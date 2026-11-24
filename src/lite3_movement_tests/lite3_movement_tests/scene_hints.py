"""Scene understanding when roam is stuck (operator 2026-11-24).

"Trigger a scene understanding when it seems stuck to allow the dog to traverse the whole course": after looking
around finds no way on, roam asks the Pi (all camera modules are on the Raspberry Pi, group8pi) what is in front of
it, and turns the answer into hints - a curtain, a gap/opening, stairs - at a bearing relative to the robot.

Request (roam -> Pi), std_msgs/String JSON on SCENE_REQUEST_TOPIC, domain 8:
  {"request_id": str, "reason": str, "map_session": str, "pose": {"x", "y", "yaw"}, "stamp": float}
The Pi does not answer requests yet; until it does, roam uses the next Pi messages that arrive after the request:
  /group8pi/perception/detections  stair_curtain {"obstacle": "stairs"|"curtain"|"unknown", "reason", optional
                                   "heading_deg", "distance_m"}  (unknown / stale / timestamp_mismatch ignored)
  /group8pi/perception/scenes      scene {"type", "evidence", ...}; optional fields from the scene prompt
                                   (~/scene_understanding/mission_prompt.txt): "curtains": [{"image_region"}],
                                   "stairs": {"observed": bool, "image_region"}, "image_quality", and
                                   "openings": [{"kind": "gap"|"passage"|"curtain"|"stairs", "image_region" or
                                   "bearing_deg"}]
Bearings: image regions map to the camera's left/centre/right thirds (80 deg view: +-25 deg). Pure: unit-tested.
"""

import json
import math

SCENE_REQUEST_TOPIC = '/exploration/scene_request'
PI_DETECTIONS_TOPIC = '/group8pi/perception/detections'
PI_SCENES_TOPIC = '/group8pi/perception/scenes'
REGION_DEG = {'left': 25.0, 'centre': 0.0, 'center': 0.0, 'right': -25.0}
KINDS = ('curtain', 'gap', 'passage', 'stairs')
UNUSABLE_REASONS = ('unknown', 'stale', 'timestamp_mismatch', 'no_confirmed_candidate')
PRIORITY = {'curtain': 0, 'gap': 1, 'passage': 1, 'stairs': 2}     # what roam acts on first


def _num(v):
    return type(v) in (int, float) and math.isfinite(v)


def _bearing(item):
    if _num(item.get('bearing_deg')):
        return float(item['bearing_deg'])
    if _num(item.get('heading_deg')):
        return float(item['heading_deg'])
    return REGION_DEG.get(str(item.get('image_region') or 'centre').strip().lower())


def parse_scene_hints(text):
    """Hints [{kind, bearing_deg, distance_m, source}] from one Pi detections or scenes message; [] if none."""
    try:
        msg = json.loads(text)
    except (TypeError, ValueError):
        return []
    if not isinstance(msg, dict):
        return []
    scene = msg.get('scene') if isinstance(msg.get('scene'), dict) else msg
    quality = scene.get('image_quality', msg.get('image_quality'))
    if isinstance(quality, str) and quality.strip().lower() == 'unusable':
        return []
    hints = []
    sc = msg.get('stair_curtain')
    if isinstance(sc, dict) and sc.get('obstacle') in ('stairs', 'curtain') \
            and str(sc.get('reason') or '').lower() not in UNUSABLE_REASONS:
        b = _bearing(sc)
        hints.append({'kind': sc['obstacle'], 'bearing_deg': 0.0 if b is None else b,
                      'distance_m': float(sc['distance_m']) if _num(sc.get('distance_m')) else None,
                      'source': 'pi_detector'})
    for item in scene.get('curtains') or []:
        if isinstance(item, dict) and not item.get('uncertain') and _bearing(item) is not None:
            hints.append({'kind': 'curtain', 'bearing_deg': _bearing(item), 'distance_m': None, 'source': 'pi_scene'})
    stairs = scene.get('stairs')
    if isinstance(stairs, dict) and stairs.get('observed') is True and _bearing(stairs) is not None:
        hints.append({'kind': 'stairs', 'bearing_deg': _bearing(stairs), 'distance_m': None, 'source': 'pi_scene'})
    for item in scene.get('openings') or []:
        if isinstance(item, dict) and item.get('kind') in KINDS and _bearing(item) is not None:
            hints.append({'kind': item['kind'], 'bearing_deg': _bearing(item),
                          'distance_m': float(item['distance_m']) if _num(item.get('distance_m')) else None,
                          'source': 'pi_scene'})
    hints = [h for h in hints if abs(h['bearing_deg']) <= 90.0]
    return sorted(hints, key=lambda h: (PRIORITY[h['kind']], abs(h['bearing_deg'])))


def pi_stair_curtain(text):
    """The detector reading inside one Pi detections message, as the advisory JSON roam already acts on (same
    detector as ~/stair_curtain on the Pi: obstacle, reason, distance_m, distance_reference,
    heading_correction_deg), plus "capture_stamp_ns": the reading's own frame
    (stair_curtain.stamp_ns). None if the message has no such reading."""
    try:
        msg = json.loads(text)
    except (TypeError, ValueError):
        return None
    sc = msg.get('stair_curtain') if isinstance(msg, dict) else None
    if not isinstance(sc, dict):
        return None
    # stair_curtain.stamp_ns is the colour frame the reading came from (Pi session 2026-11-24); the top-level
    # capture_stamp_ns belongs to the YOLO frame and can be newer. Fall back to it only when stamp_ns is missing.
    stamp = sc.get('stamp_ns') if type(sc.get('stamp_ns')) is int else msg.get('capture_stamp_ns')
    return json.dumps(dict(sc, capture_stamp_ns=stamp if type(stamp) is int else None))


def scene_request(request_id, reason, pose, map_session, stamp):
    return json.dumps({'request_id': request_id, 'reason': reason, 'map_session': map_session,
                       'pose': {'x': round(pose[0], 3), 'y': round(pose[1], 3), 'yaw': round(pose[2], 4)},
                       'stamp': round(stamp, 3)})


# ---------------------------------------------------------------------------------- scene hold (Pi, nav_hold.py)
# Operator 2026-11-24: while the Pi asks Claude about a scene (person, object, warning sign), roam stops at the next
# safe point, squares up to the 1 x 1 m course grid and waits for the result (at most the hold's expiry, 6 s) -
# except on stairs, curtains and rollers. Contract: ~/live_mission/PI_SCENE_EVENTS.md.
HOLD_TOPIC = '/group8/navigation/hold_request'
HOLD_MAX_AGE_S = 30.0          # a hold published longer ago than this is stale whatever its expiry says


def active_hold(text, now_wall_ns):
    """{'request_id', 'reason', 'encounter_ids', 'remaining_s'} while the hold applies; None when clear, expired,
    stale or unreadable (an expired message counts as clear, so a crashed hold node never strands the dog)."""
    try:
        msg = json.loads(text)
    except (TypeError, ValueError):
        return None
    if not isinstance(msg, dict) or msg.get('hold') is not True:
        return None
    expires, stamp = msg.get('expires_wall_ns'), msg.get('stamp_ns')
    if type(expires) is not int or now_wall_ns >= expires:
        return None
    if type(stamp) is int and now_wall_ns - stamp > HOLD_MAX_AGE_S * 1e9:
        return None
    return {'request_id': msg.get('request_id'), 'reason': msg.get('reason'),
            'encounter_ids': msg.get('encounter_ids') or [], 'remaining_s': round((expires - now_wall_ns) / 1e9, 2)}
