"""Mission events from the teammate's live recorder (~/live_mission, rgbd_camera.py / live_store.py).

Published on MISSION_EVENTS_TOPIC (domain 8, std_msgs/String JSON) when the recorder runs with
--publish-mission-events:
  {"event_id": str, "type": "observation" | "qr", "source": "live_scene_understanding",
   "observed_at": str, "data": {...}}
  observation data: id, label, confidence, evidence_ref, verified, xyxy,
                    location {frame_id: "map", x, y, z} (when depth + TF allowed one), location_source
  qr data:          id, text, evidence_ref, decoder, corners, location (when known)
A QR found again with a position arrives as a second event "<id>-location".
Operator 2026-11-23: roam only LOGS these (no change to how it moves). Pure: unit-tested.
"""

import json
import math

MISSION_EVENTS_TOPIC = '/mission/events'


def parse_mission_event(text):
    """{'event_id', 'type', 'value', 'confidence', 'location'} or None. location = (x, y) in map or None."""
    try:
        event = json.loads(text)
    except (TypeError, ValueError):
        return None
    if not isinstance(event, dict) or not isinstance(event.get('data'), dict):
        return None
    kind, data, event_id = event.get('type'), event['data'], event.get('event_id')
    if kind not in ('observation', 'qr') or not isinstance(event_id, str) or not event_id:
        return None
    value = data.get('label') if kind == 'observation' else data.get('text')
    if not isinstance(value, str) or not value:
        return None
    try:
        confidence = float(data.get('confidence', 1.0))
    except (TypeError, ValueError):
        confidence = None
    location = None
    loc = data.get('location')
    if isinstance(loc, dict) and loc.get('frame_id', 'map') == 'map':
        try:
            x, y = float(loc['x']), float(loc['y'])
            if math.isfinite(x) and math.isfinite(y):
                location = (x, y)
        except (KeyError, TypeError, ValueError):
            pass
    return {'event_id': event_id, 'type': kind, 'value': value, 'confidence': confidence, 'location': location}
