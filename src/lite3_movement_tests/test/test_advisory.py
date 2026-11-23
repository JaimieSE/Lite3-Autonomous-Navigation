"""Teammate's stair/curtain advisory (~/stair_curtain/stair_curtain_advisory.py) as roam reads it."""
import json

from lite3_movement_tests.advisory import ADVISORY_TOPIC, parse_advisory


def msg(**kw):
    base = {"obstacle": "stairs", "reason": "confirmed", "stamp_ns": 1, "distance_m": 0.703,
            "distance_reference": "base_link_forward", "heading_correction_deg": -3.5,
            "proposed_action": "stop_and_request_stair_gait"}
    base.update(kw)
    return json.dumps(base)


def test_topic_matches_the_detector():
    assert ADVISORY_TOPIC == '/stair_curtain/advisory'


def test_confirmed_stairs_and_curtain_are_read():
    s = parse_advisory(msg())
    assert s == {'obstacle': 'stairs', 'distance_m': 0.703, 'heading_deg': -3.5, 'reference': 'base_link_forward'}
    c = parse_advisory(msg(obstacle='curtain', distance_m=0.9, heading_correction_deg=None,
                           distance_reference='rgb_camera_optical_depth', proposed_action='request_slow_crossing'))
    assert c['obstacle'] == 'curtain' and c['heading_deg'] == 0.0 and c['reference'] == 'rgb_camera_optical_depth'


def test_anything_not_confirmed_or_malformed_is_ignored():
    assert parse_advisory(msg(reason='awaiting_consensus')) is None
    assert parse_advisory(json.dumps({"obstacle": "unknown", "reason": "stale_camera", "stamp_ns": 0,
                                      "proposed_action": "hold"})) is None
    assert parse_advisory(msg(obstacle='ramp')) is None
    assert parse_advisory(msg(distance_m=float('nan'))) is None
    assert parse_advisory(msg(distance_m=-1)) is None
    assert parse_advisory(msg(heading_correction_deg=170)) is None
    assert parse_advisory('not json') is None
    assert parse_advisory(None) is None


def test_mission_events_are_read_and_bad_ones_ignored():
    from lite3_movement_tests.mission_events import MISSION_EVENTS_TOPIC, parse_mission_event
    assert MISSION_EVENTS_TOPIC == '/mission/events'
    obs = json.dumps({"event_id": "f1-object-0", "type": "observation", "source": "live_scene_understanding",
                      "observed_at": "2026-11-23T11:00:00Z",
                      "data": {"id": "f1-object-0", "label": "person", "confidence": 0.81,
                               "location": {"frame_id": "map", "x": 1.5, "y": -0.25, "z": 0.3}}})
    assert parse_mission_event(obs) == {'event_id': 'f1-object-0', 'type': 'observation', 'value': 'person',
                                        'confidence': 0.81, 'location': (1.5, -0.25)}
    qr = json.dumps({"event_id": "qr-ab", "type": "qr", "data": {"id": "qr-ab", "text": "ROOM 3"}})
    assert parse_mission_event(qr)['value'] == 'ROOM 3' and parse_mission_event(qr)['location'] is None
    assert parse_mission_event(json.dumps({"type": "observation", "data": {"label": "x"}})) is None   # no id
    assert parse_mission_event(json.dumps({"event_id": "a", "type": "other", "data": {}})) is None
    assert parse_mission_event('nope') is None
