import json

from lite3_movement_tests.scene_hints import parse_scene_hints, scene_request


def test_pi_detector_curtain_and_stairs_are_hints_unknowns_are_not():
    det = {'frame': 3, 'stair_curtain': {'obstacle': 'curtain', 'reason': 'confirmed', 'heading_deg': -10.0,
                                         'distance_m': 0.9}}
    h = parse_scene_hints(json.dumps(det))
    assert h == [{'kind': 'curtain', 'bearing_deg': -10.0, 'distance_m': 0.9, 'source': 'pi_detector'}]
    for reason in ('timestamp_mismatch', 'stale', 'no_confirmed_candidate'):
        det['stair_curtain']['reason'] = reason
        assert parse_scene_hints(json.dumps(det)) == []
    assert parse_scene_hints(json.dumps({'stair_curtain': {'obstacle': 'unknown', 'reason': 'x'}})) == []


def test_scene_prompt_fields_and_openings_map_regions_to_bearings():
    msg = {'scene': {'type': 'corridor', 'evidence': ['wall'], 'curtains': [{'image_region': 'left'}],
                     'stairs': {'observed': True, 'image_region': 'right'},
                     'openings': [{'kind': 'gap', 'image_region': 'centre'}, {'kind': 'door?', 'bearing_deg': 0}]}}
    h = parse_scene_hints(json.dumps(msg))
    assert [(x['kind'], x['bearing_deg']) for x in h] == [('curtain', 25.0), ('gap', 0.0), ('stairs', -25.0)]
    msg['scene']['image_quality'] = 'unusable'
    assert parse_scene_hints(json.dumps(msg)) == []
    assert parse_scene_hints('not json') == [] and parse_scene_hints('[1]') == []
    # the Pi's current scene messages (room type + evidence only) carry no hint
    assert parse_scene_hints(json.dumps({'scene': {'type': 'kitchen', 'evidence': ['stove']}})) == []


def test_request_format():
    r = json.loads(scene_request('s1', 'stuck', (1.0, 2.0, 0.5), 'run-1', 12.0))
    assert r['request_id'] == 's1' and r['pose'] == {'x': 1.0, 'y': 2.0, 'yaw': 0.5} and r['map_session'] == 'run-1'
