"""Shared exploration record (exploration_record.py): states, failures, inspections, session, markers."""
import math

from lite3_movement_tests import exploration_record as E


def rec():
    return E.ExplorationRecord('run-A', now=0.0)


def test_failure_classification():
    assert E.classify_failure('Nav2 rejected the walk goal') == 'temporary'
    assert E.classify_failure('LiDAR/terrain status stale') == 'temporary'
    assert E.classify_failure('Nav2 cancellation has no terminal result') == 'temporary'
    assert E.classify_failure('walk ended after 0.02 m (obstacle ahead (walking standoff))') == 'terrain'
    assert E.classify_failure('Nav2 ABORTED') == 'terrain'                    # uneven ground is not a wall
    assert E.classify_failure('Nav2 path goes through a gap in a wall') == 'persistent'
    assert E.classify_failure('something new') == 'persistent'


def test_repeated_unsuccessful_goal_is_avoided_and_then_given_up():
    r = rec()
    r.fail(0.0, 1.0, 1.0, 'Nav2 path goes through a gap in a wall')
    assert r.avoided(1.1, 1.0, 1.0) and r.avoided(1.0, 1.0, E.PERSIST_AVOID_S - 1)
    assert not r.avoided(1.0, 1.0, E.PERSIST_AVOID_S + 1)          # may retry once later
    f = r.fail(E.PERSIST_AVOID_S + 2, 1.0, 1.0, 'Nav2 path goes through a gap in a wall')
    assert f['attempts'] == 2 and 'for this session' in f['retry']
    assert r.avoided(1.0, 1.0, 10_000.0)                            # exhausted for good
    assert not r.avoided(3.0, 3.0, 1.0)                             # elsewhere unaffected


def test_temporary_failures_retry_after_a_delay_and_are_bounded():
    r = rec()
    for k in range(E.TEMP_MAX_ATTEMPTS):
        t = k * (E.TEMP_RETRY_AFTER_S + 1)
        assert not r.avoided(2.0, 0.0, t)                           # retry allowed again
        f = r.fail(t, 2.0, 0.0, 'Nav2 rejected the walk goal')
        assert f['category'] == 'temporary' and r.avoided(2.0, 0.0, t + 1)
    assert f['attempts'] == E.TEMP_MAX_ATTEMPTS and 'given up' in f['retry']
    assert r.avoided(2.0, 0.0, 10_000.0)
    r.succeeded(2.0, 0.0)
    assert not r.avoided(2.0, 0.0, 10_000.0)                        # a later success clears it


def test_junction_branches_states_and_exhausted_branch_weights():
    r = rec()
    j = r.note_decision((0.0, 0.0, 0.0), [0.0, math.pi / 2, math.pi])
    assert j and {b['state'] for b in j['branches']} == {'UNVISITED'} and len(j['branches']) == 3
    assert r.note_decision((5.0, 5.0, 0.0), [0.0, 0.1]) is None      # one direction: no junction
    r.start_branch((0.1, 0.0, 0.0), 0.05)
    assert r._active()['state'] == 'ACTIVE'
    assert r.end_branch('walked', None) == 'DEFERRED'               # no camera confirmation
    assert r.branch_factor((0.0, 0.0, 0.0), 0.0) == E.DEFERRED_FACTOR
    r.start_branch((0.0, 0.0, 0.0), math.pi / 2)
    assert r.confirm(1.0, {'usable': True}) == 'branch'
    assert r.end_branch('walked', None) == 'INSPECTED'
    assert r.branch_factor((0.0, 0.0, 0.0), math.pi / 2) == 0.0      # exhausted
    r.start_branch((0.0, 0.0, 0.0), math.pi)
    assert r.end_branch('failed', 'temporary') == 'UNVISITED'
    r.start_branch((0.0, 0.0, 0.0), math.pi)
    assert r.end_branch('failed', 'persistent') == 'BLOCKED' and r.branch_factor((0, 0, 0), math.pi) == 0.0
    assert r.branch_factor((3.0, 0.0, 0.0), math.pi) == 1.0          # away from the junction: no rule


def req(rid, missions=('M03',), x=1.0, y=2.0, yaw=0.5, priority=1.0, **kw):
    d = {'request_id': rid, 'mission_ids': list(missions), 'evidence_ref': 'frame-7',
         'viewing_pose': {'x': x, 'y': y, 'yaw': yaw}, 'priority': priority}
    d.update(kw)
    return d


def test_inspection_requests_duplicates_updates_and_rejections():
    r = rec()
    assert r.add_inspection(0.0, req('a')) == ('added', 'a')
    assert r.add_inspection(1.0, req('a')) == ('duplicate', 'a')                   # same request again
    assert r.add_inspection(1.0, req('b', x=1.1)) == ('duplicate', 'a')            # same missions, same spot
    assert r.add_inspection(1.0, req('c', missions=('M05',), x=1.1))[0] == 'added'  # other mission: separate
    assert r.add_inspection(2.0, req('a', priority=5.0)) == ('updated', 'a')
    assert r.inspections['a']['priority'] == 5.0 and len(r.inspections) == 2
    assert r.add_inspection(2.0, {'mission_ids': ['M01']})[0].startswith('rejected')
    assert r.add_inspection(2.0, req('d', x=float('nan')))[0].startswith('rejected')
    assert r.add_inspection(2.0, req('e', map_session='run-B'))[0].startswith('rejected')


def test_inspection_lifecycle_confirmed_timed_out_and_navigation_failures():
    r = rec()
    r.add_inspection(0.0, req('a', priority=1.0))
    r.add_inspection(0.0, req('b', missions=('M01',), x=4.0, priority=3.0))
    nxt = r.next_inspection((0.0, 0.0, 0.0), 1.0, lambda x, y: True)
    assert nxt['id'] == 'b'                                                  # priority first
    assert r.next_inspection((0, 0, 0), 1.0, lambda x, y: x < 2.0)['id'] == 'a'   # b unreachable
    r.activate_inspection(1.0, 'a')
    assert r.confirm(1.5, {'usable': True, 'request_id': 'a'}) is None       # not at the viewpoint yet
    r.arrived_at_viewpoint(2.0)
    assert r.confirm(2.5, {'usable': True, 'request_id': 'zzz'}) is None     # other request: ignored
    assert r.confirm(3.0, {'usable': True, 'request_id': 'a'}) == 'inspection'
    assert r.inspections['a']['state'] == 'INSPECTED' and r.active_inspection is None
    r.activate_inspection(4.0, 'b')
    r.arrived_at_viewpoint(5.0)
    r.finish_inspection(13.0, 'DEFERRED', 'no usable-camera confirmation in 8 s')
    assert r.inspections['b']['state'] == 'DEFERRED'
    assert r.next_inspection((0, 0, 0), 14.0, lambda x, y: True) is None        # not straight away
    later = 13.0 + E.INSPECTION_DEFER_RETRY_S
    assert r.next_inspection((0, 0, 0), later, lambda x, y: True)['id'] == 'b'   # one more look later
    r.activate_inspection(later, 'b')
    r.finish_inspection(later + 1, 'UNVISITED', 'walk failed')
    assert r.inspections['b']['state'] == 'BLOCKED'                          # 2 navigation attempts used


def test_map_reset_invalidates_positions_but_keeps_requests():
    r = rec()
    r.observe_map(12.0, 1.0)
    r.add_breadcrumb(0.0, 0.0); r.add_breadcrumb(1.0, 0.0)
    r.note_decision((0, 0, 0), [0.0, math.pi])
    r.fail(1.0, 2.0, 2.0, 'wall in the way')
    r.add_inspection(1.0, req('a'))
    assert r.observe_map(11.0, 2.0) is None                                  # normal fluctuation
    why = r.observe_map(3.0, 3.0)                                            # slam reset
    assert why and 'map reset' in why
    assert not r.breadcrumbs and not r.junctions and not r.failures
    assert r.inspections['a']['state'] == 'DEFERRED' and r.inspections['a']['valid'] is False
    assert r.next_inspection((0, 0, 0), 4.0, lambda x, y: True) is None      # until re-sent for this map
    assert r.add_inspection(5.0, req('a', x=1.5))[0] == 'updated' and r.inspections['a']['valid']
    assert 'session changed' in r.observe_map(3.0, 6.0, session='run-B') and r.session == 'run-B'


def test_markers_reflect_the_record_with_labels_and_bounded_counts():
    r = rec()
    r.set_destination(0.0, {'kind': 'walk', 'x': 1.0, 'y': 0.0, 'yaw': 0.0}, 'walk', 'frontier: largest unseen gain')
    for k in range(5):
        r.add_breadcrumb(k * 0.5, 0.0)
    r.note_decision((0, 0, 0), [0.0, math.pi / 2])
    for k in range(E.MAX_FAILURE_MARKERS + 10):
        r.fail(float(k), k * 1.0, 5.0, 'wall in the way')
    r.add_inspection(0.0, req('a'))
    m = E.build_markers(r, labels=True)
    ns = {x['ns'] for x in m}
    assert {'destination', 'destination_labels', 'breadcrumbs', 'junctions', 'junction_labels', 'branches',
            'branch_labels', 'failures', 'failure_labels', 'inspections', 'inspection_labels'} <= ns
    assert any('frontier' in x['text'] for x in m if x['ns'] == 'destination_labels')
    assert sum(1 for x in m if x['ns'] == 'failures') == E.MAX_FAILURE_MARKERS
    assert any(x['text'].startswith('#1 M03') for x in m if x['ns'] == 'inspection_labels')
    assert any('UNVISITED' in x['text'] for x in m if x['ns'] == 'branch_labels')
    r.reset(1.0, 'test')
    after = {x['ns'] for x in E.build_markers(r, labels=True)}
    assert not after & {'breadcrumbs', 'junctions', 'failures', 'inspections'}       # nothing stale survives



def test_default_overlay_is_shapes_only():
    """Operator 2026-11-23: boxes and crosses instead of words; text only with labels=True."""
    r = rec()
    r.set_destination(0.0, {'kind': 'walk', 'x': 1.0, 'y': 0.0, 'yaw': 0.0}, 'walk', 'frontier')
    r.note_decision((0, 0, 0), [0.0, math.pi / 2, math.pi])
    j = r.junctions[0]
    j['branches'][0]['state'], j['branches'][1]['state'], j['branches'][2]['state'] = 'INSPECTED', 'DEFERRED', 'BLOCKED'
    f = r.fail(0.0, 2.0, 2.0, 'wall in the way'); r.fail(1.0, 2.0, 2.0, 'wall in the way')
    r.add_inspection(0.0, req('a'))
    m = E.build_markers(r, labels=False)
    assert not [x for x in m if x['type'] == 'text']                                      # no words
    dest = next(x for x in m if x['ns'] == 'destination' and x['id'] == 0)
    assert dest['type'] == 'line_list' and len(dest['points']) == 24                       # wireframe box: 12 edges
    ends = {x['type'] for x in m if x['ns'] == 'branch_ends'}
    assert ends == {'cube', 'line_list'}                                                  # INSPECTED cube; DEFERRED box, BLOCKED X
    fail = next(x for x in m if x['ns'] == 'failures')
    assert len(fail['points']) == 24 + 4                                                  # box + X
    assert sum(1 for x in m if x['ns'] == 'failure_attempts') == 2                        # 2 attempts: 2 cubes
    box = next(x for x in m if x['ns'] == 'inspections')
    assert box['type'] == 'line_list' and len(box['points']) == 24
    assert not [x for x in m if x['ns'] == 'inspection_fill']                             # not yet confirmed
    r.inspections['a']['state'] = 'INSPECTED'
    assert [x for x in E.build_markers(r, labels=False) if x['ns'] == 'inspection_fill']                # confirmed: filled
    r.set_destination(0.0, {'kind': 'spin', 'x': 0.0, 'y': 0.0, 'yaw': 1.0}, 'turn', 'look-around')
    assert next(x for x in E.build_markers(r) if x['ns'] == 'destination' and x['id'] == 0)['type'] == 'cylinder'


def test_default_overlay_has_short_tags_on_every_shape():
    """Operator 2026-11-23: annotate the shapes, without taking much space."""
    r = rec()
    r.set_destination(0.0, {'kind': 'walk', 'x': 1.0, 'y': 0.0, 'yaw': 0.0}, 'walk', 'frontier: nearest unexplored edge')
    r.note_decision((0, 0, 0), [0.0, math.pi / 2, math.pi])
    r.junctions[0]['branches'][2]['state'] = 'BLOCKED'
    r.fail(0.0, 2.0, 2.0, 'wall in the way'); r.fail(1.0, 2.0, 2.0, 'wall in the way')
    r.add_inspection(0.0, req('a'))
    m = E.build_markers(r)
    tags = [x for x in m if x['type'] == 'text']
    assert tags and {x['ns'] for x in tags} == {'tags'}                       # one namespace: one tick box in RViz
    texts = {x['text'] for x in tags}
    assert 'GOAL frontier' in texts and 'J1' in texts and 'blocked' in texts and 'AVOID x2' in texts
    assert any(t.endswith('to check') for t in texts)
    assert all(x['scale'][2] <= 0.08 and len(x['text']) <= 20 for x in tags)  # small and short
    assert len({x['id'] for x in tags}) == len(tags)
    shapes = [x for x in m if x['type'] != 'text']
    assert shapes == E.build_markers(r, labels=False)


def test_terrain_trouble_is_never_inaccessible_and_walked_spots_stay_reachable():
    """Operator 2026-11-24: uneven terrain is not a wall; where the dog has been is reachable."""
    assert E.classify_failure('walk ended after 0.02 m (terrain proximity hold)') == 'terrain'
    assert E.classify_failure('no progress for 40 s') == 'terrain'
    assert E.classify_failure('Nav2 path goes through a gap in a wall') == 'persistent'
    assert E.classify_failure('Nav2 rejected the walk goal') == 'temporary'
    r = rec()
    for k in range(6):                                            # terrain: never given up, only a growing wait
        r.fail(k * 1000.0, 2.0, 2.0, 'no progress for 40 s')
    f = next(iter(r.failures.values()))
    assert f['category'] == 'terrain' and r.avoided(2.0, 2.0, 5000.0 + 10) \
        and not r.avoided(2.0, 2.0, 5000.0 + E.TERRAIN_RETRY_MAX_S + 1)
    r = rec()
    r.add_breadcrumb(0.0, 0.0); r.add_breadcrumb(0.3, 0.0)
    f = r.fail(0.0, 0.3, 0.2, 'some unknown failure')            # next to the walked path
    assert f['category'] == 'terrain'
    for k in range(5):
        r.fail(k * 1000.0, 0.3, 0.2, 'some unknown failure')
    assert not r.avoided(0.3, 0.2, 5000.0 + E.TERRAIN_RETRY_MAX_S + 1)
    w = r.fail(0.0, 0.3, 0.2, 'Nav2 path goes through a gap in a wall')
    assert w['category'] == 'persistent'                          # a wall stays a wall
