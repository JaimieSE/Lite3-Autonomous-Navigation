import math

import pytest

from lite3_movement_tests import goal_selection as G


def cand(x, y, source, yaw=0.0, **terms):
    return G.Candidate(x, y, yaw, source, reason=source, terms=terms, goal={'x': x, 'y': y, 'yaw': yaw})


def test_nearby_proposals_merge_and_keep_every_reason():
    merged = G.merge([cand(1.0, 0.0, 'frontier', info=2.0, travel=1.0),
                      cand(1.2, 0.1, 'grid_return', grid=1.0, travel=1.2),
                      cand(3.0, 0.0, 'corridor', info=1.0)])
    assert len(merged) == 2
    m = merged[0]
    assert m.sources == ['frontier', 'grid_return'] and m.reasons == ['frontier', 'grid_return']
    assert m.terms['info'] == 2.0 and m.terms['grid'] == 1.0 and m.terms['travel'] == 1.0   # max gains, min costs
    assert m.terms['agree'] == 1.0
    assert G.merge([cand(1, 0, 'a'), cand(1, 0, 'b', yaw=math.pi)]).__len__() == 2          # opposite headings


def test_evaluate_scores_with_a_logged_breakdown_and_refuses_worthless_goals():
    d = G.evaluate([cand(1, 0, 'corridor', info=1.0, travel=1.0),
                    cand(0, 2, 'grid_return', grid=1.0, camera=2.0, travel=2.0)])
    assert d.chosen.source == 'grid_return'
    assert d.breakdown['grid'] == 1.5 and d.breakdown['travel'] == -0.8
    assert math.isclose(d.score, sum(d.breakdown.values()))
    s = d.summary()
    assert s['chosen']['source'] == 'grid_return' and len(s['alternatives']) == 1
    none = G.evaluate([cand(1, 0, 'corridor', travel=3.0)])
    assert none.chosen is None and none.ranked                                            # nothing worth it
    assert G.evaluate([]).chosen is None
    assert G.evaluate([cand(1, 0, 'x', info=100.0)]).breakdown['info'] == G.GAIN_CAP_M2   # capped gain


def test_weights_are_configurable_and_checked():
    w = G.parse_weights('info=2, grid=0')
    assert w['info'] == 2 and w['grid'] == 0 and w['camera'] == G.DEFAULT_WEIGHTS['camera']
    d = G.evaluate([cand(1, 0, 'corridor', info=1.0), cand(0, 2, 'grid_return', grid=1.0)], w)
    assert d.chosen.source == 'corridor'
    for bad in ('nope=1', 'info=-1', 'info=nan'):
        with pytest.raises(ValueError):
            G.parse_weights(bad)


def test_reselection_has_hysteresis():
    r = G.Reselection()
    assert r.should_switch(10.0, 1.0, 1.4, started_at=0.0, to_goal_m=3.0) == (False, 'not clearly better')
    assert r.should_switch(3.0, 1.0, 9.0, started_at=0.0, to_goal_m=3.0)[0] is False          # committed
    assert r.should_switch(10.0, 1.0, 9.0, started_at=0.0, to_goal_m=0.5)[0] is False         # nearly there
    ok, why = r.should_switch(10.0, 1.0, 9.0, started_at=0.0, to_goal_m=3.0)
    assert ok and 'better' in why
    r.switched(10.0)
    assert r.should_switch(15.0, 1.0, 9.0, started_at=0.0, to_goal_m=3.0)[0] is False         # switched recently
    assert r.should_switch(15.0, 1.0, 0.0, started_at=14.0, to_goal_m=3.0, invalid=True)[0]   # invalid: always
