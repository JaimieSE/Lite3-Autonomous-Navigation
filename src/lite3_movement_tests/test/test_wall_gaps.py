"""Walls with small gaps are still walls (operator 2026-11-24)."""
from lite3_movement_tests import wall_gaps as WG
from test_course_grid import grid_of


def wall_with_gaps():
    # a wall along x = 1.0 from y = -2 to 2, with a 0.2 m hole at y = 0 and a 0.8 m doorway at y = 1.2..2.0
    return grid_of(free=[(-2.0, 3.0, -2.0, 2.0)],
                   occupied=[(0.95, 1.05, -2.0, -0.1), (0.95, 1.05, 0.1, 1.2)])


def test_a_hole_narrower_than_the_dog_is_closed_a_doorway_is_not():
    g = wall_with_gaps()
    gaps, geo = WG.gap_mask(g)
    assert WG.crosses(gaps, geo, [(0.0, 0.0), (2.0, 0.0)])          # through the 0.2 m hole: a wall
    assert not WG.crosses(gaps, geo, [(0.0, 1.6), (2.0, 1.6)])      # through the 0.8 m doorway: open
    assert not WG.crosses(gaps, geo, [(0.0, -1.0), (0.5, -1.0)])    # in the room: nothing


def test_where_the_dog_has_walked_is_never_a_wall():
    g = wall_with_gaps()
    gaps, geo = WG.gap_mask(g, walked=[(1.0, 0.0)])
    assert not WG.crosses(gaps, geo, [(0.0, 0.0), (2.0, 0.0)])


def test_gap_boxes_for_rviz_nearest_first():
    gaps, geo = WG.gap_mask(wall_with_gaps())
    boxes = WG.gap_boxes(gaps, geo, near=(1.0, 0.0))
    assert boxes
    x0, y0, x1, y1 = boxes[0]
    assert x0 <= 1.0 <= x1 + 0.05 and y0 - 0.05 <= 0.0 <= y1 + 0.05


def course_and_outside():
    # course: x -2..1 inside walls at x = 1 (with a 0.2 m hole) and y = +-2; outside floor seen over the wall x 1..3
    return grid_of(free=[(-2.0, 3.0, -2.0, 2.0)],
                   occupied=[(0.95, 1.05, -2.0, -0.1), (0.95, 1.05, 0.1, 2.0), (-2.0, 3.0, 1.95, 2.05),
                             (-2.0, 3.0, -2.05, -1.95)])


def test_goals_beyond_the_course_walls_are_outside_even_through_a_small_hole():
    import numpy as np
    g = course_and_outside()
    gaps, geo = WG.gap_mask(g)
    reach = WG.reachable_mask(g, (0.0, 0.0), gaps)
    data = np.asarray(g.data, dtype=np.int16).reshape(g.height, g.width)
    assert WG.in_course(data, reach, geo, -1.0, 1.0)                  # inside the walls
    assert not WG.in_course(data, reach, geo, 2.0, 0.0)               # seen over the wall: outside
    assert not WG.in_course(data, reach, geo, 1.2, 0.0)               # just behind the wall, past the small hole
    reach_open = WG.reachable_mask(g, (0.0, 0.0))                     # without closing the hole it leaks out
    assert reach_open[int((0.0 + 3) / 0.05), int((2.0 + 3) / 0.05)]


def test_unmapped_floor_next_to_the_course_floor_is_a_frontier_goal():
    import numpy as np
    g = grid_of(free=[(-1.0, 1.0, -1.0, 1.0)])                        # the rest is unknown
    reach = WG.reachable_mask(g, (0.0, 0.0))
    data = np.asarray(g.data, dtype=np.int16).reshape(g.height, g.width)
    geo = (0.05, g.origin_x, g.origin_y)
    assert WG.in_course(data, reach, geo, 1.3, 0.0)                   # 0.3 m into the unknown: a frontier
    assert not WG.in_course(data, reach, geo, 2.5, 0.0)               # far into the unknown: no


def test_speckles_are_not_joined_into_walls():
    """Runs 2026-11-24 17:07 / 17:17: LiDAR speckles 0.2-0.4 m apart were closed into 'walls' (8372 gap cells)."""
    import numpy as np
    speckles = [(x, y, x + 0.05, y + 0.05) for x in np.arange(-1.0, 1.0, 0.3) for y in np.arange(-1.0, 1.0, 0.3)]
    g = grid_of(free=[(-2.0, 2.0, -2.0, 2.0)], occupied=[(x0, x1, y0, y1) for x0, y0, x1, y1 in speckles])
    gaps, geo = WG.gap_mask(g)
    assert gaps.sum() == 0 and not WG.crosses(gaps, geo, [(-1.5, 0.0), (1.5, 0.0)])
