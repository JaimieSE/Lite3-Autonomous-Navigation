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
