"""Camera coverage (coverage.py): what the front camera has seen, and unseen floor nearby."""
import math

from lite3_locomotion.frontiers import Grid
from lite3_movement_tests.coverage import CAMERA_RANGE_M, CameraCoverage


def grid(free=(), occupied=(), size=8.0, res=0.05):
    """Unknown grid, origin (-4, -4); boxes (x0, x1, y0, y1)."""
    n = int(size / res)
    data = [-1] * (n * n)
    for value, boxes in ((0, free), (100, occupied)):
        for x0, x1, y0, y1 in boxes:
            for j in range(n):
                for i in range(n):
                    x, y = -4 + (i + .5) * res, -4 + (j + .5) * res
                    if x0 <= x <= x1 and y0 <= y <= y1:
                        data[j * n + i] = value
    return Grid(n, n, res, data, -4.0, -4.0)


def test_marks_the_view_ahead_not_behind_or_beyond_range():
    g = grid(free=[(-3.5, 3.5, -3.5, 3.5)])
    c = CameraCoverage((0.0, 0.0), 4.0)
    c.update((0.0, 0.0, 0.0), g)
    assert c.is_seen(1.5, 0.0) and c.is_seen(2.5, 0.5)
    assert not c.is_seen(-1.0, 0.0)                         # behind
    assert not c.is_seen(0.3 + CAMERA_RANGE_M + 0.3, 0.0)   # beyond range
    assert not c.is_seen(1.0, 1.5)                          # outside the 80 deg field of view at 1 m
    assert 4.0 < c.seen_m2() < 8.0                          # ~ 80/360 * pi * 3^2 = 6.3 m^2


def test_walls_block_the_view_but_are_seen_themselves():
    g = grid(free=[(-3.5, 3.5, -3.5, 3.5)], occupied=[(1.0, 1.1, -2.0, 2.0)])
    c = CameraCoverage((0.0, 0.0), 4.0)
    c.update((0.0, 0.0, 0.0), g)
    assert c.is_seen(1.05, 0.0)                              # the wall face (targets hang on walls)
    assert not c.is_seen(2.0, 0.0)                           # behind the wall


def test_unseen_floor_by_side():
    g = grid(free=[(-3.5, 3.5, -3.5, 3.5)])
    c = CameraCoverage((0.0, 0.0), 4.0)
    full = c.unseen_free_m2(g, (0.0, 0.0), 2.5)
    assert abs(full - math.pi * 2.5 ** 2) < 0.6
    c.update((0.0, 0.0, math.pi / 2), g)                     # looked left
    left = c.unseen_free_m2(g, (0.0, 0.0), 2.5, math.pi / 2, math.radians(60))
    right = c.unseen_free_m2(g, (0.0, 0.0), 2.5, -math.pi / 2, math.radians(60))
    assert right > 4.0 and left < right / 1.5      # 120 deg sector vs 80 deg view: some left stays unseen
