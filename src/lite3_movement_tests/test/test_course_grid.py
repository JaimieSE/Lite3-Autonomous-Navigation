import math

import pytest

from lite3_locomotion.frontiers import Grid
from lite3_movement_tests.course_grid import CourseGrid
from lite3_movement_tests.coverage import CameraCoverage

RES = 0.05


def grid_of(free=(), occupied=(), x0=-3.0, y0=-3.0, size_m=8.0):
    """SLAM grid: unknown everywhere except the free / occupied rectangles (x0, x1, y0, y1)."""
    n = int(size_m / RES)
    data = [-1] * (n * n)
    for rects, val in ((free, 0), (occupied, 100)):
        for ax, bx, ay, by in rects:
            for j in range(n):
                y = y0 + (j + 0.5) * RES
                if ay <= y <= by:
                    for i in range(n):
                        x = x0 + (i + 0.5) * RES
                        if ax <= x <= bx:
                            data[j * n + i] = val
    return Grid(n, n, RES, data, x0, y0)


def test_alignment_follows_the_start_pose_not_the_map_axes():
    g = CourseGrid(1.0, 2.0, math.radians(90))
    assert g.cell(1.0, 2.0) == (0, 0)
    assert g.cell(1.0, 3.0) == (1, 0)                # +X of the grid is the start heading (map +Y here)
    assert g.cell(0.0, 2.0) == (0, 1)
    x, y = g.centre(2, 0)
    assert math.isclose(x, 1.0, abs_tol=1e-9) and math.isclose(y, 4.0, abs_tol=1e-9)
    with pytest.raises(ValueError):
        CourseGrid(0, 0, 0, cell_m=0.1)


def test_mapped_visited_and_seen_are_distinct():
    slam = grid_of(free=[(-0.5, 2.5, -0.5, 0.5)])
    cov = CameraCoverage((0.0, 0.0), 4.0)
    g = CourseGrid(0.0, 0.0, 0.0)
    g.update(slam, cov, breadcrumbs=[(0.0, 0.0)])
    a, b = g.cells[(0, 0)], g.cells[(2, 0)]
    assert a['visited'] and not b['visited']
    assert a['mapped'] > 0.9 and b['mapped'] > 0.4
    assert a['seen'] == 0.0 and b['seen'] == 0.0      # mapped and walked, but the camera saw nothing yet
    cov.update((0.0, 0.0, 0.0), slam)                 # look along +X
    g.update(slam, cov, breadcrumbs=[(0.0, 0.0)])
    assert g.cells[(2, 0)]['seen'] > 0.5 and not g.cells[(2, 0)]['visited']   # seen without being walked


def test_unfinished_means_unseen_floor_or_an_open_way_into_unmapped_space():
    slam = grid_of(free=[(-0.5, 1.8, -0.5, 0.5)])     # the floor runs on past the cell edge into (2, 0)
    cov = CameraCoverage((0.0, 0.0), 4.0)
    g = CourseGrid(0.0, 0.0, 0.0)
    g.update(slam, cov)
    assert g.unfinished((1, 0))                       # not seen yet
    for yaw in (0.0, math.pi / 2, math.pi, -math.pi / 2):
        for x in (0.0, 1.0):
            cov.update((x, 0.0, yaw), slam)
    g.update(slam, cov)
    assert g.cells[(1, 0)]['seen'] >= 0.7
    assert g.cells[(2, 0)]['mapped'] < 0.5
    assert g.unfinished((1, 0))                       # seen, but open floor runs on into mostly unmapped (2, 0)
    walled = grid_of(free=[(-0.5, 1.4, -0.5, 0.5)], occupied=[(1.4, 1.6, -0.6, 0.6), (-0.6, 1.6, 0.5, 0.65),
                                                              (-0.6, 1.6, -0.65, -0.5), (-0.65, -0.5, -0.6, 0.6)])
    cov2 = CameraCoverage((0.0, 0.0), 4.0)
    for yaw in (0.0, math.pi / 2, math.pi, -math.pi / 2):
        for x in (0.0, 1.0):
            cov2.update((x, 0.0, yaw), walled)
    g.update(walled, cov2)
    assert not g.unfinished((1, 0))                  # walled in and looked at: finished


def test_connection_states_and_failure_memory():
    slam = grid_of(free=[(-0.5, 1.5, -0.5, 0.5)], occupied=[(-0.5, 1.5, 0.48, 0.6)])
    g = CourseGrid(0.0, 0.0, 0.0)
    g.update(slam)
    assert g.connection((0, 0), (1, 0), 0.0) == 'open'
    assert g.connection((0, 0), (0, 1), 0.0) == 'blocked'           # wall along the shared edge
    assert g.connection((1, 0), (2, 0), 0.0) == 'unknown'
    g.note_failure(0.0, (0.0, 0.0), (1.0, 0.0), 'temporary')
    assert g.connection((0, 0), (1, 0), 1.0) == 'blocked'
    assert g.connection((0, 0), (1, 0), 11.0) == 'open'             # temporary: retried after 10 s
    g.note_failure(20.0, (0.0, 0.0), (2.5, 0.2), 'persistent')      # far goal: blocks the first connection
    assert g.connection((0, 0), (1, 0), 21.0) == 'blocked'
    g.note_failure(200.0, (0.0, 0.0), (1.0, 0.0), 'persistent')
    assert g.connection((0, 0), (1, 0), 1000.0) == 'blocked'        # bounded: 2 persistent failures -> stays


def test_return_point_moves_off_a_blocked_centre():
    slam = grid_of(free=[(1.0, 3.0, -1.0, 1.0)], occupied=[(1.9, 2.1, -0.1, 0.1)])   # pillar on the centre
    g = CourseGrid(0.0, 0.0, 0.0)
    g.update(slam)
    p = g.return_point((2, 0))
    assert p is not None and math.dist(p, (2.0, 0.0)) > 0.2
    assert g.return_point((5, 5)) is None


def test_map_reset_clears_memory_and_markers_are_bounded():
    slam = grid_of(free=[(-0.5, 3.5, -0.5, 0.5)])
    g = CourseGrid(0.0, 0.0, 0.0, session='a')
    g.update(slam, breadcrumbs=[(0.0, 0.0)])
    g.note_failure(0.0, (0.0, 0.0), (1.0, 0.0), 'persistent')
    m = g.markers(0.0, robot_xy=(0.0, 0.0))
    assert {x['type'] for x in m} >= {'cube_list', 'line_list', 'text'}
    assert sum(1 for x in m if x['type'] == 'text') <= 5
    assert all(len(x['text']) <= 20 for x in m)
    s = g.summary()
    assert s['cells_visited'] == 1 and s['blocked_connections'] == 1
    g.reset('b')
    assert g.cells == {} and g.edge_failures == {} and g.session == 'b'


def test_a_visited_cell_is_explored_never_a_place_to_come_back_to():
    slam = grid_of(free=[(-0.5, 2.5, -0.5, 0.5)])
    g = CourseGrid(0.0, 0.0, 0.0)
    g.update(slam, CameraCoverage((0.0, 0.0), 4.0), breadcrumbs=[(1.0, 0.0)])
    assert not g.unfinished((1, 0)) and g.unfinished((2, 0))       # walked through (1, 0); (2, 0) not yet
    g.note_failure(0.0, (0.0, 0.0), (1.0, 0.0), 'terrain')
    for t in (1.0, 100.0, 1000.0):
        g.note_failure(t, (0.0, 0.0), (1.0, 0.0), 'terrain')
    assert g.connection((0, 0), (1, 0), 5000.0) == 'open'           # terrain trouble never blocks for good
