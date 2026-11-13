import math
from types import SimpleNamespace

import pytest

from lite3_locomotion.exploration_preview import grid_from_message, main
from lite3_locomotion.frontiers import FrontierConfig, Grid, select_frontiers


def room(width=25, height=25, resolution=0.2):
    data = [-1] * (width * height)
    for y in range(5, height - 5):
        for x in range(5, width - 5):
            data[y * width + x] = 0
    return Grid(width, height, resolution, data)


def test_selects_free_side_goal_with_known_space_clearance():
    grid = room()
    result = select_frontiers(grid, *grid.world((12, 12)))
    assert result.status == 'frontiers_found'
    assert result.selected is not None
    for candidate in result.candidates:
        cell = grid.cell(candidate.x, candidate.y)
        for dy in range(-2, 3):
            for dx in range(-2, 3):
                assert grid.value((cell[0] + dx, cell[1] + dy)) == 0
        assert math.isfinite(candidate.yaw)
        assert candidate.cluster_cells >= 3


def test_disconnected_free_region_is_never_a_goal():
    width, height = 45, 25
    data = [-1] * (width * height)
    for left, right in ((3, 15), (28, 41)):
        for y in range(4, 21):
            for x in range(left, right):
                data[y * width + x] = 0
    grid = Grid(width, height, 0.2, data)
    result = select_frontiers(grid, *grid.world((8, 12)))
    assert result.selected
    assert all(grid.cell(c.x, c.y)[0] < 15 for c in result.candidates)


def test_obstacle_wall_blocks_reachability():
    width = height = 40
    data = [-1] * (width * height)
    for y in range(3, 37):
        for x in range(3, 37):
            data[y * width + x] = 100 if x == 20 else 0
    grid = Grid(width, height, 0.2, data)
    result = select_frontiers(grid, *grid.world((10, 20)))
    assert result.selected
    assert all(grid.cell(c.x, c.y)[0] < 20 for c in result.candidates)


def test_unknown_and_occupied_robot_locations_are_rejected():
    grid = room()
    assert select_frontiers(grid, *grid.world((1, 1))).status == 'robot_not_in_known_free_cell'
    assert select_frontiers(grid, -1, -1).selected is None
    data = list(grid.data)
    data[12 * 25 + 12] = 100
    blocked = Grid(25, 25, 0.2, data)
    assert select_frontiers(blocked, *blocked.world((12, 12))).selected is None


def test_uncertain_cells_are_not_traversable():
    grid = Grid(5, 5, 1.0, [50] * 25)
    assert select_frontiers(grid, 2.5, 2.5).selected is None


def test_map_edges_are_not_frontiers_and_no_goal_means_no_suggestion():
    grid = Grid(15, 15, 0.2, [0] * 225)
    result = select_frontiers(grid, 1.5, 1.5)
    assert result.status == 'no_frontiers_in_preview_area'
    assert result.selected is None


def test_insufficient_obstacle_clearance_rejected():
    grid = room()
    data = list(grid.data)
    data[12 * 25 + 13] = 100
    result = select_frontiers(Grid(25, 25, 0.2, data), *grid.world((12, 12)))
    assert result.status == 'robot_clearance_insufficient'


def test_no_goal_when_known_corridor_cannot_fit_clearance():
    data = [-1] * 400
    for x in range(3, 17):
        data[10 * 20 + x] = 0
    grid = Grid(20, 20, 0.2, data)
    assert select_frontiers(grid, *grid.world((10, 10))).selected is None


def test_radius_limits_goal_selection():
    grid = room()
    robot = grid.world((12, 12))
    result = select_frontiers(grid, *robot, FrontierConfig(radius=0.5))
    assert result.selected is None


def test_cell_budget_returns_no_partial_suggestion():
    grid = room()
    result = select_frontiers(grid, *grid.world((12, 12)), FrontierConfig(max_search_cells=5))
    assert result.status == 'search_limit_exceeded'
    assert result.visited_cells == 5
    assert result.selected is None


def test_fine_map_local_budget_is_bounded():
    grid = Grid(900, 900, 0.01, [0] * 810000)
    result = select_frontiers(grid, 4.5, 4.5)
    assert result.status == 'local_grid_limit_exceeded'


def test_rotated_origin_preserves_cell_world_mapping_and_selection():
    original = room()
    rotated = Grid(25, 25, 0.2, original.data, 10, -4, math.pi / 2)
    for cell in ((0, 0), (12, 12), (24, 24)):
        assert rotated.cell(*rotated.world(cell)) == cell
    result = select_frontiers(rotated, *rotated.world((12, 12)))
    assert result.selected
    assert rotated.value(rotated.cell(result.selected.x, result.selected.y)) == 0


@pytest.mark.parametrize('kwargs', [
    {'width': 0}, {'height': -1}, {'width': 1001, 'height': 1000},
    {'resolution': 0}, {'resolution': float('nan')}, {'origin_yaw': float('inf')},
    {'data': [0]}, {'data': [101] * 9}, {'data': [-2] * 9},
])
def test_invalid_grid_rejected(kwargs):
    fields = dict(width=3, height=3, resolution=1.0, data=[0] * 9)
    fields.update(kwargs)
    with pytest.raises(ValueError):
        Grid(**fields)


@pytest.mark.parametrize('args', [
    ['--ros-args', '-r', '/map:=/cmd_vel'], ['--output-topic', '/goal_pose'],
    ['--radius', '100'], ['--clearance', '-1'], ['--radius', 'nan'],
])
def test_preview_rejects_remapping_and_invalid_options(args, monkeypatch):
    import sys
    monkeypatch.setitem(sys.modules, 'rclpy', None)
    with pytest.raises(SystemExit) as error:
        main(args)
    assert error.value.code == 2


def message():
    return SimpleNamespace(
        header=SimpleNamespace(frame_id='map'), data=[0] * 9,
        info=SimpleNamespace(width=3, height=3, resolution=1.0,
                             origin=SimpleNamespace(
                                 position=SimpleNamespace(x=0.0, y=0.0),
                                 orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0))),
    )


def test_ros_map_conversion_is_offline_and_checks_frame_and_rotation():
    msg = message()
    assert grid_from_message(msg).width == 3
    msg.header.frame_id = 'odom'
    with pytest.raises(ValueError):
        grid_from_message(msg)
    msg.header.frame_id = 'map'
    msg.info.origin.orientation.x = 0.5
    with pytest.raises(ValueError):
        grid_from_message(msg)
