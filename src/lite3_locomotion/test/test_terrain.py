"""Synthetic-scene tests for local terrain assessment (no ROS)."""

import math

import numpy as np
import pytest

from lite3_locomotion import terrain as T

FLOOR = -0.33           # level-frame floor when standing (leg extension ~0.327 m)
P = T.TerrainParams()


def plane(x0, x1, y0, y1, z, step=0.025):
    xs, ys = np.meshgrid(np.arange(x0, x1, step), np.arange(y0, y1, step))
    return np.stack([xs.ravel(), ys.ravel(), np.full(xs.size, z)], 1)


def wall(x, y0, y1, z0, z1, step=0.025):
    ys, zs = np.meshgrid(np.arange(y0, y1, step), np.arange(z0, z1, step))
    return np.stack([np.full(ys.size, x), ys.ravel(), zs.ravel()], 1)


def floor(x0=0.3, x1=2.9):
    return plane(x0, x1, -1.0, 1.0, FLOOR)


def grid_of(points, leg_ref=FLOOR):
    floor_z, source = T.estimate_floor(points, leg_ref, P)
    return T.build_grid(points, floor_z, P, source)


def summary(points):
    return T.corridor_summary(grid_of(points), P)


def test_flat_floor_is_clear():
    s = summary(floor())
    assert s['nearest_obstacle_m'] is None and s['nearest_drop_m'] is None
    assert s['nearest_step_m'] is None and s['clear_ahead_m'] > 2.5


def test_wall_ahead_is_obstacle_at_its_distance():
    s = summary(np.vstack([floor(x1=1.5), wall(1.5, -1, 1, FLOOR, FLOOR + 1.5)]))
    assert s['nearest_obstacle_m'] == pytest.approx(1.5, abs=0.05)
    assert s['clear_ahead_m'] < 1.5


def test_low_step_is_step_and_high_riser_is_obstacle():
    low = summary(np.vstack([floor(x1=1.0), plane(1.0, 2.9, -1, 1, FLOOR + 0.07)]))
    assert low['nearest_step_m'] == pytest.approx(1.0, abs=0.05)
    assert low['nearest_obstacle_m'] is None
    assert low['max_rise_in_corridor_m'] == pytest.approx(0.07, abs=0.01)
    stair = np.vstack([floor(x1=1.0), wall(1.0, -1, 1, FLOOR, FLOOR + 0.17),
                       plane(1.0, 1.3, -1, 1, FLOOR + 0.17)])
    assert summary(stair)['nearest_obstacle_m'] == pytest.approx(1.0, abs=0.05)


def test_drop_off_is_detected():
    s = summary(np.vstack([floor(x1=1.2), plane(1.25, 2.9, -1, 1, FLOOR - 0.4)]))
    assert s['nearest_drop_m'] == pytest.approx(1.25, abs=0.06)
    assert s['clear_ahead_m'] <= 1.25


def test_curtain_reaching_the_floor_band_is_an_obstacle():
    curtain = wall(1.8, -1, 1, FLOOR + 0.08, FLOOR + 2.0)
    assert summary(np.vstack([floor(), curtain]))['nearest_obstacle_m'] == pytest.approx(1.8, abs=0.05)


def test_overhead_only_points_do_not_block():
    ceiling = plane(0.4, 2.9, -1, 1, FLOOR + 1.6)
    g = grid_of(np.vstack([ceiling, plane(-2.9, -0.5, -2.9, 2.9, FLOOR)]))
    s = T.corridor_summary(g, P)
    assert s['nearest_obstacle_m'] is None
    assert T.label_counts(g).get('overhead', 0) > 0


def test_near_field_blind_zone_is_unknown_not_free():
    s = summary(floor(x0=1.1))
    assert s['near_field_unknown_fraction'] > 0.8
    assert s['clear_ahead_m'] == 0.0
    assert s['first_observed_ahead_m'] == pytest.approx(1.1, abs=0.05)


def test_floor_estimate_uses_lidar_only_when_consistent_with_legs():
    z, source = T.estimate_floor(floor(), FLOOR + 0.03, P)
    assert z == pytest.approx(FLOOR, abs=0.01) and source.startswith('lidar')
    z, source = T.estimate_floor(plane(0.4, 2, -1, 1, FLOOR + 0.5), FLOOR, P)
    assert z == FLOOR and source == 'leg_geometry'
    assert T.estimate_floor(floor(), None, P) == (None, 'none')


def test_lidar_extrinsic_and_tilt_map_floor_to_floor_height():
    pitch = math.radians(15)
    r = T.rot_y(pitch)
    t = np.array([0.18678, 0.0, 0.12498])
    floor_base = floor()
    lidar_points = (floor_base - t) @ r          # inverse of base = R p + t
    base, level = T.to_level(lidar_points, r, t, 0.0, 0.0)
    assert np.allclose(level[:, 2], FLOOR, atol=1e-9)
    # Body pitched nose-down 0.1 rad: level transform must restore horizontal floor.
    body_pitch = 0.1
    tilted = floor_base @ T.level_rotation(0.0, body_pitch)   # level -> base is R^T
    _, back = T.to_level((tilted - t) @ r, r, t, 0.0, body_pitch)
    assert np.allclose(back[:, 2], FLOOR, atol=1e-9)


def test_self_filter_removes_body_and_legs():
    pts = np.array([[0.1, 0.0, -0.2], [0.35, 0.2, -0.3], [0.6, 0.0, -0.3]])
    assert T.self_filter(pts, P).tolist() == [False, False, True]


def test_occupancy_and_hazard_output():
    g = grid_of(np.vstack([floor(x1=1.5), wall(1.5, -1, 1, FLOOR, FLOOR + 1.0)]))
    occ = T.occupancy_values(g)
    assert occ.dtype == np.int8 and set(np.unique(occ)) <= {-1, 0, 50, 100}
    hz = T.hazard_points(g)
    assert len(hz) and np.all(hz[:, 0] >= 1.45)


def test_accumulator_keeps_points_that_left_the_view():
    acc = T.Accumulator(horizon_s=4.0, voxel_m=0.0)
    step_edge = np.array([[1.0, 0.0, FLOOR + 0.15]])
    acc.add(0.0, step_edge, np.eye(3), np.zeros(3))
    yaw = 0.3
    rz = np.array([[math.cos(yaw), -math.sin(yaw), 0], [math.sin(yaw), math.cos(yaw), 0], [0, 0, 1]])
    pos = np.array([0.6, 0.1, 0.0])
    back = acc.base_points(rz, pos)
    assert np.allclose(back @ rz.T + pos, step_edge)
    acc.add(5.0, np.zeros((0, 3)), np.eye(3), np.zeros(3))
    assert len(acc.base_points(np.eye(3), np.zeros(3))) == 0


def test_parameter_validation():
    with pytest.raises(ValueError):
        T.TerrainParams(max_step_m=0.01)
    with pytest.raises(ValueError):
        T.TerrainParams(resolution_m=1.0)


def test_leg_floor_reference_matches_validated_postures():
    from lite3_locomotion.terrain_monitor import leg_floor_reference
    lying = [0.45, 1.15, -2.75, -0.45, 1.16, -2.74, 0.45, 1.14, -2.75, -0.44, 1.12, -2.74]
    assert leg_floor_reference(lying) == pytest.approx(-0.081, abs=0.003)
    assert leg_floor_reference(lying[:11]) is None


def test_terrain_monitor_has_no_command_interfaces():
    from pathlib import Path
    src = (Path(__file__).resolve().parents[1] / 'lite3_locomotion' / 'terrain_monitor.py').read_text()
    for forbidden in ('cmd_vel', 'simple_cmd', 'Twist', 'create_service', 'create_client', 'transfer_interfaces'):
        assert forbidden not in src
