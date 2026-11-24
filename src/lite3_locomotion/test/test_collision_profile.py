"""Stopping boundary for the Nav2 collision_monitor and the single final command chain."""

from pathlib import Path

import pytest

from lite3_locomotion import collision_profile as CP

ROOT = Path(__file__).resolve().parents[1]


def test_boundary_adds_measured_terms_and_never_goes_below_backup():
    cal = CP.Calibration(sensing_age_s=0.185, processing_s=0.023, coast_m=0.0342)
    b = CP.boundary(0.40, 0.05, cal)
    assert b['required_m'] == pytest.approx(0.05 * (0.185 + 0.023 + 0.10) + 0.0342 + 0.025 + 0.40, abs=1e-3)
    assert b['boundary_m'] >= b['required_m']
    assert CP.boundary(0.0, 0.01, cal)['boundary_m'] == CP.MIN_BOUNDARY_M


def test_coast_scales_linearly_with_speed():
    cal = CP.Calibration(coast_m=0.04, coast_speed_m_s=0.05)
    assert CP.boundary(0.15, 0.10, cal)['coast_m'] == pytest.approx(0.08)
    assert CP.boundary(0.15, 0.02, cal)['coast_m'] == pytest.approx(0.04 * 0.02 / cal.coast_speed_m_s)   # linear below (2026-11-20)


@pytest.mark.parametrize('clearance, speed', [(-0.01, 0.05), (2.0, 0.05), (0.15, 1.1), (0.15, 0.0)])
def test_out_of_range_inputs_refused(clearance, speed):
    with pytest.raises(ValueError):
        CP.boundary(clearance, speed)


def test_sensor_margin_cannot_be_below_grid_resolution():
    with pytest.raises(ValueError):
        CP.Calibration(sensor_margin_m=0.01)


def test_polygon_round_trip_and_corridor():
    pts = CP.stop_polygon(0.52)
    assert CP.polygon_boundary(pts) == pytest.approx(0.52)
    assert max(abs(y) for y in pts[1::2]) == CP.CORRIDOR_HALF_WIDTH_M
    assert min(pts[0::2]) < -CP.REAR_M                     # covers the whole body


def test_collision_monitor_params_use_hazard_cloud_and_long_stop_publication():
    p = CP.collision_monitor_params(0.4, '/lite3_ops/mux_out', '/cmd_vel')['collision_monitor']['ros__parameters']
    assert p['cmd_vel_in_topic'] == '/lite3_ops/mux_out' and p['cmd_vel_out_topic'] == '/cmd_vel'
    assert p['observation_sources'] == ['hazards'] and p['hazards']['topic'] == '/lite3_terrain/hazards'
    assert p['hazards']['min_height'] <= -0.5                  # drop cells sit at floor level
    assert p['PolygonStop']['action_type'] == 'stop' and p['source_timeout'] <= 1.0
    assert p['stop_pub_timeout'] >= 3600.0      # zeros must keep flowing while the robot idles (Humble default 2 s)
    # on, every command waits for Fast-LIO TF (throughput 1/latency, blind past transform_tolerance)
    assert p['base_shift_correction'] is False


def test_calibration_file_round_trip(tmp_path):
    path = tmp_path / 'calibration.json'
    assert CP.load_calibration(path).sources == ('defaults',)
    assert CP.load_calibration('').sources == ('defaults',)
    CP.update_calibration(path, sources=['M2 x'], coast_m=0.034)
    CP.update_calibration(path, sources=['preflight y'], sensing_age_s=0.2)
    cal = CP.load_calibration(path)
    assert cal.coast_m == 0.034 and cal.sensing_age_s == 0.2 and set(cal.sources) == {'M2 x', 'preflight y'}
    with pytest.raises(ValueError):
        CP.update_calibration(path, coast_m=-1.0)


def test_launch_has_one_final_command_authority():
    src = (ROOT / 'launch' / 'navigation.launch.py').read_text()
    assert "remappings=[('cmd_vel_out', MUX_OUT)]" in src          # twist_mux no longer final
    assert "executable='collision_monitor'" in src
    assert "bound['boundary_m'], MUX_OUT, output," in src                # collision_monitor -> robot
    assert "'node_names': ['collision_monitor']" in src


def test_fast_stacks_get_a_small_stop_and_a_collision_time_approach():
    """Above 0.10 m/s: stop polygon sized for 0.10 m/s plus Nav2's approach action (no fixed slowdown
    zone): T = stop boundary / 0.10 m/s, so approach (v = d / T) reaches the stop zone at <= 0.10 m/s."""
    cal = CP.Calibration(sensing_age_s=0.158, path_latency_s=0.195, coast_m=0.035, coast_speed_m_s=0.05)
    slow = CP.tiers(0.05, 0.10, cal)
    assert slow['approach'] is None and slow['stop'] == CP.boundary(0.05, 0.10, cal)
    fast = CP.tiers(0.0, 0.50, cal)
    assert fast['stop']['boundary_m'] == CP.MIN_BOUNDARY_M
    t = fast['approach']['time_before_collision_s']
    v_at_line = fast['stop']['reaction_travel_m'] / fast['stop']['latency_s']
    assert fast['stop']['required_m'] <= CP.MIN_BOUNDARY_M and fast['stop']['boundary_m'] / t <= v_at_line + 1e-6
    ros = CP.collision_monitor_params(fast['stop']['boundary_m'], 'in', 'out', t)['collision_monitor']['ros__parameters']
    assert ros['polygons'] == ['PolygonStop', 'FootprintApproach']
    a = ros['FootprintApproach']
    assert a['action_type'] == 'approach' and a['time_before_collision'] == t
    assert a['footprint_topic'] == '/local_costmap/published_footprint' and a['simulation_time_step'] == 0.1
    assert 'PolygonSlow' not in ros
    with pytest.raises(ValueError):
        CP.collision_monitor_params(0.24, 'in', 'out', 20.0)


def test_hard_speed_ceiling_is_one_metre_per_second():
    assert CP.MAX_SPEED_M_S == 1.0               # operator decision 2026-11-18 (was 0.5)
    with pytest.raises(ValueError):
        CP.boundary(0.15, 1.1)


def test_one_metre_per_second_boundary_fits_and_is_inside_perception_reach():
    from lite3_locomotion.terrain import TerrainParams
    with pytest.raises(ValueError):              # default (unmeasured) calibration: 4.02 m, refused
        CP.boundary(0.15, 1.0)
    measured = CP.Calibration(sensing_age_s=0.17, path_latency_s=0.195, coast_m=0.035,
                              coast_speed_m_s=0.05)   # operator/calibration.json, 2026-11-18
    assert CP.boundary(0.15, 1.0, measured)['boundary_m'] == pytest.approx(2.81)   # 2.5 cm margin (half cell)
    reach = TerrainParams().size_m / 2 - CP.NOSE_M          # terrain grid ahead of the nose
    assert CP.MAX_BOUNDARY_M < reach


def test_boundary_at_validated_speeds_is_unchanged_and_grows_beyond():
    cal = CP.Calibration(sensing_age_s=0.18, processing_s=0.05, path_latency_s=0.15, coast_m=0.036,
                         coast_speed_m_s=0.05)
    at_010 = CP.boundary(0.15, 0.10, cal)
    assert at_010['coast_m'] == pytest.approx(0.072)                    # linear to 0.10, as validated
    previous = at_010['boundary_m']
    for v in (0.2, 0.3, 0.4, 0.5):
        b = CP.boundary(0.15, v, cal)
        assert b['boundary_m'] > previous and b['coast_m'] > 0.072 * (v / 0.10)   # faster than linear
        previous = b['boundary_m']
    assert CP.boundary(0.15, 0.5, cal)['boundary_m'] <= CP.MAX_BOUNDARY_M


def test_coast_measured_at_speed_replaces_the_assumption():
    measured = CP.Calibration(coast_m=0.30, coast_speed_m_s=0.30)
    assumed = CP.Calibration(coast_m=0.036, coast_speed_m_s=0.05)
    assert CP.boundary(0.15, 0.30, measured)['coast_m'] == pytest.approx(0.30)       # not inflated again


def test_docs_only_collision_monitor_is_a_pass_through():
    """Operator 2026-11-20 docs-only stops: polygon kept (readable) but never triggers, no approach."""
    p = CP.collision_monitor_params(0.05, '/in', '/out', approach_s=2.7, docs_only=True)
    ros = p['collision_monitor']['ros__parameters']
    assert ros['polygons'] == ['PolygonStop'] and 'FootprintApproach' not in ros
    assert ros['PolygonStop']['max_points'] == CP.DOCS_ONLY_MAX_POINTS
    assert ros['cmd_vel_in_topic'] == '/in' and ros['cmd_vel_out_topic'] == '/out'
