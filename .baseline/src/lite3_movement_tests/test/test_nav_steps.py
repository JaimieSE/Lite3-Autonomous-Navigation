"""Navigation-mode steps and the M0-M9 catalogue against the simulated mux/Nav2 (no ROS, no robot)."""


import pytest
import yaml

from lite3_movement_tests import physical_suite as S
from lite3_movement_tests.config import (ConfigError, authorization_template, check_authorization,
                                         load_authorization, load_sequence)
from lite3_movement_tests.nav_steps import compose, cross_track, footprint_clearance
from lite3_movement_tests.runner import MODE_PHYSICAL, MODE_SIM, SequenceRunner
from lite3_movement_tests.sim_model import FakeNavRobot, SimLite3
from lite3_movement_tests.telemetry import Telemetry


def args(**overrides):
    a = S.parse([])
    a.mode = MODE_SIM
    a.measured_coast = 0.035
    a.path_latency = 0.10
    for k, v in overrides.items():
        setattr(a, k, v)
        if k in S.M4_DESTS:
            a.m4_explicit.add(k)
    return S.resolve_m4(a)


def build(code, tmp_path, **overrides):
    a = args(**overrides)
    path = tmp_path / f'{code}.yaml'
    path.write_text(yaml.safe_dump(S.CATALOG[code].build(a), sort_keys=False))
    return load_sequence(path), a


def run(code, tmp_path, nav_fault='none', obstacles=None, start='lying', **overrides):
    seq, a = build(code, tmp_path, **overrides)
    scene = S.CATALOG[code].scene
    robot = FakeNavRobot(Telemetry(), SimLite3(start=start), nav_fault=nav_fault,
                         obstacles=obstacles if obstacles is not None else (scene(a) if scene else ()))
    report = SequenceRunner(seq, robot, MODE_SIM, log=lambda *_: None).run()
    return report, {s['id']: s for s in report['steps']}, robot


@pytest.mark.parametrize('code', ['M0', 'M1', 'M2', 'M3', 'M4', 'M5', 'M6', 'M8', 'M9', 'STAND', 'WALK', 'DET'])
def test_nominal_catalogue_passes_in_simulation(code, tmp_path):
    report, steps, _ = run(code, tmp_path)
    assert report['verdict'] == 'PASSED', [(s['id'], s['summary']) for s in report['steps']
                                           if s['status'] not in ('PASSED', 'SKIPPED')]


def test_m0_sends_only_zeros_and_idle_mux_is_silent(tmp_path):
    report, steps, robot = run('M0', tmp_path)
    assert all(e[1] == 'stop' for e in robot.sent)
    assert steps['zero_delivery']['measurements']['messages_while_idle_locked'] == 0
    assert steps['zero_delivery']['measurements']['stop_path']['problems'] == []
    assert steps['zero_delivery']['measurements']['stop_path']['warmup_s'] is not None
    assert steps['zero_delivery']['measurements']['observed_at_motion_sender'] >= 18


def test_stale_lidar_odometry_fails_nav_readiness(tmp_path):
    """2026-11-18: Fast-LIO 1.4 s behind (congested bridge) passed readiness and broke the stop path."""
    report, steps, robot = run('M0', tmp_path, nav_fault='odom_lag')
    step = steps['nav_readiness']
    assert step['status'] == 'FAILED'
    assert 'behind real time' in step['summary']
    assert report['verdict'] != 'PASSED'
    assert steps['zero_delivery']['status'] == 'NOT_RUN'


def test_zero_delivery_waits_for_stop_path_and_sends_nothing_unmatched(tmp_path):
    """2026-11-18: zeros sent before DDS matched the new stop publisher were dropped (2 of 20)."""
    report, steps, robot = run('M0', tmp_path, nav_fault='stop_unmatched')
    step = steps['zero_delivery']
    assert step['status'] == 'FAILED'
    assert 'stop path not connected' in step['summary']
    assert step['measurements']['stop_path']['waited_s'] >= 3.0
    assert report['verdict'] != 'PASSED'


def test_dead_man_ignored_by_mux_fails_m2(tmp_path):
    report, steps, _ = run('M2', tmp_path, nav_fault='deadman_ignored')
    assert steps['deadman_stop']['status'] == 'FAILED'
    assert 'kept reaching the robot' in steps['deadman_stop']['summary']
    assert report['verdict'] != 'PASSED'


def test_leaking_proximity_lock_alone_is_caught_by_collision_monitor(tmp_path):
    report, steps, _ = run('M3', tmp_path, nav_fault='mux_leak')
    assert steps['blocked_move']['status'] == 'PASSED'          # second layer holds


def test_both_layers_leaking_aborts_m3_at_first_command(tmp_path):
    report, steps, _ = run('M3', tmp_path, nav_fault='mux_leak+cm_disabled')
    assert steps['blocked_move']['status'] == 'ABORTED'
    assert 'passed the proximity lock' in steps['blocked_move']['summary']
    assert report['verdict'] == 'INTERVENTION_REQUIRED'
    assert report['safe_stop']['rest_verified']


def test_m3_refuses_without_object_in_stop_zone(tmp_path):
    report, steps, robot = run('M3', tmp_path, obstacles=())
    assert steps['detect_stop_zone']['status'] == 'FAILED'
    assert steps['blocked_move']['status'] == 'NOT_RUN'
    assert not any(e[1] == 'vel' and e[2] for e in robot.sent)


@pytest.mark.parametrize('fault, text', [
    ('nav_reverse', 'reverse command'),
    ('hazard_stale', 'hazard cloud stale'),
])
def test_nav_goal_safety_aborts(fault, text, tmp_path):
    report, steps, _ = run('M5', tmp_path, nav_fault=fault)
    step = steps['nav_straight']
    assert step['status'] == 'ABORTED' and text in step['summary'], step['summary']
    assert report['safe_stop']['rest_verified']


def test_rogue_nav2_goal_outside_nav_step_aborts(tmp_path):
    report, steps, _ = run('M1', tmp_path, nav_fault='rogue_goal')
    assert report['verdict'] != 'PASSED'
    assert any('outside a nav_goal' in s['summary'] for s in report['steps'])


def test_obstacle_in_path_fails_reach_in_straight_line_sim(tmp_path):
    # The stand-in has no planner: the proximity lock holds it and Nav2 aborts -> M5 FAIL.
    report, steps, _ = run('M5', tmp_path, obstacles=S.nose_box(0.6))
    assert steps['nav_straight']['status'] == 'FAILED'


def test_speed_limit_parameter_reaches_nav2(tmp_path):
    report, steps, robot = run('M5', tmp_path, max_speed=0.04)
    vx = [c[2] for c in robot.mux_log if c[1] == 'nav']
    assert report['verdict'] == 'PASSED' and max(vx) <= 0.0401


@pytest.mark.parametrize('argv', [['--max-speed', '1.1'],['--stop-distance', '0.15'],
                                  ['--approach-speed', '0.08'], ['--goal-distance', '9'],
                                  ['--max-yaw-rate', '1.6'], ['--timeout', '1000'],
                                  ['--final-clearance', '0.05'], ['--max-travel', '3'],
                                  ['--max-duration', '90'], ['--max-sensor-age', '1.0'],
                                  ['--min-lidar-hz', '5'], ['--detect-scans', '1'],
                                  ['--detect-scans', '2.5'], ['--response-limit', '1'],
                                  ['--detection-range', '3'], ['--trigger-distance', '0.15']])
def test_cli_cannot_exceed_hard_limits(argv):
    with pytest.raises(SystemExit):
        S.parse(argv)


def test_nav_steps_require_nav_interface(tmp_path):
    seq = S.CATALOG['M5'].build(args())
    seq['interface'] = 'direct'
    path = tmp_path / 's.yaml'
    path.write_text(yaml.safe_dump(seq))
    with pytest.raises(ConfigError):
        load_sequence(path)


def test_nav_goal_chain_must_end_with_stop_verify(tmp_path):
    seq = S.CATALOG['M5'].build(args())
    seq['steps'] = seq['steps'][:-1]
    path = tmp_path / 's.yaml'
    path.write_text(yaml.safe_dump(seq))
    with pytest.raises(ConfigError, match='stop_verify'):
        load_sequence(path)


def test_authorization_binds_nav_speed_and_distance(tmp_path):
    seq, _ = build('M5', tmp_path, goal_distance=2.0)
    template = authorization_template(seq)
    template['operator_confirmations'] = {k: True for k in template['operator_confirmations']}
    assert 'test_area_prepared_as_instructed' in template['operator_confirmations']
    assert 'accept_no_lidar_collision_gate' not in template['operator_confirmations']
    template['limits']['max_nav_distance_m'] = 1.0
    path = tmp_path / 'auth.yaml'
    path.write_text(yaml.safe_dump(template))
    problems = check_authorization(load_authorization(path), seq)
    assert any('goal distance' in p for p in problems)


def test_physical_refused_unless_stack_armed(tmp_path, monkeypatch, capsys):
    monkeypatch.delenv('LITE3_STATE_FILE', raising=False)
    assert S.main(['--physical', '--phases', 'M1', '--report-dir', str(tmp_path)]) == 3
    assert 'not running in physical mode' in capsys.readouterr().out


def test_obstacle_tests_gated_on_protective_tests(tmp_path, monkeypatch, capsys):
    state = tmp_path / 'state.env'
    run_dir = tmp_path / 'run'
    state.write_text(f'MODE=physical\nRUN_DIR={run_dir}\n')
    monkeypatch.setenv('LITE3_STATE_FILE', str(state))
    code = S.main(['--physical', '--phases', 'M4', '--report-dir', str(tmp_path)])
    assert code == 3
    assert 'needs M2, M3 to PASS first' in capsys.readouterr().out


def test_geometry_helpers():
    assert footprint_clearance([(0.505, 0.0)]) == pytest.approx(0.2)
    assert footprint_clearance([(0.0, 0.0)]) == 0.0
    x, y, yaw = compose((1.0, 1.0, 1.5707963), 1.0, 0.0, 0.0)
    assert (x, y) == pytest.approx((1.0, 2.0), abs=1e-6)
    assert cross_track((0, 0), (2, 0), (1, 0.3)) == pytest.approx(0.3)


def test_stale_hazard_feed_blocks_before_motion(tmp_path):
    seq, a = build('M5', tmp_path)
    robot = FakeNavRobot(Telemetry(), SimLite3(start='lying'))
    robot.telemetry.clearance = type(robot.telemetry.clearance)()   # never filled
    robot.telemetry.on_clearance = lambda *a_: None
    report = SequenceRunner(seq, robot, MODE_SIM, log=lambda *_: None).run()
    steps = {s['id']: s for s in report['steps']}
    assert steps['nav_readiness']['status'] == 'FAILED'
    assert 'hazard cloud stale' in steps['nav_readiness']['summary']
    assert not robot.sent


@pytest.mark.parametrize('code', ['M0', 'M1', 'M2', 'M3', 'M4', 'M5', 'M6', 'M8', 'M9', 'SIT'])
def test_generated_authorization_fits_each_test_in_physical_mode(code, tmp_path):
    """The per-test authorization the suite generates must neither block nor loosen the test."""
    seq, a = build(code, tmp_path)
    template = authorization_template(seq, hours=0.25)
    template['operator_confirmations'] = {k: True for k in template['operator_confirmations']}
    path = tmp_path / 'auth.yaml'
    path.write_text(yaml.safe_dump(template))
    auth = load_authorization(path)
    assert check_authorization(auth, seq) == []
    assert auth.max_speed_m_s <= a.max_speed + 1e-9 or code in ('M0', 'SIT')
    scene = S.CATALOG[code].scene
    robot = FakeNavRobot(Telemetry(), SimLite3(start='standing' if code == 'SIT' else 'lying'),
                         obstacles=scene(a) if scene else ())
    report = SequenceRunner(seq, robot, MODE_PHYSICAL, log=lambda *_: None, authorization=auth).run()
    assert report['verdict'] == 'PASSED', [(s['id'], s['summary']) for s in report['steps']
                                           if s['status'] not in ('PASSED', 'SKIPPED')]


def run_m2(tmp_path, fault):
    seq, a = build('M2', tmp_path)
    robot = FakeNavRobot(Telemetry(), SimLite3(start='lying', fault=fault))
    report = SequenceRunner(seq, robot, MODE_SIM, log=lambda *_: None).run()
    return report, {s['id']: s for s in report['steps']}['deadman_stop']


def test_m2_lite3_like_settle_passes_two_stage_criterion_with_measurements(tmp_path):
    """Physical signature of 2026-11-16 (commands cut ~0.45 s, stepping ends ~2 s, small posture
    settle ~3.2 s): stopped moving within 3 s, fully settled within 5 s, settle motion bounded."""
    report, step = run_m2(tmp_path, 'settle_twitch')
    assert step['status'] == 'PASSED', step['summary']
    m = step['measurements']
    assert m['rest']['stop_onset_after_release_s'] <= 3.0
    assert 3.0 < m['rest']['settle_onset_after_release_s'] <= 5.0
    assert m['rest']['settle_motion']['yaw_change_rad'] <= 0.05
    assert m['diagnostics']['phases_after_release_s']['last_disturbance'] > 3.0
    from lite3_movement_tests.report import markdown
    assert 'Stop diagnostics: deadman_stop' in markdown(report)


def test_m2_robot_still_walking_after_3s_fails(tmp_path):
    report, step = run_m2(tmp_path, 'no_silence_stop')
    assert step['status'] == 'FAILED'
    assert 'did not stop moving within 3.0 s' in step['summary']
    assert report['verdict'] == 'INTERVENTION_REQUIRED'
    assert report['safe_stop']['rest_verified']


def test_m2_large_settle_rotation_fails(tmp_path):
    report, step = run_m2(tmp_path, 'settle_twitch_large')
    assert step['status'] == 'FAILED'
    assert 'heading changed' in step['summary']


def test_m2_stop_limit_cannot_exceed_3s(tmp_path):
    seq = S.CATALOG['M2'].build(args())
    seq['steps'][-2]['stop_within_s'] = 3.5
    path = tmp_path / 's.yaml'
    path.write_text(yaml.safe_dump(seq))
    with pytest.raises(ConfigError):
        load_sequence(path)


def test_m3_needs_only_m0(tmp_path, monkeypatch, capsys):
    state = tmp_path / 'state.env'
    run_dir = tmp_path / 'run'
    state.write_text(f'MODE=physical\nRUN_DIR={run_dir}\n')
    monkeypatch.setenv('LITE3_STATE_FILE', str(state))
    assert S.CATALOG['M3'].prereqs == ['M0']
    assert set(S.CATALOG['M4'].prereqs) == {'M2', 'M3'}
    assert S.main(['--physical', '--phases', 'M3', '--report-dir', str(tmp_path)]) == 3
    assert 'needs M0 to PASS first' in capsys.readouterr().out


def test_m2_nominal_reports_phases_within_limit(tmp_path):
    report, steps, _ = run('M2', tmp_path)
    d = steps['deadman_stop']['measurements']['diagnostics']
    assert steps['deadman_stop']['status'] == 'PASSED'
    assert d['phases_after_release_s']['rest_onset'] <= 3.0
    assert d['last_dead_man_feed_s'] is not None and -0.1 <= d['last_dead_man_feed_s'] <= 0.0


# ---- M3 lock-release root cause and fix (physical abort 2026-11-16 12:09) --------------------
def test_m3_detection_dropout_withholds_commands_instead_of_leaking(tmp_path):
    report, steps, robot = run('M3', tmp_path, nav_fault='detection_dropout')
    step = steps['blocked_move']
    assert step['status'] == 'ABORTED'
    assert 'lock released during the test, command withheld' in step['summary']
    leaked = [c for c in step['measurements']['robot_commands'] if c[1] != 0.0]
    assert leaked == []                                   # nothing reached motion_sender
    ctx = step['measurements']
    assert ctx['terrain_status'] and ctx['terrain_stop_topic'] and ctx['preconditions']
    assert step['trace']


def test_m3_transient_stop_from_person_nearby_is_not_accepted(tmp_path):
    report, steps, robot = run('M3', tmp_path, obstacles=[(0.0, 0.40)])   # 0.40 m to the side
    det = steps['detect_stop_zone']
    assert det['status'] == 'FAILED'
    assert 'object ahead' in det['summary'] or 'detected ahead' in det['summary']
    assert not any(e[1] == 'vel' and e[2] for e in robot.sent)


def test_det_static_check_passes_and_fails_without_object(tmp_path):
    report, steps, robot = run('DET', tmp_path)
    assert report['verdict'] == 'PASSED' and not robot.sent
    report, steps, robot = run('DET', tmp_path, obstacles=())
    assert steps['detect_ahead']['status'] == 'FAILED'


def test_m2_records_coast_and_suite_reads_it_from_gate(tmp_path):
    from lite3_movement_tests.report import write_reports
    report, step = run_m2(tmp_path, 'settle_twitch')
    coast = step['measurements']['diagnostics']['coast_after_cut_m']
    assert coast is not None and 0.0 <= coast < 0.1
    report['physical_suite'] = {'parameters': {'max_speed': 0.05}}
    del step['measurements']['diagnostics']['coast_after_cut_m']     # older report format
    paths = write_reports(report, tmp_path / 'm2')
    run_dir = tmp_path / 'run'
    S.record_pass(run_dir, 'M2', paths['markdown'])
    value, speed, source = S.measured_coast(run_dir)
    assert value == pytest.approx(max(0.0, coast), abs=0.01) and speed == 0.05 and source


# ---- WALK / M7 (Nav2 goes around) and the boundary re-check ---------------------------------
def test_walk_reaches_goal_in_simulation(tmp_path):
    report, steps, robot = run('WALK', tmp_path)
    assert steps['walk']['status'] == 'PASSED', steps['walk']['summary']
    assert steps['walk']['measurements']['collision_monitor_stops'] == 0


def test_m7_fails_if_collision_monitor_had_to_stop(tmp_path):
    # The stand-in has no planner: it drives straight into the box and the collision_monitor
    # stops it -> M7 must FAIL (stopping is M4's behaviour, going around is M7's).
    report, steps, robot = run('M7', tmp_path, obstacles=S.nose_box(0.9))
    step = steps['nav_avoid']
    assert step['status'] == 'FAILED'
    assert 'collision_monitor had to stop the robot' in step['summary']
    assert step['measurements']['collision_monitor_stops'] >= 1


def test_readiness_refuses_boundary_too_small_for_live_delay(tmp_path):
    seq = S.CATALOG['M5'].build(args())
    seq['steps'][1].update({'cm_boundary_m': 0.37, 'cm_clearance_m': 0.40, 'cm_max_speed_m_s': 0.10,
                            'coast_m': 0.034, 'coast_speed_m_s': 0.05})
    path = tmp_path / 's.yaml'
    path.write_text(yaml.safe_dump(seq, sort_keys=False))
    robot = FakeNavRobot(Telemetry(), SimLite3(start='lying'))
    report = SequenceRunner(load_sequence(path), robot, MODE_SIM, log=lambda *_: None).run()
    step = {s['id']: s for s in report['steps']}['nav_readiness']
    assert step['status'] == 'FAILED' and 'required NOW' in step['summary']
    assert not robot.sent


def test_readiness_accepts_matching_boundary(tmp_path):
    seq = S.CATALOG['M5'].build(args())
    seq['steps'][1].update({'cm_boundary_m': 0.37, 'cm_clearance_m': 0.15, 'cm_max_speed_m_s': 0.10,
                            'coast_m': 0.034, 'coast_speed_m_s': 0.05})
    path = tmp_path / 's.yaml'
    path.write_text(yaml.safe_dump(seq, sort_keys=False))
    report = SequenceRunner(load_sequence(path), FakeNavRobot(Telemetry(), SimLite3(start='lying')),
                            MODE_SIM, log=lambda *_: None).run()
    assert report['verdict'] == 'PASSED'


def test_readiness_explains_a_boundary_that_cannot_be_met_instead_of_an_internal_error(tmp_path):
    """2026-11-17 13:34: at 0.5 m/s the live sensing delay pushed the required boundary past its 1.5 m cap;
    this surfaced as 'internal error: ValueError' and hid the cause."""
    seq = S.CATALOG['M5'].build(args())
    # Cap now 3.0 m (2026-11-18): 1.0 m/s with 1.0 m clearance needs ~3.7 m, still unmeetable.
    seq['steps'][1].update({'cm_boundary_m': 1.0, 'cm_clearance_m': 1.0, 'cm_max_speed_m_s': 1.0,
                            'coast_m': 0.034, 'coast_speed_m_s': 0.05})
    path = tmp_path / 's.yaml'
    path.write_text(yaml.safe_dump(seq, sort_keys=False))
    robot = FakeNavRobot(Telemetry(), SimLite3(start='lying'))
    report = SequenceRunner(load_sequence(path), robot, MODE_SIM, log=lambda *_: None).run()
    step = {s['id']: s for s in report['steps']}['nav_readiness']
    assert step['status'] == 'FAILED' and 'internal error' not in step['summary']
    assert 'cannot be met with the LIVE sensing delay' in step['summary'] and '--max-speed' in step['summary']
    assert not robot.sent


def test_readiness_reports_seconds_late_lidar_instead_of_an_internal_error(tmp_path):
    """2026-11-19 10:22 read-only roam: hazards 2.66 s old -> 'internal error: ValueError: sensing_age_s=2.66
    invalid'. It must refuse with the cause (LiDAR seconds late), never as an internal error."""
    seq = S.CATALOG['M5'].build(args())
    seq['steps'][1].update({'cm_boundary_m': 0.37, 'cm_clearance_m': 0.15, 'cm_max_speed_m_s': 0.10,
                            'coast_m': 0.034, 'coast_speed_m_s': 0.05})
    path = tmp_path / 's.yaml'
    path.write_text(yaml.safe_dump(seq, sort_keys=False))
    class LateLidar(FakeNavRobot):
        hazard_age_s = 2.66            # class attribute: the stand-in's warm-up ticks already see it

    robot = LateLidar(Telemetry(), SimLite3(start='lying'))
    report = SequenceRunner(load_sequence(path), robot, MODE_SIM, log=lambda *_: None).run()
    step = {s['id']: s for s in report['steps']}['nav_readiness']
    assert step['status'] == 'FAILED' and 'internal error' not in step['summary']
    assert 'seconds late' in step['summary']
    assert not robot.sent
