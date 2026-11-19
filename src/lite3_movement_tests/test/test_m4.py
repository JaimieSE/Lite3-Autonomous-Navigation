"""M4 sensor-driven proximity stop: state machine, outcomes, safeguards, configuration and the
M5 velocity-cap fix, against the simulated mux / collision_monitor / terrain monitor (no ROS,
no robot). Simulated results are software evidence only."""

from pathlib import Path

import pytest
import yaml

from lite3_movement_tests import physical_suite as S
from lite3_movement_tests.config import (ConfigError, authorization_template, check_authorization,
                                         load_authorization, load_sequence)
from lite3_movement_tests.proximity import NOT_PASS_OUTCOMES, ObstacleTracker
from lite3_movement_tests.runner import MODE_PHYSICAL, MODE_READONLY, MODE_SIM, SequenceRunner
from lite3_movement_tests.sim_model import FakeNavRobot, SimLite3, box_points
from lite3_movement_tests.telemetry import Telemetry

PROFILES = Path(__file__).resolve().parents[3] / 'operator' / 'profiles'
NOSE = 0.305


def m4_args(**overrides):
    a = S.parse([])
    a.mode, a.measured_coast, a.path_latency = MODE_SIM, 0.035, 0.10
    for k, v in overrides.items():
        setattr(a, k, v)
        if k in S.M4_DESTS:
            a.m4_explicit.add(k)
    return S.resolve_m4(a)


def build(tmp_path, **overrides):
    a = m4_args(**overrides)
    path = tmp_path / 'M4.yaml'
    path.write_text(yaml.safe_dump(S.CATALOG['M4'].build(a), sort_keys=False))
    return load_sequence(path), a


def run_m4(tmp_path, distance=1.0, obstacles=None, nav_fault='none', fault='none', mode=MODE_SIM,
           robot_setup=None, **overrides):
    seq, a = build(tmp_path, **overrides)
    if obstacles is None:
        obstacles = S.nose_box(distance) if distance is not None else ()
    robot = FakeNavRobot(Telemetry(), SimLite3(start='lying' if mode == MODE_SIM else 'standing',
                                               fault=fault),
                         nav_fault=nav_fault, obstacles=obstacles, cm_boundary=a.cm_boundary,
                         can_actuate=mode != MODE_READONLY)
    if robot_setup:
        robot_setup(robot)
    report = SequenceRunner(seq, robot, mode, log=lambda *_: None).run()
    steps = {s['id']: s for s in report['steps']}
    return report, steps, robot


def walk(steps):
    return steps['approach_stop']


def forward_sent(robot):
    return [e for e in robot.sent if e[1] == 'vel' and e[2]]


def true_clearance(robot, distance):
    """Ground-truth nose-to-obstacle-face distance in the simulator."""
    return distance - robot.model.x


# ---- PASS: walk until an obstacle anywhere in the lane stops the robot -----------------------
@pytest.mark.parametrize('distance, travel', [(0.6, 1.5), (1.0, 1.5), (1.6, 1.8), (1.75, 1.8)])
def test_stops_for_obstacle_wherever_it_starts(tmp_path, distance, travel):
    report, steps, robot = run_m4(tmp_path, distance=distance, max_travel=travel)
    step = walk(steps)
    m = step['measurements']
    assert step['status'] == 'PASSED', step['summary']
    assert report['verdict'] == 'PASSED'
    assert m['outcome'] == 'STOPPED_FOR_OBSTACLE' and all(c['ok'] for c in m['criteria'])
    assert [t['state'] for t in m['states']][-1] == 'STOP_VERIFIED'
    assert {'WALKING', 'OBSTACLE_DETECTED', 'STOPPING', 'STOP_VERIFIED'} <= {t['state'] for t in m['states']}
    assert m['stop_layer'] == 'collision_monitor'
    assert m['forward_commands_after_stop'] >= 5 and m['robot_zeros_while_input_forward'] >= 3
    assert m['final_nose_distance_m'] >= 0.15 and true_clearance(robot, distance) >= 0.15
    assert m['distance_travelled_m'] == pytest.approx(distance - 0.37, abs=0.08)


def test_obstacle_placed_after_the_walk_started_is_detected_and_stops_the_robot(tmp_path):
    # Nothing ahead at the start; 3 s into the walk a box appears 1.2 m ahead of the start.
    def setup(robot):
        robot.late_obstacles = [(3.0, box_points(NOSE + 1.2 + 0.15, 0.0, 0.3))]
    report, steps, robot = run_m4(tmp_path, distance=None, robot_setup=setup)
    m = walk(steps)['measurements']
    assert walk(steps)['status'] == 'PASSED', walk(steps)['summary']
    assert m['obstacle_tracked_at_start_m'] is None and m['obstacle_detected_after_s'] > 3.0


def test_obstacle_visible_at_start_is_tracked_from_the_first_step(tmp_path):
    report, steps, robot = run_m4(tmp_path, distance=1.2)
    m = walk(steps)['measurements']
    assert m['obstacle_tracked_at_start_m'] == pytest.approx(1.2, abs=0.05)
    assert m['states'][1]['state'] == 'OBSTACLE_DETECTED' and m['states'][1].get('at_start')


# ---- outcomes that are NOT a pass --------------------------------------------------------
def test_travel_limit_without_obstacle_is_a_separate_outcome(tmp_path):
    report, steps, robot = run_m4(tmp_path, distance=None, max_travel=0.4)
    step = walk(steps)
    assert step['status'] == 'FAILED' and step['measurements']['outcome'] == 'TRAVEL_LIMIT'
    assert 'no obstacle detected' in step['summary']
    assert report['verdict'] == 'FAILED_SAFE' and report['safe_stop']['rest_verified']
    assert step['measurements']['distance_travelled_m'] <= 0.4 + 0.02


def test_timer_is_a_separate_outcome_never_a_pass(tmp_path):
    report, steps, robot = run_m4(tmp_path, distance=None, max_duration=6.0, max_travel=1.8)
    step = walk(steps)
    assert step['status'] == 'FAILED' and step['measurements']['outcome'] == 'TIME_LIMIT'
    assert step['measurements']['walk_duration_s'] == pytest.approx(6.0, abs=0.2)


def test_travel_limit_is_enforced_conservatively_on_leg_odometry(tmp_path):
    # Obstacle 2.0 m ahead but at most 1.8 m travel, enforced as odometry <= 0.8 x 1.8 m.
    report, steps, robot = run_m4(tmp_path, distance=2.0, max_travel=1.8)
    step = walk(steps)
    assert step['measurements']['outcome'] == 'TRAVEL_LIMIT' and 'never reached' in step['summary']
    assert robot.model.x <= 1.8 * 0.8 + 0.03


def test_obstacle_tracked_but_out_of_reach_reports_time_limit(tmp_path):
    report, steps, robot = run_m4(tmp_path, distance=1.8, max_duration=8.0)
    step = walk(steps)
    assert step['measurements']['outcome'] == 'TIME_LIMIT'
    assert 'never reached' in step['summary']


def test_backup_lock_stop_is_not_a_pass(tmp_path):
    # Someone steps up beside the robot (0.40 m to the side): only the terrain lock's 0.45 m
    # radius covers it (outside the collision_monitor corridor) -> the backup stops the robot.
    def setup(robot):
        robot.late_obstacles = [(3.0, [(0.15 + d, 0.40) for d in (-0.05, 0.0, 0.05)])]
    report, steps, robot = run_m4(tmp_path, distance=1.6, max_travel=1.8, robot_setup=setup)
    step = walk(steps)
    assert step['status'] == 'FAILED', step['summary']
    assert step['measurements']['outcome'] == 'BACKUP_LOCK_STOP'
    assert step['measurements']['stop_layer'] == 'terrain_backup_lock'


def test_disabled_collision_monitor_is_caught_by_the_runner_before_the_backup(tmp_path):
    report, steps, robot = run_m4(tmp_path, nav_fault='cm_disabled')
    step = walk(steps)
    assert step['status'] == 'ABORTED' and 'protective stop failed' in step['summary']
    assert step['measurements']['outcome'] == 'SAFETY_ABORT'
    assert true_clearance(robot, 1.0) >= 0.15


def test_dead_man_lapse_is_an_unexplained_stop(tmp_path):
    report, steps, robot = run_m4(tmp_path, distance=1.5, nav_fault='deadman_lapse_walk')
    step = walk(steps)
    assert step['status'] == 'FAILED'
    assert step['measurements']['outcome'] == 'UNEXPLAINED_STOP'


@pytest.mark.parametrize('setup, outcome', [
    (lambda r: setattr(r, 'cancelled', lambda: r.first_test_cmd is not None and r.t > r.first_test_cmd + 3),
     'OPERATOR_STOP'),
])
def test_operator_stop_is_a_separate_outcome(tmp_path, setup, outcome):
    report, steps, robot = run_m4(tmp_path, robot_setup=setup)
    step = walk(steps)
    assert step['status'] == 'ABORTED' and step['measurements']['outcome'] == outcome


def test_handheld_remote_takeover_is_an_operator_stop(tmp_path):
    report, steps, robot = run_m4(tmp_path, fault='remote_takeover')
    step = walk(steps)
    assert step['status'] == 'ABORTED' and step['measurements']['outcome'] == 'OPERATOR_STOP'


# ---- independent safeguards (each must stop the robot and never PASS) ------------------------
@pytest.mark.parametrize('kwargs, text', [
    (dict(nav_fault='terrain_stale_walk'), 'stale'),
    (dict(nav_fault='lidar_slow_walk'), 'Hz <'),
    (dict(nav_fault='imu_invalid_walk'), 'IMU'),
    (dict(nav_fault='blind_after_start'), 'lost obstacle tracking'),
    (dict(fault='runaway'), ''),
])
def test_safeguards_abort_and_stop(tmp_path, kwargs, text):
    report, steps, robot = run_m4(tmp_path, **kwargs)
    step = walk(steps)
    m = step['measurements']
    assert step['status'] == 'ABORTED', step['summary']
    assert text in step['summary'], step['summary']
    assert m['outcome'] in NOT_PASS_OUTCOMES and m['outcome'] == 'SAFETY_ABORT'
    assert report['verdict'] != 'PASSED'
    assert report['safe_stop'] is not None


def test_obstacle_moving_toward_the_robot_is_an_unexpected_measurement(tmp_path):
    def setup(robot):
        robot.obstacle_velocity = (-0.15, 0.0)
    report, steps, robot = run_m4(tmp_path, distance=1.4, robot_setup=setup)
    step = walk(steps)
    assert step['status'] == 'ABORTED' and 'unexpected obstacle measurement' in step['summary']


def test_object_stepping_into_the_path_is_an_unexpected_measurement(tmp_path):
    def setup(robot):
        robot.late_obstacles = [(3.0, box_points(NOSE + 0.85, 0.0, 0.2))]   # much closer, suddenly
    report, steps, robot = run_m4(tmp_path, distance=1.6, max_travel=1.8, robot_setup=setup)
    step = walk(steps)
    assert step['status'] == 'ABORTED' and 'unexpected obstacle measurement' in step['summary']


def test_both_stop_layers_failing_is_caught_by_the_runner(tmp_path):
    report, steps, robot = run_m4(tmp_path, nav_fault='cm_disabled+mux_leak')
    step = walk(steps)
    assert step['status'] == 'ABORTED'
    assert step['measurements']['outcome'] == 'SAFETY_ABORT'
    assert true_clearance(robot, 1.0) >= 0.10


# ---- READY: refuse before any forward command ------------------------------------------------
@pytest.mark.parametrize('kwargs, text', [
    (dict(measured_coast=None), 'no measured coast'),
    (dict(min_clearance=0.40), 'is below the'),
    (dict(cm_boundary=0.21, trigger_distance=0.21), 'within 0.05 m of the 0.2 m backup lock'),
    (dict(distance=0.45), 'path not clear'),
])
def test_not_ready_sends_nothing(tmp_path, kwargs, text):
    report, steps, robot = run_m4(tmp_path, **kwargs)
    step = walk(steps)
    assert step['status'] == 'FAILED' and text in step['summary'], step['summary']
    assert not forward_sent(robot)


def test_separation_refusal_suggests_the_navigation_clearance(tmp_path):
    seq, a = build(tmp_path, cm_boundary=0.21)
    robot = FakeNavRobot(Telemetry(), SimLite3(start='lying'), obstacles=S.nose_box(1.0), cm_boundary=0.21)
    runner = SequenceRunner(seq, robot, MODE_SIM, log=lambda *_: None)
    runner.nav_clearance_hint = 0.15
    step = {s['id']: s for s in runner.run()['steps']}['approach_stop']
    assert '--clearance >= 0.19' in step['summary']


def test_running_boundary_must_match_the_trigger_distance(tmp_path):
    seq, a = build(tmp_path)
    robot = FakeNavRobot(Telemetry(), SimLite3(start='lying'), obstacles=S.nose_box(1.0), cm_boundary=0.45)
    step = {s['id']: s for s in SequenceRunner(seq, robot, MODE_SIM, log=lambda *_: None).run()['steps']}
    assert 'expected trigger distance' in step['approach_stop']['summary']
    assert not forward_sent(robot)


# ---- dry: read-only READY check and the shadow -------------------------------------------
def test_dry_m4_is_read_only_and_reports_ready(tmp_path):
    a = m4_args()
    a.mode = MODE_READONLY
    path = tmp_path / 'dry.yaml'
    path.write_text(yaml.safe_dump(S.CATALOG['M4'].build(a), sort_keys=False))
    seq = load_sequence(path)
    assert 'stand' not in [s.id for s in seq.steps]
    robot = FakeNavRobot(Telemetry(), SimLite3(start='lying'), obstacles=S.nose_box(1.0), can_actuate=False)
    report = SequenceRunner(seq, robot, MODE_READONLY, log=lambda *_: None).run()
    step = {s['id']: s for s in report['steps']}['approach_stop']
    assert step['status'] == 'PREVIEW_ONLY' and step['summary'].startswith('READY')
    assert report['verdict'] == 'READONLY_CHECKS_PASSED' and not robot.sent


def test_shadow_watch_validates_detection_without_motion(tmp_path):
    a = m4_args(shadow=True, shadow_seconds=40.0)
    path = tmp_path / 'shadow.yaml'
    path.write_text(yaml.safe_dump(S.CATALOG['M4'].build(a), sort_keys=False))
    seq = load_sequence(path)
    assert not any(s.actuating for s in seq.steps)
    robot = FakeNavRobot(Telemetry(), SimLite3(start='standing'), obstacles=S.nose_box(1.2),
                         cm_boundary=a.cm_boundary, can_actuate=False)
    robot.obstacle_velocity, robot.obstacle_motion_from = (-0.08, 0.0), robot.t + 2.0
    report = SequenceRunner(seq, robot, MODE_READONLY, log=lambda *_: None).run()
    step = {s['id']: s for s in report['steps']}['shadow_watch']
    assert step['status'] == 'PASSED', step['summary']
    m = step['measurements']
    assert m['detected_at_m'] > m['boundary_m'] and m['would_stop_after_s'] is not None
    assert not robot.sent and robot.model.x == 0.0


def test_shadow_without_an_approaching_object_fails_and_explains(tmp_path):
    a = m4_args(shadow=True, shadow_seconds=6.0)
    path = tmp_path / 'shadow.yaml'
    path.write_text(yaml.safe_dump(S.CATALOG['M4'].build(a), sort_keys=False))
    robot = FakeNavRobot(Telemetry(), SimLite3(start='standing'), obstacles=(), can_actuate=False)
    report = SequenceRunner(load_sequence(path), robot, MODE_READONLY, log=lambda *_: None).run()
    step = {s['id']: s for s in report['steps']}['shadow_watch']
    assert step['status'] == 'FAILED' and 'no obstacle was confirmed' in step['summary']


# ---- configuration: hard limits, independence, profiles ------------------------------------
def test_travel_and_duration_never_change_speed_or_stopping(tmp_path):
    seq_a, _ = build(tmp_path)
    seq_b, _ = build(tmp_path, max_travel=1.8, max_duration=60.0)
    pa = {s.id: s for s in seq_a.steps}['approach_stop'].params
    pb = {s.id: s for s in seq_b.steps}['approach_stop'].params
    fixed = ('vx', 'stop_distance_m', 'backup_stop_distance_m', 'min_clearance_m', 'max_sensor_age_s',
             'min_lidar_hz', 'detect_scans', 'response_limit_s', 'stop_within_s', 'initial_clear_m')
    assert {k: pa[k] for k in fixed} == {k: pb[k] for k in fixed}
    ta, tb = authorization_template(seq_a), authorization_template(seq_b)
    assert ta['limits']['max_speed_m_s'] == tb['limits']['max_speed_m_s'] == 0.05


@pytest.mark.parametrize('key, value', [
    ('vx', 0.06), ('max_travel_m', 2.0), ('duration_s', 61.0), ('max_sensor_age_s', 0.6),
    ('min_lidar_hz', 7.0), ('detect_scans', 1), ('detect_scans', 2.5), ('response_limit_s', 0.6),
    ('detection_range_m', 3.0), ('initial_clear_m', 0.38), ('stop_distance_m', 0.15),
    ('min_clearance_m', 0.05), ('lost_track_scans', 9), ('suppression_proof_s', 3.0),
])
def test_sequence_refuses_values_outside_the_envelope(tmp_path, key, value):
    data = S.CATALOG['M4'].build(m4_args())
    step = next(s for s in data['steps'] if s['type'] == 'approach_stop')
    step[key] = value
    path = tmp_path / 'bad.yaml'
    path.write_text(yaml.safe_dump(data, sort_keys=False))
    with pytest.raises(ConfigError):
        load_sequence(path)


def test_authorization_binds_m4_travel_and_duration(tmp_path):
    seq, _ = build(tmp_path, max_travel=1.0)
    template = authorization_template(seq)
    template['operator_confirmations'] = {k: True for k in template['operator_confirmations']}
    assert template['limits']['max_approach_travel_m'] == 1.0
    template['limits']['max_approach_travel_m'] = 0.5
    path = tmp_path / 'auth.yaml'
    path.write_text(yaml.safe_dump(template))
    assert any('travel exceeds' in p for p in check_authorization(load_authorization(path), seq))


def test_profile_values_apply_and_command_line_wins(tmp_path):
    profile = tmp_path / 'p.yaml'
    profile.write_text(yaml.safe_dump({'m4': {'approach_speed': 0.03, 'max_travel': 0.8,
                                              'max-sensor-age': 0.2, 'detect_scans': 4}}))
    a = S.parse(['--profile', str(profile), '--max-travel', '1.2'])
    assert (a.approach_speed, a.max_travel, a.max_sensor_age, a.detect_scans) == (0.03, 1.2, 0.2, 4)
    assert a.max_duration == 50.0          # 1.2 / 0.03 + 10: travel never raises the speed


@pytest.mark.parametrize('content', [{'m4': {'approach_speed': 0.08}}, {'m4': {'warp': 1}},
                                     {'m4': {'detect_scans': 2.5}}, {'m4': {'max_sensor_age': 0.9}}])
def test_profile_outside_limits_is_refused(tmp_path, content):
    profile = tmp_path / 'p.yaml'
    profile.write_text(yaml.safe_dump(content))
    with pytest.raises(ConfigError):
        S.parse(['--profile', str(profile)])


def test_shipped_profiles_are_valid_and_build(tmp_path):
    files = sorted(PROFILES.glob('m4_*.yaml'))
    assert files, f'no profiles in {PROFILES}'
    for f in files:
        a = S.parse(['--profile', str(f)])
        a.mode, a.measured_coast = MODE_SIM, 0.035
        path = tmp_path / f'{f.stem}.yaml'
        path.write_text(yaml.safe_dump(S.CATALOG['M4'].build(a), sort_keys=False))
        load_sequence(path)


def test_main_refuses_removed_start_window_and_physical_shadow(tmp_path, monkeypatch, capsys):
    monkeypatch.delenv('LITE3_STATE_FILE', raising=False)
    assert S.main(['--phases', 'M4', '--approach-start', '0.365', '--report-dir', str(tmp_path)]) == 3
    assert 'no longer needs' in capsys.readouterr().out
    assert S.main(['--phases', 'M4', '--shadow', '--physical', '--report-dir', str(tmp_path)]) == 3


def test_main_refuses_trigger_distance_that_differs_from_the_running_boundary(tmp_path, monkeypatch,
                                                                           capsys):
    state = tmp_path / 'state.env'
    state.write_text(f'MODE=dry\nRUN_DIR={tmp_path / "run"}\nBOUNDARY=0.36\nCLEARANCE=0.20\n')
    monkeypatch.setenv('LITE3_STATE_FILE', str(state))
    assert S.main(['--phases', 'M4', '--trigger-distance', '0.50', '--report-dir', str(tmp_path)]) == 3
    assert 'running collision_monitor boundary is 0.36' in capsys.readouterr().out


# ---- measured command latency (M2) feeds the boundary ----------------------------------------
def test_m2_records_delivery_latency_and_suite_uses_it(tmp_path):
    from lite3_movement_tests.report import write_reports
    a = m4_args()
    path = tmp_path / 'm2.yaml'
    path.write_text(yaml.safe_dump(S.CATALOG['M2'].build(a), sort_keys=False))
    robot = FakeNavRobot(Telemetry(), SimLite3(start='lying'))
    report = SequenceRunner(load_sequence(path), robot, MODE_SIM, log=lambda *_: None).run()
    step = {s['id']: s for s in report['steps']}['deadman_stop']
    assert step['measurements']['motion']['first_delivery_s'] is not None
    report['physical_suite'] = {'parameters': {'max_speed': 0.05}}
    paths = write_reports(report, tmp_path / 'm2')
    S.record_pass(tmp_path / 'run', 'M2', paths['markdown'])
    delivery = S.measured_command_latency(tmp_path / 'run')
    assert delivery == step['measurements']['motion']['first_delivery_s']
    assert S.path_latency_allowance(delivery) == 0.10            # never below the design allowance
    assert S.path_latency_allowance(0.12) == 0.17
    assert S.path_latency_allowance(None) == 0.15                 # unmeasured: conservative


# ---- M5 overspeed root cause and fix -----------------------------------------------------
def test_m5_cap_holds_despite_mppi_reset_at_goal_start(tmp_path):
    a = m4_args(max_speed=0.05)
    path = tmp_path / 'm5.yaml'
    path.write_text(yaml.safe_dump(S.CATALOG['M5'].build(a), sort_keys=False))
    robot = FakeNavRobot(Telemetry(), SimLite3(start='lying'))
    report = SequenceRunner(load_sequence(path), robot, MODE_SIM, log=lambda *_: None).run()
    assert report['verdict'] == 'PASSED'
    assert max(c[2] for c in robot.mux_log if c[1] == 'nav') <= 0.0501
    assert {s['id']: s for s in report['steps']}['nav_straight']['measurements']['nav_velocity_cap']['ok']


def _m5(tmp_path, clamp, republish):
    a = m4_args(max_speed=0.05)
    path = tmp_path / 'm5.yaml'
    path.write_text(yaml.safe_dump(S.CATALOG['M5'].build(a), sort_keys=False))
    robot = FakeNavRobot(Telemetry(), SimLite3(start='lying'))
    if not clamp:
        robot.set_nav_velocity_cap = lambda vx, wz: {'ok': True}      # reports ok, does not clamp
    if not republish:                                                  # old code: only before the goal
        original = robot.set_speed_limit
        robot.set_speed_limit = lambda v: original(v) if robot.goal is None else True
    seq = load_sequence(path)
    template = authorization_template(seq, hours=0.25)       # as run_physical.sh generates it
    template['operator_confirmations'] = {k: True for k in template['operator_confirmations']}
    auth_path = tmp_path / 'auth.yaml'
    auth_path.write_text(yaml.safe_dump(template))
    report = SequenceRunner(seq, robot, MODE_PHYSICAL, log=lambda *_: None,
                            authorization=load_authorization(auth_path)).run()
    return {s['id']: s for s in report['steps']}['nav_straight'], robot


def test_m5_without_either_fix_reproduces_the_physical_overspeed(tmp_path):
    """2026-11-16: the speed limit sent before the goal is lost at MPPI's goal-start reset."""
    step, _ = _m5(tmp_path, clamp=False, republish=False)
    assert step['status'] == 'ABORTED' and 'beyond authorized limits' in step['summary']


@pytest.mark.parametrize('clamp, republish', [(True, False), (False, True)])
def test_m5_each_half_of_the_fix_alone_holds_the_limit(tmp_path, clamp, republish):
    step, robot = _m5(tmp_path, clamp, republish)
    assert step['status'] == 'PASSED', step['summary']
    assert max(c[2] for c in robot.mux_log if c[1] == 'nav') <= 0.0501


def test_nav2_overspeed_aborts_against_the_authorized_session_speed(tmp_path):
    """Needs the real (physical-mode) authorization: without one the runner can only fall back to the hard
    ceiling, which is 0.5 m/s since 2026-11-17, so a simulation run would not catch 0.15 m/s."""
    a = m4_args(max_speed=0.05)
    path = tmp_path / 'm5.yaml'
    path.write_text(yaml.safe_dump(S.CATALOG['M5'].build(a), sort_keys=False))
    seq = load_sequence(path)
    template = authorization_template(seq, hours=0.25)
    template['operator_confirmations'] = {k: True for k in template['operator_confirmations']}
    auth_path = tmp_path / 'auth.yaml'
    auth_path.write_text(yaml.safe_dump(template))
    robot = FakeNavRobot(Telemetry(), SimLite3(start='lying'), nav_fault='nav_overspeed')
    report = SequenceRunner(seq, robot, MODE_PHYSICAL, log=lambda *_: None,
                            authorization=load_authorization(auth_path)).run()
    step = {s['id']: s for s in report['steps']}['nav_straight']
    assert step['status'] == 'ABORTED' and 'beyond authorized limits' in step['summary']
    assert 'caps 0.05' in step['summary'] and report['safe_stop']['rest_verified']


def test_m5_refuses_when_the_cap_cannot_be_verified(tmp_path):
    a = m4_args(max_speed=0.05)
    path = tmp_path / 'm5.yaml'
    path.write_text(yaml.safe_dump(S.CATALOG['M5'].build(a), sort_keys=False))
    robot = FakeNavRobot(Telemetry(), SimLite3(start='lying'), nav_fault='cap_unverified')
    report = SequenceRunner(load_sequence(path), robot, MODE_SIM, log=lambda *_: None).run()
    step = {s['id']: s for s in report['steps']}['nav_straight']
    assert step['status'] == 'FAILED' and 'velocity cap' in step['summary']
    assert not robot.goal_sent


# ---- tracker unit tests --------------------------------------------------------------------
def tracker(**kw):
    args = dict(detection_range_m=2.0, confirm_scans=3, tolerance_m=0.15, lost_scans=3)
    args.update(kw)
    return ObstacleTracker(**args)


def test_tracker_confirms_after_consistent_scans_and_ignores_out_of_range():
    t = tracker()
    assert t.update(0.0, 2.5, 0.0) is None and not t.candidate        # beyond range
    assert t.update(0.1, 1.00, 0.000) is None
    assert t.update(0.2, 0.99, 0.005) is None
    assert t.update(0.3, 0.99, 0.010) == 'confirmed'


def test_tracker_resets_candidate_on_inconsistent_jump():
    t = tracker()
    t.update(0.1, 1.0, 0.0)
    t.update(0.2, 0.6, 0.0)                     # 0.4 m jump: not the same object
    assert len(t.candidate) == 1 and not t.is_confirmed


def test_tracker_tolerates_odometry_under_reading_but_flags_a_moving_object():
    t = tracker()
    for i in range(3):
        t.update(i * 0.1, 1.0, 0.0)
    # Robot really walked 0.6 m, odometry says 0.51 m (15 % under-read): consistent.
    events = [t.update(1 + i * 0.1, 1.0 - 0.6 * i / 10, 0.51 * i / 10) for i in range(11)]
    assert 'unexpected' not in events
    t2 = tracker()
    for i in range(3):
        t2.update(i * 0.1, 1.0, 0.0)
    # Obstacle comes 0.3 m closer than the robot's own travel explains.
    events = [t2.update(1 + i * 0.1, 1.0 - 0.05 * i - 0.03 * i, 0.05 * i) for i in range(11)]
    assert 'unexpected' in events


def test_tracker_reports_lost_only_where_the_obstacle_must_be_visible():
    t = tracker()
    for i in range(3):
        t.update(i * 0.1, 1.0, 0.0)
    assert [t.update(1 + i * 0.1, None, 0.0) for i in range(3)][-1] == 'lost'
    edge = tracker()
    for i in range(3):
        edge.update(i * 0.1, 1.9, 0.0)
    assert [edge.update(1 + i * 0.1, None, 0.0) for i in range(5)] == [None] * 5   # at the range edge


def test_collision_monitor_chatter_at_the_polygon_edge_fails_criteria_not_safety(tmp_path):
    """Physical 2026-11-17 12:57: the robot came to rest on the polygon edge, the collision_monitor
    released for ~0.2 s and 3 forward commands reached the robot. That is inside the boundary's
    one-grid-cell sensor margin: a missed PASS criterion (and reported), not a protective-stop
    failure needing intervention."""
    report, steps, robot = run_m4(tmp_path, nav_fault='cm_edge_chatter')
    step = walk(steps)
    m = step['measurements']
    assert step['status'] == 'FAILED', step['summary']
    assert m['outcome'] == 'STOP_CRITERIA_FAILED'
    assert m['forward_at_robot_after_stop_inside_boundary'] >= 1
    assert 'polygon edge' in step['summary']
    assert report['verdict'] == 'FAILED_SAFE'
    assert true_clearance(robot, 1.0) >= 0.15


def test_late_collision_monitor_within_the_grid_margin_fails_the_response_criterion(tmp_path):
    # 0.6 s late at 0.05 m/s = 3 cm, inside the boundary's one-cell (5 cm) sensor margin: not a pass.
    report, steps, robot = run_m4(tmp_path, nav_fault='cm_slow')
    step = walk(steps)
    assert step['status'] == 'FAILED' and step['measurements']['outcome'] == 'STOP_CRITERIA_FAILED'
    assert 'unsafe commands suppressed within' in step['summary']
    assert true_clearance(robot, 1.0) >= 0.15


def test_runner_refuses_speeds_above_the_validated_range_without_a_moved_out_backup_lock(
        tmp_path, monkeypatch, capsys):
    state = tmp_path / 'state.env'
    state.write_text(f'MODE=dry\nRUN_DIR={tmp_path / "run"}\nBOUNDARY=0.98\nCLEARANCE=0.15\nMAX_SPEED=0.5\n')
    monkeypatch.setenv('LITE3_STATE_FILE', str(state))
    assert S.main(['--sim', '--phases', 'M5', '--max-speed', '0.3', '--report-dir', str(tmp_path)]) == 3
    out = capsys.readouterr().out
    assert 'above the validated 0.10 m/s' in out and '--stop-distance >= 0.93' in out
    # Within the validated range nothing changes.
    assert S.main(['--sim', '--phases', 'M0', '--max-speed', '0.05', '--report-dir', str(tmp_path)]) == 0
