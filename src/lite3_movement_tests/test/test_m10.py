"""M10 autonomous roaming against the simulated stack (no ROS, no robot): independence from
M0-M9, speed modes, operator commands, point-cloud goal veto, safety stops, watchdog logic."""

import math

import pytest
import yaml

from lite3_movement_tests import physical_suite as S
from lite3_movement_tests import roam as R
from lite3_movement_tests.config import ConfigError, load_sequence
from lite3_movement_tests.roam_sim import make_sim_robot, table
from lite3_movement_tests.runner import MODE_READONLY, MODE_SIM
from lite3_movement_tests.telemetry import Telemetry


def roam(tmp_path, script=(), stop_after=200.0, cfg=None, mode=MODE_SIM, watchdog=True, setup=None,
         **robot_kwargs):
    robot = make_sim_robot(**robot_kwargs) if mode == MODE_SIM else \
        make_sim_robot(can_actuate=False, **robot_kwargs)
    cfg = cfg or R.RoamConfig()
    path = tmp_path / 'readiness.yaml'
    path.write_text(yaml.safe_dump(R.readiness_sequence(mode, {}), sort_keys=False))
    session = R.RoamSession(robot, cfg, mode, R.ControlChannel(tmp_path / 'ctl'), log=lambda *_: None,
                            watchdog=R.WatchdogLogic(cfg.speed_limit, cfg.geofence_m) if watchdog else None,
                            script=list(script), sim_stop_after_s=stop_after)
    if setup:
        original = session._roam

        def patched(summary):
            from lite3_movement_tests.roam import Explorer
            real = Explorer.__init__

            def init(self, *a, **k):
                real(self, *a, **k)
                session.explorer = self
                setup(session)
            Explorer.__init__ = init
            try:
                return original(summary)
            finally:
                Explorer.__init__ = real
        session._roam = patched
    outcome, summary = session.run(load_sequence(path))
    return outcome, summary, robot, session


# ---- independence ----------------------------------------------------------------------
def test_m10_is_not_part_of_the_m0_m9_catalogue_and_reads_no_gates(tmp_path, monkeypatch):
    assert 'M10' not in S.CATALOG
    assert list(S.CATALOG)[:10] == ['M0', 'M1', 'M2', 'DET', 'M3', 'M4', 'M5', 'M6', 'M7', 'M8']
    monkeypatch.setattr(S, 'passed_tests', lambda *_: pytest.fail('M10 must not read test gates'))
    outcome, summary, *_ = roam(tmp_path, stop_after=20)
    assert outcome == 'STOPPED' and summary['readiness']['verdict'] == 'PASSED'


def test_own_readiness_includes_stop_delivery_and_standing():
    ids = [s['id'] for s in R.readiness_sequence(MODE_SIM, {})['steps']]
    assert ids == ['readiness', 'nav_readiness', 'telemetry_contract', 'initial_rest', 'zero_delivery',
                   'stand', 'verify_standing']


# ---- nominal roaming with operator commands -------------------------------------------
def test_roams_maps_and_obeys_pause_resume_speed_stop(tmp_path):
    script = [(40, 'pause', None), (55, 'resume', None), (70, 'speed', 'fast'), (110, 'speed', 'slow')]
    outcome, summary, robot, session = roam(tmp_path, script=script, stop_after=180)
    assert outcome == 'STOPPED' and summary['stop_reason'] == 'operator stop'
    assert summary['rest_verified'] is True
    st = summary['stats']
    assert st['goals_sent'] >= 3 and st['goals_succeeded'] >= 2 and st['pauses'] == 1
    assert summary['map_area_m2']['end'] > summary['map_area_m2']['start']
    assert st['max_from_anchor_m'] <= 3.0
    names = [e['event'] for e in summary['events']]
    assert names.index('paused') < names.index('resumed')
    # No motion command reached the robot while paused (after the 1.5 s zero burst).
    t0 = session.t_roam
    paused = [c for t, c in robot.telemetry.system_cmd.since(0) if t0 + 42 <= t <= t0 + 54.5 and not c.is_zero]
    assert paused == []
    # Speed modes were enforced at the robot side.
    fast = [c.vx for t, c in robot.telemetry.system_cmd.since(0) if t0 + 72 <= t <= t0 + 110]
    slow = [c.vx for t, c in robot.telemetry.system_cmd.since(0) if t0 + 113 <= t]
    assert max(fast) <= 0.1001 and max(fast) > 0.06
    assert max(slow) <= 0.0501


def test_speed_above_session_limit_is_refused_and_keeps_moving_slow(tmp_path):
    cfg = R.RoamConfig(speed_limit=0.06)
    outcome, summary, robot, session = roam(tmp_path, script=[(20, 'speed', 'fast')], stop_after=40, cfg=cfg)
    assert outcome == 'STOPPED' and session.speed_mode == 'slow' and summary['stats']['speed_changes'] == 0
    assert max(c.vx for _, c in robot.telemetry.system_cmd.since(0)) <= 0.0501


def test_runaway_gait_is_caught_by_actual_speed_inside_the_geofence(tmp_path):
    outcome, summary, robot, session = roam(tmp_path, stop_after=120, model_fault='runaway')
    assert outcome == 'SAFETY_STOP' and 'unexpected motion' in summary['stop_reason']


def test_dry_mode_publishes_nothing_and_reports_would_be_goals(tmp_path):
    outcome, summary, robot, session = roam(tmp_path, mode=MODE_READONLY, stop_after=15)
    assert outcome == 'STOPPED'
    assert not robot.sent and robot.model.x == 0.0
    assert any(e['event'] == 'would_send_goal' for e in summary['events'])


# ---- point-cloud enhanced exploration ---------------------------------------------------
def test_pointcloud_only_obstacle_vetoes_goals_the_2d_map_cannot_see(tmp_path):
    # A table at the first frontier: invisible in the 2D SLAM map, present in the point-cloud costmap.
    outcome, summary, robot, session = roam(tmp_path, stop_after=60, cloud_only_obstacles=table(2.1, 0.0))
    goals = [e for e in summary['events'] if e['event'] == 'goal']
    assert summary['goals_vetoed_by_pointcloud'] >= 1
    assert all(math.hypot(g['x'] - 2.1, g['y']) > 0.6 for g in goals)


# ---- safety stops ------------------------------------------------------------------------
@pytest.mark.parametrize('kwargs, text', [
    (dict(roam_fault='raw_lidar_blind'), 'raw LiDAR nearly empty'),
    (dict(roam_fault='raw_lidar_stale'), 'raw LiDAR /livox/points stale'),
    (dict(roam_fault='lio_stale'), 'Fast-LIO'),
    (dict(roam_fault='map_frozen'), 'SLAM map not updating'),
    (dict(nav_fault='hazard_stale'), 'hazard cloud stale'),
    (dict(nav_fault='nav_overspeed'), 'robot-side command'),
    (dict(nav_fault='nav_reverse'), 'reverse'),
])
def test_unsafe_conditions_stop_roaming(tmp_path, kwargs, text):
    outcome, summary, robot, session = roam(tmp_path, stop_after=120, **kwargs)
    assert outcome == 'SAFETY_STOP', summary['stop_reason']
    assert text in summary['stop_reason'], summary['stop_reason']
    assert summary['rest_verified'] is True


def test_geofence_is_enforced_on_the_slam_pose(tmp_path):
    # Goals stay inside the geofence; if the robot is pushed outside anyway (here: a goal planted
    # beyond it), the runner's map-pose geofence stops roaming.
    def setup(session):
        session.explorer.choose = lambda *a, **k: {'kind': 'frontier', 'x': 2.6, 'y': 0.0, 'yaw': 0.0}
    outcome, summary, robot, session = roam(tmp_path, stop_after=120, cfg=R.RoamConfig(geofence_m=1.5),
                                            setup=setup)
    assert outcome == 'SAFETY_STOP' and 'geofence' in summary['stop_reason']
    assert summary['stats']['max_from_anchor_m'] < 1.5 + 0.1


def test_repeated_goal_failures_pause_instead_of_pushing_on(tmp_path):
    outcome, summary, robot, session = roam(tmp_path, stop_after=200, cfg=R.RoamConfig(max_goal_failures=1),
                                            map_obstacles=[(x / 10, 0.6) for x in range(5, 30)])
    names = [e['event'] for e in summary['events']]
    if 'goal_failed' in names:
        assert 'paused' in names


# ---- configuration --------------------------------------------------------------------
@pytest.mark.parametrize('kwargs', [dict(speed_limit=0.12, stack_max_speed=0.10), dict(geofence_m=12), dict(max_minutes=500),
                                    dict(abort_clearance_m=0.02), dict(max_sensor_age_s=0.8),
                                    dict(min_lidar_hz=5), dict(modes={'slow': (0.6, 0.1)}),
                                    dict(modes={'warp': 0.05}), dict(speed_limit=0.04),
                                    dict(stack_max_speed=0.05, speed_limit=0.08)])
def test_config_outside_envelope_is_refused(kwargs):
    with pytest.raises(ConfigError):
        R.RoamConfig(**kwargs)


def test_mode_override_parsing():
    modes = R.parse_modes('slow=0.04,fast=0.09:0.18')
    assert modes['slow'] == (0.04, 0.15) and modes['fast'] == (0.09, 0.18) and modes['normal'] == (0.08, 0.20)
    with pytest.raises(ConfigError):
        R.parse_modes('turbo=0.2')


def test_control_channel_ignores_commands_from_before_the_session(tmp_path):
    R.ControlChannel.send(tmp_path, 'stop')
    channel = R.ControlChannel(tmp_path)
    assert channel.poll() == []
    R.ControlChannel.send(tmp_path, 'speed', 'fast')
    assert [(c['cmd'], c['arg']) for c in channel.poll()] == [('speed', 'fast')]
    assert channel.poll() == []


# ---- watchdog logic ------------------------------------------------------------------------
def watched(**faults):
    t = Telemetry()
    for i in range(20):
        ts = 100 + i * 0.05
        t.on_joint(ts, ('a',), (0.0,))
        t.on_odom(ts, faults.get('x', 0.0) * i / 19, 0, 0.3, 0, 0, 0, 1, 0, 0, 0)
        t.on_imu(ts, *(faults.get('quat', (0, 0, 0, 1))), (0, 0, 0), (0, 0, 9.8))
        t.raw_cloud.add(ts, faults.get('bytes', 360000))
        t.on_system_cmd(ts, faults.get('vx', 0.05), 0.0, 0.0)
    return t


@pytest.mark.parametrize('faults, text', [
    (dict(vx=0.15), 'beyond limits'), (dict(vx=-0.05), 'beyond limits'), (dict(bytes=900), 'nearly empty'),
    (dict(x=4.0), 'geofence'), (dict(quat=(0.3, 0, 0, 0.954)), 'tilt'),
])
def test_watchdog_trips(faults, text):
    logic = R.WatchdogLogic(0.10, 3.0)
    logic.origin = (0.0, 0.0)
    t = watched(**faults)
    reasons = logic.evaluate(t, 100.95)
    reasons = logic.evaluate(t, 100.95 + 0.31) if not reasons else reasons
    assert any(text in r for r in reasons), reasons


def test_watchdog_quiet_when_healthy_and_flags_lost_heartbeat_and_stale_telemetry():
    logic = R.WatchdogLogic(0.10, 3.0)
    t = watched()
    assert logic.evaluate(t, 100.95, {'state': 'ROAMING', 'heartbeat': 50.0}, 51.0) == []
    assert any('heartbeat' in r for r in logic.evaluate(t, 100.95, {'state': 'ROAMING', 'heartbeat': 50.0}, 53.0))
    assert any('stale' in r for r in logic.evaluate(t, 102.0))


def test_actual_speed_limit_is_tied_to_the_selected_mode(tmp_path):
    """Slow mode (0.05): sustained odometry speed above max(2 x 0.05, 0.05 + 0.05) = 0.10 m/s stops
    roaming even though 0.10 is within the session limit."""
    outcome, summary, robot, session = roam(tmp_path, stop_after=30)
    from lite3_movement_tests.telemetry import Stream
    t = robot.telemetry
    now = robot.now()
    session.speed_changed = -1e9
    session.map = (now, session.map[1])
    t.odom = Stream()
    for i in range(12):
        t.on_odom(now - 0.55 + 0.05 * i, 0, 0, 0.3, 0, 0, 0, 1, 0.09, 0, 0)
    assert session.roam_violation(now) is None              # 0.09 m/s: within 2 x slow
    t.odom = Stream()
    for i in range(12):
        t.on_odom(now - 0.55 + 0.05 * i, 0, 0, 0.3, 0, 0, 0, 1, 0.13, 0, 0)
    assert 'unexpected motion' in (session.roam_violation(now) or '')


# ---- found in the live dry run (2026-11-17 13:11) -------------------------------------------
def test_robot_starting_next_to_an_obstacle_still_gets_a_goal(tmp_path):
    # Robot 0.22 m (footprint clearance) from an object beside it, as on the live robot.
    beside = [(x / 20, 0.42) for x in range(-6, 7)]
    outcome, summary, robot, session = roam(tmp_path, stop_after=40, map_obstacles=beside)
    goals = [e for e in summary['events'] if e['event'] == 'goal']
    assert goals, [e for e in summary['events']]
    assert summary['stats']['min_clearance_m'] >= 0.08


def test_distance_ignores_pose_jitter(tmp_path):
    import random
    rng = random.Random(3)
    real = make_sim_robot.__globals__['SimRoamRobot'].map_pose

    def jitter(self):
        x, y, yaw = real(self)
        return x + rng.uniform(-0.02, 0.02), y + rng.uniform(-0.02, 0.02), yaw
    make_sim_robot.__globals__['SimRoamRobot'].map_pose = jitter
    try:
        outcome, summary, robot, session = roam(tmp_path, mode=MODE_READONLY, stop_after=20)
    finally:
        make_sim_robot.__globals__['SimRoamRobot'].map_pose = real
    assert robot.model.x == 0.0                     # dry: the robot never moved
    assert summary['stats']['distance_m'] < 0.1     # was ~0.06 m/s of pure jitter before


# ---- hard speed ceiling raised to 0.5 m/s (operator decision 2026-11-17) -------------------------
def test_ceiling_is_half_a_metre_per_second_and_defaults_stay_low():
    from lite3_movement_tests.config import HARD_MAX_SPEED_M_S
    assert HARD_MAX_SPEED_M_S == 0.5
    cfg = R.RoamConfig()
    assert cfg.speed_limit == 0.10 and cfg.mode_speed(cfg.start_mode)[0] == 0.05     # nothing got faster
    assert R.DEFAULT_MODES['fast'][0] == 0.10


def test_higher_speed_needs_the_stack_started_for_it():
    with pytest.raises(ConfigError):
        R.RoamConfig(speed_limit=0.3)                                   # stack boundary only covers 0.10
    cfg = R.RoamConfig(speed_limit=0.3, stack_max_speed=0.5, modes={'fast': (0.3, 0.2)})
    assert cfg.mode_speed('fast')[0] == 0.3
    with pytest.raises(ConfigError):
        R.RoamConfig(speed_limit=0.6, stack_max_speed=0.6)              # above the hard ceiling


def test_backup_lock_moves_out_only_above_the_validated_speed():
    low, high = R.RoamConfig(), R.RoamConfig(speed_limit=0.3, stack_max_speed=0.5)
    assert R.backup_distance(low, 0.47, MODE_SIM) == 0.30
    assert R.backup_distance(high, 0.98, MODE_SIM) == 0.30              # sim/dry: not settable, unchanged
    from lite3_movement_tests.runner import MODE_PHYSICAL
    assert R.backup_distance(low, 0.47, MODE_PHYSICAL) == 0.30
    assert R.backup_distance(high, 0.98, MODE_PHYSICAL) == 0.93         # 5 cm inside the boundary
    assert R.backup_distance(high, 1.50, MODE_PHYSICAL) == 1.45


def test_roaming_at_a_higher_session_limit_still_enforces_the_selected_mode(tmp_path):
    cfg = R.RoamConfig(speed_limit=0.3, stack_max_speed=0.5, modes={'normal': (0.2, 0.2)})
    script = [(20, 'speed', 'normal')]
    outcome, summary, robot, session = roam(tmp_path, script=script, stop_after=70, cfg=cfg)
    assert outcome == 'STOPPED' and session.speed_mode == 'normal'
    t0 = session.t_roam
    speeds = [c.vx for t, c in robot.telemetry.system_cmd.since(0) if t >= t0 + 25]
    assert max(speeds) <= 0.2001 and max(speeds) > 0.1
