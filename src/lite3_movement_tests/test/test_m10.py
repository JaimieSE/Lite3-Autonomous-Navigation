"""M10 autonomous roaming against the simulated stack (no ROS, no robot): independence from
M0-M9, speed modes, operator commands, point-cloud goal veto, safety stops, watchdog logic."""

import math

import pytest
import yaml

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
def test_m10_runs_standalone_with_its_own_readiness(tmp_path):
    # The M0-M9 suite was removed 2026-11-21 (operator: "roam should be enough"); roam needs no gates.
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
    # Since the 2026-11-21 margin (0.25 m/s) the independent watchdog's 0.20 m/s runaway limit fires first.
    assert outcome == 'SAFETY_STOP' and ('unexpected motion' in summary['stop_reason']
                                         or 'runaway' in summary['stop_reason'])


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
                                    dict(abort_clearance_m=0.01), dict(max_sensor_age_s=0.8),
                                    dict(min_lidar_hz=5), dict(modes={'slow': (0.6, 0.1)}),
                                    dict(modes={'warp': 0.05}), dict(speed_limit=0.04),
                                    dict(stack_max_speed=0.05, speed_limit=0.08)])
def test_config_outside_envelope_is_refused(kwargs):
    with pytest.raises(ConfigError):
        R.RoamConfig(**kwargs)


def test_mode_override_parsing():
    modes = R.parse_modes('slow=0.04,fast=0.09:0.18')
    assert modes['slow'] == (0.04, 0.15) and modes['fast'] == (0.09, 0.18) and modes['normal'] == (0.08, 1.5)   # manufacturer turn limit
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


def test_watchdog_tolerates_observed_link_gaps_but_not_longer_ones():
    """Robot<->onboard-PC gaps up to 0.78 s (telemetry) / 0.83 s (LiDAR) were measured in preflights;
    they must not trip the watchdog. Longer outages still must."""
    logic = R.WatchdogLogic(0.10, 3.0)
    t = watched()                                   # last sample at 100.95
    assert not any('stale' in r for r in logic.evaluate(t, 100.95 + 0.85))
    late = logic.evaluate(t, 100.95 + 1.3)
    assert any('joints telemetry stale' in r for r in late), late
    assert any('raw LiDAR' in r and 'stale' in r for r in late), late


def test_actual_speed_limit_is_tied_to_the_selected_mode(tmp_path):
    """Slow mode (0.05): sustained odometry speed above max(2 x 0.05, 0.05 + ODOM_MARGIN_M_S 0.25) = 0.30 m/s
    stops roaming (margin was 0.05 until balance steps on the uneven course tripped it, 2026-11-21)."""
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
    assert session.roam_violation(now) is None  # latched velocity with independent rest evidence
    t.odom = Stream()
    for i in range(12):
        t.on_odom(now - 0.55 + 0.05 * i, 0.13 * 0.05 * i, 0, 0.3, 0, 0, 0, 1, 0.13, 0, 0)
    assert session.roam_violation(now) is None              # balance steps on lips: allowed now
    t.odom = Stream()
    for i in range(12):
        t.on_odom(now - 0.55 + 0.05 * i, 0.35 * 0.05 * i, 0, 0.3, 0, 0, 0, 1, 0.35, 0, 0)
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


# ---- hard speed ceiling raised to 1.0 m/s (operator decisions 2026-11-17: 0.5, 2026-11-18: 1.0) --
def test_ceiling_is_one_metre_per_second_and_defaults_stay_low():
    from lite3_movement_tests.config import HARD_MAX_SPEED_M_S
    assert HARD_MAX_SPEED_M_S == 1.0
    cfg = R.RoamConfig()
    assert cfg.speed_limit == 0.10 and cfg.mode_speed(cfg.start_mode)[0] == 0.05     # nothing got faster
    assert R.DEFAULT_MODES['fast'][0] == 0.10


def test_higher_speed_needs_the_stack_started_for_it():
    with pytest.raises(ConfigError):
        R.RoamConfig(speed_limit=0.3)                                   # stack boundary only covers 0.10
    cfg = R.RoamConfig(speed_limit=0.3, stack_max_speed=0.5, modes={'fast': (0.3, 0.2)})
    assert cfg.mode_speed('fast')[0] == 0.3
    with pytest.raises(ConfigError):
        R.RoamConfig(speed_limit=1.1, stack_max_speed=1.1)              # above the hard ceiling
    assert R.RoamConfig(speed_limit=1.0, stack_max_speed=1.0).speed_limit == 1.0
    from lite3_movement_tests.runner import MODE_PHYSICAL
    fast = R.RoamConfig(speed_limit=1.0, stack_max_speed=1.0)
    assert R.backup_distance(fast, 2.83, MODE_PHYSICAL) == 2.78        # follows the 1.0 m/s boundary


def test_backup_lock_moves_out_only_above_the_validated_speed():
    low, high = R.RoamConfig(), R.RoamConfig(speed_limit=0.3, stack_max_speed=0.5)
    assert R.backup_distance(low, 0.47, MODE_SIM) == 0.05
    assert R.backup_distance(high, 0.98, MODE_SIM) == 0.05              # sim/dry: not settable, unchanged
    from lite3_movement_tests.runner import MODE_PHYSICAL
    assert R.backup_distance(low, 0.47, MODE_PHYSICAL) == 0.05
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


# ---- spin-and-walk strategy (2026-11-18: robot stood still on a sparse 1-scan map) ----------------
def _grid(width_m=8.0, height_m=6.0, res=0.05, free=(), occupied=()):
    """Unknown grid with free / occupied axis-aligned boxes (x0, x1, y0, y1) in metres; origin (-4, -3)."""
    from lite3_locomotion.frontiers import Grid
    w, h = int(width_m / res), int(height_m / res)
    data = [-1] * (w * h)
    for value, boxes in ((0, free), (100, occupied)):
        for x0, x1, y0, y1 in boxes:
            for j in range(h):
                for i in range(w):
                    x, y = -4 + (i + 0.5) * res, -3 + (j + 0.5) * res
                    if x0 <= x <= x1 and y0 <= y <= y1:
                        data[j * w + i] = value
    return Grid(w, h, res, data, -4.0, -3.0)


def _walker(geofence=3.0):
    return R.ScanWalk(R.RoamConfig(geofence_m=geofence, strategy='scan_walk'), (0.0, 0.0))


def test_scan_walk_walks_toward_the_open_side_and_stops_short_of_the_wall():
    grid = _grid(free=[(-0.5, 2.0, -0.5, 0.5)], occupied=[(2.0, 2.1, -1.0, 1.0)])
    w = _walker()
    goal = w.choose(grid, (0.0, 0.0, 0.0), 0.0)
    assert goal['kind'] == 'walk' and goal['ends_at'] == 'obstacle'
    assert abs(goal['yaw']) < 0.01                                  # straight at the open side
    assert 1.0 < goal['x'] <= 2.0 - R.ScanWalk.STANDOFF_M + 0.06    # nose ~0.35 m before the wall
    assert abs(goal['y']) < 0.01


def test_scan_walk_does_not_walk_into_unseen_space_or_through_a_narrow_gap():
    # Seen-free strip 0.2 m wide beside obstacles: narrower than the corridor -> blocked.
    grid = _grid(free=[(-0.5, 2.5, -0.1, 0.1)], occupied=[(0.6, 2.5, 0.2, 0.25), (0.6, 2.5, -0.25, -0.2)])
    assert _walker().choose(grid, (0.0, 0.0, 0.0), 0.0) is None
    grid = _grid(free=[(-0.4, 0.4, -0.4, 0.4)])                    # nothing seen beyond 0.4 m
    assert _walker().choose(grid, (0.0, 0.0, 0.0), 0.0) is None


def test_scan_walk_prefers_not_to_go_back_where_it_came_from():
    grid = _grid(free=[(-2.5, 2.5, -0.5, 0.5)])                    # open both ways, equal length
    w = _walker()
    w.came_from = 0.0                                               # just walked toward +x ...
    goal = w.choose(grid, (0.0, 0.0, 0.0), 0.0)
    assert abs(goal['yaw']) < 0.01                                  # ... so keep going, not back


def test_scan_walk_respects_the_geofence_and_costmap_hazards():
    grid = _grid(free=[(-0.5, 3.9, -0.5, 0.5)])
    goal = _walker(geofence=1.5).choose(grid, (0.0, 0.0, 0.0), 0.0)
    assert math.hypot(goal['x'], goal['y']) <= 1.5 - 0.4
    cost = _grid(free=[(-4, 4, -3, 3)], occupied=[(1.2, 1.3, -3, 3)])   # point-cloud-only wall
    goal = _walker().choose(grid, (0.0, 0.0, 0.0), 0.0, cost)
    assert goal['ends_at'] == 'obstacle'
    ahead = (1.2 - goal['x']) / math.cos(goal['yaw'])               # to the hazard along the heading
    assert ahead >= R.ScanWalk.STANDOFF_M - 0.06


def test_scan_walk_stops_short_of_the_live_proximity_stop_zone():
    """A walk must end outside the terrain stop's ahead zone, or the lock holds even in-place turns."""
    grid = _grid(free=[(-0.5, 3.0, -0.6, 0.6)], occupied=[(3.0, 3.1, -1.0, 1.0)])
    w = _walker()
    w.stop_ahead_m = 1.45                     # backup lock at 0.5 m/s (1.14 m nose + 0.305)
    goal = w.choose(grid, (0.0, 0.0, 0.0), 0.0)
    assert goal['x'] <= 3.0 - (1.45 + R.ScanWalk.STOP_AHEAD_MARGIN_M) + 0.06
    room = _grid(free=[(-0.5, 3.0, -0.6, 0.6)],
                 occupied=[(3.0, 3.1, -1, 1), (-0.7, -0.6, -1, 1), (-0.7, 3.1, 0.7, 0.8), (-0.7, 3.1, -0.8, -0.7)])
    w.stop_ahead_m = 2.65                     # zone reaches past every walk (>= MIN_WALK_M) in a 3.6 m room
    assert w.choose(room, (0.0, 0.0, 0.0), 0.0) is None


def test_scan_walk_starts_walking_on_an_open_map(tmp_path):
    outcome, summary, robot, _ = roam(tmp_path, stop_after=90, cfg=R.RoamConfig(strategy='scan_walk'))
    names = [e['event'] for e in summary['events']]
    assert names[:4] == ['costmap_cleared', 'scan_walk', 'roaming', 'walk']   # Nav2 clear at start
    assert summary['stats']['goals_succeeded'] >= 1 and robot.costmap_clears >= 1
    assert outcome == 'STOPPED' and summary['rest_verified']
    assert max(c.vx for _, c in robot.telemetry.system_cmd.since(0)) <= 0.0501


@pytest.mark.parametrize('side', [-1, 1])
def test_walk_goals_can_turn_either_way(side):
    grid = _grid(free=[(-0.25, 0.25, min(0, side * 2), max(0, side * 2))])
    goal = _walker().choose(grid, (0, 0, 0), 0)
    assert goal['y'] * side > 0.5
    assert goal['yaw'] * side > 0


def test_no_opening_scans_both_sides_then_pauses(tmp_path, monkeypatch):
    monkeypatch.setattr(R, 'blocked_passage', lambda *a, **kw: None)          # no curtain here
    monkeypatch.setattr(R.RoamSession, '_frontier_goal', lambda *a, **kw: None)   # scan path only
    monkeypatch.setattr(R.ScanWalk, 'choose', lambda *a, **kw: None)
    monkeypatch.setattr(R.ScanWalk, 'straight_ahead', lambda *a, **kw: None)
    outcome, summary, _, _ = roam(tmp_path, cfg=R.RoamConfig(strategy='scan_walk'), stop_after=100)
    turns = [e['degrees'] for e in summary['events'] if e['event'] == 'spin']
    assert turns == [30, 30, -30, -30, -30, -30]            # one round only, even after the clear
    assert summary['stats']['walk_goals'] == 0
    assert summary['stats']['costmap_clears'] == 2           # at start and before giving up
    assert summary['stats']['pauses'] == 1 and outcome == 'STOPPED'


def test_new_opening_cancels_scan_before_walking(tmp_path, monkeypatch):
    from lite3_movement_tests.roam_sim import SimRoamRobot
    original = SimRoamRobot.map_grid
    blind = _grid(occupied=[(0.6, 0.65, -1.0, 1.0)])   # known wall ahead: no go-straight

    def map_after_turn(robot):
        stamp, grid = original(robot)
        return stamp, blind if abs(robot.model.yaw) < 0.05 else grid

    monkeypatch.setattr(SimRoamRobot, 'map_grid', map_after_turn)
    _, summary, _, _ = roam(tmp_path, stop_after=35, cfg=R.RoamConfig(strategy='scan_walk'))
    events = summary['events']
    opened = next(i for i, e in enumerate(events) if e['event'] == 'spin_opening')
    assert events[opened]['turned_deg'] < 30
    assert events[opened + 1]['event'] == 'walk'


def test_one_sided_partial_map_starts_search_to_the_right(tmp_path, monkeypatch):
    from lite3_movement_tests.roam_sim import SimRoamRobot
    # The right side has more known space but still too little to send a walk goal.
    grid = _grid(free=[(-0.2, 0.51, -0.36, 0.05)], occupied=[(0.55, 0.6, 0.1, 1.0)])   # wall ahead-left
    monkeypatch.setattr(SimRoamRobot, 'map_grid', lambda robot: (robot.now(), grid))
    _, summary, _, _ = roam(tmp_path, stop_after=3, cfg=R.RoamConfig(strategy='scan_walk'))
    turns = [event['degrees'] for event in summary['events'] if event['event'] == 'spin']
    assert turns and turns[0] == -30
    assert summary['stats']['walk_goals'] == 0


def test_stalled_search_pauses(tmp_path, monkeypatch):
    from lite3_movement_tests.sim_model import FakeNavRobot
    monkeypatch.setattr(R.RoamSession, '_frontier_goal', lambda *a, **kw: None)   # spin/backtrack path
    monkeypatch.setattr(R.ScanWalk, 'choose', lambda *a, **kw: None)
    monkeypatch.setattr(R.ScanWalk, 'straight_ahead', lambda *a, **kw: None)
    monkeypatch.setattr(FakeNavRobot, '_spin_tick', lambda self: None)
    _, summary, _, _ = roam(tmp_path, stop_after=25, cfg=R.RoamConfig(strategy='scan_walk', spin_stall_s=12.0))
    names = [e['event'] for e in summary['events']]
    assert 'spin_stalled' in names and 'paused' in names and 'walk' not in names


def test_cli_defaults_to_scan_walk_and_frontier_stays_available():
    a = R.parse(['run', '--sim'])
    assert R.build_config(a, {}).strategy == 'scan_walk'
    assert R.build_config(R.parse(['run', '--sim', '--strategy', 'frontier']), {}).strategy == 'frontier'
    with pytest.raises(ConfigError):
        R.RoamConfig(strategy='wander')


# ---- speed plan: one source for the limit chain, stop zones and turning clearance ----------------
def test_default_roam_speed_is_capped_by_session_and_fast_can_go_higher():
    assert R.session_modes('', 0.10)['fast'] == R.DEFAULT_MODES['fast']        # validated: unchanged
    assert R.session_modes('', 0.50)['fast'][0] == 0.50
    assert R.session_modes('fast=0.3', 0.50)['fast'][0] == 0.3                # explicit --modes wins
    a = R.parse(['run', '--speed-limit', '0.5'])
    cfg = R.build_config(a, {'MAX_SPEED': '0.5'})
    assert cfg.start_mode == 'normal' and cfg.mode_speed(cfg.start_mode)[0] == 0.5   # operator 2026-11-23: regular speed
    assert R.session_modes('', 0.8)['normal'][0] == 0.5
    assert R.session_modes('', 0.8)['fast'][0] == 0.8
    assert R.session_modes('', 0.07)['normal'][0] == 0.07
    with pytest.raises(ConfigError):                                          # above the stack boundary
        R.build_config(R.parse(['run', '--speed-limit', '0.5']), {'MAX_SPEED': '0.10'})


def test_effective_settings_scale_with_speed():
    from lite3_locomotion import collision_profile as CP
    cal = CP.Calibration(sensing_age_s=0.158, path_latency_s=0.195, coast_m=0.035, coast_speed_m_s=0.05)
    slow, fast = R.effective_settings(0.10, 0.0, cal), R.effective_settings(0.50, 0.0, cal)
    assert slow['validated'] and not fast['validated'] and 'ASSUMED' in fast['coast_basis']
    assert slow['terrain_stop_from_nose_m'] == 0.05                              # roaming's 5 cm floor (2026-11-20)
    assert fast['terrain_stop_from_nose_m'] == 0.05                      # 5 cm stop line
    # Above 0.10 the STOP polygon stays sized for 0.10 m/s and Nav2's approach action (collision-time,
    # no fixed slowdown zone) brings the robot down to it, so turning clearance does NOT grow with speed.
    assert fast['approach_time_s'] >= 2.0 and slow['approach_time_s'] is None
    assert fast['boundary_parts']['required_m'] > slow['boundary_parts']['required_m']
    assert fast['turn_clearance_radius_m'] <= slow['turn_clearance_radius_m'] < 0.7   # 10 cm line at speed
    assert fast['turn_clearance_radius_m'] >= math.hypot(fast['stop_zone_from_centre_m'], R.ScanWalk.SPIN_CORRIDOR_HALF_M)
    assert fast['modes']['fast']['smoother_cap'][0] == 0.50 and fast['watchdog_cmd_abort_above_m_s'] == 0.52
    with pytest.raises(ValueError):                                             # boundary cap 3.0 m
        R.effective_settings(1.0, 0.5, CP.Calibration(sensing_age_s=1.5))


def test_plan_command_prints_without_a_stack(capsys, tmp_path, monkeypatch):
    monkeypatch.setenv('LITE3_STATE_FILE', str(tmp_path / 'state.env'))
    assert R.main(['plan', '--max-speed', '0.3']) == 0
    out = capsys.readouterr().out
    assert 'NOT VALIDATED' in out and 'fast    0.30 m/s' in out and 'open floor' in out
    assert R.main(['plan', '--max-speed', '0.1', '--speed-limit', '0.2']) == 2


def test_nav2_recovery_spin_is_within_every_mode_but_overspeed_is_not():
    """2026-11-19 13:28: Nav2's recovery Spin (0.20 rad/s, bypasses the smoother's mode cap) during a
    slow-mode walk stopped roaming. The validated 0.20 rad/s turn is allowed in every mode; forward
    speed is still held to the mode and turning beyond 0.20 still stops."""
    slow_v, slow_w = R.DEFAULT_MODES['slow']
    assert R.command_above_mode(0.0, 0.20, slow_v, slow_w, 'slow') is None
    assert R.command_above_mode(0.0, -0.21, slow_v, slow_w, 'slow') is None        # within 0.02 margin
    assert R.command_above_mode(0.0, 1.5, slow_v, slow_w, 'slow') is None         # manufacturer 1.5 rad/s
    assert 'above the selected' in R.command_above_mode(0.0, 1.6, slow_v, slow_w, 'slow')
    assert 'above the selected' in R.command_above_mode(slow_v + 0.05, 0.0, slow_v, slow_w, 'slow')


def test_roaming_uses_its_own_freshness_limits_and_keeps_the_rest():
    from lite3_movement_tests.runner import MODE_PHYSICAL
    seq = R.readiness_sequence(MODE_PHYSICAL, {})
    assert seq['telemetry'] == {'max_age_s': R.ROAM_TELEMETRY_AGE_S} and R.ROAM_TELEMETRY_AGE_S == 1.0
    nav = next(s for s in seq['steps'] if s['id'] == 'nav_readiness')
    assert nav['max_odom_lag_s'] == R.ROAM_MAX_ODOM_LAG_S == 1.0
    assert R.HARD_MIN_LIDAR_HZ == 6.0 and R.WatchdogLogic.TELEMETRY_AGE_S == 1.0   # was 8.0 (12:37 run)
    assert R.ROAM_RAW_LIDAR_STALE_S == 1.0 and R.ROAM_LIDAR_RATE_WINDOW_S == 5.0


def test_observed_floor_opens_directions_the_slam_map_left_unknown():
    """Physical 2026-11-20: open directions had no /scan returns in the 0.43-0.83 m band, so the SLAM map
    left them unknown and every scan found no open direction. Floor the 3D LiDAR observed (terrain grid)
    now counts as seen free; obstacles still end the walk."""
    grid = _grid(free=[(-0.3, 0.3, -0.3, 0.3)])                         # SLAM knows only the robot's spot
    walk = R.ScanWalk(R.RoamConfig(strategy='scan_walk'), (0.0, 0.0))
    assert walk.choose(grid, (0.0, 0.0, 0.0), 0.0) is None              # before: nothing seen free
    walk.floor_free = lambda x, y: math.hypot(x, y) < 3.0               # observed floor all round
    goal = walk.choose(grid, (0.0, 0.0, 0.0), 0.0)
    assert goal is not None and goal['free_m'] >= 1.0
    walled = _grid(free=[(-0.3, 0.3, -0.3, 0.3)], occupied=[(1.0, 1.1, -3, 3), (-1.1, -1.0, -3, 3),
                                                           (-3, 3, 1.0, 1.1), (-3, 3, -1.1, -1.0)])
    walk = R.ScanWalk(R.RoamConfig(strategy='scan_walk'), (0.0, 0.0))
    walk.floor_free = lambda x, y: True
    goal = walk.choose(walled, (0.0, 0.0, 0.0), 0.0)                    # walls still stop it short
    assert goal is None or max(abs(goal['x']), abs(goal['y'])) < 1.0


def test_a_pause_keeps_the_heartbeat_alive(tmp_path):
    """Physical 2026-11-20: an ordinary pause blocked the loop > 2 s and the watchdog ended the session
    ("runner heartbeat lost (2.0 s)"). Every step of a pause now writes a heartbeat."""
    beats = []

    def setup(session):
        original = session.channel.write_status
        session.channel.write_status = lambda st: (beats.append((session.robot.now(), st.get('state'))), original(st))
    outcome, summary, robot, session = roam(tmp_path, stop_after=40, script=[(10.0, 'pause', None)], setup=setup)
    roaming = [t for t, state in beats if state in ('ROAMING', 'PAUSED')]   # the watchdog's heartbeat phase
    gaps = [b - a for a, b in zip(roaming, roaming[1:])]
    assert summary['stats']['pauses'] >= 1, summary['stop_reason']
    assert max(gaps) < 1.0, max(gaps)


def test_a_metre_clear_ahead_keeps_it_walking_straight_on():
    """Operator 2026-11-20: with 1 m clear ahead of the nose it keeps walking forward, even where there is
    no room for a full turn at the end (a corridor); less than that and it searches as before."""
    corridor = _grid(free=[(-0.5, 3.0, -0.6, 0.6)], occupied=[(-0.5, 3.0, 0.6, 0.65), (-0.5, 3.0, -0.65, -0.6)])
    walk = R.ScanWalk(R.RoamConfig(strategy='scan_walk'), (0.0, 0.0))
    goal = walk.choose(corridor, (0.0, 0.0, 0.0), 0.0)
    assert goal is not None and abs(goal['yaw']) < 1e-6 and goal['x'] >= 0.5
    short = _grid(free=[(-0.5, 1.0, -0.6, 0.6)], occupied=[(1.0, 1.05, -0.7, 0.7), (-0.5, 1.0, 0.6, 0.65),
                                                           (-0.5, 1.0, -0.65, -0.6)])
    walk = R.ScanWalk(R.RoamConfig(strategy='scan_walk'), (0.0, 0.0))
    walk.stop_ahead_m = 0.355                              # live roaming stop line (nose + 5 cm)
    goal = walk.choose(short, (0.0, 0.0, 0.0), 0.0)
    # < 1 m ahead: no straight-on rule; a forward walk is fine only if it stops short and the turn fits
    # (1.2 m wide here; 0.92 m of open floor is enough since the 1.5 m course loosening, 2026-11-20).
    assert goal is None or abs(goal['yaw']) > 1e-6 or (
        goal['x'] <= 1.0 - walk.standoff() + 1e-6 and 0.6 >= walk.spin_radius())


def test_go_straight_when_only_the_blind_near_field_is_unknown():
    """Operator 2026-11-20 "GO STRAIGHT": standing, the floor 0.45-~1 m ahead is never seen, so nothing ahead
    is confirmed free; it now steps straight on unless a KNOWN obstacle is in the way."""
    walk = R.ScanWalk(R.RoamConfig(strategy='scan_walk'), (0.0, 0.0))
    unknown = _grid(free=[(-0.3, 0.3, -0.3, 0.3)])                     # only the robot's own spot is known
    assert walk.choose(unknown, (0.0, 0.0, 0.0), 0.0) is None
    goal = walk.straight_ahead(unknown, (0.0, 0.0, 0.0))
    # Long leg on the held heading (operator 2026-11-20 "straight line"), up to frontier_radius_m.
    assert goal is not None and goal['yaw'] == 0.0 and 0.5 <= goal['x'] <= walk.cfg.frontier_radius_m + 1e-9
    wall = _grid(free=[(-0.3, 0.3, -0.3, 0.3)], occupied=[(0.8, 0.85, -1, 1)])
    assert walk.straight_ahead(wall, (0.0, 0.0, 0.0)) is None             # known obstacle ahead: no step


# ---- gait (Motion Host Communication Interface s1.2.5) ----------------------------------------
def test_gait_is_sent_once_after_the_stand_check_and_logged(tmp_path):
    from lite3_movement_tests.ros_io import GAITS
    assert GAITS['rug_general'] == 0x21010401 and GAITS['rug_grip'] == 0x21010402
    cfg = R.RoamConfig(strategy='scan_walk', gait='rug_general')
    _, summary, robot, _ = roam(tmp_path, stop_after=20, cfg=cfg)
    sent = [s for s in robot.sent if s[1] == 'gait']
    assert [s[2] for s in sent] == ['rug_general']
    toggles = [s[0] for s in robot.sent if s[1] == 'toggle']
    assert not toggles or sent[0][0] > max(toggles)          # after standing up, never before
    events = [e for e in summary['events'] if e['event'] == 'gait']
    assert events and events[0]['code'] == '0x21010401' and events[0]['published']


def test_no_gait_by_default_and_unknown_gait_refused(tmp_path):
    _, _, robot, _ = roam(tmp_path, stop_after=10, cfg=R.RoamConfig(strategy='scan_walk'))
    assert not [s for s in robot.sent if s[1] == 'gait']
    with pytest.raises(ConfigError):
        R.RoamConfig(gait='moonwalk')


def test_gait_change_only_while_paused(tmp_path):
    script = [(10, 'gait', 'rug_grip'), (15, 'pause', None), (20, 'gait', 'rug_hstep')]
    _, summary, robot, _ = roam(tmp_path, script=script, stop_after=30, cfg=R.RoamConfig(strategy='scan_walk'))
    assert [s[2] for s in robot.sent if s[1] == 'gait'] == ['rug_hstep']


# ---- documented command codes (Motion Host Communication Interface) ------------------------
def test_documented_command_codes_and_heartbeat_defaults():
    from lite3_movement_tests import ros_io
    assert ros_io.POSTURE_TOGGLE == 0x21010202 and ros_io.HEARTBEAT_CODE == 0x21040001      # s1.2.2, s1.2.1
    assert ros_io.HEARTBEAT_PERIOD_S <= 0.5                                                 # >= 2 Hz
    assert ros_io.GAITS == {'flat_slow': 0x21010300, 'flat_medium': 0x21010307, 'flat_fast': 0x21010303,
                            'rug_general': 0x21010401, 'rug_grip': 0x21010402, 'rug_hstep': 0x21010407}
    assert R.RoamConfig().heartbeat is False                                                # opt-in
    robot = object.__new__(ros_io.RosRobot)            # no ROS: only the rate guard runs
    robot.can_actuate = True
    with pytest.raises(ValueError):
        robot.start_heartbeat(period_s=1.0)


def test_documented_ai_switches_are_off_by_default_and_encoded_as_documented():
    import socket, struct
    from lite3_movement_tests import ros_io
    assert ros_io.AI_OPTIONS_CODE == 0x21012109
    assert ros_io.AI_OPTIONS == {'all_off': 0x00, 'auto_stop': 0x20, 'obstacle_avoidance': 0x40}
    assert R.RoamConfig().ai_options == ()
    with pytest.raises(ConfigError):
        R.RoamConfig(ai_options=('tracking_please',))
    rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)       # loopback stand-in for the perception host
    rx.bind(('127.0.0.1', 0))
    rx.settimeout(2)
    assert ros_io.send_perception_udp(0x21012109, 0x40, rx.getsockname())
    assert struct.unpack('<III', rx.recv(64)) == (0x21012109, 0x40, 0)
    rx.close()


# ---- exploration gaps (2026-11-21): exit detector, failed-walk memory, dead-end backtracking -------------
def test_exit_detector_ignores_corridors_unknown_space_and_the_start():
    corridor = _grid(free=[(-0.5, 3.5, -0.75, 0.75)], occupied=[(-0.5, 3.5, 0.75, 0.8), (-0.5, 3.5, -0.8, -0.75)])
    assert R.detect_exit(corridor, (2.5, 0.0, 0.0), (0.0, 0.0)) is None          # 1.5 m wide: never "open"
    unknown = _grid(free=[(-0.5, 0.5, -0.5, 0.5)])                                  # unseen is not open
    assert R.detect_exit(unknown, (0.0, 0.0, 0.0), (0.0, 0.0)) is None
    room = _grid(free=[(-0.5, 3.5, -2.9, 2.9)])                                     # 4 x 5.8 m mapped open area
    assert R.detect_exit(room, (2.0, 0.0, 0.0), (0.0, 0.0), min_from_start_m=10.0) is None   # all near start


def test_exit_detector_finds_a_mapped_open_area_beyond_the_maze():
    # 1.2 m corridor from the start (x -3.9..-1.0) opening into a mapped 4.9 x 5.8 m area
    world = _grid(free=[(-3.9, -1.0, -0.6, 0.6), (-1.0, 3.9, -2.9, 2.9)],
                  occupied=[(-3.9, -1.0, 0.6, 0.65), (-3.9, -1.0, -0.65, -0.6)])
    start = (-3.5, 0.0)
    found = R.detect_exit(world, (-1.5, 0.0, 0.0), start)
    assert found and found['area_m2'] >= 3.0 and found['x'] > -1.0 and not found['reached']
    assert R.detect_exit(world, (1.5, 0.0, 0.0), start)['reached']


def test_scan_walk_remembers_failed_walks_and_avoids_them():
    grid = _grid(free=[(-0.5, 3.0, -0.6, 0.6)])
    w = _walker()
    goal = w.straight_ahead(grid, (0.0, 0.0, 0.0), now=0.0)
    assert goal is not None
    w.block(goal['x'], goal['y'], 0.0, R.FAILED_WALK_MEMORY_S)
    assert w.straight_ahead(grid, (0.0, 0.0, 0.0), now=1.0) is None                # same goal: avoided
    assert w.straight_ahead(grid, (0.0, 0.0, 0.0), now=R.FAILED_WALK_MEMORY_S + 1) is not None   # expired


def test_dead_end_backtracks_along_the_trail_instead_of_pausing(tmp_path, monkeypatch):
    monkeypatch.setattr(R, 'blocked_passage', lambda *a, **kw: None)          # no curtain here
    from lite3_movement_tests.roam_sim import SimRoamRobot
    monkeypatch.setattr(R.RoamSession, '_frontier_goal', lambda *a, **kw: None)   # spin/backtrack path
    monkeypatch.setattr(R.ScanWalk, 'choose', lambda *a, **kw: None)
    monkeypatch.setattr(R.ScanWalk, 'straight_ahead', lambda *a, **kw: None)
    original = R.RoamSession._search_turn

    def seeded(self, now, pose):
        if not self.trail and not self.backtracks:
            self.trail = [(pose[0] - 1.5, pose[1])]                              # a place we walked from
        return original(self, now, pose)
    monkeypatch.setattr(R.RoamSession, '_search_turn', seeded)
    outcome, summary, _, _ = roam(tmp_path, cfg=R.RoamConfig(strategy='scan_walk'), stop_after=100)
    names = [e['event'] for e in summary['events']]
    assert 'backtrack' in names
    assert names.index('backtrack') < (names.index('paused') if 'paused' in names else len(names))


def test_stairs_command_runs_the_routine_from_pause_and_resumes_roaming(tmp_path, monkeypatch):
    from lite3_movement_tests import stairs as ST
    original = ST.StairRoutine.step

    def on_stairs(self, t, pitch, roll, travelled):         # 0.45 m incline starting 5 cm ahead
        return original(self, t, 0.17 if 0.05 <= travelled < 0.5 else 0.0, roll, travelled)
    monkeypatch.setattr(ST.StairRoutine, 'step', on_stairs)
    script = [(5, 'pause', None), (6, 'stairs', 'up')]
    _, summary, robot, _ = roam(tmp_path, script=script, stop_after=80,
                                cfg=R.RoamConfig(strategy='scan_walk', stair_mode='hstep', auto_stairs=False))
    names = [e['event'] for e in summary['events']]
    assert 'safety_stop' not in names
    assert names.index('stairs_start') < names.index('stairs_switch_in') < names.index('stairs_done')
    assert [s[2] for s in robot.sent if s[1] == 'gait'][:2] == ['rug_hstep', 'flat_medium']
    assert max(c.vx for _, c in robot.telemetry.system_cmd.since(0)) <= 0.0501        # slow mode respected


def test_scene_detections_are_parsed_and_logged_with_the_map_pose(tmp_path, monkeypatch):
    import json
    from lite3_movement_tests.roam_sim import SimRoamRobot
    text = json.dumps({'objects': [{'label': 'person', 'confidence': 0.9}, {'label': 'chair', 'confidence': 0.2}],
                       'qr': 'EXIT-3'})
    assert R.parse_scene(text) == [('object', 'person', 0.9), ('qr', 'EXIT-3', 1.0)]
    assert R.parse_scene('not json') == [] and R.parse_scene('{}') == []
    monkeypatch.setattr(SimRoamRobot, 'scene', [(0.0, text), (0.1, text)], raising=False)
    _, summary, _, _ = roam(tmp_path, stop_after=10, cfg=R.RoamConfig(strategy='scan_walk'))
    kinds = [(s['kind'], s['value']) for s in summary['sightings']]
    assert kinds == [('object', 'person'), ('qr', 'EXIT-3')]                   # repeats suppressed
    assert all(s['x'] is not None for s in summary['sightings'])


def test_detected_step_is_crossed_automatically_in_ai_mode(tmp_path, monkeypatch):
    """Operator 2026-11-21: at the steps switch to AI mode on its own (Comm Interface s1.2.2, s1.2.3.2, s1.2.7)."""
    from lite3_movement_tests import ros_io as IO, stairs as ST
    from lite3_movement_tests.telemetry import Telemetry
    original_terrain = Telemetry.on_terrain
    window = {'until': None}

    def on_terrain(self, stamp, stop, reason, ahead, stop_ahead, problems, processing_s=None,
                   step_ahead_m=None, step_rise_m=None):
        if window['until'] is None:
            window['until'] = stamp + 12.0                           # a 0.11 m ledge lip 0.5 m ahead, first 12 s
        if stamp < window['until']:
            step_ahead_m, step_rise_m = 0.5, 0.11
        return original_terrain(self, stamp, stop, reason, ahead, stop_ahead, problems, processing_s,
                                step_ahead_m, step_rise_m)
    monkeypatch.setattr(Telemetry, 'on_terrain', on_terrain)
    original_step = ST.StairRoutine.step

    def on_ledge(self, t, pitch, roll, travelled):                  # body tilts while the feet climb
        return original_step(self, t, 0.2 if 0.02 <= travelled < 0.2 else 0.0, roll, travelled)
    monkeypatch.setattr(ST.StairRoutine, 'step', on_ledge)
    _, summary, robot, _ = roam(tmp_path, stop_after=60, cfg=R.RoamConfig(strategy='scan_walk', auto_stairs=True))
    names = [e['event'] for e in summary['events']]
    assert 'safety_stop' not in names and 'paused' not in names
    assert names.index('stairs_detected') < names.index('stairs_start') < names.index('stairs_done')
    simple = [s[2] for s in robot.sent if s[1] == 'simple']
    assert simple[:2] == [IO.AI_ENTER, IO.AI_GAIT_BASIC] and simple[2:4] == [IO.AI_EXIT, IO.NAVIGATION_MODE]
    axis = [s for s in robot.sent if s[1] == 'axis']
    assert axis and all(s[2] == IO.AXIS_FORWARD for s in axis) and max(s[3] for s in axis) > IO.AXIS_FORWARD_DEAD_ZONE
    assert axis[-1][3] == 0                                          # stopped before leaving AI state
    assert [s[2] for s in robot.sent if s[1] == 'gait'][-1] == 'flat_medium'
    assert names.count('stairs_start') == 1                          # cooldown: the same edge does not re-trigger


def test_safety_stop_during_ai_stairs_still_exits_ai_mode(tmp_path, monkeypatch):
    """Physical 2026-11-21: a safety stop mid-crossing left the robot in AI mode."""
    from lite3_movement_tests import ros_io as IO
    from lite3_movement_tests.telemetry import Telemetry
    original = Telemetry.on_terrain

    def on_terrain(self, stamp, stop, reason, ahead, stop_ahead, problems, processing_s=None, *_):
        return original(self, stamp, stop, reason, ahead, stop_ahead, problems, processing_s, 0.5, 0.11)
    monkeypatch.setattr(Telemetry, 'on_terrain', on_terrain)
    script = [(3, 'stop', None)]                                  # stop while still in AI state
    _, summary, robot, _ = roam(tmp_path, script=script, stop_after=30, cfg=R.RoamConfig(strategy='scan_walk', auto_stairs=True))
    simple = [s[2] for s in robot.sent if s[1] == 'simple']
    assert IO.AI_ENTER in simple and simple[-2:] == [IO.AI_EXIT, IO.NAVIGATION_MODE]


def test_own_ai_axis_commands_are_not_read_as_remote_takeover(tmp_path, monkeypatch):
    from lite3_movement_tests.roam_sim import SimRoamRobot
    from lite3_movement_tests.telemetry import Telemetry
    original_terrain = Telemetry.on_terrain
    window = {'until': None}

    def on_terrain(self, stamp, stop, reason, ahead, stop_ahead, problems, processing_s=None, *_):
        window['until'] = window['until'] or stamp + 12.0
        step = (0.5, 0.11) if stamp < window['until'] else (None, None)
        return original_terrain(self, stamp, stop, reason, ahead, stop_ahead, problems, processing_s, *step)
    monkeypatch.setattr(Telemetry, 'on_terrain', on_terrain)
    original_axis = SimRoamRobot.send_axis

    def echoing_axis(self, code, value):                          # the robot reports our axis on /handle_state
        original_axis(self, code, value)
        self.telemetry.on_handle(self.now(), 0.5 if value else 0.0, 0.0, 0.0)
    monkeypatch.setattr(SimRoamRobot, 'send_axis', echoing_axis)
    _, summary, _, _ = roam(tmp_path, stop_after=40, cfg=R.RoamConfig(strategy='scan_walk', auto_stairs=True))
    reasons = [e.get('reason') for e in summary['events'] if e['event'] == 'safety_stop']
    assert not any('remote' in (r or '') for r in reasons), reasons


def test_consecutive_lips_keep_ai_mode_on_until_the_ground_is_flat(tmp_path, monkeypatch):
    """Course 2026-11-21: a run of 0.10-0.15 m lips; exit AI mode only once no lip is ahead."""
    from lite3_movement_tests import ros_io as IO, stairs as ST
    from lite3_movement_tests.telemetry import Telemetry
    original_terrain = Telemetry.on_terrain
    window = {'until': None}

    def on_terrain(self, stamp, stop, reason, ahead, stop_ahead, problems, processing_s=None, *_):
        window['until'] = window['until'] or stamp + 25.0                 # lips keep appearing for 25 s
        step = (0.5, 0.12) if stamp < window['until'] else (None, None)
        return original_terrain(self, stamp, stop, reason, ahead, stop_ahead, problems, processing_s, *step)
    monkeypatch.setattr(Telemetry, 'on_terrain', on_terrain)
    _, summary, robot, _ = roam(tmp_path, stop_after=70, cfg=R.RoamConfig(strategy='scan_walk', auto_stairs=True,
                                                                         step_crossing=False))   # AI path only
    names = [e['event'] for e in summary['events']]
    assert 'stairs_continue' in names and 'safety_stop' not in names
    simple = [s[2] for s in robot.sent if s[1] == 'simple']
    assert simple.count(IO.AI_ENTER) == 1 and simple.count(IO.AI_EXIT) == 1      # one AI session for the run
    assert names.index('stairs_continue') < names.index('stairs_done')


def test_ai_mode_is_off_by_default_even_when_a_step_is_reported(tmp_path, monkeypatch):
    """Operator 2026-11-21: AI mode only on stairs (detection later), never on the uneven course."""
    from lite3_movement_tests import ros_io as IO
    from lite3_movement_tests.telemetry import Telemetry
    original = Telemetry.on_terrain

    def on_terrain(self, stamp, stop, reason, ahead, stop_ahead, problems, processing_s=None, *_):
        return original(self, stamp, stop, reason, ahead, stop_ahead, problems, processing_s, 0.5, 0.12)
    monkeypatch.setattr(Telemetry, 'on_terrain', on_terrain)
    assert R.RoamConfig().auto_stairs is False
    _, summary, robot, _ = roam(tmp_path, stop_after=20, cfg=R.RoamConfig(strategy='scan_walk'))
    assert IO.AI_ENTER not in [s[2] for s in robot.sent if s[1] == 'simple']
    # A 0.12 m step may be crossed by step crossing (2026-11-23), which uses the high-step gait, never AI mode.
    assert all(e['mode'] == 'hstep' for e in summary['events'] if e['event'] == 'stairs_start')


def test_scan_walk_falls_back_to_a_frontier_before_spinning():
    """Operator 2026-11-21: held heading and nearest turn blocked -> Nav2 goal to a frontier, not a spin."""
    grid = _grid(free=[(-0.6, 0.3, -0.3, 2.0)], occupied=[(0.35, 0.45, -1.0, 1.0)])
    cfg = R.RoamConfig(strategy='scan_walk')
    sw = R.ScanWalk(cfg, (0.0, 0.0))
    class Stub:
        pass
    r = Stub()
    r.cfg, r.map, r.frontiers = cfg, (0.0, grid), R.Explorer(cfg, (0.0, 0.0))
    r._costmap = lambda: None
    goal = R.RoamSession._frontier_goal(r, 0.0, (0.0, 0.0, 0.0))
    assert goal is not None and goal['ends_at'] == 'frontier' and goal['kind'] == 'walk'
    assert goal['y'] > 0.4                         # along the open corridor, not into the wall ahead


def test_walk_cut_short_by_the_standoff_is_remembered():
    """Course 2026-11-21: a standoff-cancelled walk must enter the failed-walk memory (it was lost)."""
    class Stub:
        pass
    sw = R.ScanWalk(R.RoamConfig(strategy='scan_walk'), (0.0, 0.0))
    s = Stub()
    s.cfg, s.explorer, s.frontiers, s.failures, s.stats = R.RoamConfig(strategy='scan_walk'), sw, None, 0, \
        {'goals_failed': 0, 'goals_succeeded': 0, 'walks_to_obstacle': 0}
    s.record = None
    s.goal = {'kind': 'walk', 'x': 0.77, 'y': -0.08, 'yaw': 0.0, 'ends_at': 'straight_on'}
    s.walk_start = (0.2, 0.0)
    s.events = []
    s.event = lambda name, **kw: s.events.append(name)
    s._terrain_now = lambda now: None
    s._pause = lambda why: None
    s._walk_failed = lambda now, why, goal=None: R.RoamSession._walk_failed(s, now, why, goal)
    R.RoamSession._end_walk(s, 10.0, (0.22, 0.0, 0.0), 'obstacle ahead (walking standoff)')
    assert s.events == ['walk_failed'] and s.goal is None
    assert sw.avoided(0.77, -0.12, 11.0)            # the next straight-on goal there is skipped


def test_frontier_fallback_accepts_goals_in_wall_inflation():
    """12:37 run: in a ~1.5 m course every frontier sat in inflation cost >= 70; the fallback judges like Nav2."""
    class Cost:
        resolution = 0.05
        def cell(self, x, y):
            return int(x / 0.05), int(y / 0.05)
        def contains(self, c):
            return True
        def value(self, c):
            return 80                              # inflated, not inscribed
    e = R.Explorer(R.RoamConfig(strategy='scan_walk'), (0.0, 0.0))
    assert R.COSTMAP_VETO == 99 and e.hazard_free(Cost(), 1.0, 0.0)   # inflation is passable (Nav2 rule)
    Cost.value = lambda self, c: 99                                    # inscribed is not
    assert not e.hazard_free(Cost(), 1.0, 0.0)


def test_dead_end_faces_the_wall_with_unseen_space_behind_it():
    """Operator 2026-11-21: stop in front of the curtain (a LiDAR 'wall' with the course unseen behind it)."""
    grid = _grid(free=[(-0.45, 0.55, -0.55, 0.55), (0.65, 2.0, -1.5, 1.5), (-0.45, 2.0, 0.65, 1.5),
                       (-0.45, 2.0, -1.5, -0.65)],
                 occupied=[(-0.55, -0.45, -0.6, 0.6), (0.55, 0.65, -0.6, 0.6), (-0.5, 0.6, 0.55, 0.65),
                           (-0.5, 0.6, -0.65, -0.55)])
    p = R.blocked_passage(grid, (0.0, 0.0, 0.0))
    assert p is not None and abs(abs(p['yaw']) - math.pi) < math.radians(15)
    assert 0.4 <= p['wall_m'] <= 0.6 and p['unknown'] >= 0.6
    open_room = _grid(free=[(-2.0, 2.0, -2.0, 2.0)])
    assert R.blocked_passage(open_room, (0.0, 0.0, 0.0)) is None


def test_dead_end_with_a_passage_stops_in_front_of_it(tmp_path, monkeypatch):
    """No opening, no frontier, but a wall with unseen space behind: face it and pause (no backtrack)."""
    monkeypatch.setattr(R.ScanWalk, 'choose', lambda *a, **kw: None)
    monkeypatch.setattr(R.ScanWalk, 'straight_ahead', lambda *a, **kw: None)
    monkeypatch.setattr(R.RoamSession, '_frontier_goal', lambda *a, **kw: None)
    monkeypatch.setattr(R, 'blocked_passage', lambda grid, pose, **kw: {
        'yaw': pose[2] + math.pi / 2, 'wall_m': 0.40, 'unknown': 0.9, 'width_deg': 25})
    outcome, summary, _, _ = roam(tmp_path, cfg=R.RoamConfig(strategy='scan_walk', passage_wait_s=0.0),
                                  stop_after=100)
    names = [e['event'] for e in summary['events']]
    assert 'blocked_passage' in names and 'backtrack' not in names
    paused = [e for e in summary['events'] if e['event'] == 'paused']
    assert paused and 'blocked passage' in paused[0]['reason']
    assert summary['stats']['passage_stops'] == 1


def test_patrol_fallback_goes_only_to_white_space_not_yet_walked():
    """Operator 2026-11-21: walk to the unvisited white area; none left -> no goal (curtain stop next)."""
    grid = _grid(free=[(-0.5, 2.5, -0.5, 0.5)])
    e = R.Explorer(R.RoamConfig(strategy='scan_walk'), (0.0, 0.0))
    e.clearance_m, e.veto, e.visited = R.FRONTIER_FALLBACK_CLEARANCE_M, R.FRONTIER_FALLBACK_VETO, [(0.0, 0.0)]
    goal = e._patrol(grid, (0.0, 0.0, 0.0))
    assert goal is not None and goal['x'] >= 1.0
    e.visited = [(x / 10, 0.0) for x in range(-5, 26, 3)]                # walked the whole strip
    assert e._patrol(grid, (2.0, 0.0, 0.0)) is None


def _advise(monkeypatch, after_s, **fields):
    """Feed the advisory topic in the simulator from after_s on (set where roam reads scene input)."""
    import json as _json
    original = R.RoamSession._take_scene
    data = {"obstacle": "stairs", "reason": "confirmed", "stamp_ns": 1, "distance_m": 0.7,
            "distance_reference": "base_link_forward", "heading_correction_deg": 0.0}
    data.update(fields)

    def take(self, now, pose):
        if self.t_roam is not None and now - self.t_roam >= after_s:
            self.robot.advisory = (self.robot.now(), _json.dumps(data))
        return original(self, now, pose)
    monkeypatch.setattr(R.RoamSession, '_take_scene', take)


def test_confirmed_stairs_nearby_start_the_stair_routine(tmp_path, monkeypatch):
    _advise(monkeypatch, 3.0, distance_m=0.7, heading_correction_deg=2.0)
    _, summary, robot, _ = roam(tmp_path, cfg=R.RoamConfig(strategy='scan_walk'), stop_after=40)
    names = [e['event'] for e in summary['events']]
    assert 'stairs_detected' in names and 'stairs_align' not in names
    start = next(e for e in summary['events'] if e['event'] == 'stairs_start')
    assert start['trigger'] == 'advisory' and start['direction'] == 'up'
    # flat simulator: no incline within 1 m, so the routine aborts and roam pauses out of AI mode
    assert 'stairs_aborted' in names and summary['rest_verified']


def test_stairs_at_an_angle_are_faced_first(tmp_path, monkeypatch):
    _advise(monkeypatch, 3.0, distance_m=0.7, heading_correction_deg=30.0)   # > 20 deg off the grid axis
    _, summary, _, _ = roam(tmp_path, cfg=R.RoamConfig(strategy='scan_walk'), stop_after=40)
    names = [e['event'] for e in summary['events']]
    assert names.index('stairs_align') < names.index('stairs_start')
    spin = [e for e in summary['events'] if e['event'] == 'spin'][-1]
    assert spin['degrees'] == 30


def test_far_stairs_are_walked_to_first(tmp_path, monkeypatch):
    _advise(monkeypatch, 3.0, distance_m=1.8, heading_correction_deg=0.0)
    _, summary, _, _ = roam(tmp_path, cfg=R.RoamConfig(strategy='scan_walk'), stop_after=25)
    walk = next(e for e in summary['events'] if e['event'] == 'walk' and e['ends_at'] == 'stairs')
    assert abs(walk['distance_m'] - (1.8 - R.ADVISORY_STAIRS_WALK_TO_M)) < 0.05


def test_near_curtain_is_crossed_straight_then_costmaps_cleared(tmp_path, monkeypatch):
    _advise(monkeypatch, 3.0, obstacle='curtain', distance_m=0.5, heading_correction_deg=None,
            distance_reference='rgb_camera_optical_depth')
    _, summary, robot, _ = roam(tmp_path, cfg=R.RoamConfig(strategy='scan_walk'), stop_after=40)
    ev = summary['events']
    start = next(e for e in ev if e['event'] == 'curtain_crossing')
    assert abs(start['curtain_m'] - 0.75) < 0.01 and abs(start['length_m'] - (0.75 + R.CURTAIN_PAST_M)) < 0.01
    done = next(e for e in ev if e['event'] == 'curtain_crossed')
    assert done['travelled_m'] >= 0.75 + R.CURTAIN_PAST_M
    assert any(e['event'] == 'costmap_cleared' and e['why'] == 'curtain crossed' for e in ev)
    assert summary['stats']['curtain_crossings'] == 1


def test_far_curtain_and_log_mode_do_not_take_over(tmp_path, monkeypatch):
    _advise(monkeypatch, 3.0, obstacle='curtain', distance_m=2.0, heading_correction_deg=None,
            distance_reference='rgb_camera_optical_depth')
    _, summary, _, _ = roam(tmp_path, cfg=R.RoamConfig(strategy='scan_walk'), stop_after=20)
    names = [e['event'] for e in summary['events']]
    assert 'advisory' in names and 'curtain_crossing' not in names


def test_log_mode_only_logs(tmp_path, monkeypatch):
    _advise(monkeypatch, 3.0, distance_m=0.7)
    _, summary, _, _ = roam(tmp_path, cfg=R.RoamConfig(strategy='scan_walk', advisory='log'), stop_after=20)
    names = [e['event'] for e in summary['events']]
    assert 'advisory' in names and 'stairs_start' not in names


def test_blocked_curtain_stops_and_pauses(tmp_path, monkeypatch):
    """A 'curtain' that does not give way (a wall): no progress -> stop and pause, never push on."""
    from lite3_movement_tests.sim_model import SimLite3
    _advise(monkeypatch, 3.0, obstacle='curtain', distance_m=0.5, heading_correction_deg=None,
            distance_reference='rgb_camera_optical_depth')
    original = R.RoamSession._crossing_tick

    def stuck(self, now):
        if self.crossing is not None:
            self.crossing['origin'] = (self.robot.telemetry.odom.latest()[1].x,
                                       self.robot.telemetry.odom.latest()[1].y)   # never gets anywhere
        return original(self, now)
    monkeypatch.setattr(R.RoamSession, '_crossing_tick', stuck)
    _, summary, _, _ = roam(tmp_path, cfg=R.RoamConfig(strategy='scan_walk'), stop_after=30)
    failed = next(e for e in summary['events'] if e['event'] == 'curtain_crossing_failed')
    assert 'no progress' in failed['reason']
    assert any(e['event'] == 'paused' and 'curtain crossing stopped' in e['reason'] for e in summary['events'])


def test_blocked_passage_that_is_a_curtain_is_crossed_not_paused_at(tmp_path, monkeypatch):
    import json as _json
    monkeypatch.setattr(R.ScanWalk, 'choose', lambda *a, **kw: None)
    monkeypatch.setattr(R.ScanWalk, 'straight_ahead', lambda *a, **kw: None)
    monkeypatch.setattr(R.RoamSession, '_frontier_goal', lambda *a, **kw: None)
    monkeypatch.setattr(R, 'blocked_passage', lambda grid, pose, **kw: {
        'yaw': pose[2] + math.pi / 2, 'wall_m': 0.40, 'unknown': 0.9, 'width_deg': 25})
    original = R.RoamSession._take_scene

    def take(self, now, pose):
        if self.passage is not None and 'wait_until' in self.passage:       # detector confirms while waiting
            self.robot.advisory = (self.robot.now(), _json.dumps({
                "obstacle": "curtain", "reason": "confirmed", "distance_m": 0.12,
                "distance_reference": "rgb_camera_optical_depth", "heading_correction_deg": None}))
        return original(self, now, pose)
    monkeypatch.setattr(R.RoamSession, '_take_scene', take)
    _, summary, _, _ = roam(tmp_path, cfg=R.RoamConfig(strategy='scan_walk'), stop_after=120)
    names = [e['event'] for e in summary['events']]
    assert names.index('passage_waiting') < names.index('curtain_crossing') < names.index('curtain_crossed')
    crossed = names.index('curtain_crossed')
    assert not any(e['event'] == 'paused' for e in summary['events'][:crossed])   # crossed, not stopped at



def test_advisory_stairs_go_up_over_the_landing_and_down_then_roam_on(tmp_path, monkeypatch):
    """Operator 2026-11-23: after the climb it comes back down the other side; it does not stay on top."""
    from lite3_movement_tests import ros_io as IO, stairs as ST
    _advise(monkeypatch, 3.0, distance_m=0.7)
    original_step = ST.StairRoutine.step

    def staircase(self, t, pitch, roll, travelled):
        p = 0.0
        if 0.1 <= travelled < 0.5:
            p = -0.17                                   # up
        elif 0.75 <= travelled < 1.15:
            p = 0.17                                    # down
        return original_step(self, t, p, roll, travelled)
    monkeypatch.setattr(ST.StairRoutine, 'step', staircase)
    _, summary, robot, _ = roam(tmp_path, cfg=R.RoamConfig(strategy='scan_walk'), stop_after=90)
    names = [e['event'] for e in summary['events']]
    assert next(e for e in summary['events'] if e['event'] == 'stairs_start')['over'] is True
    assert names.index('stairs_start') < names.index('stairs_landing') < names.index('stairs_descend') \
        < names.index('stairs_done')
    first = names[:names.index('stairs_done') + 1]          # (the fake keeps reporting stairs: later re-climbs)
    assert first.count('stairs_switch_in') == 1 and first.count('stairs_switch_out') == 1   # AI held over the top
    assert 'paused' not in first
    simple = [s[2] for s in robot.sent if s[1] == 'simple']
    assert simple[:4] == [IO.AI_ENTER, IO.AI_GAIT_BASIC, IO.AI_EXIT, IO.NAVIGATION_MODE]


def test_mission_events_are_logged_once_and_do_not_change_movement(tmp_path, monkeypatch):
    import json as _json
    original = R.RoamSession._take_scene
    sent = {'n': 0}

    def take(self, now, pose):
        if self.t_roam is not None and now - self.t_roam >= 3.0 and sent['n'] < 3:
            sent['n'] += 1                                   # same event three times: logged once
            self.robot.mission = [(self.robot.now(), _json.dumps({
                "event_id": "f9-object-0", "type": "observation",
                "data": {"label": "person", "confidence": 0.9,
                         "location": {"frame_id": "map", "x": 2.0, "y": 1.0, "z": 0.2}}}))]
        return original(self, now, pose)
    monkeypatch.setattr(R.RoamSession, '_take_scene', take)
    _, summary, _, _ = roam(tmp_path, cfg=R.RoamConfig(strategy='scan_walk'), stop_after=20)
    logged = [e for e in summary['events'] if e['event'] == 'mission']
    assert len(logged) == 1 and logged[0]['value'] == 'person' and logged[0]['at'] == [2.0, 1.0]
    assert summary['stats']['mission_events'] == 1
    assert any(s.get('kind') == 'mission_observation' for s in summary['sightings'])
    assert 'paused' not in [e['event'] for e in summary['events']]


def _lane_with_side_room():
    """Big course 2026-11-21 in miniature: a 1.4 m lane along x (y -0.7..0.7) from x -0.5 to 3.0, closed at
    both ends, with a 1.4 m opening on its left at x 1.0..2.4 into an unexplored room (y up to 2.5)."""
    return _grid(free=[(-0.5, 3.0, -0.7, 0.7), (1.0, 2.4, 0.7, 2.5)],
                 occupied=[(-0.6, -0.5, -0.8, 0.8), (3.0, 3.1, -0.8, 0.8), (-0.6, 3.1, -0.8, -0.7),
                           (-0.6, 1.0, 0.7, 0.8), (2.4, 3.1, 0.7, 0.8)])


def test_novelty_marks_walked_lanes():
    w = R.ScanWalk(R.RoamConfig(strategy='scan_walk'), (0.0, 0.0))
    assert w.novelty((0, 0, 0), 0.0, 2.0) == 1.0                         # no memory: everything new
    w.visited = [(x / 10, 0.0) for x in range(0, 31, 3)]
    assert w.novelty((0.0, 0.0, 0.0), 0.0, 2.5) == 0.0
    assert w.novelty((1.7, 0.0, 0.0), math.pi / 2, 2.0) > 0.5


def test_walked_lane_is_not_walked_again_while_a_new_opening_exists():
    """Operator 2026-11-23 coverage: at the far end of a lane already walked both ways, turn into the side
    room, not back down the lane (the big-course pacing)."""
    grid = _lane_with_side_room()
    w = R.ScanWalk(R.RoamConfig(strategy='scan_walk', geofence_m=6.0), (0.0, 0.0))
    w.visited = [(x / 10, 0.0) for x in range(0, 26, 3)]                 # lane walked to x 2.5 and back
    w.heading_ref = math.pi                                               # last walked back towards x 0
    pose = (1.7, 0.0, math.pi)
    assert w.straight_ahead(grid, pose, None, 0.0) is None and w.last_status == 'walked_lane'
    goal = w.choose(grid, pose, 0.0)
    assert goal is not None and goal['y'] > 0.5                           # into the side room
    assert abs(math.atan2(math.sin(goal['yaw'] - math.pi / 2), math.cos(goal['yaw'] - math.pi / 2))) < 0.6


def test_fully_walked_area_still_offers_a_walk_as_last_resort():
    grid = _grid(free=[(-0.5, 3.0, -0.7, 0.7)], occupied=[(3.0, 3.1, -1, 1), (-0.6, -0.5, -1, 1)])
    w = R.ScanWalk(R.RoamConfig(strategy='scan_walk', geofence_m=6.0), (0.0, 0.0))
    w.visited = [(x / 10, y / 10) for x in range(-5, 31, 3) for y in (-5, 0, 5)]
    assert w.choose(grid, (1.0, 0.0, 0.0), 0.0) is None and w.last_status == 'no_new_direction'
    assert w.choose(grid, (1.0, 0.0, 0.0), 0.0, allow_visited=True) is not None



def test_roam_shares_one_walked_path_for_coverage_and_patrol(tmp_path, monkeypatch):
    seen = []
    original = R.RoamSession._scan_walk

    def check(self, now, pose):
        seen.append(self.explorer.visited is self.frontiers.visited and len(self.frontiers.visited) >= 1)
        return original(self, now, pose)
    monkeypatch.setattr(R.RoamSession, '_scan_walk', check)
    roam(tmp_path, cfg=R.RoamConfig(strategy='scan_walk'), stop_after=15)
    assert seen and all(seen)


def test_goes_back_past_walked_lanes_to_find_unexplored_space():
    """Operator 2026-11-23: "it should be able to backtrack to find things if needed". At the dead end of a
    lane already walked, an unvisited room near the START is still a goal (Nav2 routes back down the lane)."""
    grid = _grid(free=[(-0.5, 3.0, -0.7, 0.7), (-0.5, 0.9, 0.7, 2.5)],          # room off the lane's start
                 occupied=[(3.0, 3.1, -0.8, 0.8), (-0.6, -0.5, -0.8, 2.6), (0.9, 3.1, 0.7, 0.8),
                           (-0.6, 3.1, -0.8, -0.7)])
    cfg = R.RoamConfig(strategy='scan_walk', geofence_m=6.0)
    walked = [(x / 10, 0.0) for x in range(0, 26, 3)]                    # walked the lane out to x 2.5
    w = R.ScanWalk(cfg, (0.0, 0.0))
    w.visited, w.heading_ref = walked, 0.0
    pose = (2.5, 0.0, 0.0)
    assert w.straight_ahead(grid, pose, None, 0.0) is None                # dead end ahead
    e = R.Explorer(cfg, (0.0, 0.0))
    e.clearance_m, e.veto, e.visited = R.FRONTIER_FALLBACK_CLEARANCE_M, R.FRONTIER_FALLBACK_VETO, walked
    goal = e.choose(grid, pose, 0.0)
    assert goal is not None and goal['x'] < 1.0 and goal['y'] > 0.7     # back near the start, in the room


def test_patrol_skips_floor_the_camera_has_seen():
    from lite3_movement_tests.coverage import CameraCoverage
    g = _grid(free=[(-0.5, 2.5, -0.5, 0.5)])
    e = R.Explorer(R.RoamConfig(strategy='scan_walk'), (0.0, 0.0))
    e.clearance_m, e.veto = R.FRONTIER_FALLBACK_CLEARANCE_M, R.FRONTIER_FALLBACK_VETO
    e.seen = CameraCoverage((0.0, 0.0), 4.0)
    assert e._patrol(g, (0.0, 0.0, 0.0)) is not None
    e.seen.seen[:] = True                                    # camera has looked everywhere
    assert e._patrol(g, (0.0, 0.0, 0.0)) is None


def test_look_around_after_walks_only_where_unseen_and_never_twice_in_one_spot(tmp_path):
    _, summary, _, _ = roam(tmp_path, cfg=R.RoamConfig(strategy='scan_walk', stop_at_exit=False), stop_after=120)
    ev = summary['events']
    assert [e['event'] for e in ev].count('exit_reached') <= 2      # once per arrival, not every check
    looks = [e for e in ev if e['event'] == 'look_around']
    assert looks, 'the simulator room has unseen sides after the first walk'
    for e in looks:
        assert e['turns_deg'] in ([45.0, 45.0], [-45.0, -45.0], [45.0, 45.0, -45.0, -45.0, -45.0, -45.0])
        assert (e['left_unseen_m2'] >= R.LOOK_MIN_SIDE_M2) == (45.0 in e['turns_deg'])
    names = [e['event'] for e in ev]
    first = names.index('look_around')
    assert 'walk_end' in names[:first] or 'walk_failed' in names[:first]   # only after a walk
    assert summary['stats']['camera_seen_m2'] > 0
    assert summary['stats']['look_arounds'] == len(looks)


def test_look_around_turns_are_not_cut_short_by_an_opening(tmp_path, monkeypatch):
    monkeypatch.setattr(R.RoamSession, '_walk_candidate',
                        lambda self, now, pose, allow_visited=False: {'kind': 'walk', 'x': pose[0] + 1, 'y': pose[1],
                                                                       'yaw': pose[2], 'score': 1.0, 'free_m': 1.0,
                                                                       'ends_at': 'obstacle'})
    _, summary, _, _ = roam(tmp_path, cfg=R.RoamConfig(strategy='scan_walk', stop_at_exit=False), stop_after=90)
    names = [e['event'] for e in summary['events']]
    if 'look_around' in names:
        after = names[names.index('look_around') + 1:]
        assert after[0] == 'spin' and after[1] in ('spin_done', 'spin_incomplete')   # not 'spin_opening'



def test_look_steps_pause_briefly_and_the_last_step_waits_for_the_map(tmp_path):
    _, summary, _, _ = roam(tmp_path, cfg=R.RoamConfig(strategy='scan_walk', stop_at_exit=False), stop_after=120)
    ev = summary['events']
    i = next(k for k, e in enumerate(ev) if e['event'] == 'look_around')
    steps = len(ev[i]['turns_deg'])
    spins = [e for e in ev[i + 1:] if e['event'] == 'spin'][:steps]
    assert [s['degrees'] for s in spins] == ev[i]['turns_deg']           # every 45 deg step taken
    gaps = [b['t_s'] - a['t_s'] for a, b in zip(spins, spins[1:])]
    assert all(g >= R.LOOK_DWELL_S for g in gaps)                         # stood still facing each step


def test_fast_turns_do_not_count_as_seen(tmp_path, monkeypatch):
    """Coverage only while turning slower than COVER_MAX_YAW_RATE."""
    marked = []
    from lite3_movement_tests import coverage as C
    real = C.CameraCoverage.update

    def spy(self, pose, grid, *a, **k):
        marked.append(pose)
        return real(self, pose, grid, *a, **k)
    monkeypatch.setattr(C.CameraCoverage, 'update', spy)
    monkeypatch.setattr(R, 'COVER_MAX_YAW_RATE', -1.0)                    # nothing is slow enough
    _, summary, _, _ = roam(tmp_path, cfg=R.RoamConfig(strategy='scan_walk', stop_at_exit=False), stop_after=20)
    assert marked == [] and summary['stats'].get('camera_seen_m2', 0.0) == 0.0


def test_hazard_staleness_matches_terrain_limit_in_docs_mode():
    """2026-11-23 14:33 / 14:43: 0.64-0.66 s link dropouts tripped the 0.5 s hazard limit; docs mode allows 1.0 s
    like the terrain status. Full stop layers keep 0.5 s."""
    class Stub:
        pass
    s = Stub()
    s.cfg = R.RoamConfig(strategy='scan_walk', docs_stops=True)
    assert R.RoamSession._hazard_max_age(s) == R.HAZARD_STALE_DOCS_S == R.ROAM_RAW_LIDAR_STALE_S == 1.0
    s.cfg = R.RoamConfig(strategy='scan_walk')
    assert R.RoamSession._hazard_max_age(s) == R.HAZARD_STALE_S == 0.5


def test_a_hazard_cloud_that_stops_for_good_still_stops_roaming_in_docs_mode(tmp_path):
    outcome, summary, _, _ = roam(tmp_path, stop_after=120, nav_fault='hazard_stale',
                                  cfg=R.RoamConfig(docs_stops=True))
    assert outcome == 'SAFETY_STOP' and 'hazard cloud stale' in summary['stop_reason'] or \
        'stale' in summary['stop_reason'], summary['stop_reason']
    assert summary['rest_verified'] is True


def _terrain_step(monkeypatch, rise, ahead=0.5, seconds=12.0, obstacle_after=None):
    """Telemetry sees a step (ahead, rise) for `seconds`; optionally an obstacle at the nose after `obstacle_after`."""
    from lite3_movement_tests.telemetry import Telemetry
    original = Telemetry.on_terrain
    w = {'t0': None}

    def on_terrain(self, stamp, stop, reason, obstacle, stop_ahead, problems, processing_s=None,
                   step_ahead_m=None, step_rise_m=None):
        w['t0'] = stamp if w['t0'] is None else w['t0']
        if stamp < w['t0'] + seconds:
            step_ahead_m, step_rise_m = ahead, rise
        if obstacle_after is not None and stamp > w['t0'] + obstacle_after:
            obstacle = 0.1
        return original(self, stamp, stop, reason, obstacle, stop_ahead, problems, processing_s, step_ahead_m,
                        step_rise_m)
    monkeypatch.setattr(Telemetry, 'on_terrain', on_terrain)


def test_terrain_step_is_crossed_in_the_high_step_gait_never_ai(tmp_path, monkeypatch):
    """Operator 2026-11-23: a 20 cm block must be crossed; AI mode only for stairs; nothing course-specific."""
    from lite3_movement_tests import ros_io as IO, stairs as ST
    _terrain_step(monkeypatch, 0.20)
    original_step = ST.StairRoutine.step
    monkeypatch.setattr(ST.StairRoutine, 'step', lambda self, t, p, r, d: original_step(
        self, t, 0.2 if 0.05 <= d < 0.3 else 0.0, r, d))                  # body tilts while stepping up
    _, summary, robot, _ = roam(tmp_path, stop_after=60, cfg=R.RoamConfig(strategy='scan_walk'))
    ev = summary['events']
    start = next(e for e in ev if e['event'] == 'stairs_start')
    assert start['mode'] == 'hstep' and start['trigger'] == 'terrain step'
    assert any(e['event'] == 'step_detected' and abs(e['rise_m'] - 0.2) < 1e-6 for e in ev)
    simple = [s[2] for s in robot.sent if s[1] == 'simple']
    assert IO.AI_ENTER not in simple and IO.AI_GAIT_BASIC not in simple    # never AI mode
    assert 'stairs_done' in [e['event'] for e in ev]


def test_step_crossing_ends_when_a_wall_is_ahead(tmp_path, monkeypatch):
    from lite3_movement_tests import stairs as ST
    _terrain_step(monkeypatch, 0.20, obstacle_after=2.0)
    original_step = ST.StairRoutine.step
    monkeypatch.setattr(ST.StairRoutine, 'step', lambda self, t, p, r, d: original_step(
        self, t, 0.2 if 0.05 <= d < 0.2 else 0.0, r, d))
    _, summary, _, _ = roam(tmp_path, stop_after=60, cfg=R.RoamConfig(strategy='scan_walk'))
    names = [e['event'] for e in summary['events']]
    assert 'step_crossed_wall_ahead' in names
    assert names.index('stairs_start') < names.index('step_crossed_wall_ahead') < names.index('stairs_done')


def test_steps_outside_the_configured_range_are_not_crossed_this_way(tmp_path, monkeypatch):
    _terrain_step(monkeypatch, 0.40)                                     # wall-like: not a step to climb
    _, summary, _, _ = roam(tmp_path, stop_after=25, cfg=R.RoamConfig(strategy='scan_walk'))
    assert 'step_detected' not in [e['event'] for e in summary['events']]
    _terrain_step(monkeypatch, 0.20)
    _, off, _, _ = roam(tmp_path / 'off', stop_after=25,
                        cfg=R.RoamConfig(strategy='scan_walk', step_crossing=False)) \
        if (tmp_path / 'off').mkdir() is None else (None, None, None, None)
    assert 'step_detected' not in [e['event'] for e in off['events']]


def test_step_crossing_options_come_from_the_command_line():
    a = R.parse(['run', '--speed-limit', '0.5', '--step-min-rise', '0.15', '--step-max-rise', '0.22',
                 '--step-trigger', '0.7', '--step-speed', '0.25', '--step-crossing', 'off'])
    cfg = R.build_config(a, {'MAX_SPEED': '0.5'})
    assert (cfg.step_crossing, cfg.step_min_rise_m, cfg.step_max_rise_m, cfg.step_trigger_m, cfg.step_speed_m_s) == \
        (False, 0.15, 0.22, 0.7, 0.25)
    d = R.build_config(R.parse(['run', '--speed-limit', '0.5']), {'MAX_SPEED': '0.5'})
    assert (d.step_crossing, d.step_min_rise_m, d.step_max_rise_m) == (True, 0.12, 0.30)   # 25 cm blocks on a pallet
    with pytest.raises(ConfigError):
        R.RoamConfig(step_min_rise_m=0.3, step_max_rise_m=0.2)


def test_a_passage_that_opens_while_waiting_is_explored_instead_of_paused_at(tmp_path, monkeypatch):
    """Operator 2026-11-23: think longer before giving up. While waiting at a blocked passage, a map update that
    no longer shows it blocked sends roam back to exploring instead of pausing."""
    monkeypatch.setattr(R.ScanWalk, 'choose', lambda *a, **kw: None)
    monkeypatch.setattr(R.ScanWalk, 'straight_ahead', lambda *a, **kw: None)
    monkeypatch.setattr(R.RoamSession, '_frontier_goal', lambda *a, **kw: None)
    state = {'calls': 0}

    def passage(grid, pose, **kw):
        state['calls'] += 1
        return {'yaw': 0.0, 'wall_m': 0.40, 'unknown': 0.9, 'width_deg': 25} if state['calls'] == 1 else None
    monkeypatch.setattr(R, 'blocked_passage', passage)
    _, summary, _, _ = roam(tmp_path, cfg=R.RoamConfig(strategy='scan_walk', passage_wait_s=30.0), stop_after=60)
    names = [e['event'] for e in summary['events']]
    assert 'passage_waiting' in names and 'passage_cleared' in names
    assert not any('blocked passage' in (e.get('reason') or '') for e in summary['events'] if e['event'] == 'paused')


def test_give_up_timings_are_longer_by_default_and_set_per_run():
    cfg = R.RoamConfig()
    assert cfg.stuck_s == 40.0 and cfg.spin_stall_s == 25.0 and cfg.passage_wait_s == 10.0
    assert cfg.goal_time_s(2.0, 0.5) == 2.0 / 0.5 * 5 + 60              # was x3 + 30
    assert cfg.goal_time_s(100.0, 0.1) == R.HARD_MAX_GOAL_S
    d = R.build_config(R.parse(['run', '--speed-limit', '0.5']), {'MAX_SPEED': '0.5'})
    assert (d.stuck_s, d.goal_time_factor, d.goal_time_base_s, d.spin_stall_s, d.passage_wait_s) == (40, 5, 60, 25, 10)
    a = R.parse(['run', '--speed-limit', '0.5', '--stuck', '30', '--goal-time-factor', '4', '--goal-time-base',
                 '45', '--spin-stall', '20', '--passage-wait', '6'])
    c = R.build_config(a, {'MAX_SPEED': '0.5'})
    assert (c.stuck_s, c.goal_time_factor, c.goal_time_base_s, c.spin_stall_s, c.passage_wait_s) == (30, 4, 45, 20, 6)
    with pytest.raises(R.ConfigError):
        R.RoamConfig(stuck_s=500)
