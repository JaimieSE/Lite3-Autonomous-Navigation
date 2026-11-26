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
    assert names[:3] == ['scan_walk', 'roaming', 'walk']
    assert summary['stats']['goals_succeeded'] >= 1
    assert outcome == 'STOPPED' and summary['rest_verified']
    assert max(c.vx for _, c in robot.telemetry.system_cmd.since(0)) <= 0.0501


@pytest.mark.parametrize('side', [-1, 1])
def test_walk_goals_can_turn_either_way(side):
    grid = _grid(free=[(-0.25, 0.25, min(0, side * 2), max(0, side * 2))])
    goal = _walker().choose(grid, (0, 0, 0), 0)
    assert goal['y'] * side > 0.5
    assert goal['yaw'] * side > 0


def test_no_opening_scans_both_sides_then_pauses(tmp_path, monkeypatch):
    monkeypatch.setattr(R.ScanWalk, 'choose', lambda *a, **kw: None)
    monkeypatch.setattr(R.ScanWalk, 'straight_ahead', lambda *a, **kw: None)
    outcome, summary, _, _ = roam(tmp_path, cfg=R.RoamConfig(strategy='scan_walk'), stop_after=100)
    turns = [e['degrees'] for e in summary['events'] if e['event'] == 'spin']
    assert turns == [30, 30, -30, -30, -30, -30]
    assert summary['stats']['walk_goals'] == 0
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
    monkeypatch.setattr(R.ScanWalk, 'choose', lambda *a, **kw: None)
    monkeypatch.setattr(R.ScanWalk, 'straight_ahead', lambda *a, **kw: None)
    monkeypatch.setattr(FakeNavRobot, '_spin_tick', lambda self: None)
    _, summary, _, _ = roam(tmp_path, stop_after=25, cfg=R.RoamConfig(strategy='scan_walk'))
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
    assert cfg.start_mode == 'normal' and cfg.mode_speed(cfg.start_mode)[0] == 0.5
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
    assert R.HARD_MIN_LIDAR_HZ == 8.0 and R.WatchdogLogic.TELEMETRY_AGE_S == 1.0
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
