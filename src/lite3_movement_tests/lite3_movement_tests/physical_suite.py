"""Supervised physical navigation tests (M0-M9) on the existing Nav2 / twist_mux / bridge path.

DEFAULT IS DRY: prints each test's plan and PASS criteria, then checks the live robot and
stack READ-ONLY (no publishers) and lists every blocker. ``--sim`` rehearses the tests
against a software stand-in. Only ``--physical`` moves the robot, and only when:
  * navigation was started with operator/start_navigation.sh --physical (separately armed),
  * the test's prerequisites passed on this navigation run (M0 -> M1 -> M2 before any
    obstacle test), and
  * the operator types the test ID at the confirmation prompt for EACH test.

Terminal parameters can only narrow the hard limits in config.py (speed <= 0.5 m/s, validated to 0.10,
stop distance >= 0.20 m from the nose, ...). Values outside them are refused, not clamped.
"""

import argparse
import datetime as dt
import json
import math
import os
from pathlib import Path
import signal
import sys
import threading
import time

import yaml

from .config import (ConfigError, HARD_MAX_APPROACH_S, HARD_MAX_APPROACH_SPEED_M_S,
                     HARD_MAX_APPROACH_TRAVEL_M, HARD_MAX_DETECTION_RANGE_M, HARD_MAX_MOVE_S,
                     HARD_MAX_NAV_STEP_S, HARD_MAX_RESPONSE_S, HARD_MAX_SENSOR_AGE_S,
                     HARD_MAX_SPEED_M_S, HARD_MIN_LIDAR_HZ, HARD_MIN_STOP_DISTANCE_M,
                     NAV2_MAX_YAW_RATE_RAD_S, NOSE_OFFSET_M, authorization_template,
                     check_authorization, load_authorization, load_sequence)
from .report import write_reports
from .runner import MODE_PHYSICAL, MODE_READONLY, MODE_SIM, SequenceRunner

VALIDATED_SPEED_M_S = 0.10            # coast, latency and the 0.30 m backup lock were validated up to here
DEFAULT_STOP_AHEAD_M = 0.505          # terrain.StopParams default (centre), restored afterwards
EXIT = {'PASSED': 0, 'READONLY_CHECKS_PASSED': 0, 'FAILED_SAFE': 1, 'INTERVENTION_REQUIRED': 2}

CONFIRM_ITEMS = (
    ('emergency_stop_in_hand_and_tested',
     'The handheld remote / emergency stop is in YOUR hand, was tested today, and you can reach '
     'the robot.'),
    ('navigation_mode_selected',
     'The robot is in Navigation (auto) mode on flat ground; no one else is commanding it.'),
    ('exclusive_robot_control', 'No other group or program is using this robot now.'),
    ('area_clear_radius_1m', 'People stay outside the test area; at least 1 m clear all round.'),
    ('test_area_prepared_as_instructed', 'The scene is set up exactly as in "Setup" above.'),
    ('operator_supervising_entire_session',
     'You will watch the robot for the whole test and stop it if anything looks wrong.'),
    ('accept_unverified_firmware_command_timeout',
     'You accept the robot may coast ~1.6 s after commands stop (measured, not a firmware '
     'guarantee).'),
)


# --------------------------------------------------------------------------- sequence parts
BOUNDARY_CHECK = {}      # filled in main() from the running navigation (state.env + calibration)


def head(stop_distance, posture='standing'):
    steps = [
        {'id': 'readiness', 'type': 'readiness', 'timeout_s': 10, 'quiet_s': 2.0},
        dict({'id': 'nav_readiness', 'type': 'nav_readiness', 'timeout_s': 15,
              'stop_distance_m': stop_distance}, **BOUNDARY_CHECK),
        {'id': 'telemetry_contract', 'type': 'telemetry_validation', 'timeout_s': 5},
        {'id': 'initial_rest', 'type': 'stationary_check', 'timeout_s': 6, 'hold_s': 2.0},
    ]
    if posture == 'standing':
        steps += [{'id': 'stand', 'type': 'posture_transition', 'target': 'standing',
                   'timeout_s': 12, 'hold_s': 1.5},
                  {'id': 'verify_standing', 'type': 'posture_check', 'expect': 'standing',
                   'timeout_s': 8, 'hold_s': 2.0}]
    return steps


def stop(step_id):
    return {'id': step_id, 'type': 'stop_verify', 'timeout_s': 8, 'hold_s': 1.5, 'zero_min_s': 1.0}


def nav_timeout(args, distance):
    if args.timeout:
        return args.timeout
    return min(HARD_MAX_NAV_STEP_S, round(distance / args.max_speed * 2.5 + 30))


def nav_goal(args, step_id, forward, left=0.0, yaw=0.0, expect='reached', min_lateral=0.0,
             timeout=None, geofence=None, max_cm_stops=-1):
    distance = math.hypot(forward, left)
    goal = {'id': step_id, 'type': 'nav_goal', 'forward_m': forward, 'left_m': left, 'yaw_deg': yaw,
            'max_speed': args.max_speed, 'xy_tolerance_m': args.xy_tolerance,
            'yaw_tolerance_deg': args.yaw_tolerance_deg, 'expect': expect,
            'min_clearance_m': args.clearance, 'min_lateral_m': min_lateral,
            'timeout_s': timeout or nav_timeout(args, distance), 'max_cm_stops': max_cm_stops}
    if geofence:
        goal['geofence_m'] = geofence
    return goal


def sequence(name, description, steps, finalize=(), geofence=0.5):
    return {'name': name, 'description': description, 'interface': 'nav',
            'safety': {'geofence_radius_m': geofence, 'max_tilt_rad': 0.35, 'command_period_s': 0.05,
                       'zero_seconds': 2.0, 'stop_timeout_s': 6.0, 'stop_hold_s': 1.5},
            'steps': list(steps), 'finalize': list(finalize)}


def parse_waypoints(text):
    points = []
    for part in (text or '').split(';'):
        if not part.strip():
            continue
        values = [float(v) for v in part.split(',')]
        if len(values) not in (2, 3):
            raise ConfigError(f'waypoint "{part}" must be forward,left[,yaw_deg]')
        points.append(tuple(values) + ((0.0,) if len(values) == 2 else ()))
    return points


# --------------------------------------------------------------------------- the catalogue
class Test:
    def __init__(self, code, title, moves, prereqs, setup, criteria, build, scene=None):
        self.code, self.title, self.moves, self.prereqs = code, title, moves, prereqs
        self.setup, self.criteria, self.build, self.scene = setup, criteria, build, scene


def nose_box(distance_from_nose, size=0.3):
    """Sim scene: box whose front face is ``distance_from_nose`` ahead of the robot's nose."""
    from .sim_model import box_points
    return box_points(NOSE_OFFSET_M + distance_from_nose + size / 2, 0.0, size)


def _m0(a):
    return sequence('M0_command_path', 'Command path and stop delivery (zeros only)',
                    head(a.stop_distance, posture=None) +
                    [{'id': 'zero_delivery', 'type': 'zero_delivery', 'duration_s': 1.0}])


def _m1(a):
    v, w, d = a.max_speed, a.max_yaw_rate, a.move_seconds
    fwd = [round(max(0.02, 0.3 * v * d), 3), round(3 * v * d + 0.05, 3)]
    yaw = [round(0.3 * w * d, 3), round(2.5 * w * d + 0.1, 3)]
    moves = [
        {'id': 'forward', 'type': 'velocity_move', 'vx': v, 'wz': 0.0, 'duration_s': d,
         'expect': {'forward_m': fwd, 'lateral_m': [-0.10, 0.10], 'yaw_change_rad': [-0.25, 0.25]}},
        stop('stop_forward'),
        {'id': 'turn_left', 'type': 'velocity_move', 'vx': 0.0, 'wz': w, 'duration_s': d,
         'expect': {'yaw_change_rad': yaw, 'forward_m': [-0.10, 0.10], 'lateral_m': [-0.10, 0.10]}},
        stop('stop_turn_left'),
        {'id': 'turn_right', 'type': 'velocity_move', 'vx': 0.0, 'wz': -w, 'duration_s': d,
         'expect': {'yaw_change_rad': [-yaw[1], -yaw[0]], 'forward_m': [-0.10, 0.10],
                    'lateral_m': [-0.10, 0.10]}},
        stop('stop_turn_right'),
    ]
    return sequence('M1_basic_motion', 'Low-speed forward and turns in a clear area via the mux',
                    head(a.stop_distance) + moves)


def _m2(a):
    return sequence('M2_stop_paths', 'Dead-man release must cut live commands and stop the robot',
                    head(a.stop_distance) + [
                        {'id': 'deadman_stop', 'type': 'deadman_stop', 'vx': a.max_speed,
                         'duration_s': min(HARD_MAX_MOVE_S, a.move_seconds + 1.0),
                         'release_after_s': 1.0},
                        stop('stop_after_deadman')])


def detection(expect, distance, window=3.0):
    return {'id': f'detect_{expect}', 'type': 'detection_check', 'expect': expect,
            'distance_m': [round(distance[0], 3), round(distance[1], 3)], 'window_s': window}


def _m3(a):
    return sequence('M3_proximity_block', 'Object inside the stop zone blocks a forward command',
                    head(HARD_MIN_STOP_DISTANCE_M) + [
                        detection('stop_zone', (0.10, HARD_MIN_STOP_DISTANCE_M)),
                        {'id': 'blocked_move', 'type': 'blocked_move',
                         'vx': min(a.max_speed, HARD_MAX_APPROACH_SPEED_M_S),
                         'duration_s': a.move_seconds},
                        stop('stop_after_block')])


def m4_params(a):
    """approach_stop / proximity_watch parameters from the resolved options (see resolve_m4)."""
    coast = a.measured_coast if a.measured_coast is not None else -1.0
    return {'vx': a.approach_speed, 'duration_s': a.max_duration, 'max_travel_m': a.max_travel,
            'stop_distance_m': a.cm_boundary, 'backup_stop_distance_m': HARD_MIN_STOP_DISTANCE_M,
            'min_clearance_m': a.clearance_m4, 'initial_clear_m': a.initial_clear,
            'detection_range_m': a.detection_range, 'max_sensor_age_s': a.max_sensor_age,
            'min_lidar_hz': a.min_lidar_hz, 'detect_scans': int(a.detect_scans),
            'track_tolerance_m': a.track_tolerance, 'lost_track_scans': int(a.lost_track_scans),
            'suppression_proof_s': a.suppression_proof, 'response_limit_s': a.response_limit,
            'coast_m': round(coast, 4), 'coast_speed_m_s': a.coast_speed,
            'path_latency_s': a.path_latency}


def _m4(a):
    params = m4_params(a)
    geofence = min(2.0, round(a.max_travel + 0.2, 2))
    if a.shadow:
        watch = {'id': 'shadow_watch', 'type': 'proximity_watch', 'window_s': a.shadow_seconds, **params}
        return sequence('M4_shadow', 'Dry shadow of the M4 stop logic: robot still, object moved toward it',
                        head(HARD_MIN_STOP_DISTANCE_M, posture=None) + [watch], geofence=geofence)
    posture = None if a.mode == MODE_READONLY else 'standing'
    return sequence('M4_sensor_stop', 'Walk forward while the path is clear; the collision_monitor must '
                    'stop the robot for a LiDAR-detected obstacle',
                    head(HARD_MIN_STOP_DISTANCE_M, posture) + [
                        {'id': 'approach_stop', 'type': 'approach_stop', **params},
                        stop('stop_after_walk')], geofence=geofence)


def _det(a):
    if a.object_distance is None:
        window = (0.10, 3.0)
    else:
        window = (max(0.0, a.object_distance - 0.05), a.object_distance + 0.05)
    return sequence('DET_object_detection', 'Static check: is the placed object detected steadily?',
                    head(a.stop_distance, posture=None)[:3] + [detection('ahead', window, window=5.0)])


def _m5(a):
    d = a.goal_distance or 1.0
    return sequence('M5_straight', f'Nav2 straight path following {d} m',
                    head(a.stop_distance) + [nav_goal(a, 'nav_straight', d), stop('stop_at_goal')])


def _m6(a):
    f = a.goal_distance or 1.0
    left = 0.8 if a.goal_left is None else a.goal_left
    yaw = 90.0 if a.goal_yaw_deg is None else a.goal_yaw_deg
    return sequence('M6_turn_junction', f'Nav2 goal around a corner ({f} fwd, {left} left, {yaw} deg)',
                    head(a.stop_distance) + [nav_goal(a, 'nav_junction', f, left, yaw),
                                             stop('stop_at_goal')])


def _m7(a):
    d = a.goal_distance or 2.0
    return sequence('M7_avoidance', f'Nav2 goal {d} m ahead with a box on the straight line',
                    head(a.stop_distance) + [nav_goal(a, 'nav_avoid', d, min_lateral=0.15, max_cm_stops=0),
                                             stop('stop_at_goal')])


def _m8(a):
    d = a.goal_distance or 2.0
    return sequence('M8_blocked_recovery', 'Unreachable goal: Nav2 recoveries, no collision, safe stop',
                    head(a.stop_distance) + [nav_goal(a, 'nav_blocked', d, expect='not_reached',
                                                      timeout=a.timeout or 90),
                                             stop('stop_after_blocked')])


def _m9(a):
    points = parse_waypoints(a.waypoints) or [(1.0, 0.0, 0.0), (2.0, 0.0, 0.0)]
    radius = a.course_radius or min(10.0, max(math.hypot(p[0], p[1]) for p in points) + 1.0)
    goals = [nav_goal(a, f'waypoint_{i + 1}', f, left, yaw, geofence=radius,
                      timeout=nav_timeout(a, math.hypot(f - (points[i - 1][0] if i else 0.0),
                                                        left - (points[i - 1][1] if i else 0.0))))
             for i, (f, left, yaw) in enumerate(points)]
    return sequence('M9_course', f'Obstacle-course traversal through {len(points)} waypoints',
                    head(a.stop_distance) + goals + [stop('stop_at_finish')])


def _walk(a):
    d = a.goal_distance or 2.0
    return sequence('WALK', f'Walk {d} m ahead with Nav2, going around obstacles',
                    head(HARD_MIN_STOP_DISTANCE_M) +
                    [nav_goal(a, 'walk', d, a.goal_left or 0.0, a.goal_yaw_deg or 0.0),
                     stop('stop_at_goal')])


def _stand(a):
    return sequence('STAND', 'Stand up (posture toggle, verified)', head(a.stop_distance))


def _sit(a):
    return sequence('SIT', 'Lie down (posture toggle, verified)', head(a.stop_distance, None) + [
        {'id': 'lie_down', 'type': 'posture_transition', 'target': 'lying', 'timeout_s': 12,
         'hold_s': 1.5}])


def _sim_wall(x=1.6, half=1.0):
    from .sim_model import box_points
    pts = []
    for i in range(int(2 * half / 0.3) + 1):
        pts += box_points(x, -half + i * 0.3, 0.3)
    return pts


CATALOG = {t.code: t for t in [
    Test('M0', 'Command path and stop delivery', False, [],
         'Robot at rest (lying or standing). Nobody within 1 m.',
         'Nav2/terrain/LiDAR ready; twist_mux is the only /cmd_vel source; zeros on the stop input '
         'reach motion_sender within 0.5 s; nothing reaches it while idle (dead-man locked).',
         _m0),
    Test('M1', 'Basic low-speed motion', True, ['M0'],
         'Clear, flat area: 1 m all round. Robot will stand (if lying), walk forward ~0.1 m, '
         'turn left and right in place.',
         'Each move reaches motion_sender through the mux; leg odometry displacement/yaw in range; '
         'rest verified after every move (also proves Navigation mode: no response = FAIL).',
         _m1),
    Test('M2', 'Stop paths (dead-man)', True, ['M1'],
         'Clear area 1 m all round. Robot walks forward; the test then releases the dead-man while '
         'still commanding forward.',
         'Commands at motion_sender stop within 0.8 s of the release; the robot STOPS MOVING (no '
         'translation, no foot lifts) within 3 s and is FULLY SETTLED within 5 s, turning <= 0.05 '
         'rad and shifting <= 0.02 m while settling; no operator action.',
         _m2),
    Test('DET', 'Object detection stability (no motion)', False, [],
         'Place the object straight ahead{object_at}. Nobody within 1 m of the robot or the object.',
         'Object detected ahead in >= 95% of LiDAR scans over 5 s, no status gap > 0.3 s, distance '
         'spread <= 0.05 m (and within 0.05 m of --object-distance if given).',
         _det, lambda a: nose_box(a.object_distance or 0.6)),
    Test('M3', 'Proximity lock blocks motion', True, ['M0'],
         'Place a box (>= 20 cm tall) straight ahead, front face {block_at:.2f} m from the robot\'s '
         'nose (inside the {stop:.2f} m stop zone). The robot stays still.',
         'First 3 s without commands: the object is seen in >= 95% of scans and holds the stop for '
         'the object AHEAD (the lock input never false). Then: before EVERY command the lock is '
         're-confirmed (otherwise the test stops commanding and aborts); no forward command reaches '
         'motion_sender; robot moves < 3 cm.',
         _m3, lambda a: nose_box(max(0.15, a.stop_distance - 0.10))),
    Test('M4', 'Sensor-driven proximity stop (walk until an obstacle stops the robot)', True,
         ['M2', 'M3'],
         'Straight, flat lane ahead: {travel:.1f} m + 0.5 m long, 0.6 m wide, nobody in it. Nothing '
         'within {initial:.2f} m of the nose at the start. Put the obstacle (chair/box >= 20 cm tall) '
         'ANYWHERE in the lane up to {range:.1f} m ahead (no exact distance needed). The robot walks '
         'straight ahead at {approach_speed} m/s until something stops it.',
         'READY -> WALKING -> OBSTACLE_DETECTED -> STOPPING -> STOP_VERIFIED. PASS only if the robot '
         'moved; the obstacle was detected ({scans} consistent scans) and tracked; the collision_monitor '
         '(trigger {boundary:.2f} m from the nose, re-checked against live latency + M2 coast) zeroed '
         'forward commands BECAUSE of it while the test still published them; unsafe commands were '
         'suppressed within {resp:.2f} s; the robot stopped moving within 3 s; minimum and final '
         'clearance >= {final_clearance:.2f} m. Stops by the timer ({dur:.0f} s), the travel limit '
         '({travel:.1f} m), the backup lock, the operator or a safeguard are separate outcomes, never '
         'PASS.',
         _m4, lambda a: nose_box(a.object_distance or 1.0)),
    Test('M5', 'Nav2 straight path following', True, ['M2'],
         'Clear straight lane: {goal:.1f} m ahead + 1 m beyond, 0.6 m either side.',
         'Nav2 SUCCEEDED; final map pose within {xy:.2f} m / {yawtol:.0f} deg; leg odometry agrees; '
         'footprint clearance >= {clearance:.2f} m; robot-side speed <= {speed} m/s; no reverse.',
         _m5),
    Test('M6', 'Nav2 turn / junction', True, ['M5'],
         'Clear L-shaped area covering the goal {goal:.1f} m ahead and {left:.1f} m left (+1 m '
         'margin). Walls may form the junction.',
         'Same as M5 for a goal around a corner with a heading change.',
         _m6),
    Test('M7', 'Obstacle avoidance', True, ['M5'],
         'Lane {goal:.1f} m ahead, >= 1.2 m wide each side. Box (>= 20 cm tall, ~30 cm wide) '
         'on the straight line about halfway.',
         'Goal reached as in M5 AND the path deviates >= 0.15 m from the straight line; clearance '
         '>= {clearance:.2f} m throughout.',
         _m7, None),
    Test('M8', 'Blocked goal and recoveries', True, ['M5'],
         'Wall/barrier fully across the lane about 1.2 m ahead; goal {goal:.1f} m ahead behind it.',
         'Goal NOT reached; no collision (clearance >= {clearance:.2f} m); Nav2 aborts or times out; '
         'robot at rest; recoveries recorded. Then remove the barrier and run M5 again.',
         _m8, lambda a: _sim_wall()),
    Test('M9', 'Obstacle-course traversal', True, ['M6', 'M7'],
         'Course laid out along the waypoints (forward,left[,yaw] from the start pose): '
         '{waypoints}. Stairs are NOT supported (no stair gait; seen as obstacles).',
         'Every waypoint reached in order (M5 criteria each); course geofence held.',
         _m9),
    Test('WALK', 'Walk to a point ahead, going around obstacles (Nav2)', True, ['M5'],
         'Lane toward the goal {goal:.1f} m ahead (--goal-distance; --goal-left to offset). Obstacles may '
         'be on the way; leave >= 1.2 m free beside them. Nobody in the lane.',
         'Nav2 reaches the goal within {xy:.2f} m; leg odometry agrees; obstacle clearance >= '
         '{clearance:.2f} m; speed <= {speed} m/s; collision_monitor stops are recorded (the safety '
         'net), not required.',
         _walk),
    Test('STAND', 'Stand up', True, ['M0'], 'Robot lying; 1 m clear all round.',
         'Standing verified by joint geometry, IMU and rest.', _stand),
    Test('SIT', 'Lie down', True, [], 'Robot standing still; 1 m clear all round.',
         'Lying verified by joint geometry, IMU and rest.', _sit),
]}


def fill(text, a):
    return text.format(block_at=0.20, stop=HARD_MIN_STOP_DISTANCE_M,
                       approach_speed=a.approach_speed, boundary=a.cm_boundary,
                       final_clearance=a.clearance_m4, travel=a.max_travel, initial=a.initial_clear,
                       range=a.detection_range, scans=int(a.detect_scans),
                       resp=min(a.response_limit, a.path_latency + 0.05), dur=a.max_duration,
                       object_at='' if a.object_distance is None else
                       f', nearest part {a.object_distance:.2f} m from the nose',
                       goal=a.goal_distance or 1.0, left=0.8 if a.goal_left is None else a.goal_left,
                       xy=a.xy_tolerance, yawtol=a.yaw_tolerance_deg, clearance=a.clearance,
                       speed=a.max_speed, waypoints=a.waypoints or '1,0; 2,0')


# --------------------------------------------------------------------------- gates
def gates_path(run_dir):
    return Path(run_dir) / 'physical' / 'gates.json'


def passed_tests(run_dir):
    path = gates_path(run_dir)
    return json.loads(path.read_text()) if run_dir and path.exists() else {}


def record_pass(run_dir, code, report_path):
    path = gates_path(run_dir)
    path.parent.mkdir(parents=True, exist_ok=True)
    data = passed_tests(run_dir)
    data[code] = {'time': dt.datetime.now().isoformat(timespec='seconds'), 'report': str(report_path)}
    path.write_text(json.dumps(data, indent=1))


def measured_coast(run_dir):
    """Coast after the command cut measured by this navigation run's passed M2: (m, m/s, report)."""
    entry = passed_tests(run_dir).get('M2') if run_dir else None
    if not entry:
        return None, None, None
    path = Path(entry['report']).with_suffix('.json')
    try:
        report = json.loads(path.read_text())
        step = next(s for s in report['steps'] if s['type'] == 'deadman_stop')
        diag = step['measurements']['diagnostics']
        coast = diag.get('coast_after_cut_m')
        if coast is None:                     # report written before coast was recorded
            trace, cut = step['trace'], diag['robot_commands_s']['last_nonzero']
            release = trace[0]['t'] + diag['release_s_after_motion_start']
            a = trace[0]
            fwd = [(r['t'] - release, (r['x'] - a['x']) * math.cos(a['yaw']) +
                    (r['y'] - a['y']) * math.sin(a['yaw'])) for r in trace]
            at_cut = min(fwd, key=lambda q: abs(q[0] - cut))[1]
            coast = max(v for t, v in fwd if t >= cut) - at_cut
        speed = report['physical_suite']['parameters']['max_speed']
        return round(max(0.0, coast), 4), speed, str(path)
    except (OSError, KeyError, StopIteration, ValueError, TypeError):
        return None, None, None


def measured_command_latency(run_dir):
    """First-command delivery latency (publish -> motion_sender) of this run's passed M2, or None."""
    entry = passed_tests(run_dir).get('M2') if run_dir else None
    if not entry:
        return None
    try:
        report = json.loads(Path(entry['report']).with_suffix('.json').read_text())
        step = next(s for s in report['steps'] if s['type'] == 'deadman_stop')
        value = step['measurements']['motion'].get('first_delivery_s')
        return None if value is None else float(value)
    except (OSError, KeyError, StopIteration, ValueError, TypeError):
        return None


def path_latency_allowance(delivery_s):
    """Command-path latency the stopping boundary assumes: measured delivery + one collision_monitor
    cycle (it reacts on its next input, 50 ms), never below the 0.10 s design allowance; 0.15 s
    while unmeasured."""
    if delivery_s is None:
        return UNMEASURED_PATH_LATENCY_S
    return round(max(0.10, delivery_s + 0.05), 3)


UNMEASURED_PATH_LATENCY_S = 0.15


def save_coast_calibration(state):
    """Store this run's measured M2 coast (and command latency) for the next start-up's boundary."""
    coast, speed, source = measured_coast(state.get('RUN_DIR'))
    delivery = measured_command_latency(state.get('RUN_DIR'))
    path = Path(state.get('LITE3_CALIBRATION') or
                Path(os.environ.get('LITE3_STATE_FILE', '')).parent / 'calibration.json')
    if coast is None or not path.parent.exists():
        return
    try:
        from .nav_steps import CP
        latency = path_latency_allowance(delivery) if delivery is not None else None
        CP.update_calibration(path, sources=[f'M2 {source}'], coast_m=coast, coast_speed_m_s=speed,
                              path_latency_s=latency)
        print(f'Calibration: coast {coast} m at {speed} m/s'
              + (f', command path latency {latency} s' if latency is not None else '') + f' saved to {path}')
    except (ValueError, OSError) as error:
        print(f'Calibration not saved: {error}')


# --------------------------------------------------------------------------- CLI
def bounded(low, high, name, integer=False):
    def convert(text):
        value = float(text)
        if not math.isfinite(value) or not low <= value <= high:
            raise argparse.ArgumentTypeError(f'{name} must be within [{low}, {high}] (hard limit)')
        if integer and value != int(value):
            raise argparse.ArgumentTypeError(f'{name} must be a whole number')
        return int(value) if integer else value
    return convert


# M4 options: (flag, dest, low, high, default or None = derived, integer, help). Every one can be
# given on the command line or in a YAML profile (--profile); the command line wins. Ranges are
# the validated envelope: values outside are REFUSED, never clamped. Raising travel or duration
# never changes the speed, the stopping boundary or any safeguard.
M4_OPTIONS = (
    ('--approach-speed', 'approach_speed', 0.02, HARD_MAX_APPROACH_SPEED_M_S, 0.05, False,
     'M4 forward walking speed, m/s'),
    ('--max-travel', 'max_travel', 0.2, HARD_MAX_APPROACH_TRAVEL_M, 1.5, False,
     'M4 maximum travel distance, m (leg odometry / 0.8 and SLAM pose; geofence = travel + 0.2 m)'),
    ('--max-duration', 'max_duration', 5.0, HARD_MAX_APPROACH_S, None, False,
     'M4 maximum movement duration, s (default: travel / speed + 10, at most 60)'),
    ('--min-clearance', 'min_clearance', 0.10, 1.0, None, False,
     'M4 desired minimum obstacle clearance from the nose, m (default: navigation --clearance)'),
    ('--trigger-distance', 'trigger_distance', HARD_MIN_STOP_DISTANCE_M, 1.5, None, False,
     'M4 proximity-stop trigger distance from the nose, m; must equal the running collision_monitor '
     'boundary (set by start_navigation.sh --clearance); default: the running boundary'),
    ('--initial-clear', 'initial_clear', 0.35, HARD_MAX_DETECTION_RANGE_M, None, False,
     'M4: the path must be clear this far ahead at the start, m (default: boundary + 0.15)'),
    ('--detection-range', 'detection_range', 0.5, HARD_MAX_DETECTION_RANGE_M, 2.0, False,
     'M4 validated LiDAR detection region ahead of the nose, m'),
    ('--max-sensor-age', 'max_sensor_age', 0.10, HARD_MAX_SENSOR_AGE_S, 0.30, False,
     'M4 maximum age of LiDAR/terrain status and lock input, s'),
    ('--min-lidar-hz', 'min_lidar_hz', HARD_MIN_LIDAR_HZ, 20.0, HARD_MIN_LIDAR_HZ, False,
     'M4 minimum LiDAR/terrain rate, Hz'),
    ('--detect-scans', 'detect_scans', 2, 10, 3, True,
     'M4 consecutive consistent scans needed to confirm an obstacle'),
    ('--track-tolerance', 'track_tolerance', 0.05, 0.25, 0.15, False,
     'M4 allowed disagreement between obstacle distance and own travel, m'),
    ('--lost-track-scans', 'lost_track_scans', 1, 5, 3, True,
     'M4 consecutive missing scans of a tracked obstacle that count as lost tracking'),
    ('--suppression-proof', 'suppression_proof', 0.5, 1.5, 1.0, False,
     'M4: keep publishing forward this long after the stop to prove it is suppressed, s'),
    ('--response-limit', 'response_limit', 0.10, HARD_MAX_RESPONSE_S, 0.30, False,
     'M4 limit from obstacle crossing the boundary to the last forward command at the robot, s '
     '(effective: min(this, command-path latency + 0.05))'),
)
M4_DESTS = {o[1]: o for o in M4_OPTIONS}
M4_PROFILE_ALIASES = {'final_clearance': 'min_clearance', 'speed': 'approach_speed'}


def load_profile(path):
    """M4 options from a YAML profile ({m4: {option: value}} or flat). Unknown keys and values
    outside the hard limits are refused."""
    raw = yaml.safe_load(Path(path).read_text()) or {}
    if not isinstance(raw, dict):
        raise ConfigError(f'profile {path}: must be a mapping')
    section = raw.get('m4', raw) if set(raw) <= {'m4', 'description'} else raw
    if not isinstance(section, dict):
        raise ConfigError(f'profile {path}: m4 must be a mapping')
    values = {}
    for key, value in section.items():
        if key == 'description':
            continue
        dest = key.lstrip('-').replace('-', '_')
        dest = M4_PROFILE_ALIASES.get(dest, dest)
        if dest not in M4_DESTS:
            raise ConfigError(f'profile {path}: unknown option {key!r} (known: {", ".join(M4_DESTS)})')
        flag, _, low, high, _, integer, _ = M4_DESTS[dest]
        try:
            values[dest] = bounded(low, high, flag, integer)(value)
        except (argparse.ArgumentTypeError, TypeError, ValueError) as error:
            raise ConfigError(f'profile {path}: {key}: {error}') from error
    return values


def resolve_m4(a, explicit_only=False):
    """Fill derived M4 values from the (possibly updated) running boundary and clearance."""
    ex = a.m4_explicit
    for dest, (_, _, _, _, default, _, _) in M4_DESTS.items():
        if dest not in ex and default is not None:
            setattr(a, dest, default)
    if 'max_duration' not in ex:
        a.max_duration = float(min(HARD_MAX_APPROACH_S, math.ceil(a.max_travel / a.approach_speed + 10)))
    a.clearance_m4 = a.min_clearance if 'min_clearance' in ex else (a.nav_clearance or 0.15)
    if 'initial_clear' not in ex:
        a.initial_clear = round(min(a.detection_range, a.cm_boundary + 0.15), 3)
    return a


def parse(argv):
    p = argparse.ArgumentParser(prog='run_physical', description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter,
                                epilog=M4_HELP)
    p.add_argument('--phases', default='M0', help='comma list, e.g. M0,M1,M2 (default M0)')
    p.add_argument('--list-phases', action='store_true')
    mode = p.add_mutually_exclusive_group()
    mode.add_argument('--physical', action='store_true', help='ACTUATE (armed stack + confirmation)')
    mode.add_argument('--sim', action='store_true', help='rehearse against the software stand-in')
    p.add_argument('--manual', action='store_true', help='dry/sim: pause for Enter before each test')
    p.add_argument('--max-speed', type=bounded(0.02, HARD_MAX_SPEED_M_S, 'max-speed'), default=0.05)
    p.add_argument('--max-yaw-rate', type=bounded(0.05, NAV2_MAX_YAW_RATE_RAD_S, 'max-yaw-rate'),
                   default=0.15)
    p.add_argument('--stop-distance', type=bounded(HARD_MIN_STOP_DISTANCE_M, 1.5, 'stop-distance'),
                   default=HARD_MIN_STOP_DISTANCE_M, help='proximity stop, metres from the nose')
    p.add_argument('--clearance', type=bounded(0.0, 1.0, 'clearance'), default=0.10,
                   help='minimum obstacle-to-footprint clearance for PASS')
    p.add_argument('--move-seconds', type=bounded(1.0, HARD_MAX_MOVE_S, 'move-seconds'), default=2.0)
    p.add_argument('--goal-distance', type=bounded(0.3, 6.0, 'goal-distance'), default=None)
    p.add_argument('--goal-left', type=bounded(-6.0, 6.0, 'goal-left'), default=None)
    p.add_argument('--goal-yaw-deg', type=bounded(-180.0, 180.0, 'goal-yaw-deg'), default=None)
    p.add_argument('--xy-tolerance', type=bounded(0.05, 1.0, 'xy-tolerance'), default=0.25)
    p.add_argument('--yaw-tolerance-deg', type=bounded(5.0, 180.0, 'yaw-tolerance-deg'), default=30.0)
    p.add_argument('--timeout', type=bounded(10.0, HARD_MAX_NAV_STEP_S, 'timeout'), default=None,
                   help='per Nav2 goal, seconds')
    p.add_argument('--waypoints', default=None, help='M9: "f,l[,yaw];f,l[,yaw];..." from the start')
    p.add_argument('--course-radius', type=bounded(1.0, 10.0, 'course-radius'), default=None)
    m4 = p.add_argument_group('M4 sensor-driven proximity stop (also settable in a YAML --profile)')
    m4.add_argument('--profile', default=None, help='YAML profile with M4 options (operator/profiles/)')
    for flag, dest, low, high, default, integer, text in M4_OPTIONS:
        names = [flag] + (['--final-clearance'] if dest == 'min_clearance' else [])
        shown = 'derived' if default is None else default
        m4.add_argument(*names, dest=dest, type=bounded(low, high, flag.lstrip('-'), integer),
                        default=None, help=f'{text} [{low}-{high}; default {shown}]')
    m4.add_argument('--shadow', action='store_true',
                    help='M4 dry/sim only: robot stays still, nothing is published; move an object '
                         'toward the robot and see where it is detected and when the stop would trigger')
    m4.add_argument('--shadow-seconds', type=bounded(5.0, 120.0, 'shadow-seconds'), default=60.0)
    m4.add_argument('--approach-start', default=None, help=argparse.SUPPRESS)   # removed (refused)
    p.add_argument('--direct-telemetry', action='store_true',
                   help='read robot telemetry in domain 64 (automatic if navigation was started '
                        'with --direct-telemetry)')
    p.add_argument('--object-distance', type=bounded(0.1, 3.0, 'object-distance'), default=None,
                   help='DET: expected distance of the object from the nose (m); --sim M4: where the '
                        'simulated obstacle is placed (default 1.0)')
    p.add_argument('--report-dir', default=None)
    args = p.parse_args(argv)
    explicit = {dest for dest in M4_DESTS if getattr(args, dest) is not None}
    if args.profile:
        for dest, value in load_profile(args.profile).items():
            if dest not in explicit:
                setattr(args, dest, value)
                explicit.add(dest)
    args.m4_explicit = explicit
    args.mode = MODE_PHYSICAL if args.physical else MODE_SIM if args.sim else MODE_READONLY
    args.measured_coast, args.coast_speed = None, 0.05     # filled from this run's M2 in main()
    args.path_latency = UNMEASURED_PATH_LATENCY_S          # from this run's M2 in main()
    args.cm_boundary = 0.37          # running collision_monitor boundary (state.env BOUNDARY)
    args.nav_clearance = None        # state.env CLEARANCE
    return resolve_m4(args)


M4_HELP = """M4 (sensor-driven stop), typical use:
  operator/run_physical.sh --phases M4                      dry: READY check, nothing published
  operator/run_physical.sh --phases M4 --shadow             dry: move an object toward the robot
  operator/run_physical.sh --sim --phases M4 --object-distance 1.6
  operator/run_physical.sh --physical --phases M4 --profile operator/profiles/m4_first_session.yaml
Options combine with --profile; the command line wins. Values outside the hard limits are refused."""


def list_phases(args):
    for t in CATALOG.values():
        print(f'{t.code:<6}{t.title}  ({"MOVES" if t.moves else "no motion"}; needs '
              f'{", ".join(t.prereqs) or "nothing"})')
        print(f'      Setup: {fill(t.setup, args)}')
        print(f'      PASS:  {fill(t.criteria, args)}')


def print_plan(test, seq, args):
    print(f'\n=== {test.code} {test.title} ===')
    print(f'Setup:  {fill(test.setup, args)}')
    print(f'PASS:   {fill(test.criteria, args)}')
    print(f'Limits: speed <= {args.max_speed} m/s, stop distance {args.stop_distance:.2f} m from the '
          f'nose, clearance >= {args.clearance} m, session <= {sum(s.timeout_s for s in seq.all_steps):.0f} s')
    print('Steps:  ' + ' -> '.join(s.id + ('*' if s.actuating else '') for s in seq.steps)
          + '   (* = sends commands)')


def confirm(test):
    if not sys.stdin.isatty():
        print('REFUSED: physical tests need an interactive terminal for the operator confirmation.')
        return False
    print('\nConfirm ALL of the following:')
    for _, text in CONFIRM_ITEMS:
        print(f'  [ ] {text}')
    previous = signal.signal(signal.SIGINT, signal.default_int_handler)   # Ctrl+C leaves the prompt
    try:
        answer = input(f'Type {test.code} and press Enter to START (anything else skips): ').strip()
    except (EOFError, KeyboardInterrupt):
        print()
        return False
    finally:
        signal.signal(signal.SIGINT, previous)
    return answer.upper() == test.code


def stack_state():
    state = {}
    path = os.environ.get('LITE3_STATE_FILE')
    if path and Path(path).exists():
        for line in Path(path).read_text().splitlines():
            if '=' in line:
                k, v = line.split('=', 1)
                state[k] = v
    return state


def run_one(test, args, out_root, robot_factory, mode, state):
    seq_dict = test.build(args)
    out = out_root / f'{dt.datetime.now():%H%M%S}-{test.code}'
    out.mkdir(parents=True, exist_ok=True)
    seq_path = out / 'sequence.yaml'
    seq_path.write_text(yaml.safe_dump(seq_dict, sort_keys=False))
    seq = load_sequence(seq_path)                    # hard limits enforced here
    print_plan(test, seq, args)
    auth = None
    if mode == MODE_PHYSICAL:
        missing = [p for p in test.prereqs if p not in passed_tests(state.get('RUN_DIR'))]
        if missing:
            print(f'REFUSED: {test.code} needs {", ".join(missing)} to PASS first on this navigation '
                  'run (protective mechanisms before obstacle tests).')
            return 3, None
        if not confirm(test):
            print(f'{test.code} skipped by operator; nothing was sent.')
            return 3, None
        template = authorization_template(seq, hours=0.25)
        template['operator_confirmations'] = {k: True for k in template['operator_confirmations']}
        template['confirmed_at'] = dt.datetime.now().isoformat(timespec='seconds')
        auth_path = out / 'authorization.yaml'
        auth_path.write_text('# Generated from the operator\'s typed confirmation of ' + test.code +
                             '\n' + yaml.safe_dump(template, sort_keys=False))
        auth = load_authorization(auth_path)
        problems = check_authorization(auth, seq)
        if problems:
            print(f'REFUSED: {problems}')
            return 3, None
    elif args.manual:
        try:
            input(f'Press Enter to run the {"simulation" if mode == MODE_SIM else "read-only check"} '
                  f'of {test.code} (Ctrl+C stops)... ')
        except EOFError:
            pass
    robot = robot_factory(test)
    runner = SequenceRunner(seq, robot, mode, log=lambda m: print(m, flush=True), authorization=auth)
    runner.nav_clearance_hint = args.nav_clearance
    report = runner.run()
    report['physical_suite'] = {'test': test.code, 'title': test.title,
                                'parameters': {k: v for k, v in vars(args).items()
                                               if k not in ('phases',)}}
    paths = write_reports(report, out)
    print(f'\n{test.code} VERDICT: {report["verdict"]}  ({report["evidence_class"]})')
    for s in report['steps']:
        print(f'  {s["status"]:<13} {s["id"]:<20} {s["summary"]}')
    print_m4_outcome(test, report)
    print(f'Report: {paths["markdown"]}')
    return EXIT[report['verdict']], paths['markdown']


def print_m4_outcome(test, report):
    for s in report['steps']:
        m = s.get('measurements') or {}
        if s['type'] != 'approach_stop' or 'outcome' not in m:
            continue
        print(f'{test.code} OUTCOME: {m["outcome"]} - {m["outcome_explanation"]}')
        for c in m.get('criteria', []):
            print(f'    [{"x" if c["ok"] else " "}] {c["criterion"]}')
        print(f'    detected at {m.get("obstacle_detected_at_m")} m | stop at '
              f'{m.get("nose_distance_at_stop_m")} m ({m.get("stop_layer")}) | travelled '
              f'{m.get("distance_travelled_m")} m in {m.get("walk_duration_s")} s | response '
              f'{m.get("response_s")} s | stopped moving after {m.get("stopped_moving_after_s")} s | '
              f'final clearance {m.get("final_nose_distance_m")} m (min {m.get("min_nose_distance_m")} m)')


def main(argv=None):
    try:
        return _main(argv)
    finally:
        BOUNDARY_CHECK.clear()                 # module-level: never leak into later sequences


def _main(argv=None):
    try:
        args = parse(sys.argv[1:] if argv is None else argv)
    except ConfigError as error:
        print(f'REFUSED: {error}')
        return 3
    if args.approach_start is not None:
        print('REFUSED: --approach-start was removed. M4 no longer needs the obstacle at a start '
              'distance: place it anywhere in the lane (see --list-phases).')
        return 3
    if args.shadow and args.physical:
        print('REFUSED: --shadow is the dry (non-moving) M4; run it without --physical.')
        return 3
    if args.list_phases:
        list_phases(args)
        return 0
    chosen = [c.strip().upper() for c in args.phases.split(',') if c.strip()]
    unknown = [c for c in chosen if c not in CATALOG]
    if unknown or not chosen:
        print(f'Unknown test(s) {unknown or chosen}; choose from {", ".join(CATALOG)}', file=sys.stderr)
        return 3
    order = list(CATALOG)
    tests = [CATALOG[c] for c in sorted(set(chosen), key=order.index)]
    state = stack_state()
    BOUNDARY_CHECK.clear()                     # module-level: never carry over from an earlier call
    mode = args.mode
    if state.get('CLEARANCE'):
        args.nav_clearance = float(state['CLEARANCE'])
        resolve_m4(args)
    latency_source = None
    if mode == MODE_SIM:
        args.measured_coast, args.coast_speed, source = 0.035, 0.05, 'simulation default'
        args.path_latency, latency_source = 0.10, 'simulation default'
        # Simulate the boundary start_navigation.sh would compute for this clearance.
        from .nav_steps import CP
        cal_path = Path(os.environ.get('LITE3_STATE_FILE', '/nonexistent')).parent / 'calibration.json'
        args.cm_boundary = args.trigger_distance if 'trigger_distance' in args.m4_explicit else \
            CP.boundary(args.clearance_m4, 0.10, CP.load_calibration(cal_path))['boundary_m']
        print(f'Simulated collision_monitor boundary {args.cm_boundary} m (clearance {args.clearance_m4} m)')
    else:
        args.measured_coast, args.coast_speed, source = measured_coast(state.get('RUN_DIR'))
        args.coast_speed = args.coast_speed or 0.05
        delivery = measured_command_latency(state.get('RUN_DIR'))
        args.path_latency = path_latency_allowance(delivery)
        if delivery is not None:
            latency_source = f'M2 on this navigation run: first command delivered in {delivery} s'
    if state.get('BOUNDARY'):
        if mode != MODE_SIM or 'trigger_distance' not in args.m4_explicit:
            args.cm_boundary = float(state['BOUNDARY'])    # rehearse with the running boundary
        if mode != MODE_SIM:
            from .nav_steps import CP
            cal = CP.load_calibration(Path(os.environ.get('LITE3_STATE_FILE', '')).parent / 'calibration.json')
            BOUNDARY_CHECK.update({'cm_boundary_m': args.cm_boundary,
                                   'cm_clearance_m': float(state.get('CLEARANCE', 0.15)),
                                   'cm_max_speed_m_s': float(state.get('MAX_SPEED', 0.10)),
                                   'coast_m': cal.coast_m, 'coast_speed_m_s': cal.coast_speed_m_s})
        if args.max_speed > float(state.get('MAX_SPEED', 0.10)) + 1e-9:
            print(f'REFUSED: --max-speed {args.max_speed} exceeds the {state.get("MAX_SPEED")} m/s the '
                  'collision stop boundary was computed for (restart navigation with --max-speed).')
            return 3
        if args.max_speed > VALIDATED_SPEED_M_S + 1e-9 and \
                args.stop_distance < args.cm_boundary - 0.05 - 1e-9:
            print(f'REFUSED: --max-speed {args.max_speed} is above the validated 0.10 m/s, where the 0.30 m '
                  f'backup lock cannot act as a backup. Pass --stop-distance >= {args.cm_boundary - 0.05:.2f} '
                  f'(collision_monitor boundary {args.cm_boundary} m, which must have been started with '
                  f'--max-speed {args.max_speed} or higher).')
            return 3
    resolve_m4(args)
    if any(t.code == 'M4' for t in tests):
        if 'trigger_distance' in args.m4_explicit and mode != MODE_SIM and \
                abs(args.trigger_distance - args.cm_boundary) > 0.005:
            print(f'REFUSED: --trigger-distance {args.trigger_distance} m but the running '
                  f'collision_monitor boundary is {args.cm_boundary} m. The boundary is fixed when '
                  'navigation starts: operator/stop_navigation.sh, then start_navigation.sh '
                  '--physical --clearance C (the boundary grows with C; it is printed at start-up).')
            return 3
        print(f'M4: speed {args.approach_speed} m/s | travel <= {args.max_travel} m | duration <= '
              f'{args.max_duration:.0f} s | trigger {args.cm_boundary} m from the nose | min clearance '
              f'{args.clearance_m4} m | path clear >= {args.initial_clear} m at start | detection '
              f'range {args.detection_range} m | sensor age <= {args.max_sensor_age} s, LiDAR >= '
              f'{args.min_lidar_hz} Hz | confirm {args.detect_scans} scans'
              + (f' | profile {args.profile}' if args.profile else ''))
        print(f'M4 stopping boundary uses coast {args.measured_coast} m at {args.coast_speed} m/s '
              f'({source or "NOT MEASURED: M2 must pass on this navigation run"}) and command-path '
              f'latency {args.path_latency} s ({latency_source or "not measured yet: conservative 0.15 s"})')
    root = Path(args.report_dir) if args.report_dir else (
        Path(state['RUN_DIR']) / 'physical' if state.get('RUN_DIR')
        else Path.home() / 'lite3_ws' / 'log' / 'physical')
    out_root = root / f'{dt.datetime.now():%Y%m%d-%H%M%S}-{mode}'
    print(f'Mode: {mode.upper()}' + {MODE_PHYSICAL: '  (the robot WILL move after each confirmation)',
                                     MODE_SIM: '  (software stand-in; no robot)',
                                     MODE_READONLY: '  (dry: live robot read-only, nothing published)'}[mode])
    if mode == MODE_PHYSICAL:
        if state.get('MODE') != 'physical':
            print('REFUSED: navigation is not running in physical mode. Start it with '
                  'operator/start_navigation.sh --physical (separate arming step).')
            return 3
        if os.environ.get('ROS_LOCALHOST_ONLY') == '1':
            print('REFUSED: ROS_LOCALHOST_ONLY=1 cannot reach the robot.')
            return 3
    direct = args.direct_telemetry or state.get('TELEMETRY') == 'direct'
    args.telemetry_source = 'direct (domain 64)' if direct else 'bridge (domain 8)'
    if mode != MODE_SIM:
        print(f'Robot telemetry for the test runner: {args.telemetry_source}')
    cancel = threading.Event()
    old = {s: signal.signal(s, lambda *_: cancel.set()) for s in (signal.SIGINT, signal.SIGTERM)}
    shared = {}

    def factory(test):
        if mode == MODE_SIM:
            from .sim_model import FakeNavRobot, SimLite3
            from .telemetry import Telemetry
            robot = FakeNavRobot(Telemetry(), SimLite3(start='lying'),
                                 obstacles=test.scene(args) if test.scene else (),
                                 cm_boundary=args.cm_boundary)
            if test.code == 'M4' and args.shadow:     # someone slides the object toward the robot
                robot.obstacle_velocity, robot.obstacle_motion_from = (-0.08, 0.0), robot.t + 2.0
            robot.cancelled = cancel.is_set
            return robot
        if 'robot' not in shared:
            from .ros_io import NAV_WIRING, NavRobot, direct_telemetry
            wiring = direct_telemetry(NAV_WIRING) if direct else NAV_WIRING
            shared['robot'] = NavRobot(wiring, allow_commands=(mode == MODE_PHYSICAL),
                                       cancel_flag=cancel)
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline and shared['robot'].telemetry.joints.rate(time.monotonic()) < 50:
                time.sleep(0.2)
        return shared['robot']

    results, code = [], 0
    try:
        for test in tests:
            if cancel.is_set():
                break
            try:
                code, report = run_one(test, args, out_root, factory, mode, state)
            except ConfigError as error:
                print(f'REFUSED {test.code}: {error}')
                code, report = 3, None
            results.append((test.code, code, report))
            if code == 0 and mode == MODE_PHYSICAL:
                record_pass(state.get('RUN_DIR'), test.code, report)
                if test.code == 'M2':
                    save_coast_calibration(state)
            if code != 0 and mode == MODE_PHYSICAL:
                print('Stopping the session: later tests depend on this one passing.')
                break
    finally:
        robot = shared.get('robot')
        if robot is not None:
            if mode == MODE_PHYSICAL:
                robot.set_stop_distance(DEFAULT_STOP_AHEAD_M)
            robot.close()
        for s, h in old.items():
            signal.signal(s, h)
    print('\n========== SESSION SUMMARY ==========')
    for c, rc, report in results:
        label = {0: 'PASS', 1: 'FAIL (safe)', 2: 'INTERVENTION REQUIRED', 3: 'NOT RUN'}[rc]
        print(f'  {c:<6} {label:<22} {report or ""}')
    if mode == MODE_READONLY:
        print('Dry run only: nothing was published. Blockers are the FAILED items above.')
    return max((rc for _, rc, _ in results), default=3)


if __name__ == '__main__':
    raise SystemExit(main())
