"""Test 4: live, NON-ACTUATING navigation test with real obstacles (five minutes).

Run via operator/run_test4.sh while operator/start_navigation.sh (dry) is up.
Nav2 receives goals ahead of the robot; what it WOULD command goes only to
/lite3_locomotion/dry_cmd_vel. The robot stays where it is (lying is fine).

Live line every second: phase, elapsed time, odometry/terrain rates, obstacle
distances, proximity-stop state, Nav2's would-be command, robot-side command count.

Non-actuation guards (test aborts at once if violated):
  * group domain 8: /cmd_vel must have NO publishers
  * system domain 64: /cmd_vel (motion_sender input) must receive NO messages

Ctrl+C aborts immediately: the goal is cancelled and a partial report is written.
Exit: 0 PASS, 1 FAIL, 2 ABORTED / NON-ACTUATION VIOLATION, 3 INCONCLUSIVE (scene not clear).
"""

import argparse
import datetime as dt
import json
import math
from pathlib import Path
import signal
import sys
import threading
import time

# Distances are from the robot body centre (base_link). The body front edge is 0.305 m ahead
# of the centre (Nav2 footprint), so "from the nose" = centre distance - 0.30.
NOSE = 0.30
PHASES = [
    ('P1', 'CLEAR', 60, 'Nothing within 0.6 m all round the robot and 2.5 m ahead. Stand to the side, '
                        'outside that area.'),
    ('P2', 'BOX FAR', 60, 'Place a box (at least 20 cm tall) straight ahead, its front face {p2:.2f} m '
                          'from the robot\'s nose. Then step away.'),
    ('P3', 'BOX NEAR', 60, 'Move the box to {p3:.2f} m from the robot\'s nose (straight ahead). '
                           'Expect PROXIMITY STOP.'),
    ('P4', 'LOW OBJECT', 60, 'Remove the box. Place a low object 5-8 cm tall (e.g. thick book) '
                             '1.0 m from the nose, straight ahead.'),
    ('P5', 'WALK ACROSS', 60, 'Remove the object. Walk slowly across, 1-1.5 m in front of the robot, '
                              'once or twice, then leave the area clear.'),
]
# Short reference capture, added automatically when P2/P4/P5 run without P1.
BASE_PHASE = ('BASE', 'BASELINE', 20, 'Reference scene: nothing within 0.6 m all round and 2.5 m ahead. '
                                     'Stay outside that area.')
NEEDS_BASELINE = {'P2', 'P4', 'P5'}
CAPS = {'vx_max': 0.101, 'vx_min': -0.001, 'vy_abs': 0.001, 'wz_abs': 0.201}
GROUP_DOMAIN, SYSTEM_DOMAIN = 8, 64
DRY = '/lite3_locomotion/dry_cmd_vel'
NOSE_M = 0.305
DEFAULT_STOP_NOSE_M = 0.30          # terrain StopParams 0.60 m from centre (floor; cannot go lower)
# What each perception phase corresponds to in the physical suite (operator/run_physical.sh).
PHYSICAL_EQUIVALENT = {
    'P1': (['M1'], 'basic low-speed motion in the same clear area'),
    'P2': (['M7'], 'Nav2 drives toward a goal past a box on the straight line (avoidance)'),
    'P3': (['M3', 'M4'], 'object in the stop zone blocks a forward command (M3), then a slow '
                         'approach the proximity lock must stop (M4)'),
    'P4': ([], 'no physical test yet: low objects are classified but stepping over them is not '
               'validated (blocker)'),
    'P5': ([], 'not offered: people moving close to a moving robot'),
}


def placements(stop_nose):
    """Nose distances for P2/P3 so that P2 stays well outside and P3 well inside the stop zone."""
    return {'p2': max(1.2, stop_nose + 0.6), 'p3': max(0.15, stop_nose - 0.05)}


class Recorder:
    def __init__(self):
        self.lock = threading.Lock()
        self.data = {k: [] for k in ('odom', 'status', 'stop', 'dry', 'nav_in', 'robot_cmd', 'goals')}

    def add(self, key, value):
        with self.lock:
            self.data[key].append(value)

    def window(self, key, start, end=math.inf):
        with self.lock:
            return [v for v in self.data[key] if start <= v[0] <= end]


def median(values):
    values = sorted(v for v in values if v is not None)
    return values[len(values) // 2] if values else None


def stop_episodes(rec, a=0.0, b=math.inf):
    """Continuous runs of stop=true samples as (first_true, last_true)."""
    episodes, current = [], None
    for t, value in rec.window('stop', a, b):
        if value:
            current = [t, t] if current is None else [current[0], t]
        elif current is not None:
            episodes.append(tuple(current))
            current = None
    if current is not None:
        episodes.append(tuple(current))
    return episodes


def phase_window(index):
    start = sum(p[2] for p in PHASES[:index])
    return start, start + PHASES[index][2]


def default_windows(t_end):
    """Back-to-back timing of all PHASES (used when no recorded windows are given)."""
    return {PHASES[i][0]: phase_window(i) for i in range(len(PHASES)) if phase_window(i)[1] <= t_end}


def baseline(rec, window):
    """Scene with nothing placed (P1 or BASE, last 30 s): what is already there."""
    a, b = window
    tail = max(a, b - 30)
    vals = rec.window('status', tail, b)
    return {'obstacle': median([s[1].get('nearest_obstacle_m') for s in vals]),
            'step': median([s[1].get('nearest_step_m') for s in vals]),
            'stops': sum(v for _, v in rec.window('stop', tail, b)),
            'reasons': sorted({s[2] for s in vals if s[2]})}


def differs(value, base, tolerance=0.2):
    return value is not None and (base is None or abs(value - base) > tolerance)


def evaluate(rec, t_end, aborted, windows=None, p2_nose=1.2):
    """`windows`: {phase code: (start, end)} of COMPLETED phases, in seconds since test start."""
    windows = default_windows(t_end) if windows is None else windows
    results = []

    def check(name, ok, detail, judged=True):
        results.append({'check': name, 'result': ('PASS' if ok else 'FAIL') if judged else 'INCONCLUSIVE',
                        'detail': detail})

    def values(a, b, key):
        return [s[1].get(key) for s in rec.window('status', a, b)]

    reference = windows.get('P1') or windows.get('BASE')
    base = baseline(rec, reference) if reference else None
    if base is not None:
        a, b = reference
        label = 'P1' if 'P1' in windows else 'BASE'
        commands = rec.window('dry', a, b)
        check(f'{label} surroundings clear: no proximity stop with nothing placed', base['stops'] == 0,
              'no stops' if not base['stops'] else
              f"{base['stops']} stop samples ({', '.join(base['reasons'])}): something is within "
              '0.6 m ahead or 0.45 m around the robot')
        check(f'{label} Nav2 produced would-be commands', len(commands) > 10, f'{len(commands)} commands')
        results.append({'check': f'{label} baseline scene', 'result': 'INFO',
                        'detail': f"nearest obstacle {base['obstacle']} m, nearest step {base['step']} m "
                                  '(later phases must differ from this)'})
    if 'P2' in windows:
        a, b = windows['P2']
        d = median(values(max(a, b - 30), b, 'nearest_obstacle_m'))
        centre = p2_nose + NOSE_M
        in_range = d is not None and centre - 0.3 <= d <= centre + 0.3
        if in_range and not differs(d, base and base['obstacle']):
            check(f'P2 box detected ~{centre:.1f} m from body centre', False,
                  f'obstacle {d} m equals the baseline obstacle; cannot attribute it to the box',
                  judged=False)
        else:
            check(f'P2 box detected ~{centre:.1f} m from body centre ({p2_nose:.1f} m from nose)', in_range,
                  f'median nearest obstacle {d} m (baseline {base and base["obstacle"]} m)')
        far_stops = [e for e in stop_episodes(rec, max(a, b - 30), b)]
        check('P2 no proximity stop for a box beyond 0.6 m', not far_stops,
              f'{len(far_stops)} stop episodes in last 30 s')
    if 'P3' in windows:
        a, b = windows['P3']
        samples = rec.window('stop', max(a, b - 30), b)
        frac = sum(v for _, v in samples) / len(samples) if samples else 0.0
        ahead = [s for s in rec.window('status', max(a, b - 30), b) if s[2] and 'ahead' in s[2]]
        check('P3 proximity stop with box inside the stop zone (reason: ahead)',
              frac >= 0.8 and bool(ahead),
              f'stop active {frac:.0%} of last 30 s; "ahead" samples {len(ahead)}; '
              f'nearest obstacle {median(values(max(a, b - 30), b, "nearest_obstacle_m"))} m')
    # Lock: in every continuous stop episode, nothing may pass the mux after 0.6 s.
    leaked, blocked, exercised = 0, 0, 0
    for first, last in stop_episodes(rec, 0, t_end):
        if last - first <= 0.6:
            continue
        leaked += len(rec.window('dry', first + 0.6, last))
        nav = [n for n in rec.window('nav_in', first + 0.6, last) if any(abs(x) > 1e-6 for x in n[1:])]
        blocked += len(nav)
        exercised += bool(nav)
    if leaked:
        check('LOCK: no Nav2 command passes the mux during a stop', False,
              f'{leaked} commands passed during stop episodes')
    elif exercised:
        check('LOCK: no Nav2 command passes the mux during a stop (proven)', True,
              f'{blocked} Nav2 commands blocked across {exercised} stop episodes, 0 passed')
    else:
        check('LOCK: no Nav2 command passes the mux during a stop', True,
              'no stop episode occurred while Nav2 was commanding (not exercised)', judged=False)
    if 'P4' in windows:
        a, b = windows['P4']
        step = median(values(max(a, b - 30), b, 'nearest_step_m'))
        obstacle = median(values(max(a, b - 30), b, 'nearest_obstacle_m'))
        step_hit = step is not None and 1.0 <= step <= 1.7 and differs(step, base and base['step'])
        obstacle_hit = (obstacle is not None and 1.0 <= obstacle <= 1.7 and
                        differs(obstacle, base and base['obstacle']))
        check('P4 low object detected ~1.3 m from centre (as step, or obstacle if >10 cm)',
              step_hit or obstacle_hit,
              f'step {step} m (baseline {base and base["step"]}), obstacle {obstacle} m '
              f'(baseline {base and base["obstacle"]})')
    if 'P5' in windows:
        a, b = windows['P5']
        reference = base and base['obstacle']
        seen = [o for o in values(a, b - 10, 'nearest_obstacle_m')
                if o is not None and o <= 2.0 and differs(o, reference)]
        if reference is not None and reference <= 2.0 and not seen:
            check('P5 walking person detected', False, f'baseline already has an obstacle at {reference} m',
                  judged=False)
        else:
            check('P5 walking person detected', bool(seen), f'{len(seen)} samples with a new obstacle <= 2.0 m')
        check('P5 area clear at end (no stop in last 10 s)',
              not any(v for _, v in rec.window('stop', b - 10, b)), 'stop samples in last 10 s')
    odom = [o[0] for o in rec.window('odom', 0, t_end)]
    gaps = [b - a for a, b in zip(odom, odom[1:])]
    hz = len(odom) / max(t_end, 1)
    check('LiDAR odometry rate >= 8 Hz, <= 3 gaps > 0.5 s', hz >= 8 and sum(g > 0.5 for g in gaps) <= 3,
          f'{hz:.1f} Hz, {sum(g > 0.5 for g in gaps)} gaps, max {max(gaps, default=0):.2f} s')
    status = rec.window('status', 0, t_end)
    problems = sorted({p for s in status for p in (s[3] or [])})
    check('terrain monitor inputs valid', not problems, ', '.join(problems) or 'none')
    dry = rec.window('dry', 0, t_end)
    vx = [d[1] for d in dry]
    caps_ok = (not dry or (max(vx) <= CAPS['vx_max'] and min(vx) >= CAPS['vx_min'] and
                           max(abs(d[2]) for d in dry) <= CAPS['vy_abs'] and
                           max(abs(d[3]) for d in dry) <= CAPS['wz_abs']))
    check('Nav2 commands within 0.10 m/s, 0.2 rad/s, no reverse/lateral', caps_ok,
          f'vx [{min(vx, default=0):.3f}, {max(vx, default=0):.3f}], '
          f'|wz| <= {max((abs(d[3]) for d in dry), default=0):.3f}')
    robot = rec.window('robot_cmd', 0, t_end)
    check('NON-ACTUATING: zero messages on robot-side /cmd_vel (domain 64)', not robot,
          f'{len(robot)} messages')
    judged = [r for r in results if r['result'] in ('PASS', 'FAIL')]
    unsure = [r for r in results if r['result'] == 'INCONCLUSIVE']
    verdict = ('ABORTED' if aborted else 'FAIL' if any(r['result'] == 'FAIL' for r in judged)
               else 'INCONCLUSIVE' if unsure else 'PASS')
    return verdict, results


def set_stop_distance(node, ahead_m):
    """terrain_monitor parameter stop_ahead_m (from centre); the node refuses values below 0.405."""
    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
    from rcl_interfaces.srv import SetParameters
    client = node.create_client(SetParameters, '/lite3_terrain_monitor/set_parameters')
    try:
        if not client.wait_for_service(timeout_sec=5.0):
            return {'ok': False, 'reason': 'terrain_monitor parameter service not available '
                                           '(rebuild + restart navigation after updating)'}
        future = client.call_async(SetParameters.Request(parameters=[Parameter(
            name='stop_ahead_m', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                                      double_value=round(ahead_m, 3)))]))
        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not future.done():
            return {'ok': False, 'reason': 'timeout'}
        result = future.result().results[0]
        return {'ok': result.successful, 'reason': result.reason}
    finally:
        node.destroy_client(client)


def explain_physical(args):
    chosen = [c.strip().upper() for c in args.phases.split(',') if c.strip()]
    print('Test 4 is PERCEPTION-ONLY: the robot stays where it is and Nav2 output goes to a dry topic.\n'
          'Its phases cannot be made physical. The physical counterparts are separate, supervised\n'
          'tests (operator/run_physical.sh), run on a separately armed stack:\n')
    mapped = []
    for code in chosen:
        tests, text = PHYSICAL_EQUIVALENT.get(code, ([], 'unknown phase'))
        print(f'  {code} -> {", ".join(tests) or "(none)"}: {text}')
        mapped += [t for t in tests if t not in mapped]
    if mapped:
        extra = f' --stop-distance {args.stop_distance:.2f}' if args.stop_distance != DEFAULT_STOP_NOSE_M else ''
        print('\nThese need M0, M1 and M2 to pass first on the same physical navigation run:\n'
              '  operator/stop_navigation.sh && operator/start_navigation.sh --physical\n'
              f'  operator/run_physical.sh --physical --phases M0,M1,M2\n'
              f'  operator/run_physical.sh --physical --phases {",".join(mapped)}{extra}\n'
              'Each physical test shows its setup and asks you to type its ID before it moves.\n'
              'Without --physical, run_physical.sh only plans and checks readiness (nothing moves).')
    return 2


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--goal-distance', type=float, default=2.5)
    parser.add_argument('--goal-period', type=float, default=30.0)
    parser.add_argument('--report-dir', default=str(Path.home() / 'lite3_ws' / 'log' / 'test4'))
    parser.add_argument('--phases', default='P1,P2,P3,P4,P5',
                        help='comma-separated subset to run, e.g. P3 or P2,P3 (default: all five)')
    parser.add_argument('--manual', action='store_true',
                        help='wait for Enter before each phase (time to place objects)')
    parser.add_argument('--list-phases', action='store_true', help='show the phases and exit')
    parser.add_argument('--stop-distance', type=float, default=DEFAULT_STOP_NOSE_M,
                        help='proximity stop distance from the nose for this test, 0.30-1.50 m '
                             '(cannot be reduced below 0.30)')
    parser.add_argument('--physical', action='store_true',
                        help='explain the physical counterparts of the selected phases and exit')
    parser.add_argument('--phase-seconds', type=float, default=None, help=argparse.SUPPRESS)
    args = parser.parse_args()
    if not math.isfinite(args.stop_distance) or not DEFAULT_STOP_NOSE_M <= args.stop_distance <= 1.5:
        print(f'--stop-distance must be within [{DEFAULT_STOP_NOSE_M}, 1.5] m from the nose', file=sys.stderr)
        return 2
    place = placements(args.stop_distance)
    if args.physical:
        return explain_physical(args)
    if args.list_phases:
        for code, name, seconds, text in PHASES:
            print(f'{code}  {name:<12} {seconds:>3.0f} s  {text.format(**place)}')
        print(f'(P2, P4 and P5 need a reference scene: without P1 a {BASE_PHASE[2]} s BASE capture runs first)')
        return 0
    known = {p[0]: p for p in PHASES}
    chosen = [c.strip().upper() for c in args.phases.split(',') if c.strip()]
    unknown = [c for c in chosen if c not in known]
    if unknown or not chosen:
        print(f'Unknown phase(s) {unknown or chosen}; choose from {", ".join(known)}', file=sys.stderr)
        return 2
    sequence = [(a, b, c, d.format(**place)) for a, b, c, d in
                [known[c] for c in known if c in chosen]]         # canonical order
    if NEEDS_BASELINE & set(chosen) and 'P1' not in chosen:
        sequence.insert(0, BASE_PHASE)
    if args.phase_seconds:      # shortened phases are for software checks only
        sequence = [(a, b, args.phase_seconds, d) for a, b, _, d in sequence]

    import rclpy
    from rclpy.action import ActionClient
    from rclpy.context import Context
    from rclpy.duration import Duration
    from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from rclpy.signals import SignalHandlerOptions
    from rclpy.time import Time
    from geometry_msgs.msg import Twist
    from nav2_msgs.action import NavigateToPose
    from nav_msgs.msg import Odometry
    from std_msgs.msg import Bool, String
    from tf2_ros import Buffer, TransformListener

    rec = Recorder()
    t0 = time.monotonic()

    def now():
        return time.monotonic() - t0

    ctx = Context()
    rclpy.init(args=[], context=ctx, domain_id=GROUP_DOMAIN, signal_handler_options=SignalHandlerOptions.NO)
    node = Node('lite3_test4', context=ctx, start_parameter_services=False)
    sys_ctx = Context()
    rclpy.init(args=[], context=sys_ctx, domain_id=SYSTEM_DOMAIN, signal_handler_options=SignalHandlerOptions.NO)
    sys_node = Node('lite3_test4_observer', context=sys_ctx, start_parameter_services=False)
    buffer = Buffer()
    TransformListener(buffer, node)
    node.create_subscription(Odometry, '/odom', lambda m: rec.add('odom', (now(),)), qos_profile_sensor_data)
    node.create_subscription(Bool, '/lite3_terrain/stop', lambda m: rec.add('stop', (now(), m.data)),
                             qos_profile_sensor_data)

    def on_status(msg):
        s = json.loads(msg.data)
        c = s.get('corridor') or {}
        rec.add('status', (now(), {k: c.get(k) for k in ('nearest_obstacle_m', 'nearest_step_m',
                                                         'nearest_drop_m', 'clear_ahead_m')},
                           (s.get('stop') or {}).get('reason'), s.get('problems')))
    node.create_subscription(String, '/lite3_terrain/status', on_status, 10)
    node.create_subscription(Twist, DRY, lambda m: rec.add('dry', (now(), m.linear.x, m.linear.y,
                                                                   m.angular.z)), 50)
    node.create_subscription(Twist, '/nav_cmd_vel', lambda m: rec.add('nav_in', (now(), m.linear.x,
                                                                                 m.angular.z)), 50)
    sys_node.create_subscription(Twist, '/cmd_vel', lambda m: rec.add('robot_cmd', (now(), m.linear.x)),
                                 qos_profile_sensor_data)
    client = ActionClient(node, NavigateToPose, '/navigate_to_pose')
    executor = MultiThreadedExecutor(num_threads=3, context=ctx)
    executor.add_node(node)
    sys_executor = SingleThreadedExecutor(context=sys_ctx)
    sys_executor.add_node(sys_node)
    for ex in (executor, sys_executor):
        threading.Thread(target=ex.spin, daemon=True).start()

    stop_flag = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop_flag.set())

    def violation():
        pubs = [p.node_name for p in node.get_publishers_info_by_topic('/cmd_vel')]
        if pubs:
            return f'/cmd_vel (group domain) has publishers {pubs}'
        if rec.window('robot_cmd', 0):
            return 'messages observed on robot-side /cmd_vel (domain 64)'
        if [p.node_name for p in node.get_publishers_info_by_topic(DRY)] != ['collision_monitor']:
            return f'collision_monitor is not the only publisher of the dry topic {DRY}'
        return None

    print('Test 4 (non-actuating). Connecting...', flush=True)
    time.sleep(3.0)
    applied = ({'ok': True} if args.stop_distance == DEFAULT_STOP_NOSE_M
               else set_stop_distance(node, args.stop_distance + NOSE_M))
    if not applied['ok']:
        print(f'REFUSED before start: could not set the proximity stop distance: {applied["reason"]}',
              flush=True)
        return 2
    if args.stop_distance != DEFAULT_STOP_NOSE_M:
        print(f'Proximity stop set to {args.stop_distance:.2f} m from the nose for this test '
              '(restored afterwards).', flush=True)
    aborted, abort_reason = False, None
    first = violation()
    if first or not client.wait_for_server(timeout_sec=15):
        print(f'REFUSED before start: {first or "Nav2 /navigate_to_pose not available"}', flush=True)
        return 2
    t0 = time.monotonic()
    rec.data = {k: [] for k in rec.data}
    handle = [None]
    goals_sent = [0]

    def send_goal():
        if handle[0] is not None:
            handle[0].cancel_goal_async()
        try:
            tf = buffer.lookup_transform('map', 'base_link', Time(), timeout=Duration(seconds=2))
        except Exception as error:  # noqa: BLE001
            rec.add('goals', (now(), f'no map->base_link: {error}'))
            return
        q = tf.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x = tf.transform.translation.x + args.goal_distance * math.cos(yaw)
        goal.pose.pose.position.y = tf.transform.translation.y + args.goal_distance * math.sin(yaw)
        goal.pose.pose.orientation = q
        future = client.send_goal_async(goal)
        deadline = time.monotonic() + 5
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        handle[0] = future.result() if future.done() else None
        goals_sent[0] += 1
        rec.add('goals', (now(), bool(handle[0] and handle[0].accepted)))

    windows = {}
    state = {'next_goal': 0.0, 'last_check': 0.0, 'last_line': 0.0, 'abort': None}
    enter = threading.Event()

    def read_enter():
        while True:
            line = sys.stdin.readline()
            if not line:            # stdin closed: no operator present
                return
            enter.set()

    if args.manual:
        threading.Thread(target=read_enter, daemon=True).start()

    def fmt(v):
        return f'{v:.2f}' if v is not None else '--'

    def tick(label, progress):
        """Goals, non-actuation guard and the live line; shared by running and waiting."""
        t = now()
        if t >= state['next_goal']:
            send_goal()
            state['next_goal'] = t + args.goal_period
        if t - state['last_check'] >= 1.0:
            state['last_check'] = t
            reason = violation()
            if reason:
                state['abort'] = 'NON-ACTUATION VIOLATION: ' + reason
                print(f'\n\a!!! {state["abort"]} - aborting. Use the robot e-stop if it moves. !!!', flush=True)
                return False
        if t - state['last_line'] >= 1.0:
            state['last_line'] = t
            odom_hz = len(rec.window('odom', t - 2, t)) / 2
            status = rec.window('status', t - 2, t)
            s = status[-1] if status else (0, {}, None, None)
            stop_now = rec.window('stop', t - 1, t)
            stop_txt = ('STOP ' + (s[2] or '')) if stop_now and stop_now[-1][1] else 'clear'
            dry = rec.window('dry', t - 1, t)
            cmd = f'vx {dry[-1][1]:+.2f} wz {dry[-1][3]:+.2f}' if dry else 'none (blocked/idle)'
            print(f'[{progress}] {label} | odom {odom_hz:4.1f} Hz terrain {len(status) / 2:4.1f} Hz | '
                  f'obstacle {fmt(s[1].get("nearest_obstacle_m"))} step {fmt(s[1].get("nearest_step_m"))} '
                  f'drop {fmt(s[1].get("nearest_drop_m"))} m | {stop_txt[:32]:<32} | Nav2 {cmd} | '
                  f'robot cmd_vel msgs {len(rec.window("robot_cmd", 0))}', flush=True)
        return True

    print(f'Phases: {", ".join(p[0] for p in sequence)}'
          f'{"  (manual: press Enter to start each phase)" if args.manual else ""}', flush=True)
    try:
        for number, (code, name, seconds, text) in enumerate(sequence, 1):
            print(f'\n\a=== {code} {name} ({seconds:.0f} s, phase {number}/{len(sequence)}) ===\n'
                  f'    {text}\n', flush=True)
            if args.manual:
                print('    Set up the scene, then press Enter to start this phase (Ctrl+C aborts).',
                      flush=True)
                enter.clear()
                while not enter.is_set() and not stop_flag.is_set():
                    if not tick(f'{code} waiting for Enter', f'{now():5.0f}s'):
                        break
                    time.sleep(0.1)
            if stop_flag.is_set() or state['abort']:
                break
            start = now()
            state['next_goal'] = start          # fresh goal at each phase start
            while now() - start < seconds and not stop_flag.is_set():
                if not tick(code, f'{now() - start:3.0f}/{seconds:.0f}s'):
                    break
                time.sleep(0.1)
            if stop_flag.is_set() or state['abort']:
                break
            windows[code] = (start, now())
            if code in ('P1', 'BASE'):
                base = baseline(rec, windows[code])
                if base['stops'] or (base['obstacle'] is not None and base['obstacle'] < 2.0):
                    print(f"\n\a!!! BASELINE NOT CLEAR: nearest obstacle {base['obstacle']} m, "
                          f"{base['stops']} stop samples ({', '.join(base['reasons']) or '-'}). "
                          'Clear 0.6 m all round and 2.5 m ahead, then restart for valid results '
                          '(Ctrl+C). !!!', flush=True)
        aborted = bool(stop_flag.is_set() or state['abort'])
        abort_reason = state['abort'] or ('aborted by operator (Ctrl+C / stop)' if aborted else None)
    finally:
        if handle[0] is not None:
            future = handle[0].cancel_goal_async()
            deadline = time.monotonic() + 3
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.05)
        t_end = now()
        if args.stop_distance != DEFAULT_STOP_NOSE_M:
            set_stop_distance(node, DEFAULT_STOP_NOSE_M + NOSE_M - 0.005)   # back to 0.60 from centre
        executor.shutdown(timeout_sec=1)
        sys_executor.shutdown(timeout_sec=1)
        verdict, results = evaluate(rec, t_end, aborted, windows, p2_nose=place['p2'])
        out = Path(args.report_dir) / dt.datetime.now().strftime('%Y%m%d-%H%M%S')
        out.mkdir(parents=True, exist_ok=True)
        report = {
            'test': 'Test 4 non-actuating navigation with real obstacles',
            'evidence_class': 'live robot sensors; navigation output dry (no robot commands)',
            'finished_local': dt.datetime.now().isoformat(timespec='seconds'),
            'duration_s': round(t_end, 1), 'verdict': verdict, 'abort_reason': abort_reason,
            'goals_sent': goals_sent[0], 'checks': results,
            'phases_run': {k: [round(v[0], 1), round(v[1], 1)] for k, v in windows.items()},
            'manual': args.manual, 'stop_distance_from_nose_m': args.stop_distance,
            'distances_note': 'obstacle distances are from the body centre; nose = centre - 0.30 m',
        }
        with rec.lock:
            raw = {k: v[::2] if k in ('status', 'dry', 'nav_in') else v for k, v in rec.data.items()
                   if k != 'odom'}
        (out / 'report.json').write_text(json.dumps({**report, 'raw': raw}, indent=1, default=str))
        lines = [f'# Test 4 result: {verdict}', '', f'- Phases completed: {", ".join(windows) or "none"}',
                 f'- Duration: {t_end:.0f} s, goals sent {goals_sent[0]}',
                 f'- Abort reason: {abort_reason or "none"}', '', '| Result | Check | Detail |',
                 '| --- | --- | --- |'] + [f'| {r["result"]} | {r["check"]} | {r["detail"]} |' for r in results]
        (out / 'summary.md').write_text('\n'.join(lines) + '\n')
        print(f'\n========== TEST 4 {verdict} ==========')
        for r in results:
            print(f'  {r["result"]:<4} {r["check"]}: {r["detail"]}')
        print(f'Report: {out / "summary.md"}\nData:   {out / "report.json"}', flush=True)
        node.destroy_node()
        sys_node.destroy_node()
        rclpy.shutdown(context=ctx)
        rclpy.shutdown(context=sys_ctx)
    return 0 if verdict == 'PASS' else 2 if aborted else 3 if verdict == 'INCONCLUSIVE' else 1


if __name__ == '__main__':
    sys.exit(main())
