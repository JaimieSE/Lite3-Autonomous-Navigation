#!/usr/bin/env python3
"""Checks used by the Lite3 operator scripts (read-only, except `cancel-goals`).

  preflight     environment, robot topics, LiDAR rates and IP-fragment health
  wait-ready    after start: Nav2 lifecycle, data flow and NON-ACTUATION verified
  status        one-shot (or --watch) system status
  cancel-goals  cancel every NavigateToPose goal
  graph-clear   exit 0 when none of this workflow's nodes remain in the ROS graph

Exit codes: 0 ok/ready, 1 not ready / failed, 2 degraded (usable with warnings).
Nothing here publishes velocity, posture or mode commands.
"""

import argparse
from concurrent.futures import ThreadPoolExecutor
import struct
import json
import os
from pathlib import Path
import sys
import threading
import time

GROUP_DOMAIN = 8
SYSTEM_DOMAIN = 64
DRY_TOPIC = '/lite3_locomotion/dry_cmd_vel'
# Nodes started by `navigation.launch.py` (university bringup + navigation + ours).
WORKFLOW_NODES = {
    'lite3_sensor_relay', 'lite3_livox_cloud', 'laser_mapping', 'odom_relay', 'pointcloud_to_laserscan', 'lite3_terrain_monitor',
    'twist_mux', 'nav2_container', 'slam_toolbox', 'controller_server', 'planner_server',
    'bt_navigator', 'velocity_smoother', 'behavior_server', 'smoother_server',
    'waypoint_follower', 'lifecycle_manager_navigation', 'lifecycle_manager_slam',
    'collision_monitor', 'lifecycle_manager_collision_monitor',
}
CALIBRATION_FILE = Path(__file__).resolve().parents[1] / 'log' / 'operator' / 'calibration.json'
NAV_LIFECYCLE = ('controller_server', 'planner_server', 'bt_navigator', 'velocity_smoother',
                 'behavior_server', 'smoother_server', 'collision_monitor')
REQUIRED_PACKAGES = ('lite3_launcher', 'fast_lio', 'lite3_locomotion', 'twist_mux', 'nav2_bringup',
                     'nav2_collision_monitor', 'lite3_livox_cloud',
                     'slam_toolbox', 'pointcloud_to_laserscan', 'livox_ros_driver2', 'nav2_msgs')
ROBOT_TOPICS = ('/joint_states', '/imu/data', '/leg_odom2')
CLOUD_TOPICS = ('/livox/points', '/livox/lidar')
HAZARD_PIPELINE_S = 0.30   # cloud arrival -> hazards at collision_monitor (relay + terrain_monitor), see check_network
FRAG_FIX = ('sudo sysctl -w net.ipv4.ipfrag_high_thresh=67108864 '
            'net.ipv4.ipfrag_low_thresh=50331648 net.ipv4.ipfrag_time=3')
FRAG_RECOMMENDED_HIGH = 32 * 1024 * 1024

GREEN, YELLOW, RED, BOLD, RESET = ('\033[32m', '\033[33m', '\033[31m', '\033[1m', '\033[0m') \
    if sys.stdout.isatty() else ('', '', '', '', '')


class Report:
    def __init__(self):
        self.rows = []

    def add(self, level, item, detail, fix=None):
        self.rows.append({'level': level, 'item': item, 'detail': detail, 'fix': fix})
        colour = {'OK': GREEN, 'NOTE': YELLOW, 'WARN': YELLOW, 'FAIL': RED}[level]
        print(f'  {colour}{level:<4}{RESET} {item:<34} {detail}', flush=True)
        if fix and level != 'OK':
            print(f'       -> {fix}', flush=True)

    def verdict(self):
        if any(r['level'] == 'FAIL' for r in self.rows):
            return 'NOT READY', 1
        if any(r['level'] == 'WARN' for r in self.rows):
            return 'DEGRADED', 2
        return 'READY', 0


# ---------------------------------------------------------------- kernel / network

# Robot telemetry may not pause longer than this (was a fixed 0.25 s). Operator 2026-11-23: 0.35 s allowed
# for the stair test on a link with ~0.3 s dropouts, per command (LITE3_TELEMETRY_MAX_GAP_S); never above 0.35.
TELEMETRY_MAX_GAP_DEFAULT_S = 0.25
TELEMETRY_MAX_GAP_CAP_S = 0.35
try:
    TELEMETRY_MAX_GAP_S = min(max(float(os.environ.get('LITE3_TELEMETRY_MAX_GAP_S',
                                                        TELEMETRY_MAX_GAP_DEFAULT_S)), 0.1), TELEMETRY_MAX_GAP_CAP_S)
except ValueError:
    TELEMETRY_MAX_GAP_S = TELEMETRY_MAX_GAP_DEFAULT_S

def read_sysctl(name):
    try:
        return int(Path('/proc/sys/' + name.replace('.', '/')).read_text().split()[0])
    except (OSError, ValueError):
        return None


def reasm_counters():
    lines = Path('/proc/net/snmp').read_text().splitlines()
    for header, values in zip(lines, lines[1:]):
        if header.startswith('Ip:') and values.startswith('Ip:'):
            table = dict(zip(header.split()[1:], map(int, values.split()[1:])))
            return {k: table[k] for k in ('ReasmReqds', 'ReasmOKs', 'ReasmFails')}
    return None


def frag_memory():
    for line in Path('/proc/net/sockstat').read_text().splitlines():
        if line.startswith('FRAG:'):
            parts = line.split()
            return int(parts[parts.index('memory') + 1])
    return None


# ---------------------------------------------------------------- ROS helpers
class Ros:
    def __init__(self, domain=GROUP_DOMAIN, name='lite3_operator_check'):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        self.rclpy = rclpy
        self.context = Context()
        rclpy.init(args=[], context=self.context, domain_id=domain)
        self.node = Node(name, context=self.context, start_parameter_services=False)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.executor.add_node(self.node)
        self.thread = threading.Thread(target=self._spin, daemon=True)
        self.thread.start()

    def _spin(self):
        try:
            self.executor.spin()
        except Exception:  # noqa: BLE001 - shutdown
            pass

    def nodes(self):
        return {name for name, _ in self.node.get_node_names_and_namespaces()}

    def publishers(self, topic):
        return sorted(p.node_name for p in self.node.get_publishers_info_by_topic(topic))

    def subscribers(self, topic):
        return sorted(p.node_name for p in self.node.get_subscriptions_info_by_topic(topic))

    def measure(self, topics, seconds):
        """Raw (no deserialization) message timing for {topic: (type, reliable)}."""
        from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
        stamps = {t: [] for t in topics}
        subs = []
        for topic, (msg_type, reliable) in topics.items():
            qos = QoSProfile(depth=10, history=HistoryPolicy.KEEP_LAST,
                             reliability=ReliabilityPolicy.RELIABLE if reliable
                             else ReliabilityPolicy.BEST_EFFORT)
            subs.append(self.node.create_subscription(
                msg_type, topic, lambda m, t=topic: stamps[t].append(time.monotonic()), qos, raw=True))
        time.sleep(seconds)
        for sub in subs:
            self.node.destroy_subscription(sub)
        out = {}
        for topic, times in stamps.items():
            gaps = [b - a for a, b in zip(times, times[1:])]
            out[topic] = {'hz': round(len(times) / seconds, 1),
                          'max_gap_s': round(max(gaps), 3) if gaps else None}
        return out

    def stamp_ages(self, topic, seconds):
        """Age on arrival for CDR messages beginning with std_msgs/Header."""
        from rclpy.qos import qos_profile_sensor_data
        ages = []

        def sample(raw):
            if len(raw) >= 12 and raw[:2] in (bytes((0, 1)), bytes((0, 0))):
                sec, nsec = struct.unpack_from('<iI' if raw[1] else '>iI', raw, 4)
                ages.append(time.time() - (sec + nsec * 1e-9))

        sub = self.node.create_subscription(msg_types()[topic][0], topic, sample,
                                            qos_profile_sensor_data, raw=True)
        time.sleep(seconds)
        self.node.destroy_subscription(sub)
        return sorted(ages)

    def lifecycle(self, name, timeout=3.0):
        from lifecycle_msgs.srv import GetState
        client = self.node.create_client(GetState, f'/{name}/get_state')
        try:
            if not client.wait_for_service(timeout_sec=timeout):
                return 'unavailable'
            future = client.call_async(GetState.Request())
            deadline = time.monotonic() + timeout
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.05)
            return future.result().current_state.label if future.done() else 'timeout'
        finally:
            self.node.destroy_client(client)

    def close(self):
        self.executor.shutdown(timeout_sec=1.0)
        self.thread.join(timeout=2.0)       # spin thread must end before the context goes away
        self.node.destroy_node()
        self.rclpy.shutdown(context=self.context)


def msg_types():
    from livox_ros_driver2.msg import CustomMsg
    from nav_msgs.msg import Odometry, OccupancyGrid
    from sensor_msgs.msg import Imu, JointState, LaserScan, PointCloud2
    from std_msgs.msg import String
    return {'/joint_states': (JointState, False), '/imu/data': (Imu, False),
            '/leg_odom2': (Odometry, False), '/livox/points': (PointCloud2, False),
            '/livox/lidar': (CustomMsg, True), '/odom': (Odometry, False),
            '/scan': (LaserScan, False),
            '/lite3_terrain/status': (String, False), '/map': (OccupancyGrid, True)}


# ---------------------------------------------------------------- commands
def check_environment(report):
    domain = os.environ.get('ROS_DOMAIN_ID')
    report.add('OK' if os.environ.get('ROS_DISTRO') == 'humble' else 'FAIL', 'ROS 2 distribution',
               os.environ.get('ROS_DISTRO', 'not sourced'), 'source operator/env.sh')
    report.add('OK' if domain == str(GROUP_DOMAIN) else 'FAIL', 'ROS_DOMAIN_ID', str(domain),
               'source operator/env.sh (group domain is 8)')
    report.add('FAIL' if os.environ.get('ROS_LOCALHOST_ONLY') == '1' else 'OK', 'ROS_LOCALHOST_ONLY',
               os.environ.get('ROS_LOCALHOST_ONLY', 'unset'), 'unset ROS_LOCALHOST_ONLY')
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
    missing = []
    for package in REQUIRED_PACKAGES:
        try:
            get_package_share_directory(package)
        except PackageNotFoundError:
            missing.append(package)
    report.add('FAIL' if missing else 'OK', 'ROS packages', 'missing: ' + ', '.join(missing)
               if missing else f'{len(REQUIRED_PACKAGES)} found',
               'run operator/build.sh; university packages come from ~/ros2_ws/install')


SYSTEM_DOMAIN = 64
TERRAIN_STALE_S = 0.5                             # terrain_monitor marks inputs stale after this


def check_network(report, ros, seconds, direct=False):
    high = read_sysctl('net.ipv4.ipfrag_high_thresh')
    timeout = read_sysctl('net.ipv4.ipfrag_time')
    memory_before = frag_memory()
    before = reasm_counters()
    types = msg_types()
    # Direct mode: the clouds are read in domain 64 by sensor_relay, so measure them there (pulling the
    # bridged copies here would only make the bridge send them again over the robot link).
    rates = {} if direct else ros.measure({t: types[t] for t in ROBOT_TOPICS + CLOUD_TOPICS}, seconds)
    if not direct:
        after = reasm_counters()
        memory_after = frag_memory()
    if not direct:
        for topic in ROBOT_TOPICS:
            r = rates[topic]
            ok = r['hz'] >= 100 and (r['max_gap_s'] or 9) < TELEMETRY_MAX_GAP_S
            report.add('OK' if ok else 'FAIL', f'robot telemetry {topic}',
                       f"{r['hz']} Hz, max gap {r['max_gap_s']} s",
                       'check the robot and lab telemetry bridge')
    if direct:
        sys_ros = Ros(domain=SYSTEM_DOMAIN, name='lite3_operator_check_direct')
        cloud_ros = Ros(domain=SYSTEM_DOMAIN, name='lite3_operator_check_clouds')
        try:
            time.sleep(2.5)
            # Measure joints while native scans are also flowing, on separate participants.
            with ThreadPoolExecutor(max_workers=1) as pool:
                telemetry = pool.submit(sys_ros.measure, {t: types[t] for t in ROBOT_TOPICS}, seconds)
                direct_rates = cloud_ros.measure({'/livox/lidar': (types['/livox/lidar'][0], False)}, seconds)
                direct_rates.update(telemetry.result())
            after = reasm_counters()
            memory_after = frag_memory()
            ages = cloud_ros.stamp_ages('/livox/lidar', 3.0)
        finally:
            cloud_ros.close()
            sys_ros.close()
        for topic in ROBOT_TOPICS:
            r = direct_rates[topic]
            ok = r['hz'] >= 100 and (r['max_gap_s'] or 9) < TELEMETRY_MAX_GAP_S
            report.add('OK' if ok else 'FAIL', f'robot telemetry {topic} (direct, domain 64)',
                       f"{r['hz']} Hz, max gap {r['max_gap_s']} s",
                       'direct telemetry gap at group8: compare source and receiver during the same interval')
        rates = {**rates, **{f'{t}@64': r for t, r in direct_rates.items()}}
    for topic in (('/livox/lidar',) if direct else CLOUD_TOPICS):
        r = direct_rates[topic] if direct else rates[topic]
        level = 'OK' if r['hz'] >= 8 and (r['max_gap_s'] or 9) < 0.5 else \
            'WARN' if r['hz'] >= 5 else 'FAIL'
        report.add(level, f'LiDAR {topic}' + (' (direct, domain 64)' if direct else ''),
                   f"{r['hz']} Hz, max gap {r['max_gap_s']} s",
                   'see "LiDAR rate low" in operator/README.md (IP fragment check below)')
    if not direct:
        ages = ros.stamp_ages('/livox/points', 3.0)
    if ages:
        p95 = ages[min(len(ages) - 1, int(0.95 * len(ages)))]
        ok = 0.0 <= p95 <= 1.0
        report.add('OK' if ok else 'WARN', 'LiDAR data age on arrival (p95)', f'{p95:.3f} s over {len(ages)} clouds',
                   'clock offset or long delay: the stop boundary grows with it')
        if ok:
            try:
                sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src' / 'lite3_locomotion'))
                from lite3_locomotion import collision_profile as CP
                # The live readiness check (nav_steps._boundary_problems) measures the delay until the
                # HAZARDS reach collision_monitor: cloud arrival + sensor_relay + terrain_monitor queue and
                # processing. Measured 2026-11-19 on healthy data: hazards p95 0.27-0.31 s (3 s windows up
                # to 0.46 s) vs cloud arrival p95 0.13-0.15 s. Sizing the boundary from arrival alone made
                # healthy data fail that check ("0.21 m < 0.32 m required NOW"), so add the pipeline.
                CP.update_calibration(CALIBRATION_FILE, sources=[f'preflight {time.strftime("%F %T")}'],
                                      sensing_age_s=round(p95 + HAZARD_PIPELINE_S, 3))
            except (ImportError, ValueError, OSError) as error:
                report.add('WARN', 'calibration update', str(error))
    d_ok = after['ReasmOKs'] - before['ReasmOKs']
    d_fail = after['ReasmFails'] - before['ReasmFails']
    ratio = d_fail / (d_ok + d_fail) if d_ok + d_fail else 0.0
    saturated = high and max(memory_before or 0, memory_after or 0) >= 0.9 * high
    report.add('FAIL' if saturated or ratio > 0.2 else 'WARN' if ratio > 0.02 else 'OK',
               'IP fragment reassembly',
               f'{d_ok} ok / {d_fail} failed in {seconds:.0f} s; cache {memory_after} of {high} bytes',
               f'fragment cache overloaded; apply the fix in operator/README.md: {FRAG_FIX}')
    # Advisory only: the measured reassembly result above decides readiness. The default 4 MB
    # cache has collapsed under sustained two-stream LiDAR load (2026-11-13), so the fix is advised.
    report.add('OK' if high and high >= FRAG_RECOMMENDED_HIGH and timeout and timeout <= 5 else 'NOTE',
               'ipfrag sysctls (preventive)', f'high_thresh={high} time={timeout}s',
               f'advised before long runs (README "LiDAR fragment fix"): {FRAG_FIX}')
    return rates, {'reasm_ok': d_ok, 'reasm_fail': d_fail, 'frag_memory': memory_after,
                   'ipfrag_high_thresh': high, 'ipfrag_time': timeout}


def cmd_preflight(args):
    print(f'{BOLD}Lite3 preflight (group domain {GROUP_DOMAIN}, read-only){RESET}')
    report = Report()
    if args.domain is None:
        check_environment(report)
        if any(r['level'] == 'FAIL' for r in report.rows):
            return finish(report, args)
    ros = Ros(domain=args.domain if args.domain is not None else GROUP_DOMAIN)
    try:
        time.sleep(2.5)   # discovery
        nodes = ros.nodes()
        report.add('OK' if 'lite3_domain_bridge_8' in nodes else 'FAIL', 'university domain bridge',
                   'lite3_domain_bridge_8 present' if 'lite3_domain_bridge_8' in nodes else 'absent',
                   'robot-side bridge not visible: robot/system computer off or lab bridge down')
        running = sorted(WORKFLOW_NODES & nodes)
        report.add('WARN' if running else 'OK', 'navigation stack', 'already running: ' + ', '.join(running)
                   if running else 'not running', 'use operator/status.sh, or stop_navigation.sh first')
        cmd_pubs = ros.publishers('/cmd_vel')
        report.add('OK' if not cmd_pubs else 'FAIL', 'robot /cmd_vel publishers (group)',
                   str(cmd_pubs or 'none'), 'something is commanding the robot; find and stop it')
        tf = ros.publishers('/tf_static')
        report.add('OK' if tf else 'FAIL', '/tf_static (sensor extrinsics)', str(tf or 'none'),
                   'driver static TFs not bridged; check robot-side drivers')
        rates, frag = check_network(report, ros, args.seconds,
                                    direct=getattr(args, 'direct_telemetry', False))
    finally:
        ros.close()
    return finish(report, args, {'rates': rates, 'fragments': frag})


def cmd_wait_ready(args):
    print(f'{BOLD}Waiting for navigation stack (up to {args.timeout:.0f} s){RESET}')
    ros = Ros()
    deadline = time.monotonic() + args.timeout
    lio_node = 'group8_lio_outputs_8' if os.environ.get('LITE3_LIO_SOURCE', 'local') == 'system' else 'laser_mapping'
    try:
        while True:
            states = {n: ros.lifecycle(n, 2.0) for n in NAV_LIFECYCLE}
            nodes = ros.nodes()
            done = all(s == 'active' for s in states.values()) and {
                'slam_toolbox', 'twist_mux', 'lite3_terrain_monitor', lio_node,
                'collision_monitor', 'lite3_livox_cloud', 'lite3_sensor_relay'} <= nodes
            if done or time.monotonic() > deadline:
                break
            print(f'  ... {sum(s == "active" for s in states.values())}/{len(states)} Nav2 nodes active',
                  flush=True)
            time.sleep(3)
        report = Report()
        for name, state in states.items():
            report.add('OK' if state == 'active' else 'FAIL', f'Nav2 {name}', state,
                       'see launch log; operator/README.md "Nav2 not active"')
        scan_nodes = () if lio_node == 'group8_lio_outputs_8' else ('pointcloud_to_laserscan',)
        for name in (lio_node, 'odom_relay', *scan_nodes, 'slam_toolbox',
                     'lite3_terrain_monitor', 'lite3_livox_cloud', 'lite3_sensor_relay', 'twist_mux', 'collision_monitor'):
            report.add('OK' if name in nodes else 'FAIL', f'node {name}',
                       'running' if name in nodes else 'missing', 'see launch log')
        check_flow(report, ros, 6.0)
        (check_physical_path if args.mode == 'physical' else check_non_actuation)(report, ros)
    finally:
        ros.close()
    return finish(report, args)


def check_flow(report, ros, seconds):
    types = msg_types()
    rates = ros.measure({t: types[t] for t in ('/odom', '/scan', '/lite3_terrain/status')}, seconds)
    for topic, r in rates.items():
        level = 'OK' if r['hz'] >= 8 and (r['max_gap_s'] or 9) < 0.5 else \
            'WARN' if r['hz'] >= 3 else 'FAIL'
        report.add(level, f'data {topic}', f"{r['hz']} Hz, max gap {r['max_gap_s']} s",
                   'LiDAR input degraded: run operator/preflight.sh')
    services = {name for name, _ in ros.node.get_service_names_and_types()}
    server = '/navigate_to_pose/_action/send_goal' in services
    report.add('OK' if server else 'FAIL', 'NavigateToPose action', 'available' if server else 'missing',
               'bt_navigator not active')
    return rates


def check_non_actuation(report, ros):
    cmd_pubs = ros.publishers('/cmd_vel')
    report.add('OK' if not cmd_pubs else 'FAIL', 'NON-ACTUATING: /cmd_vel publishers',
               str(cmd_pubs or 'none (robot command input not driven)'),
               'STOP NOW: operator/stop_navigation.sh; use the robot e-stop if it moves')
    dry = ros.publishers(DRY_TOPIC)
    report.add('OK' if dry == ['collision_monitor'] else 'FAIL', 'NON-ACTUATING: final output',
               f'twist_mux -> collision_monitor -> {DRY_TOPIC}' if dry == ['collision_monitor']
               else f'{DRY_TOPIC} publishers {dry}',
               'restart with operator/start_navigation.sh (dry only)')
    dry_subs = [s for s in ros.subscribers(DRY_TOPIC) if not s.startswith('lite3_')]
    report.add('OK' if not dry_subs else 'FAIL', 'NON-ACTUATING: dry topic consumers',
               str(dry_subs or 'none'), 'unknown node consumes the dry topic; investigate')


def check_physical_path(report, ros):
    """Physical mode: twist_mux is the only /cmd_vel source and the dead-man lock is wired."""
    cmd_pubs = ros.publishers('/cmd_vel')
    report.add('OK' if cmd_pubs == ['collision_monitor'] else 'FAIL', 'PHYSICAL: /cmd_vel publishers',
               str(cmd_pubs or 'none'), 'only collision_monitor may drive /cmd_vel; stop and investigate')
    mux = ros.publishers('/lite3_ops/mux_out')
    cm_in = 'collision_monitor' in ros.subscribers('/lite3_ops/mux_out')
    report.add('OK' if mux == ['twist_mux'] and cm_in else 'FAIL', 'PHYSICAL: command chain',
               'twist_mux -> collision_monitor -> /cmd_vel' if mux == ['twist_mux'] and cm_in
               else f'mux_out publishers {mux}, collision_monitor subscribed {cm_in}',
               'restart with operator/start_navigation.sh --physical')
    # (the terrain proximity lock /lite3_terrain/stop was removed 2026-11-24)
    mux_subs = [t for t in ('/lite3_ops/hold', '/lite3_ops/test_cmd_vel', '/lite3_ops/stop_cmd_vel')
                if 'twist_mux' not in ros.subscribers(t)]
    report.add('OK' if not mux_subs else 'FAIL', 'PHYSICAL: dead-man lock wired',
               'twist_mux has all lock/stop inputs' if not mux_subs else f'missing {mux_subs}',
               'restart with operator/start_navigation.sh --physical')
    foreign = {t: [p for p in ros.publishers(t) if p != 'lite3_movement_test']
               for t in ('/lite3_ops/hold', '/lite3_ops/test_cmd_vel', '/lite3_ops/stop_cmd_vel')}
    foreign = {t: p for t, p in foreign.items() if p}
    joy = ros.publishers('/joy_cmd_vel')
    report.add('OK' if not foreign and not joy else 'FAIL', 'PHYSICAL: no foreign command sources',
               'none' if not foreign and not joy else f'{foreign} joystick {joy}',
               'another program could command the robot; stop it first')
    runner = 'lite3_movement_test' in ros.publishers('/lite3_ops/hold')
    report.add('OK', 'PHYSICAL: dead-man', 'held by a supervised test' if runner else
               'LOCKED (no supervised test running; Nav2 cannot move the robot)')


def cmd_status(args):
    while True:
        print(f'{BOLD}Lite3 navigation status {time.strftime("%H:%M:%S")}{RESET}')
        report = Report()
        ros = Ros()
        try:
            time.sleep(2.0)
            nodes = ros.nodes()
            running = sorted(WORKFLOW_NODES & nodes)
            report.add('OK' if running else 'WARN', 'workflow nodes', f'{len(running)} running'
                       if running else 'navigation not running', 'operator/start_navigation.sh')
            if running:
                states = {n: ros.lifecycle(n, 2.0) for n in NAV_LIFECYCLE}
                active = sum(s == 'active' for s in states.values())
                report.add('OK' if active == len(states) else 'FAIL', 'Nav2 lifecycle',
                           f'{active}/{len(states)} active', 'operator/README.md "Nav2 not active"')
                check_flow(report, ros, 4.0)
                (check_physical_path if args.mode == 'physical' else check_non_actuation)(report, ros)
            else:
                cmd_pubs = ros.publishers('/cmd_vel')
                report.add('OK' if not cmd_pubs else 'FAIL', '/cmd_vel publishers', str(cmd_pubs or 'none'))
        finally:
            ros.close()
        code = finish(report, args)
        if not args.watch:
            return code
        time.sleep(max(0.0, args.watch - 6))


def cmd_cancel_goals(args):
    from action_msgs.srv import CancelGoal
    ros = Ros(name='lite3_operator_cancel')
    try:
        client = ros.node.create_client(CancelGoal, '/navigate_to_pose/_action/cancel_goal')
        if not client.wait_for_service(timeout_sec=args.timeout):
            print('  no NavigateToPose server reachable (nothing to cancel)')
            return 0
        future = client.call_async(CancelGoal.Request())   # zero id + zero stamp = cancel all
        deadline = time.monotonic() + args.timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.05)
        if not future.done():
            print('  cancel request timed out')
            return 1
        print(f'  cancelled {len(future.result().goals_canceling)} goal(s)')
        return 0
    finally:
        ros.close()


def cmd_graph_clear(args):
    deadline = time.monotonic() + args.timeout
    remaining = []
    while time.monotonic() < deadline:
        ros = Ros(name='lite3_operator_graph')
        try:
            time.sleep(2.0)
            remaining = sorted(WORKFLOW_NODES & ros.nodes())
        finally:
            ros.close()
        if not remaining:
            print('  ROS graph: no workflow nodes remain')
            return 0
        time.sleep(2.0)
    print(f'  ROS graph still lists: {", ".join(remaining)} (may be stale discovery or another user)')
    return 1


def finish(report, args, extra=None):
    verdict, code = report.verdict()
    colour = {0: GREEN, 1: RED, 2: YELLOW}[code]
    print(f'{BOLD}RESULT: {colour}{verdict}{RESET}', flush=True)
    if getattr(args, 'json', None):
        Path(args.json).parent.mkdir(parents=True, exist_ok=True)
        Path(args.json).write_text(json.dumps({'verdict': verdict, 'time': time.strftime('%F %T'),
                                               'checks': report.rows, **(extra or {})}, indent=1))
    return code


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = parser.add_subparsers(dest='command', required=True)
    p = sub.add_parser('preflight')
    p.add_argument('--seconds', type=float, default=8.0)
    p.add_argument('--json')
    p.add_argument('--domain', type=int, default=None, help=argparse.SUPPRESS)   # validation only
    p.add_argument('--direct-telemetry', action='store_true',
                   help='test runner reads robot telemetry in domain 64 (bypasses the degraded relay)')
    p = sub.add_parser('wait-ready')
    p.add_argument('--timeout', type=float, default=120.0)
    p.add_argument('--json')
    p.add_argument('--mode', choices=('dry', 'physical'), default='dry')
    p = sub.add_parser('status')
    p.add_argument('--mode', choices=('dry', 'physical'), default='dry')
    p.add_argument('--watch', type=float, default=0.0, help='repeat every N seconds')
    p.add_argument('--json')
    p = sub.add_parser('cancel-goals')
    p.add_argument('--timeout', type=float, default=6.0)
    p = sub.add_parser('graph-clear')
    p.add_argument('--timeout', type=float, default=20.0)
    args = parser.parse_args()
    handler = {'preflight': cmd_preflight, 'wait-ready': cmd_wait_ready, 'status': cmd_status,
               'cancel-goals': cmd_cancel_goals, 'graph-clear': cmd_graph_clear}[args.command]
    try:
        return handler(args)
    except KeyboardInterrupt:
        print('\ninterrupted')
        return 1


if __name__ == '__main__':
    sys.exit(main())
