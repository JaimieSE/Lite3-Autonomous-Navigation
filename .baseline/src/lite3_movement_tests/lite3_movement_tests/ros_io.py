"""ROS 2 interface to the EXISTING Lite3 bridge: subscribers plus, only when allowed, publishers.

Live wiring (verified read-only on 2026-11-13):
  group domain 8:  lite3_domain_bridge_8 subscribes /cmd_vel (reliable, volatile) and
                   relays telemetry (/joint_states, /leg_odom2, /imu/data, /handle_state,
                   /sensor_status/*) from the system computer.
  system domain 64: lite3_domain_bridge_64 republishes /cmd_vel to the vendor
                   motion_sender; motion_receiver publishes the telemetry.
Velocity goes through the existing domain-8 bridge (as in both supervised trials).
The posture toggle is published once, volatile, directly to the motion_sender's
/simple_cmd in domain 64 - the path used in both trials. Publishing it in domain 8
is avoided because that bridge subscription is TRANSIENT_LOCAL, so a durable
toggle could be replayed later.
"""

from dataclasses import dataclass, replace
import threading
import time

from .telemetry import Telemetry, TwistSample

POSTURE_TOGGLE = 0x21010202   # documented stand/sit toggle; also used by university drain_patrol
# Motion Host Communication Interface V1.1.0 s1.2.5: gait switches, simple commands (value 0, type 0),
# valid when NOT in AI state and in move mode. Same /simple_cmd path as the posture toggle.
GAITS = {'flat_slow': 0x21010300, 'flat_medium': 0x21010307, 'flat_fast': 0x21010303,
         'rug_general': 0x21010401, 'rug_grip': 0x21010402, 'rug_hstep': 0x21010407}
NODE_NAME = 'lite3_movement_test'
SENSORS = ('imu', 'joint', 'odom2', 'odom', 'ultrasound')


@dataclass(frozen=True)
class Wiring:
    telemetry_domain: int
    velocity_domain: int
    system_domain: int
    velocity_topic: str = '/cmd_vel'
    posture_topic: str = '/simple_cmd'
    motion_node: str = 'motion_sender'
    system_cmd_publisher: str = None      # bridge expected to publish /cmd_vel in system domain
    velocity_bridge_subscriber: str = None
    robot_topic: str = '/cmd_vel'         # motion_sender input observed in the system domain


LIVE_WIRING = Wiring(telemetry_domain=8, velocity_domain=8, system_domain=64,
                     system_cmd_publisher='lite3_domain_bridge_64',
                     velocity_bridge_subscriber='lite3_domain_bridge_8')


def sim_wiring(domain):
    return Wiring(domain, domain, domain)


class RosRobot:
    node_name = NODE_NAME      # subclasses that must not share the test runner's name override this

    def __init__(self, wiring, allow_commands, cancel_flag=None):
        import rclpy
        from rclpy.context import Context
        from rclpy.executors import SingleThreadedExecutor
        from rclpy.node import Node
        from rclpy.qos import (QoSProfile, ReliabilityPolicy, DurabilityPolicy,
                               HistoryPolicy, qos_profile_sensor_data)
        from rclpy.signals import SignalHandlerOptions
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Imu, JointState
        from std_msgs.msg import Bool

        self._rclpy = rclpy
        self.wiring = wiring
        self.can_actuate = bool(allow_commands)
        self.telemetry = Telemetry()
        self._cancel = cancel_flag or threading.Event()
        self._Twist = Twist
        self._volatile = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                    durability=DurabilityPolicy.VOLATILE,
                                    history=HistoryPolicy.KEEP_LAST)
        self._nodes = {}
        self._contexts = []
        for domain in sorted({wiring.telemetry_domain, wiring.velocity_domain, wiring.system_domain}):
            context = Context()
            rclpy.init(args=[], context=context, domain_id=domain,
                       signal_handler_options=SignalHandlerOptions.NO)
            name = self.node_name if domain == wiring.velocity_domain else f'{self.node_name}_d{domain}'
            node = Node(name, context=context, use_global_arguments=False,
                        start_parameter_services=False)
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            thread = threading.Thread(target=self._spin, args=(executor,), daemon=True)
            self._nodes[domain] = node
            self._contexts.append((context, node, executor, thread))
        t = self.telemetry
        now = time.monotonic
        tel = self._nodes[wiring.telemetry_domain]
        sensor = qos_profile_sensor_data   # best effort: matches reliable and best-effort publishers
        tel.create_subscription(JointState, '/joint_states',
                                lambda m: t.on_joint(now(), m.name, m.position), sensor)
        tel.create_subscription(Odometry, '/leg_odom2', lambda m: t.on_odom(
            now(), m.pose.pose.position.x, m.pose.pose.position.y, m.pose.pose.position.z,
            m.pose.pose.orientation.x, m.pose.pose.orientation.y, m.pose.pose.orientation.z,
            m.pose.pose.orientation.w, m.twist.twist.linear.x, m.twist.twist.linear.y,
            m.twist.twist.angular.z), sensor)
        tel.create_subscription(Imu, '/imu/data', lambda m: t.on_imu(
            now(), m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w,
            (m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z),
            (m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z)), sensor)
        tel.create_subscription(Twist, '/handle_state', lambda m: t.on_handle(
            now(), m.linear.x, m.linear.y, m.angular.z), sensor)
        for name in SENSORS:
            tel.create_subscription(Bool, f'/sensor_status/{name}_isalive',
                                    lambda m, n=name: t.on_sensor_alive(now(), n, m.data), sensor)
        system = self._nodes[wiring.system_domain]
        system.create_subscription(Twist, wiring.robot_topic, lambda m: t.on_system_cmd(
            now(), m.linear.x, m.linear.y, m.angular.z), sensor)
        self._posture_seen = []
        if self.can_actuate:
            from transfer_interfaces.msg import MotionSimpleCMD
            self._Simple = MotionSimpleCMD
            system.create_subscription(MotionSimpleCMD, wiring.posture_topic,
                                       lambda m: self._posture_seen.append((now(), m.cmd_code)),
                                       self._volatile)
            self._velocity_pub = self._nodes[wiring.velocity_domain].create_publisher(
                Twist, wiring.velocity_topic, self._volatile)
        self._extra_setup()
        for *_, thread in self._contexts:
            thread.start()

    def _extra_setup(self):
        """Hook for subclasses: create more endpoints before the executors start."""

    def _spin(self, executor):
        try:
            executor.spin()
        except Exception:  # noqa: BLE001 - executor shutdown during close
            pass

    # Interface used by SequenceRunner ------------------------------------
    def now(self):
        return time.monotonic()

    def wait(self, seconds):
        time.sleep(max(0.0, seconds))

    def cancelled(self):
        return self._cancel.is_set()

    def send_velocity(self, vx, wz):
        if not self.can_actuate:
            raise RuntimeError('read-only interface has no publishers')
        msg = self._Twist()
        msg.linear.x = float(vx)
        msg.angular.z = float(wz)
        self._velocity_pub.publish(msg)

    def send_posture_toggle(self):
        """Publish exactly one volatile toggle; report delivery evidence, never success."""
        return self.send_simple_command(POSTURE_TOGGLE)

    def send_gait(self, name):
        """One documented gait switch (GAITS); the deployed bridge does not report the active gait."""
        return self.send_simple_command(GAITS[name])

    def send_simple_command(self, code):
        """Publish exactly one volatile simple command; report delivery evidence, never success."""
        if not self.can_actuate:
            raise RuntimeError('read-only interface has no publishers')
        node = self._nodes[self.wiring.system_domain]
        pub = node.create_publisher(self._Simple, self.wiring.posture_topic, self._volatile)
        try:
            deadline = time.monotonic() + 3.0
            while time.monotonic() < deadline:
                subs = node.get_subscriptions_info_by_topic(self.wiring.posture_topic)
                if any(s.node_name == self.wiring.motion_node for s in subs) and \
                        pub.get_subscription_count() >= 2:   # motion_sender + our observer
                    break
                time.sleep(0.05)
            else:
                return {'published': False, 'definitely_not_sent': True,
                        'reason': 'motion_sender not matched on posture topic'}
            msg = self._Simple()
            msg.cmd_code, msg.size, msg.type = code, 0, 0
            sent = time.monotonic()
            pub.publish(msg)
            deadline = sent + 1.0
            while time.monotonic() < deadline and not any(
                    t >= sent and c == code for t, c in self._posture_seen):
                time.sleep(0.01)
            return {'published': True, 'cmd_code': hex(code),
                    'matched_subscribers': pub.get_subscription_count(),
                    'observed_in_system_domain': any(t >= sent for t, _ in self._posture_seen)}
        finally:
            node.destroy_publisher(pub)

    def command_path_health(self):
        """Exclusive, expected wiring of every motion input; any surprise is unhealthy."""
        w = self.wiring
        problems = []
        vel = self._nodes[w.velocity_domain]
        system = self._nodes[w.system_domain]

        def names(infos):
            return sorted(f'{i.node_namespace.rstrip("/")}/{i.node_name}' for i in infos)

        ours = f'/{NODE_NAME}'
        vel_pubs = names(vel.get_publishers_info_by_topic(w.velocity_topic))
        allowed_vel = [ours] if self.can_actuate else []
        if w.velocity_domain == w.system_domain and w.system_cmd_publisher:
            allowed_vel.append('/' + w.system_cmd_publisher)
        if sorted(set(vel_pubs)) != sorted(set(allowed_vel)) and \
                not (not self.can_actuate and not vel_pubs):
            problems.append(f'{w.velocity_topic} publishers {vel_pubs} != expected {allowed_vel}')
        vel_subs = names(vel.get_subscriptions_info_by_topic(w.velocity_topic))
        needed = w.velocity_bridge_subscriber or w.motion_node
        if '/' + needed not in vel_subs:
            problems.append(f'{needed} not subscribed to {w.velocity_topic}')
        if w.system_domain != w.velocity_domain:
            sys_pubs = names(system.get_publishers_info_by_topic(w.velocity_topic))
            if sys_pubs != ['/' + w.system_cmd_publisher]:
                problems.append(f'system {w.velocity_topic} publishers {sys_pubs}')
        sys_subs = names(system.get_subscriptions_info_by_topic(w.velocity_topic))
        if '/' + w.motion_node not in sys_subs:
            problems.append(f'{w.motion_node} not subscribed to {w.velocity_topic}')
        for topic in ('/cmd_vel_corrected', '/complex_cmd'):
            for node in {vel, system}:
                pubs = names(node.get_publishers_info_by_topic(topic))
                if pubs:
                    problems.append(f'unexpected publishers on {topic}: {pubs}')
        system_ours = '/' + system.get_name()
        allowed_posture = {system_ours} | ({'/' + w.system_cmd_publisher} if w.system_cmd_publisher else set())
        posture_pubs = names(system.get_publishers_info_by_topic(w.posture_topic))
        if set(posture_pubs) - allowed_posture:
            problems.append(f'unexpected {w.posture_topic} publishers {posture_pubs}')
        if w.velocity_domain != w.system_domain:
            group_posture = names(vel.get_publishers_info_by_topic(w.posture_topic))
            if group_posture:
                problems.append(f'group-domain {w.posture_topic} publishers {group_posture}')
        return not problems, {'problems': problems, 'velocity_publishers': vel_pubs,
                              'system_subscribers': sys_subs}

    def close(self):
        for context, node, executor, thread in self._contexts:
            executor.shutdown(timeout_sec=1.0)
            thread.join(timeout=2.0)
            node.destroy_node()
            self._rclpy.shutdown(context=context)


# ---------------------------------------------------------------------------------------------
# Navigation mode: commands go through the EXISTING twist_mux (physical config) -> /cmd_vel ->
# domain bridge -> motion_sender. This class never publishes /cmd_vel itself.
OPS_TEST_TOPIC = '/lite3_ops/test_cmd_vel'
OPS_STOP_TOPIC = '/lite3_ops/stop_cmd_vel'
OPS_HOLD_TOPIC = '/lite3_ops/hold'
MUX_OUT_TOPIC = '/lite3_ops/mux_out'           # twist_mux -> collision_monitor
CM_NODE = 'collision_monitor'                  # the ONLY publisher of /cmd_vel
TERRAIN_NODE = 'lite3_terrain_monitor'
NAV2_PUBLISHERS = {'/velocity_smoother', '/behavior_server'}
SMOOTHER_NODE = 'velocity_smoother'

NAV_WIRING = Wiring(telemetry_domain=8, velocity_domain=8, system_domain=64,
                    velocity_topic=OPS_TEST_TOPIC, system_cmd_publisher='lite3_domain_bridge_64',
                    velocity_bridge_subscriber='lite3_domain_bridge_8')


def direct_telemetry(wiring):
    """Same wiring, but robot telemetry (joints, odometry, IMU, handle, sensor status) is read
    at its source in the system domain instead of through the group bridge relay. Read-only."""
    return replace(wiring, telemetry_domain=wiring.system_domain)


class NavRobot(RosRobot):
    """Supervised navigation-mode interface (see nav_steps.py for the command path)."""

    def __init__(self, wiring=NAV_WIRING, allow_commands=False, cancel_flag=None):
        self._goal = {'handle': None, 'result': None, 'recoveries': 0}
        self._last_feed = -1.0
        super().__init__(wiring, allow_commands, cancel_flag)

    def _extra_setup(self):
        import json
        import numpy as np
        from rclpy.action import ActionClient
        from rclpy.qos import qos_profile_sensor_data
        from geometry_msgs.msg import Twist
        from nav2_msgs.action import NavigateToPose
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import PointCloud2
        from std_msgs.msg import Bool, String
        from tf2_ros import Buffer, TransformListener
        from .nav_steps import FOOTPRINT_HALF

        node = self._nodes[self.wiring.velocity_domain]
        t = self.telemetry
        now = time.monotonic

        def on_status(msg):
            try:
                s = json.loads(msg.data)
            except ValueError:
                return
            stop = s.get('stop') or {}
            t.on_terrain(now(), stop.get('active', True), stop.get('reason'),
                         (s.get('corridor') or {}).get('nearest_obstacle_m'), stop.get('ahead_m'),
                         s.get('problems'), s.get('processing_s'))

        def on_hazards(msg):
            t.hazard_age.add(now(), time.time() - (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9))
            pts = np.frombuffer(bytes(msg.data), '<f4')
            if msg.point_step != 12 or pts.size % 3:
                return
            xy = pts.reshape(-1, 3)[:, :2]
            if not len(xy):
                t.on_clearance(now(), float('inf'))
                return
            dx = np.maximum(np.abs(xy[:, 0]) - FOOTPRINT_HALF[0], 0.0)
            dy = np.maximum(np.abs(xy[:, 1]) - FOOTPRINT_HALF[1], 0.0)
            t.on_clearance(now(), float(np.hypot(dx, dy).min()))

        node.create_subscription(String, '/lite3_terrain/status', on_status, 10)
        node.create_subscription(PointCloud2, '/lite3_terrain/hazards', on_hazards, 5)
        node.create_subscription(Bool, '/lite3_terrain/stop',
                                 lambda m: t.terrain_stop.add(now(), bool(m.data)), 10)
        node.create_subscription(Twist, MUX_OUT_TOPIC, lambda m: t.mux_out.add(
            now(), TwistSample(m.linear.x, m.linear.y, m.angular.z)), 50)
        from rcl_interfaces.msg import Log

        def on_log(msg):
            if msg.name == CM_NODE and 'Robot to' in msg.msg:
                kind = ('stop' if 'Robot to stop' in msg.msg else
                        'continue' if 'continue' in msg.msg else 'other')
                t.cm_events.add(now(), (kind, msg.msg))
        node.create_subscription(Log, '/rosout', lambda m: on_log(m), 50)
        node.create_subscription(Twist, '/nav_cmd_vel', lambda m: t.on_nav_cmd(
            now(), m.linear.x, m.linear.y, m.angular.z), 50)
        # value = stamp age on arrival (wall clock; robot computer and this PC are NTP-synced, the
        # Livox IMU measured 5 ms old in domain 64): how far SLAM odometry / odom TF trails reality
        node.create_subscription(Odometry, '/odom', lambda m: t.lidar_odom.add(
            now(), time.time() - (m.header.stamp.sec + m.header.stamp.nanosec * 1e-9)),
            qos_profile_sensor_data)
        self._tf = Buffer()
        self._tf_listener = TransformListener(self._tf, node)
        self._NavigateToPose = NavigateToPose
        self._action = ActionClient(node, NavigateToPose, '/navigate_to_pose')
        from nav2_msgs.action import Spin
        self._Spin = Spin
        self._spin_action = ActionClient(node, Spin, '/spin')       # behavior_server (in place, <= 0.2 rad/s)
        if self.can_actuate:
            from nav2_msgs.msg import SpeedLimit
            from rcl_interfaces.srv import SetParameters
            self._Bool, self._SpeedLimit = Bool, SpeedLimit
            self._stop_pub = node.create_publisher(Twist, OPS_STOP_TOPIC, self._volatile)
            self._hold_pub = node.create_publisher(Bool, OPS_HOLD_TOPIC, self._volatile)
            self._speed_pub = node.create_publisher(SpeedLimit, '/speed_limit', 10)
            self._param_client = node.create_client(SetParameters,
                                                    f'/{TERRAIN_NODE}/set_parameters')

    def stop_path_status(self):
        """Is the commanded-stop path discovered end to end? Publishers created moments ago drop
        everything sent before DDS matches them (M10 zero_delivery 2026-11-18: started 0.35 s after
        launch, first zero at 0.53 s, 2 of 20 delivered). Read-only graph queries."""
        status = {'stop_input_subscribers': 0, 'hold_subscribers': 0, 'robot_cmd_publishers': [],
                  'problems': []}
        if not self.can_actuate:
            status['problems'].append('read-only interface has no publishers')
            return status
        status['stop_input_subscribers'] = self._stop_pub.get_subscription_count()
        status['hold_subscribers'] = self._hold_pub.get_subscription_count()
        system = self._nodes[self.wiring.system_domain]
        status['robot_cmd_publishers'] = sorted(
            '/' + i.node_name for i in system.get_publishers_info_by_topic(self.wiring.robot_topic))
        if status['stop_input_subscribers'] < 1:
            status['problems'].append(f'twist_mux not matched on {OPS_STOP_TOPIC}')
        if status['hold_subscribers'] < 1:
            status['problems'].append(f'twist_mux not matched on {OPS_HOLD_TOPIC}')
        want = '/' + self.wiring.system_cmd_publisher
        if want not in status['robot_cmd_publishers']:
            status['problems'].append(f'{want} not yet discovered on {self.wiring.robot_topic} '
                                      f'(domain {self.wiring.system_domain})')
        return status

    # ---- commands (all through the mux) -----------------------------------------------------
    def send_stop(self):
        """Zero velocity on the mux safety_stop input (priority 255, above every lock)."""
        if not self.can_actuate:
            raise RuntimeError('read-only interface has no publishers')
        self._stop_pub.publish(self._Twist())

    def feed_deadman(self):
        """Release the operator_hold lock for <= 0.5 s. Called only after all checks passed."""
        if not self.can_actuate:
            return
        now = time.monotonic()
        if now - self._last_feed >= 0.05:
            self._hold_pub.publish(self._Bool(data=False))
            self._last_feed = now

    def set_speed_limit(self, speed):
        """Nav2 speed limit (absolute m/s; 0 = Nav2 configured maximum, itself capped at the hard ceiling)."""
        if not self.can_actuate:
            return False
        deadline = time.monotonic() + 2.0
        while self._speed_pub.get_subscription_count() < 1 and time.monotonic() < deadline:
            time.sleep(0.05)
        msg = self._SpeedLimit(percentage=False, speed_limit=float(speed))
        for _ in range(3):
            self._speed_pub.publish(msg)
            time.sleep(0.02)
        return self._speed_pub.get_subscription_count() > 0

    @staticmethod
    def _call(client, request, timeout=3.0):
        future = client.call_async(request)
        deadline = time.monotonic() + timeout
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        return future.result() if future.done() else None

    def set_nav_velocity_cap(self, vx, wz):
        """Hard ceiling on Nav2's output at the velocity_smoother (dynamic max_velocity), read back.

        Humble MPPI discards /speed_limit at every goal start (optimizer reset restores vx_max);
        the smoother sits after MPPI, clamps every output to max_velocity and is not reset."""
        from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
        from rcl_interfaces.srv import GetParameters, SetParameters
        if not self.can_actuate:
            return {'ok': False, 'reason': 'read-only interface'}
        node = self._nodes[self.wiring.velocity_domain]
        want = [float(vx), 0.0, float(wz)]
        set_client = node.create_client(SetParameters, f'/{SMOOTHER_NODE}/set_parameters')
        get_client = node.create_client(GetParameters, f'/{SMOOTHER_NODE}/get_parameters')
        try:
            if not set_client.wait_for_service(timeout_sec=3.0):
                return {'ok': False, 'reason': 'velocity_smoother parameter service unavailable'}
            result = self._call(set_client, SetParameters.Request(parameters=[Parameter(
                name='max_velocity', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                                                          double_array_value=want))]))
            if result is None:
                return {'ok': False, 'reason': 'set_parameters timeout'}
            if not result.results[0].successful:
                return {'ok': False, 'reason': f'rejected: {result.results[0].reason}'}
            got = self._call(get_client, GetParameters.Request(names=['max_velocity']))
            values = list(got.values[0].double_array_value) if got and got.values else []
            ok = len(values) == 3 and abs(values[0] - want[0]) < 1e-6 and abs(values[2]) <= want[2] + 1e-6
            return {'ok': ok, 'requested': want, 'max_velocity': values,
                    'reason': None if ok else f'read back {values}'}
        finally:
            node.destroy_client(set_client)
            node.destroy_client(get_client)

    def set_stop_distance(self, ahead_m):
        """Ask terrain_monitor to use this proximity-stop distance (it refuses values below its floor)."""
        from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
        from rcl_interfaces.srv import SetParameters
        if not self.can_actuate or not self._param_client.wait_for_service(timeout_sec=3.0):
            return {'ok': False, 'reason': 'terrain_monitor parameter service unavailable'}
        request = SetParameters.Request(parameters=[Parameter(
            name='stop_ahead_m', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                                      double_value=float(ahead_m)))])
        future = self._param_client.call_async(request)
        deadline = time.monotonic() + 3.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            return {'ok': False, 'reason': 'timeout'}
        result = future.result().results[0]
        return {'ok': result.successful, 'reason': result.reason, 'stop_ahead_m': ahead_m}

    def nav_send_goal(self, x, y, yaw):
        import math
        if not self.can_actuate:
            raise RuntimeError('read-only interface sends no goals')
        if not self._action.wait_for_server(timeout_sec=5.0):
            return False
        goal = self._NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x, goal.pose.pose.position.y = float(x), float(y)
        goal.pose.pose.orientation.z = math.sin(yaw / 2)
        goal.pose.pose.orientation.w = math.cos(yaw / 2)
        self._goal = {'handle': None, 'result': None, 'recoveries': 0}

        def feedback(msg):
            self._goal['recoveries'] = msg.feedback.number_of_recoveries

        future = self._action.send_goal_async(goal, feedback_callback=feedback)
        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        handle = future.result() if future.done() else None
        if handle is None or not handle.accepted:
            return False
        self._goal['handle'] = handle
        names = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}
        handle.get_result_async().add_done_callback(
            lambda f: self._goal.update(result=names.get(f.result().status, f'STATUS_{f.result().status}')))
        return True

    def nav_spin(self, angle_rad, time_allowance_s):
        """Nav2 behavior_server Spin: turn in place by angle_rad (relative). Shares the goal slot, so
        nav_status / nav_cancel cover it; feedback 'spun_rad' is the angle turned so far. Its output
        goes nav_cmd_vel -> twist_mux (dead-man + terrain locks) -> collision_monitor like any goal."""
        if not self.can_actuate:
            raise RuntimeError('read-only interface sends no goals')
        if not self._spin_action.wait_for_server(timeout_sec=5.0):
            return False
        goal = self._Spin.Goal()
        goal.target_yaw = float(angle_rad)
        goal.time_allowance.sec = int(time_allowance_s)
        self._goal = {'handle': None, 'result': None, 'recoveries': 0, 'spun_rad': 0.0}

        def feedback(msg):
            self._goal['spun_rad'] = abs(msg.feedback.angular_distance_traveled)

        future = self._spin_action.send_goal_async(goal, feedback_callback=feedback)
        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        handle = future.result() if future.done() else None
        if handle is None or not handle.accepted:
            return False
        self._goal['handle'] = handle
        names = {4: 'SUCCEEDED', 5: 'CANCELED', 6: 'ABORTED'}
        handle.get_result_async().add_done_callback(
            lambda f: self._goal.update(result=names.get(f.result().status, f'STATUS_{f.result().status}')))
        return True

    def nav_status(self):
        g = self._goal
        return {'active': g['handle'] is not None and g['result'] is None, 'result': g['result'],
                'recoveries': g['recoveries'], 'spun_rad': g.get('spun_rad')}

    def nav_cancel(self, timeout_s=3.0):
        handle = self._goal.get('handle')
        if handle is None or self._goal.get('result'):
            return True
        future = handle.cancel_goal_async()
        deadline = time.monotonic() + timeout_s
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        return future.done()

    def map_pose(self):
        import math
        from rclpy.time import Time
        try:
            tf = self._tf.lookup_transform('map', 'base_link', Time())
        except Exception:  # noqa: BLE001 - no SLAM/localization yet
            return None
        q = tf.transform.rotation
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        return tf.transform.translation.x, tf.transform.translation.y, yaw

    def collision_boundary(self, polygon='PolygonStop'):
        """Nose distance of a running collision_monitor polygon (None if unavailable)."""
        from rcl_interfaces.srv import GetParameters
        node = self._nodes[self.wiring.velocity_domain]
        client = node.create_client(GetParameters, f'/{CM_NODE}/get_parameters')
        try:
            if not client.wait_for_service(timeout_sec=2.0):
                return None
            future = client.call_async(GetParameters.Request(names=[f'{polygon}.points']))
            deadline = time.monotonic() + 2.0
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.02)
            values = future.result().values if future.done() else []
            points = list(values[0].double_array_value) if values else []
            return round(max(points[0::2]) - 0.305, 3) if points else None
        finally:
            node.destroy_client(client)

    def collision_approach_time(self):
        """time_before_collision of the running collision_monitor approach action (None if absent)."""
        from rcl_interfaces.srv import GetParameters
        node = self._nodes[self.wiring.velocity_domain]
        client = node.create_client(GetParameters, f'/{CM_NODE}/get_parameters')
        try:
            if not client.wait_for_service(timeout_sec=2.0):
                return None
            future = client.call_async(GetParameters.Request(names=['FootprintApproach.time_before_collision']))
            deadline = time.monotonic() + 2.0
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.02)
            values = future.result().values if future.done() else []
            value = values[0].double_value if values and values[0].type == 3 else 0.0   # 3 = PARAMETER_DOUBLE
            return round(value, 3) if value > 0 else None
        finally:
            node.destroy_client(client)

    def nav_ready(self):
        node = self._nodes[self.wiring.velocity_domain]
        problems = []
        if not self._action.server_is_ready():
            problems.append('Nav2 /navigate_to_pose action server not available')
        if node.count_subscribers('/speed_limit') < 1:
            problems.append('Nav2 controller not subscribed to /speed_limit')
        if self.map_pose() is None:
            problems.append('no map -> base_link transform (SLAM not running?)')
        mux_inputs = {OPS_HOLD_TOPIC, OPS_TEST_TOPIC, OPS_STOP_TOPIC, '/lite3_terrain/stop'}
        for topic in sorted(mux_inputs):
            subs = {i.node_name for i in node.get_subscriptions_info_by_topic(topic)}
            if 'twist_mux' not in subs:
                problems.append(f'twist_mux not subscribed to {topic} '
                                '(navigation not started with --physical?)')
        for topic in (MUX_OUT_TOPIC, '/lite3_terrain/hazards'):
            subs = {i.node_name for i in node.get_subscriptions_info_by_topic(topic)}
            if CM_NODE not in subs:
                problems.append(f'collision_monitor not subscribed to {topic} (not running/active?)')
        services = {name for name, _ in node.get_service_names_and_types()}
        if f'/{TERRAIN_NODE}/set_parameters' not in services:
            problems.append('terrain_monitor parameter service not found')
        return {'problems': problems}

    def command_path_health(self):
        """Physical navigation wiring: twist_mux is the ONLY /cmd_vel source; nothing competes."""
        w = self.wiring
        vel = self._nodes[w.velocity_domain]
        system = self._nodes[w.system_domain]

        def names(infos):
            return sorted({f'{i.node_namespace.rstrip("/")}/{i.node_name}' for i in infos})

        problems = []
        ours = f'/{NODE_NAME}'
        cmd_pubs = names(vel.get_publishers_info_by_topic('/cmd_vel'))
        sys_pubs = names(system.get_publishers_info_by_topic(w.robot_topic))
        mux_pubs = names(vel.get_publishers_info_by_topic(MUX_OUT_TOPIC))
        if mux_pubs != ['/twist_mux']:
            problems.append(f'{MUX_OUT_TOPIC} publishers {mux_pubs} != [/twist_mux]')
        if f'/{CM_NODE}' not in names(vel.get_subscriptions_info_by_topic(MUX_OUT_TOPIC)):
            problems.append(f'collision_monitor not subscribed to {MUX_OUT_TOPIC}')
        if not cmd_pubs:
            # The domain bridge creates its /cmd_vel relay only once a publisher exists, so the
            # bridge checks below are meaningless until twist_mux drives /cmd_vel.
            problems.append('no /cmd_vel publisher: navigation is dry or not running (physical tests '
                            'need operator/start_navigation.sh --physical); bridge relay not yet created')
        else:
            if cmd_pubs != [f'/{CM_NODE}']:
                problems.append(f'/cmd_vel publishers {cmd_pubs} != [/{CM_NODE}] (the collision monitor '
                                'must be the only, final command authority)')
            if f'/{w.velocity_bridge_subscriber}' not in names(
                    vel.get_subscriptions_info_by_topic('/cmd_vel')):
                problems.append(f'{w.velocity_bridge_subscriber} not subscribed to /cmd_vel')
            if sys_pubs != [f'/{w.system_cmd_publisher}']:
                problems.append(f'system {w.robot_topic} publishers {sys_pubs} (bridge relay missing)')
        if f'/{w.motion_node}' not in names(system.get_subscriptions_info_by_topic(w.robot_topic)):
            problems.append(f'{w.motion_node} not subscribed to {w.robot_topic}')
        for topic in (OPS_TEST_TOPIC, OPS_STOP_TOPIC, OPS_HOLD_TOPIC):
            extra = set(names(vel.get_publishers_info_by_topic(topic))) - {ours}
            if extra:
                problems.append(f'foreign publishers on {topic}: {sorted(extra)}')
        extra = set(names(vel.get_publishers_info_by_topic('/nav_cmd_vel'))) - NAV2_PUBLISHERS
        if extra:
            problems.append(f'unexpected /nav_cmd_vel publishers {sorted(extra)}')
        joy = names(vel.get_publishers_info_by_topic('/joy_cmd_vel'))
        if joy:
            problems.append(f'/joy_cmd_vel publishers {joy} (joystick input bypasses the locks)')
        stop_pubs = names(vel.get_publishers_info_by_topic('/lite3_terrain/stop'))
        if stop_pubs != [f'/{TERRAIN_NODE}']:
            problems.append(f'/lite3_terrain/stop publishers {stop_pubs}')
        for topic in ('/cmd_vel_corrected', '/complex_cmd'):
            for node in (vel, system):
                if names(node.get_publishers_info_by_topic(topic)):
                    problems.append(f'unexpected publishers on {topic}')
        allowed_posture = {'/' + system.get_name(), '/' + w.system_cmd_publisher}
        posture_pubs = names(system.get_publishers_info_by_topic(w.posture_topic))
        if set(posture_pubs) - allowed_posture:
            problems.append(f'unexpected {w.posture_topic} publishers {posture_pubs}')
        if names(vel.get_publishers_info_by_topic(w.posture_topic)):
            problems.append(f'group-domain {w.posture_topic} publishers present')
        return not problems, {'problems': problems, 'cmd_vel_publishers': cmd_pubs,
                              'system_publishers': sys_pubs}

    def close(self):
        if self.can_actuate:
            try:
                self.nav_cancel()
                for _ in range(3):        # engage the dead-man lock at once (silence would in 0.5 s)
                    self._hold_pub.publish(self._Bool(data=True))
                    time.sleep(0.02)
            except Exception:  # noqa: BLE001 - closing anyway
                pass
        super().close()
