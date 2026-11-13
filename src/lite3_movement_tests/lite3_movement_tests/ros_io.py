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

from dataclasses import dataclass
import threading
import time

from .telemetry import Telemetry

POSTURE_TOGGLE = 0x21010202   # documented stand/sit toggle; also used by university drain_patrol
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


LIVE_WIRING = Wiring(telemetry_domain=8, velocity_domain=8, system_domain=64,
                     system_cmd_publisher='lite3_domain_bridge_64',
                     velocity_bridge_subscriber='lite3_domain_bridge_8')


def sim_wiring(domain):
    return Wiring(domain, domain, domain)


class RosRobot:
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
            name = NODE_NAME if domain == wiring.velocity_domain else f'{NODE_NAME}_d{domain}'
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
        system.create_subscription(Twist, wiring.velocity_topic, lambda m: t.on_system_cmd(
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
        for *_, thread in self._contexts:
            thread.start()

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
            msg.cmd_code, msg.size, msg.type = POSTURE_TOGGLE, 0, 0
            sent = time.monotonic()
            pub.publish(msg)
            deadline = sent + 1.0
            while time.monotonic() < deadline and not any(
                    t >= sent and c == POSTURE_TOGGLE for t, c in self._posture_seen):
                time.sleep(0.01)
            return {'published': True, 'cmd_code': hex(POSTURE_TOGGLE),
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
