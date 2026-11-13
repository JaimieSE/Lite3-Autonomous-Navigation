"""Opt-in high-level velocity transport; no posture, mode or joint commands.

Wire layout matches the pinned vendor source, NOT verified deployed firmware.
The worker survives a stalled ROS executor, not process/host/network failure.
"""

import argparse
import ipaddress
import json
import math
import signal
import socket
import sys
import threading
import time
import uuid

from lite3_locomotion.command_policy import PolicyConfig, Velocity, ZERO
from lite3_legacy.protocol_preview import preview_packets


COMMAND = '/lite3_robot_link/cmd_vel'
READY = '/lite3_robot_link/ready'
STATUS = '/lite3_robot_link/status'
ENABLE = '/lite3_robot_link/enable'
STOP = '/lite3_robot_link/stop'
AUTHORIZE = '/lite3_robot_link/session_ack'


def parse_options(args):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--target-ip', required=True)
    parser.add_argument('--target-port', required=True, type=int)
    parser.add_argument('--allow-robot-output', action='store_true',
                        help='Explicitly permit a non-loopback destination; does not arm')
    parser.add_argument('--session-seconds', type=float, default=30.0)
    options = parser.parse_args(args)
    try:
        address = ipaddress.IPv4Address(options.target_ip)
        if address.is_unspecified or address.is_multicast or int(address) == 0xffffffff:
            raise ValueError('Target must be a unicast IPv4 address')
        if not address.is_loopback and not options.allow_robot_output:
            raise ValueError('Non-loopback target requires --allow-robot-output')
        if not 1 <= options.target_port <= 65535:
            raise ValueError('Target port must be in [1, 65535]')
        if not math.isfinite(options.session_seconds) or not 0 < options.session_seconds <= 60:
            raise ValueError('Session must be in (0, 60] seconds')
    except ValueError as error:
        parser.error(str(error))
    return options


class TransportGate:
    """Caller serializes access. Clock and sender injection permit offline tests."""

    def __init__(self, send, *, session_seconds=30.0, clock=time.monotonic):
        if not math.isfinite(session_seconds) or not 0 < session_seconds <= 60:
            raise ValueError('Invalid session duration')
        self.send = send
        self.clock = clock
        self.session_seconds = session_seconds
        self.config = PolicyConfig(0.03, 0.0, 0.10, 0.25)
        self.enabled = False
        self.reason = 'disarmed'
        self.graph_received = None
        self.graph_ok = False
        self.started = self.received = None
        self.command = ZERO
        self.transmitted = False
        self.zero_until = None
        self.guard_seen = False
        self.session_id = None
        self.authorized = False

    def graph(self, healthy):
        self.graph_ok = healthy
        self.graph_received = self.clock()

    def graph_fresh(self):
        return (self.graph_ok and self.graph_received is not None and
                0 <= self.clock() - self.graph_received < 0.5)

    def enable(self):
        if not self.graph_fresh():
            return False
        if not self.enabled:
            self.enabled = True
            self.started = self.clock()
            self.received = None
            self.command = ZERO
            self.guard_seen = False
            self.authorized = False
            self.session_id = uuid.uuid4().hex
            self.reason = 'waiting_for_command'
        return True  # Repeated enable cannot extend a session.

    def stop(self, reason='operator_stop'):
        if self.enabled and self.transmitted:
            self.zero_until = self.clock() + 1.0
        self.enabled = False
        self.authorized = False
        self.command = ZERO
        self.received = None
        self.reason = reason

    def authorize(self, session_id):
        self.supervise()
        if self.enabled and session_id == self.session_id:
            self.authorized = True
            return True
        return False

    def guard_enabled(self, enabled):
        # Pre-authorization heartbeats may belong to the previous adapter arm.
        # Only the nonce acknowledgment establishes this session's arm intent.
        if self.enabled and self.authorized:
            if enabled:
                self.guard_seen = True
            elif self.guard_seen:
                self.stop('adapter_disarmed')

    def accept(self, command):
        self.supervise()
        if not self.enabled or not self.authorized:
            return False
        try:
            # Reject lateral commands rather than silently changing direction.
            if command.linear_y != 0:
                raise ValueError('Lateral movement disabled')
            preview_packets(command, self.config)
        except (ValueError, AttributeError, TypeError, OverflowError):
            self.stop('invalid_command')
            return False
        self.command = command
        self.received = self.clock()
        self.reason = 'active'
        return True

    def supervise(self):
        if not self.enabled:
            return
        now = self.clock()
        if not self.graph_fresh():
            self.stop('graph_missing_unexpected_or_stale')
        elif not 0 <= now - self.started < self.session_seconds:
            self.stop('session_deadline')
        elif self.received is None and now - self.started >= 10.0:
            self.stop('first_command_timeout')
        elif self.received is not None and not 0 <= now - self.received < 0.25:
            self.stop('command_timeout')

    def step(self):
        self.supervise()
        command = None
        if self.enabled and self.received is not None:
            command = self.command
        elif self.zero_until is not None and self.clock() < self.zero_until:
            command = ZERO
        if command is None:
            return
        # Set before sending: even a partially sent triplet needs stop attempts.
        self.transmitted = True
        stopping = not self.enabled
        for packet in preview_packets(command, self.config):
            try:
                if self.send(packet) != len(packet):
                    raise OSError('Short UDP send')
            except OSError:
                self.stop('transport_error')
                if not stopping:
                    break  # Never send the remaining nonzero axes after a failure.
                # During a zero burst, attempt every axis even if another fails.


def create_node(options, context):
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from std_msgs.msg import Bool, String
    from std_srvs.srv import Trigger
    from rclpy.qos import DurabilityPolicy, QoSProfile

    class MotionSender(Node):
        def __init__(self):
            super().__init__('motion_sender', namespace='/lite3_robot_link', context=context,
                             use_global_arguments=False, enable_rosout=False,
                             start_parameter_services=False)
            for name in (COMMAND, READY, STATUS, AUTHORIZE, '/lite3_locomotion/autonomy_enabled'):
                if self.resolve_topic_name(name) != name:
                    raise RuntimeError('Motion sender topic isolation failed')
            for name in (ENABLE, STOP):
                if self.resolve_service_name(name) != name:
                    raise RuntimeError('Motion sender service isolation failed')
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.setblocking(False)
            self.target = (options.target_ip, options.target_port)
            self.lock = threading.Lock()
            self.gate = TransportGate(lambda packet: self.sock.sendto(packet, self.target),
                                      session_seconds=options.session_seconds)
            self.quit = threading.Event()
            self.ready_pub = self.create_publisher(String, READY, 1)
            self.status_pub = self.create_publisher(String, STATUS, 1)
            self.command_sub = self.create_subscription(Twist, COMMAND, self.command_callback, 1)
            qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
            self.guard_sub = self.create_subscription(Bool, '/lite3_locomotion/autonomy_enabled',
                                                      self.guard_callback, qos)
            self.auth_sub = self.create_subscription(String, AUTHORIZE, self.authorize_callback, 1)
            self.enable_service = self.create_service(Trigger, ENABLE, self.enable_callback)
            self.stop_service = self.create_service(Trigger, STOP, self.stop_callback)
            self.timer = self.create_timer(0.1, self.tick)
            self.worker = threading.Thread(target=self.work, name='velocity_watchdog', daemon=True)
            self.worker.start()
            self.get_logger().info('DISARMED transport to %s:%s; no stand or mode commands' % self.target)

        def command_callback(self, message):
            with self.lock:
                self.gate.accept(Velocity(message.linear.x, message.linear.y, message.linear.z,
                                          message.angular.x, message.angular.y, message.angular.z))

        def guard_callback(self, message):
            with self.lock:
                self.gate.guard_enabled(message.data)

        def authorize_callback(self, message):
            with self.lock:
                self.gate.authorize(message.data)

        def graph_healthy(self):
            pubs = self.get_publishers_info_by_topic(COMMAND)
            subs = self.get_subscriptions_info_by_topic(COMMAND)
            auth = self.get_publishers_info_by_topic(AUTHORIZE)
            return ([(p.node_name, p.node_namespace) for p in pubs] == [('lite3_guarded_adapter', '/')] and
                    [(p.node_name, p.node_namespace) for p in auth] == [('lite3_guarded_adapter', '/')] and
                    [(p.node_name, p.node_namespace) for p in subs] == [('motion_sender', '/lite3_robot_link')])

        def enable_callback(self, request, response):
            healthy = self.graph_healthy()
            with self.lock:
                self.gate.graph(healthy)
                response.success = self.gate.enable()
            response.message = ('Sender enabled; start autonomy within 10 seconds; no mode change sent'
                                if response.success else 'Not enabled: expected one guarded adapter publisher only')
            return response

        def stop_callback(self, request, response):
            with self.lock:
                self.gate.stop()
            response.success = True
            response.message = 'Sender disarmed; bounded zero burst requested, not physical acknowledgment'
            return response

        def tick(self):
            healthy = self.graph_healthy()
            with self.lock:
                self.gate.graph(healthy)
                self.gate.supervise()
                enabled, reason = self.gate.enabled, self.gate.reason
                session_id, authorized = self.gate.session_id, self.gate.authorized
            ready = String()
            ready.data = json.dumps(dict(enabled=enabled, session_id=session_id))
            self.ready_pub.publish(ready)
            status = String()
            status.data = json.dumps(dict(enabled=enabled, reason=reason, target=list(self.target),
                                           session_id=session_id, authorized=authorized,
                                           max_linear_x=0.03, max_angular_z=0.10,
                                           physical_stop_verified=False))
            self.status_pub.publish(status)

        def work(self):
            while not self.quit.is_set():
                with self.lock:
                    self.gate.step()
                self.quit.wait(0.05)

        def close(self):
            with self.lock:
                self.gate.stop('shutdown')
            # Keep the worker alive for a bounded best-effort zero burst.
            time.sleep(1.1)
            self.quit.set()
            self.worker.join(timeout=1.0)
            self.sock.close()

    return MotionSender()


def main(args=None):
    options = parse_options(sys.argv[1:] if args is None else args)
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
    from rclpy.signals import SignalHandlerOptions
    context = Context()
    node = executor = None
    rclpy.init(args=[], context=context, signal_handler_options=SignalHandlerOptions.NO)
    def interrupt(signum, frame):
        raise KeyboardInterrupt
    previous = {sig: signal.signal(sig, interrupt) for sig in (signal.SIGINT, signal.SIGTERM)}
    try:
        node = create_node(options, context)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(node)
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        # Ignore a second interrupt while bounded shutdown zeros are attempted.
        for sig in previous:
            signal.signal(sig, signal.SIG_IGN)
        if node is not None:
            node.close()
        if executor is not None:
            executor.shutdown()
        if node is not None:
            node.destroy_node()
        context.try_shutdown()
        for sig, handler in previous.items():
            signal.signal(sig, handler)


if __name__ == '__main__':
    main()
