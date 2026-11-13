"""One-shot commissioning pulse; default output is a separate non-actuating topic.

Not a collision guard or firmware watchdog. Never changes mode or posture.
Physical operation requires explicit, fresh operator confirmations.
"""

import argparse
from dataclasses import dataclass
import json
import math
import signal
import threading
import time


DRY_TOPIC = '/lite3_locomotion/pulse_test_cmd_vel'


@dataclass(frozen=True)
class PulsePlan:
    speed: float = 0.03
    duration: float = 1.0
    period: float = 0.05
    zero_seconds: float = 2.0
    warmup_seconds: float = 0.5
    max_gap: float = 0.15

    def __post_init__(self):
        for name, upper in [('speed', 0.03), ('duration', 2.0),
                            ('period', 0.05), ('zero_seconds', 5.0),
                            ('warmup_seconds', 2.0), ('max_gap', 0.15)]:
            value = getattr(self, name)
            if (isinstance(value, bool) or not isinstance(value, (int, float)) or
                    not math.isfinite(value) or not 0 < value <= upper):
                raise ValueError(f'{name} must be finite and in (0, {upper}]')
        if self.zero_seconds < 2.0 or self.max_gap <= self.period:
            raise ValueError('Need >=2 seconds of zeros and max_gap > period')


def run_pulse(emit, *, plan=PulsePlan(), clock=time.monotonic, sleep=time.sleep,
              pump=lambda: None, healthy=lambda: True, cancelled=lambda: False):
    """Same emit callback for warmup, forward and final zeros; never rearms.

    A delayed tick latches forward off. Cleanup is best effort: process/host
    failure can still prevent zeros. Timestamps describe local publication,
    not robot receipt or verified stopping.
    """
    events = []
    reason = 'duration_complete'

    def publish(value):
        emit(value)
        events.append((clock(), value))

    def wait_tick():
        sleep(plan.period)
        pump()

    try:
        warmup_end = clock() + plan.warmup_seconds
        while clock() < warmup_end:
            publish(0.0)
            wait_tick()
            if cancelled() or not healthy():
                reason = 'cancelled_or_unhealthy_before_forward'
                break
        else:
            start = last = clock()
            forward_end = start + plan.duration
            while True:
                pump()
                if cancelled() or not healthy():
                    reason = 'cancelled_or_unhealthy'
                    break
                now = clock()
                if now < last or now - last >= plan.max_gap:
                    reason = 'scheduler_gap'
                    break
                if now >= forward_end:
                    break
                publish(plan.speed)
                last = now
                # Wake at the deadline even when duration is not a period multiple.
                sleep(min(plan.period, max(0.0, forward_end - clock())))
    finally:
        # No subprocess, publisher replacement, discovery wait or sleep here.
        cleanup_error = None
        try:
            publish(0.0)
        except Exception as error:
            cleanup_error = error
        end = clock() + plan.zero_seconds
        while clock() < end:
            sleep(min(plan.period, max(0.0, end - clock())))
            try:
                publish(0.0)
            except Exception as error:
                cleanup_error = error
            try:
                pump()
            except Exception as error:
                cleanup_error = error
        if cleanup_error is not None:
            raise RuntimeError('Zero cleanup encountered an error; physical stop is unverified') from cleanup_error
    forward = [stamp for stamp, value in events if value != 0]
    after = [stamp for stamp, value in events if value == 0 and forward and stamp >= forward[-1]]
    return {
        'reason': reason, 'forward_messages': len(forward),
        'zero_messages': sum(value == 0 for _, value in events),
        'first_forward_to_first_zero_s': after[0] - forward[0] if forward and after else None,
        'last_forward_to_first_zero_s': after[0] - forward[-1] if forward and after else None,
        'physical_stop_verified': False,
    }


def parse_options(args=None):
    parser = argparse.ArgumentParser(description=__doc__, allow_abbrev=False)
    parser.add_argument('--physical', action='store_true')
    parser.add_argument('--duration', type=float, default=1.0,
                        help='Requested forward duration in seconds, at most 2; default 1')
    for name in ('navigation-confirmed', 'operator-ready', 'standing-confirmed',
                 'exclusive-control', 'accept-no-lidar'):
        parser.add_argument('--' + name, action='store_true')
    options = parser.parse_args(args)
    try:
        PulsePlan(duration=options.duration)
    except ValueError as error:
        parser.error(str(error))
    if options.physical and not all((options.navigation_confirmed, options.operator_ready,
                                    options.standing_confirmed, options.exclusive_control,
                                    options.accept_no_lidar)):
        parser.error('Physical test requires every explicit confirmation flag; home screen is not mode feedback')
    return options


def main(args=None):
    options = parse_options(args)
    plan = PulsePlan(duration=options.duration)
    import rclpy
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from rclpy.signals import SignalHandlerOptions
    from geometry_msgs.msg import Twist

    topic = '/cmd_vel' if options.physical else DRY_TOPIC
    domain = 8 if options.physical else None
    context = Context()
    rclpy.init(args=[], context=context, domain_id=domain,
                signal_handler_options=SignalHandlerOptions.NO)
    node = Node('lite3_commissioning_pulse', context=context, use_global_arguments=False)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.VOLATILE)
    publisher = node.create_publisher(Twist, topic, qos)
    monitor_context = None
    monitor_node = None
    monitor_executor = None
    monitor_thread = None
    observed = []
    cancelled = [False]
    old_signals = {}
    for sig in (signal.SIGINT, signal.SIGTERM):
        old_signals[sig] = signal.signal(sig, lambda *_: cancelled.__setitem__(0, True))

    def pump():
        executor.spin_once(timeout_sec=0)

    def healthy():
        if not options.physical:
            return True
        local_pubs = node.get_publishers_info_by_topic('/cmd_vel')
        corrected = monitor_node.get_publishers_info_by_topic('/cmd_vel_corrected')
        downstream = monitor_node.get_subscriptions_info_by_topic('/cmd_vel')
        remote_pubs = monitor_node.get_publishers_info_by_topic('/cmd_vel')
        return ([(p.node_name, p.node_namespace) for p in local_pubs] ==
                [('lite3_commissioning_pulse', '/')] and not corrected and
                [(p.node_name, p.node_namespace) for p in remote_pubs] ==
                [('lite3_domain_bridge_64', '/')] and
                any((p.node_name, p.node_namespace) == ('motion_sender', '/') for p in downstream)
                and publisher.get_subscription_count() > 0)

    def emit(value):
        msg = Twist()
        msg.linear.x = value
        publisher.publish(msg)

    try:
        if options.physical:
            monitor_context = Context()
            rclpy.init(args=[], context=monitor_context, domain_id=64,
                        signal_handler_options=SignalHandlerOptions.NO)
            monitor_node = Node('lite3_pulse_observer', context=monitor_context,
                                use_global_arguments=False)
            monitor_executor = SingleThreadedExecutor(context=monitor_context)
            monitor_executor.add_node(monitor_node)
            monitor_node.create_subscription(Twist, '/cmd_vel',
                lambda msg: observed.append((time.monotonic(), msg.linear.x)), qos)
            monitor_thread = threading.Thread(target=monitor_executor.spin, daemon=True)
            monitor_thread.start()
            # Discovery before any command; no mode/posture commands are sent.
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline and not cancelled[0]:
                pump()
                time.sleep(0.05)
            if cancelled[0] or not healthy():
                raise RuntimeError('Preflight failed: absent sender/link or competing velocity publisher')
        print(json.dumps({'topic': topic, 'physical': options.physical,
                          'lidar_protected': False, 'requested_speed_m_s': 0.03,
                          'requested_duration_s': plan.duration}), flush=True)
        result = run_pulse(emit, plan=plan, pump=pump, healthy=healthy,
                           cancelled=lambda: cancelled[0])
        # Allow delayed observations of trailing zeros; this does not send commands.
        deadline = time.monotonic() + 0.3
        while time.monotonic() < deadline:
            pump()
            time.sleep(0.005)
        result['domain64_forward_observed'] = sum(value != 0 for _, value in observed)
        result['domain64_zero_observed'] = sum(value == 0 for _, value in observed)
        forward_observed = [stamp for stamp, value in observed if value != 0]
        trailing_observed = [stamp for stamp, value in observed
                             if forward_observed and stamp >= forward_observed[-1] and value == 0]
        result['domain64_last_forward_to_first_zero_s'] = (
            trailing_observed[0] - forward_observed[-1]
            if forward_observed and trailing_observed else None)
        if options.physical and result['forward_messages'] and not (
                forward_observed and trailing_observed):
            result['reason'] = 'domain64_forward_or_trailing_zero_unobserved'
        print(json.dumps(result, sort_keys=True), flush=True)
        return 0 if result['reason'] == 'duration_complete' else 1
    finally:
        for sig, handler in old_signals.items():
            signal.signal(sig, handler)
        if monitor_executor is not None:
            monitor_executor.shutdown()
            monitor_thread.join(timeout=2)
            monitor_node.destroy_node()
            rclpy.shutdown(context=monitor_context)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown(context=context)


if __name__ == '__main__':
    raise SystemExit(main())
