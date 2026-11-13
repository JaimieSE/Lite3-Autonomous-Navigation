"""Deterministic, non-ROS timing tests: no robot/network access."""

import math
import os
import time
import threading
import pytest

from lite3_locomotion.commissioning_pulse import PulsePlan, parse_options, run_pulse


def fixture_clock():
    now = [0.0]
    emitted = []
    def sleep(dt):
        now[0] += dt
    return now, emitted, sleep


def test_one_process_deadline_and_zero_gap():
    now, emitted, sleep = fixture_clock()
    result = run_pulse(lambda value: emitted.append((now[0], value)),
                       clock=lambda: now[0], sleep=sleep)
    assert result['reason'] == 'duration_complete'
    assert 19 <= result['forward_messages'] <= 21
    assert result['first_forward_to_first_zero_s'] == pytest.approx(1.0)
    assert result['last_forward_to_first_zero_s'] <= 0.0500001
    assert all(value == 0 for _, value in emitted[:10])
    assert emitted[-1][0] - 1.5 >= 2.0 - 1e-6
    assert not result['physical_stop_verified']


def test_cancel_before_forward_is_zero_only():
    now, emitted, sleep = fixture_clock()
    result = run_pulse(lambda value: emitted.append((now[0], value)),
                       clock=lambda: now[0], sleep=sleep, cancelled=lambda: True)
    assert not result['forward_messages']
    assert all(value == 0 for _, value in emitted)


def test_cancel_during_forward_latches_zeros():
    now, emitted, sleep = fixture_clock()
    result = run_pulse(lambda value: emitted.append((now[0], value)),
                       clock=lambda: now[0], sleep=sleep,
                       cancelled=lambda: now[0] >= 0.7)
    assert result['reason'] == 'cancelled_or_unhealthy'
    last_forward = max(i for i, (_, value) in enumerate(emitted) if value)
    assert all(value == 0 for _, value in emitted[last_forward + 1:])


def test_scheduler_delay_aborts_instead_of_resuming_forward():
    now, emitted, sleep = fixture_clock()
    def delayed(dt):
        sleep(dt + (0.3 if now[0] >= 0.65 else 0))
    result = run_pulse(lambda value: emitted.append((now[0], value)),
                       clock=lambda: now[0], sleep=delayed)
    assert result['reason'] == 'scheduler_gap'
    assert result['forward_messages'] < 20


def test_publish_exception_attempts_same_publisher_zero_cleanup():
    now, emitted, sleep = fixture_clock()
    def emit(value):
        emitted.append((now[0], value))
        if value:
            raise RuntimeError('publish failed')
    with pytest.raises(RuntimeError, match='publish failed'):
        run_pulse(emit, clock=lambda: now[0], sleep=sleep)
    assert emitted[-1][1] == 0
    assert now[0] >= 2.5


def test_slow_health_check_cannot_publish_after_deadline():
    now, emitted, sleep = fixture_clock()
    checks = [0]
    def healthy():
        checks[0] += 1
        if now[0] >= 0.6:
            sleep(1.0)
        return True
    result = run_pulse(lambda value: emitted.append((now[0], value)),
                       clock=lambda: now[0], sleep=sleep, healthy=healthy)
    assert result['reason'] == 'scheduler_gap'
    assert all(stamp < 1.0 for stamp, value in emitted if value != 0)


def test_observer_exception_does_not_prevent_zero_burst():
    now, emitted, sleep = fixture_clock()
    def pump():
        raise RuntimeError('observer failed')
    with pytest.raises(RuntimeError, match='Zero cleanup'):
        run_pulse(lambda value: emitted.append((now[0], value)),
                   clock=lambda: now[0], sleep=sleep, pump=pump)
    assert all(value == 0 for _, value in emitted)
    assert now[0] >= 2.05


@pytest.mark.parametrize('kwargs', [
    {'speed': math.nan}, {'speed': 0.031}, {'duration': 1.01},
    {'zero_seconds': 0.5}, {'duration': True}, {'max_gap': 0.01},
])
def test_invalid_plan(kwargs):
    with pytest.raises(ValueError):
        PulsePlan(**kwargs)


def test_physical_requires_explicit_confirmations_and_default_is_dry():
    assert not parse_options([]).physical
    with pytest.raises(SystemExit):
        parse_options(['--physical'])
    assert parse_options(['--physical', '--navigation-confirmed', '--operator-ready',
                          '--standing-confirmed', '--exclusive-control', '--accept-no-lidar']).physical


@pytest.mark.skipif(os.environ.get('LITE3_RUN_PULSE_LOOPBACK') != '1',
                    reason='Opt-in isolated ROS loopback timing check')
def test_real_ros_loopback_timing():
    # Refuse the test unless discovery/traffic is constrained to loopback.
    assert os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    assert os.environ.get('FASTRTPS_DEFAULT_PROFILES_FILE', '').endswith('/test/loopback_dds.xml')
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node
    from rclpy.executors import SingleThreadedExecutor
    from geometry_msgs.msg import Twist
    context = Context()
    rclpy.init(args=[], context=context, domain_id=97)
    node = Node('isolated_pulse_timing_test', context=context)
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    received = []
    topic = '/lite3_locomotion/pulse_test_cmd_vel'
    pub = node.create_publisher(Twist, topic, 100)
    node.create_subscription(Twist, topic,
        lambda msg: received.append((time.monotonic(), msg.linear.x)), 100)
    observer = threading.Thread(target=executor.spin, daemon=True)
    observer.start()
    def emit(value):
        msg = Twist()
        msg.linear.x = value
        pub.publish(msg)
    try:
        deadline = time.monotonic() + 5
        while pub.get_subscription_count() == 0 and time.monotonic() < deadline:
            time.sleep(0.02)
        assert pub.get_subscription_count() == 1
        result = run_pulse(emit)
        time.sleep(0.3)
        forward = [stamp for stamp, value in received if value != 0]
        assert forward
        first_zero = next(stamp for stamp, value in received if stamp > forward[-1] and value == 0)
        gap = first_zero - forward[-1]
        span = first_zero - forward[0]
        print({'local': result, 'receiver_last_forward_to_zero_s': gap,
               'receiver_first_forward_to_zero_s': span})
        assert result['reason'] == 'duration_complete'
        assert result['first_forward_to_first_zero_s'] < 1.15
        assert gap < 0.15
        assert 0.9 <= span < 1.15
    finally:
        executor.shutdown()
        observer.join(timeout=2)
        node.destroy_node()
        rclpy.shutdown(context=context)
