"""Transport policy and real UDP framing without robot or ROS connections."""

import math
import socket
import struct

import pytest

from lite3_locomotion.command_policy import Velocity
from lite3_locomotion.motion_sender import TransportGate, parse_options


@pytest.fixture
def gate():
    now = [0.0]
    packets = []
    def send(packet):
        packets.append(struct.unpack('<iiid', packet))
        return len(packet)
    instance = TransportGate(send, clock=lambda: now[0])
    instance.graph(True)
    return instance, now, packets


def refresh(instance, now, value):
    now[0] = value
    instance.graph(True)


def arm(instance):
    assert instance.enable()
    assert instance.authorize(instance.session_id)


def test_startup_and_disarmed_commands_are_silent(gate):
    instance, _, packets = gate
    assert not instance.accept(Velocity(linear_x=0.03))
    instance.step()
    instance.stop()
    instance.step()
    assert not packets
    assert instance.enable()
    instance.step()
    assert not packets  # Arming alone must not transmit, even zero packets.


def test_limits_wire_layout_and_yaw_sign(gate):
    instance, _, packets = gate
    arm(instance)
    assert instance.accept(Velocity(linear_x=10, angular_z=-10))
    instance.step()
    assert packets == [(320, 8, 1, 0.03), (325, 8, 1, 0.0), (321, 8, 1, 0.10)]


@pytest.mark.parametrize('command', [
    Velocity(linear_x=math.nan), Velocity(angular_z=math.inf),
    Velocity(linear_y=0.01), Velocity(linear_z=0.1),
    Velocity(angular_x=0.1), Velocity(angular_y=0.1),
])
def test_invalid_command_latches_stop(gate, command):
    instance, _, packets = gate
    arm(instance)
    instance.accept(Velocity(linear_x=0.02))
    instance.step()
    assert not instance.accept(command)
    assert not instance.enabled
    assert instance.reason == 'invalid_command'
    instance.step()
    assert all(p[3] == 0 for p in packets[-3:])
    assert not instance.accept(Velocity(linear_x=0.02))


def test_command_timeout_zero_burst_and_no_automatic_rearm(gate):
    instance, now, packets = gate
    arm(instance)
    instance.accept(Velocity(linear_x=0.02))
    instance.step()
    refresh(instance, now, 0.25)
    instance.step()
    assert instance.reason == 'command_timeout'
    assert all(p[3] == 0 for p in packets[-3:])
    assert not instance.accept(Velocity(linear_x=0.02))
    refresh(instance, now, 1.26)
    count = len(packets)
    instance.step()
    assert len(packets) == count
    arm(instance)
    assert instance.accept(Velocity(linear_x=0.02))


def test_graph_heartbeat_timeout_survives_fresh_input(gate):
    instance, now, _ = gate
    arm(instance)
    for stamp in (0.0, 0.2, 0.4):
        now[0] = stamp
        instance.accept(Velocity(linear_x=0.02))
        instance.step()
    now[0] = 0.5
    instance.step()
    assert not instance.enabled
    assert instance.reason == 'graph_missing_unexpected_or_stale'


def test_unexpected_graph_prevents_enable_and_stops(gate):
    instance, _, _ = gate
    instance.graph(False)
    assert not instance.enable()
    instance.graph(True)
    assert instance.enable()
    instance.graph(False)
    assert not instance.accept(Velocity(linear_x=0.02))
    assert not instance.enabled


def test_first_command_deadline_and_reenable_does_not_extend_session(gate):
    instance, now, _ = gate
    arm(instance)
    refresh(instance, now, 9.9)
    instance.enable()
    refresh(instance, now, 10.0)
    instance.step()
    assert instance.reason == 'first_command_timeout'
    arm(instance)
    instance.accept(Velocity())
    refresh(instance, now, 40.0)
    instance.step()
    assert instance.reason == 'session_deadline'


def test_adapter_disarm_latches_sender(gate):
    instance, _, _ = gate
    arm(instance)
    instance.guard_enabled(False)  # Adapter has not yet armed.
    assert instance.enabled
    instance.guard_enabled(True)
    instance.guard_enabled(False)
    assert not instance.enabled
    assert instance.reason == 'adapter_disarmed'


def test_udp_failure_latches_and_attempts_zeros():
    packets = []
    def send(packet):
        packets.append(struct.unpack('<iiid', packet))
        if len(packets) == 2:
            raise BlockingIOError('simulated UDP failure')
        return len(packet)
    instance = TransportGate(send, clock=lambda: 0.0)
    instance.graph(True)
    arm(instance)
    instance.accept(Velocity(linear_x=0.02))
    instance.step()
    assert instance.reason == 'transport_error'
    assert not instance.enabled
    instance.step()
    assert all(p[3] == 0 for p in packets[-3:])


def test_real_loopback_datagrams_match_vendor_abi():
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
        receiver.bind(('127.0.0.1', 0))
        receiver.settimeout(1)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sender:
            instance = TransportGate(lambda packet: sender.sendto(packet, receiver.getsockname()))
            instance.graph(True)
            arm(instance)
            instance.accept(Velocity(linear_x=0.02, angular_z=0.04))
            instance.step()
            packets = [receiver.recvfrom(100)[0] for _ in range(3)]
        assert [len(p) for p in packets] == [20, 20, 20]
        assert [struct.unpack('<iiid', p) for p in packets] == [
            (320, 8, 1, 0.02), (325, 8, 1, 0.0), (321, 8, 1, -0.04)]


@pytest.mark.parametrize('args', [
    [], ['--target-ip', '192.168.1.120', '--target-port', '43893'],
    ['--target-ip', '0.0.0.0', '--target-port', '43893'],
    ['--target-ip', '224.0.0.1', '--target-port', '43893'],
    ['--target-ip', '127.0.0.1', '--target-port', '0'],
    ['--target-ip', '127.0.0.1', '--target-port', '12345', '--session-seconds', 'nan'],
    ['--target-ip', '127.0.0.1', '--target-port', '12345', '--ros-args'],
])
def test_bad_or_implicit_physical_configuration_rejected(args):
    with pytest.raises(SystemExit):
        parse_options(args)


def test_explicit_targets_and_robot_opt_in():
    assert parse_options(['--target-ip', '127.0.0.1', '--target-port', '12345']).target_port == 12345
    options = parse_options(['--target-ip', '192.168.1.120', '--target-port', '43893',
                             '--allow-robot-output'])
    assert options.allow_robot_output


def test_reenable_requires_new_session_ack_even_before_disabled_heartbeat(gate):
    instance, _, packets = gate
    arm(instance)
    old_session = instance.session_id
    instance.accept(Velocity(linear_x=0.02))
    instance.step()
    instance.stop()
    instance.enable()  # Simulate stop + enable between two ROS heartbeat ticks.
    assert instance.session_id != old_session
    assert not instance.accept(Velocity(linear_x=0.02))
    assert not instance.authorize(old_session)
    instance.step()
    assert all(p[3] == 0 for p in packets[-3:])
    assert instance.authorize(instance.session_id)
    assert instance.accept(Velocity(linear_x=0.02))


def test_disarmed_adapter_zeros_cannot_clear_new_session_deadline(gate):
    instance, now, _ = gate
    instance.enable()
    for stamp in range(11):
        refresh(instance, now, float(stamp))
        assert not instance.accept(Velocity())
    assert not instance.enabled
    assert instance.reason == 'first_command_timeout'


def test_previous_arm_heartbeats_cannot_disarm_new_unauthorized_session(gate):
    instance, _, _ = gate
    instance.enable()
    instance.guard_enabled(True)
    instance.guard_enabled(False)
    assert instance.enabled
    assert not instance.authorized
    assert not instance.accept(Velocity(linear_x=0.02))


def test_zero_failure_does_not_skip_other_stop_axes(gate):
    instance, _, _ = gate
    arm(instance)
    instance.accept(Velocity(linear_x=0.02, angular_z=0.04))
    instance.step()
    instance.stop()
    attempts = []
    def send(packet):
        code, _, _, value = struct.unpack('<iiid', packet)
        attempts.append((code, value))
        if code == 320:
            raise OSError('Forward-zero send failure')
        return len(packet)
    instance.send = send
    instance.step()
    assert attempts == [(320, 0.0), (325, 0.0), (321, 0.0)]
