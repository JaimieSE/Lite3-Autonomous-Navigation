"""Actual Nav2 -> actual robot-mode adapter -> UDP sender, localhost only."""

import os
from pathlib import Path
import signal
import socket
import struct
import subprocess

import pytest

pytestmark = pytest.mark.skipif(os.environ.get('LITE3_RUN_SENDER_INTEGRATION') != '1',
                                reason='Use isolated loopback sender runner')


def test_nav2_guard_handshake_and_stop_to_loopback_udp():
    from test_autonomy_integration import World
    world = World(output_mode='robot')
    # Robot-mode adapter permits ONLY the actual controller_server input owner.
    world.node.destroy_publisher(world.command_pub)
    root = Path(__file__).resolve().parents[1]
    sender = log = None
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as receiver:
            receiver.bind(('127.0.0.1', 0))
            receiver.setblocking(False)
            log = (Path(os.environ['LITE3_SENDER_RESULTS']) / 'robot_path_sender.log').open('w')
            sender = subprocess.Popen([
                str(root / '.colcon/install/lite3_locomotion/lib/lite3_locomotion/motion_sender'),
                '--target-ip', '127.0.0.1', '--target-port', str(receiver.getsockname()[1])],
                stdout=log, stderr=subprocess.STDOUT)

            def drain():
                packets = []
                while True:
                    try:
                        packet, _ = receiver.recvfrom(100)
                        packets.append(struct.unpack('<iiid', packet))
                    except BlockingIOError:
                        return packets

            def enable(command='enable'):
                client = world.node.create_client(world.trigger, '/lite3_robot_link/' + command)
                try:
                    world.wait(client.service_is_ready)
                    future = client.call_async(world.trigger.Request())
                    world.wait(future.done)
                    response = future.result()
                    assert response.success, response.message
                finally:
                    world.node.destroy_client(client)

            world.wait(lambda: world.mission.get('nav2_active'), 25)
            world.spin(0.3)
            assert not world.enabled
            assert drain() == []
            enable()
            world.wait(lambda: world.status.get('health') == 'healthy', 3)
            world.control('start')
            world.wait(lambda: world.enabled, 3)
            world.spin(0.5)
            packets = drain()
            assert any(p[0] in (320, 321) and abs(p[3]) > 1e-6 for p in packets)
            assert all(p[1:3] == (8, 1) for p in packets)
            assert all(abs(p[3]) <= (0.03 if p[0] == 320 else 0.10) for p in packets)
            assert all(p[3] == 0 for p in packets if p[0] == 325)
            world.control('stop')
            world.wait(lambda: not world.enabled, 2)
            world.spin(0.2)
            drain()
            world.spin(0.25)
            assert all(p[3] == 0 for p in drain())
            # New sender session MUST disarm the adapter and demand a new arm.
            enable()
            world.spin(0.3)
            assert not world.enabled
            assert all(p[3] == 0 for p in drain())
            world.wait(lambda: world.status.get('health') == 'healthy', 3)
            world.control('start')
            world.wait(lambda: world.enabled, 3)
            world.spin(0.4)
            assert any(abs(p[3]) > 1e-6 for p in drain())
            # Stop/re-enable ONLY the sender while the real adapter is armed.
            # A missed disabled heartbeat must not allow previous authorization
            # to resume movement: the new token forces a fresh adapter arm.
            assert world.enabled
            enable('stop')
            enable()
            world.wait(lambda: not world.enabled, 2)
            world.spin(0.2)
            drain()
            world.spin(0.3)
            assert all(p[3] == 0 for p in drain())
            world.wait(lambda: world.status.get('health') == 'healthy', 3)
            world.control('start')
            world.wait(lambda: world.enabled, 3)
            world.spin(0.4)
            assert any(abs(p[3]) > 1e-6 for p in drain())
            world.control('stop')
    finally:
        if sender is not None and sender.poll() is None:
            sender.send_signal(signal.SIGINT)
            try:
                sender.wait(timeout=4)
            except subprocess.TimeoutExpired:
                sender.kill()
                sender.wait(timeout=2)
        if log is not None:
            log.close()
        world.close()
