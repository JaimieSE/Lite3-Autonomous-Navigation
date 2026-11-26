"""Loopback-only, no-command startup/shutdown check for the review derivative."""

import json
import os
from pathlib import Path
import signal
import socket
import subprocess
import time
import uuid


def main():
    root = Path(__file__).resolve().parents[1]
    results = root / '.colcon' / 'review'
    results.mkdir(parents=True, exist_ok=True)
    executable = root / '.colcon/vendor_bridge_build/jetson2motion_shutdown_review'
    profile = str(root / 'test/loopback_dds.xml')
    env = os.environ.copy()
    for name in ('ROS_DISCOVERY_SERVER', 'ROS_SUPER_CLIENT', 'FASTDDS_BUILTIN_TRANSPORTS'):
        env.pop(name, None)
    env.update(ROS_DOMAIN_ID='77', ROS_LOCALHOST_ONLY='1',
               RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
               FASTRTPS_DEFAULT_PROFILES_FILE=profile,
               FASTDDS_DEFAULT_PROFILES_FILE=profile,
               ROS_LOG_DIR=str(results / 'shutdown_ros_logs'))
    namespace = '/lite3_shutdown_check_' + uuid.uuid4().hex[:8]
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as monitor:
        monitor.bind(('127.0.0.1', 0))
        monitor.settimeout(0.05)
        with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as reservation:
            reservation.bind(('127.0.0.1', 0))
            receive_port = reservation.getsockname()[1]
        command = [str(executable), '--ros-args', '-r', '__ns:=' + namespace,
                   '-p', 'target_ip:=127.0.0.1',
                   '-p', 'target_port:=' + str(monitor.getsockname()[1]),
                   '-p', 'local_port:=' + str(receive_port)]
        for topic in ('/imu/data', '/handle_state', '/us_publisher/ultrasound_distance', '/tf', '/tf_static'):
            command.extend(['-r', topic + ':=' + namespace + topic])
        with (results / 'shutdown_check.log').open('w') as log:
            process = subprocess.Popen(command, env=env, stdout=log, stderr=subprocess.STDOUT)
            try:
                time.sleep(1.0)
                if process.poll() is not None:
                    raise RuntimeError('Bridge failed before the shutdown test; inspect its log')
                start = time.monotonic()
                process.send_signal(signal.SIGINT)
                returncode = process.wait(timeout=2.0)
                elapsed = time.monotonic() - start
                if returncode != 0:
                    raise RuntimeError(f'Unexpected bridge exit: {returncode}')
                try:
                    monitor.recvfrom(4096)
                except socket.timeout:
                    pass
                else:
                    raise RuntimeError('Unexpected outbound UDP packet during no-command test')
            finally:
                if process.poll() is None:
                    process.kill()
                    process.wait(timeout=2.0)
    report = dict(result='PASS', target='127.0.0.1 only', commands_published=0,
                  outbound_udp_packets=0, sigint_exit_code=returncode,
                  shutdown_seconds=round(elapsed, 3), independent_physical_stop_verified=False)
    (results / 'shutdown_check.json').write_text(json.dumps(report, indent=2) + '\n')
    print(json.dumps(report, indent=2))


if __name__ == '__main__':
    main()
