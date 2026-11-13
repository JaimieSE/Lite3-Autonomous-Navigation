"""Opt-in sender checks: isolated DDS and localhost UDP, never the robot."""

import os
from pathlib import Path
import secrets
import signal
import subprocess
import sys


def main():
    root = Path(__file__).resolve().parents[1]
    results = root / '.colcon/motion_sender_integration'
    results.mkdir(parents=True, exist_ok=True)
    prefix = root / '.colcon/install/lite3_locomotion'
    if not (prefix / 'lib/lite3_locomotion/motion_sender').is_file():
        raise SystemExit('Build into .colcon/install first')
    env = os.environ.copy()
    for key in ('ROS_DISCOVERY_SERVER', 'ROS_SUPER_CLIENT', 'FASTDDS_BUILTIN_TRANSPORTS',
                'RMW_FASTRTPS_USE_QOS_FROM_XML'):
        env.pop(key, None)
    profile = str(root / 'test/loopback_dds.xml')
    env.update(ROS_DOMAIN_ID=str(40 + secrets.randbelow(50)), ROS_LOCALHOST_ONLY='1',
               RMW_IMPLEMENTATION='rmw_fastrtps_cpp', FASTRTPS_DEFAULT_PROFILES_FILE=profile,
               FASTDDS_DEFAULT_PROFILES_FILE=profile, ROS_LOG_DIR=str(results / 'ros_logs'),
               PYTHONDONTWRITEBYTECODE='1', LITE3_RUN_SENDER_INTEGRATION='1',
               LITE3_SENDER_RESULTS=str(results),
               LITE3_AUTONOMY_RESULTS=str(results),
               AMENT_PREFIX_PATH=str(prefix) + os.pathsep + env.get('AMENT_PREFIX_PATH', ''),
               PYTHONPATH=str(prefix / 'lib/python3.10/site-packages') + os.pathsep + env.get('PYTHONPATH', ''))
    print('Sender test: isolated domain ' + env['ROS_DOMAIN_ID'] + ', loopback only', flush=True)
    with (results / 'pytest.log').open('w') as log:
        process = subprocess.Popen([sys.executable, '-m', 'pytest', '-v', '-s', '--tb=short',
                                    '-p', 'no:cacheprovider', '--junitxml=' + str(results / 'results.xml'),
                                    str(root / 'test/test_motion_sender_integration.py'),
                                    str(root / 'test/test_robot_output_loopback.py')],
                                   cwd=results, env=env, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        try:
            code = process.wait(timeout=90)
        except (subprocess.TimeoutExpired, KeyboardInterrupt):
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3)
            code = 1
    print((results / 'pytest.log').read_text())
    return code


if __name__ == '__main__':
    raise SystemExit(main())
