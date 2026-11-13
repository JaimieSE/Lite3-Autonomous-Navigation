"""Bounded Nav2 dry-run integration in a loopback-only random ROS domain."""
import os
from pathlib import Path
import secrets
import signal
import subprocess
import sys


def main():
    root = Path(__file__).resolve().parents[1]
    results = root / '.colcon/autonomy_integration'
    results.mkdir(parents=True, exist_ok=True)
    prefix = root / '.colcon/install/lite3_locomotion'
    if not (prefix / 'lib/lite3_locomotion/guarded_movement_adapter').is_file():
        raise SystemExit('Build the package into .colcon/install first')
    env = os.environ.copy()
    for key in ('ROS_DISCOVERY_SERVER', 'ROS_SUPER_CLIENT', 'FASTDDS_BUILTIN_TRANSPORTS',
                'RMW_FASTRTPS_USE_QOS_FROM_XML'):
        env.pop(key, None)
    profile = str(root / 'test/loopback_dds.xml')
    domain = str(40 + secrets.randbelow(50))
    env.update(ROS_DOMAIN_ID=domain, ROS_LOCALHOST_ONLY='1', RMW_IMPLEMENTATION='rmw_fastrtps_cpp',
               FASTRTPS_DEFAULT_PROFILES_FILE=profile, FASTDDS_DEFAULT_PROFILES_FILE=profile,
               ROS_LOG_DIR=str(results / 'ros_logs'), PYTHONDONTWRITEBYTECODE='1',
               AMENT_PREFIX_PATH=str(prefix) + os.pathsep + env.get('AMENT_PREFIX_PATH', ''),
               PYTHONPATH=str(prefix / 'lib/python3.10/site-packages') + os.pathsep + env.get('PYTHONPATH', ''),
               LITE3_RUN_AUTONOMY_INTEGRATION='1', LITE3_AUTONOMY_RESULTS=str(results))
    print(f'Nav2 dry-run: domain {domain}, loopback UDP only; results {results}', flush=True)
    with (results / 'pytest.log').open('w') as log:
        process = subprocess.Popen([sys.executable, '-m', 'pytest', '-v', '-s', '--tb=short',
                                    '-p', 'no:cacheprovider', '--junitxml=' + str(results / 'results.xml'),
                                    str(root / 'test/test_autonomy_integration.py')],
                                   cwd=results, env=env, stdout=log, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        try:
            code = process.wait(timeout=100)
        except (subprocess.TimeoutExpired, KeyboardInterrupt):
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3)
            code = 1
    print((results / 'pytest.log').read_text())
    return code


if __name__ == '__main__':
    raise SystemExit(main())
