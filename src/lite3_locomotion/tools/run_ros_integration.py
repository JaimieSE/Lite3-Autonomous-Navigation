"""Run opt-in integration tests with local-only DDS and package-local results."""

import os
from pathlib import Path
import secrets
import signal
import subprocess
import sys


def main():
    root = Path(__file__).resolve().parents[1]
    results = root / '.colcon' / 'integration'
    results.mkdir(parents=True, exist_ok=True)
    prefix = root / '.colcon' / 'install' / 'lite3_locomotion'
    executable = prefix / 'lib' / 'lite3_locomotion' / 'locomotion_controller'
    site_packages = prefix / 'lib' / f'python{sys.version_info.major}.{sys.version_info.minor}' / 'site-packages'
    if not executable.is_file() or not site_packages.is_dir():
        raise SystemExit('Build the package into .colcon/install first; see README.md')
    profile = str(root / 'test' / 'loopback_dds.xml')
    env = os.environ.copy()
    # No discovery-server forwarding, custom transport overrides, daemon, or SHM.
    for name in (
        'ROS_DISCOVERY_SERVER', 'ROS_SUPER_CLIENT', 'FASTDDS_BUILTIN_TRANSPORTS',
        'RMW_FASTRTPS_USE_QOS_FROM_XML',
    ):
        env.pop(name, None)
    domain = str(40 + secrets.randbelow(50))  # Separate from the audited robot domain 8.
    env.update({
        'ROS_DOMAIN_ID': domain,
        'ROS_LOCALHOST_ONLY': '1',
        'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
        'FASTRTPS_DEFAULT_PROFILES_FILE': profile,
        'FASTDDS_DEFAULT_PROFILES_FILE': profile,
        'ROS_LOG_DIR': str(results / 'ros_logs'),
        'PYTHONDONTWRITEBYTECODE': '1',
        'PYTHONPATH': str(site_packages) + os.pathsep + env.get('PYTHONPATH', ''),
        'LITE3_RUN_ROS_INTEGRATION': '1',
        'LITE3_TEST_EXECUTABLE': str(executable),
    })
    print(f'Isolated test: domain={domain}, loopback UDP only, no built-in DDS transports', flush=True)
    print(f'Results: {results}', flush=True)
    command = [
        sys.executable, '-m', 'pytest', '-v', '--tb=short', '-p', 'no:cacheprovider',
        '--junitxml=' + str(results / 'results.xml'),
        str(root / 'test' / 'test_ros_integration.py'),
    ]
    with (results / 'pytest.log').open('w') as log:
        process = subprocess.Popen(
            command, cwd=results, env=env, stdout=log, stderr=subprocess.STDOUT,
            start_new_session=True,
        )
        try:
            returncode = process.wait(timeout=90)
        except (subprocess.TimeoutExpired, KeyboardInterrupt):
            try:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3)
            except ProcessLookupError:
                pass
            print('FAIL: integration suite interrupted or exceeded 90 seconds', flush=True)
            return 1
    print((results / 'pytest.log').read_text(), end='')
    return returncode


if __name__ == '__main__':
    raise SystemExit(main())
