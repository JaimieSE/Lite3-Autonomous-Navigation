"""Verify twist_mux + terrain lock behavior in an isolated loopback DDS domain (no robot).

Starts the real twist_mux with the university topic config and params/twist_mux_locks.yaml,
output remapped to /mux_out. Exit 0 only if every expectation holds.
"""

import os
from pathlib import Path
import secrets
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
UNIVERSITY_TOPICS = Path('/home/user/ros2_ws/src/lite3_launcher/config/twist_mux_topics.yaml')


def isolated_env():
    env = os.environ.copy()
    profile = ROOT / 'test' / 'loopback_dds.xml'
    for name in ('ROS_DISCOVERY_SERVER', 'ROS_SUPER_CLIENT', 'FASTDDS_BUILTIN_TRANSPORTS'):
        env.pop(name, None)
    env.update({'ROS_DOMAIN_ID': str(40 + secrets.randbelow(50)), 'ROS_LOCALHOST_ONLY': '1',
                'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
                'FASTRTPS_DEFAULT_PROFILES_FILE': str(profile),
                'FASTDDS_DEFAULT_PROFILES_FILE': str(profile), 'LITE3_MUX_CHECK': '1'})
    return env


def scenario():
    import threading
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from geometry_msgs.msg import Twist, Vector3
    from std_msgs.msg import Bool

    rclpy.init()
    node = Node('lite3_mux_lock_check')
    nav = node.create_publisher(Twist, '/nav_cmd_vel', 10)
    joy = node.create_publisher(Twist, '/joy_cmd_vel', 10)
    lock = node.create_publisher(Bool, '/lite3_terrain/stop', 10)
    out = []
    node.create_subscription(Twist, '/mux_out', lambda m: out.append((time.monotonic(), m.linear.x)), 100)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    def phase(seconds, nav_x=None, joy_x=None, lock_value=None):
        start = time.monotonic()
        while time.monotonic() - start < seconds:
            if nav_x is not None:
                nav.publish(Twist(linear=Vector3(x=nav_x)))
            if joy_x is not None:
                joy.publish(Twist(linear=Vector3(x=joy_x)))
            if lock_value is not None:
                lock.publish(Bool(data=lock_value))
            time.sleep(0.05)
        # Outputs received during the second half of the phase (after any transition).
        return [x for t, x in out if start + seconds * 0.5 <= t < start + seconds]

    phase(3.0, lock_value=False)                                        # discovery
    r = {
        'unlocked_nav (0.081)': phase(2.0, nav_x=0.081, lock_value=False),
        'locked_nav (0.082)': phase(2.0, nav_x=0.082, lock_value=True),
        'lock_silent_nav (0.083)': phase(2.0, nav_x=0.083),
        'released_nav (0.084)': phase(2.0, nav_x=0.084, lock_value=False),
        'locked_nav+joy (0.085/0.05)': phase(2.0, nav_x=0.085, joy_x=0.05, lock_value=True),
    }
    for k, v in r.items():
        print(f'{k:<28} messages={len(v):<4} values={sorted(set(round(x, 3) for x in v))}')
    v = list(r.values())
    ok = (v[0] and set(v[0]) == {0.081} and not any(x == 0.082 for x in v[1]) and
          not any(x == 0.083 for x in v[2]) and v[3] and set(v[3]) == {0.084} and
          v[4] and set(v[4]) == {0.05})
    blocked_outputs = v[1] + v[2]
    print('mux output while blocked:', 'silent' if not blocked_outputs else sorted(set(blocked_outputs)))
    print('RESULT:', 'PASS' if ok else 'FAIL')
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


def main():
    if os.environ.get('LITE3_MUX_CHECK') == '1':
        return scenario()
    env = isolated_env()
    print(f'Isolated domain {env["ROS_DOMAIN_ID"]}, loopback only', flush=True)
    mux = subprocess.Popen(
        ['ros2', 'run', 'twist_mux', 'twist_mux', '--ros-args',
         '--params-file', str(UNIVERSITY_TOPICS),
         '--params-file', str(ROOT / 'params' / 'twist_mux_locks.yaml'),
         '-r', 'cmd_vel_out:=/mux_out'],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    try:
        return subprocess.call([sys.executable, __file__], env=env, timeout=60)
    finally:
        os.killpg(mux.pid, signal.SIGINT)
        try:
            mux.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(mux.pid, signal.SIGKILL)


if __name__ == '__main__':
    sys.exit(main())
