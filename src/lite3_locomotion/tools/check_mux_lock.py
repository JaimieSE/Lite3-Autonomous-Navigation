"""Verify twist_mux + terrain lock behavior in an isolated loopback DDS domain (no robot).

Starts the real twist_mux with the university topic config and params/twist_mux_locks.yaml,
output remapped to /mux_out. Exit 0 only if every expectation holds.

--physical additionally loads params/twist_mux_physical.yaml (as navigation.launch.py does
with allow_robot_motion:=true) and verifies the operator dead-man lock, the lockable test
input and the zeros-only safety_stop input that overrides the locks.
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


def physical_scenario(phase, node):
    from geometry_msgs.msg import Twist, Vector3
    from std_msgs.msg import Bool
    test = node.create_publisher(Twist, '/lite3_ops/test_cmd_vel', 10)
    stop = node.create_publisher(Twist, '/lite3_ops/stop_cmd_vel', 10)
    hold = node.create_publisher(Bool, '/lite3_ops/hold', 10)

    def run(seconds, hold_value=None, test_x=None, stop_zero=False, **kw):
        extra = []
        if hold_value is not None:
            extra.append(lambda: hold.publish(Bool(data=hold_value)))
        if test_x is not None:
            extra.append(lambda: test.publish(Twist(linear=Vector3(x=test_x))))
        if stop_zero:
            extra.append(lambda: stop.publish(Twist()))
        return phase(seconds, extra=extra, **kw)

    run(2.0, hold_value=False, lock_value=False)
    r = {
        'idle: hold silent, nav 0.061': run(2.0, nav_x=0.061, lock_value=False),
        'hold silent, test 0.062': run(2.0, test_x=0.062, lock_value=False),
        'hold released, test 0.063': run(2.0, hold_value=False, test_x=0.063, lock_value=False),
        'hold released, nav 0.064': run(2.0, hold_value=False, nav_x=0.064, lock_value=False),
        'hold released, terrain stop, test 0.065': run(2.0, hold_value=False, test_x=0.065,
                                                       lock_value=True),
        'hold true, test 0.066': run(2.0, hold_value=True, test_x=0.066, lock_value=False),
        'locked + stop zeros (test 0.067)': run(2.0, test_x=0.067, stop_zero=True, lock_value=False),
        'released + stop zeros (test 0.068)': run(2.0, hold_value=False, test_x=0.068, stop_zero=True,
                                                  lock_value=False),
        'released, stop silent, test 0.069': run(2.0, hold_value=False, test_x=0.069,
                                                 lock_value=False),
    }
    for k, v in r.items():
        print(f'{k:<42} messages={len(v):<4} values={sorted(set(round(x, 3) for x in v))}')
    v = list(r.values())
    blocked = [v[0], v[1], v[4], v[5]]
    ok = (all(not x for x in blocked) and v[2] and set(v[2]) == {0.063} and v[3] and
          set(v[3]) == {0.064} and v[6] and set(v[6]) == {0.0} and v[7] and set(v[7]) == {0.0}
          and v[8] and set(v[8]) == {0.069})
    return ok


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

    def phase(seconds, nav_x=None, joy_x=None, lock_value=None, extra=()):
        start = time.monotonic()
        while time.monotonic() - start < seconds:
            for publish in extra:
                publish()
            if nav_x is not None:
                nav.publish(Twist(linear=Vector3(x=nav_x)))
            if joy_x is not None:
                joy.publish(Twist(linear=Vector3(x=joy_x)))
            if lock_value is not None:
                lock.publish(Bool(data=lock_value))
            time.sleep(0.05)
        # Outputs received during the second half of the phase (after any transition).
        return [x for t, x in out if start + seconds * 0.5 <= t < start + seconds]

    never_held = []
    if os.environ.get('LITE3_MUX_PHYSICAL') == '1':
        # Fresh mux, dead-man NEVER published: Nav2 commands from the very start must not pass.
        never_held = phase(3.0, nav_x=0.051, lock_value=False)
        never_held = [x for t, x in out if x == 0.051]
        print(f'fresh mux, dead-man never published, nav 0.051: messages={len(never_held)}')
    else:
        phase(3.0, lock_value=False)                                    # discovery
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
    if os.environ.get('LITE3_MUX_PHYSICAL') == '1':
        # Dead-man not held in these phases: navigation must be blocked even when the terrain
        # lock is clear; the handheld joystick (priority 100) still passes.
        ok = not any(v[:4]) and v[4] and set(v[4]) == {0.05} and not never_held
        print('base phases with dead-man not held: navigation blocked throughout' if not any(v[:4])
              else 'base phases: NAVIGATION PASSED WITHOUT DEAD-MAN')
    blocked_outputs = v[1] + v[2]
    print('mux output while blocked:', 'silent' if not blocked_outputs else sorted(set(blocked_outputs)))
    if os.environ.get('LITE3_MUX_PHYSICAL') == '1':
        physical = physical_scenario(phase, node)
        print('physical dead-man / test / safety_stop inputs:', 'PASS' if physical else 'FAIL')
        ok = ok and physical
    print('RESULT:', 'PASS' if ok else 'FAIL')
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


def main():
    if os.environ.get('LITE3_MUX_CHECK') == '1':
        return scenario()
    env = isolated_env()
    physical = '--physical' in sys.argv[1:]
    env['LITE3_MUX_PHYSICAL'] = '1' if physical else '0'
    print(f'Isolated domain {env["ROS_DOMAIN_ID"]}, loopback only', flush=True)
    mux = subprocess.Popen(
        ['ros2', 'run', 'twist_mux', 'twist_mux', '--ros-args',
         '--params-file', str(UNIVERSITY_TOPICS),
         '--params-file', str(ROOT / 'params' / 'twist_mux_locks.yaml')] +
        (['--params-file', str(ROOT / 'params' / 'twist_mux_physical.yaml')] if physical else []) +
        ['-r', 'cmd_vel_out:=/mux_out'],
        env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
    try:
        return subprocess.call([sys.executable, __file__], env=env, timeout=120)
    finally:
        os.killpg(mux.pid, signal.SIGINT)
        try:
            mux.wait(timeout=5)
        except subprocess.TimeoutExpired:
            os.killpg(mux.pid, signal.SIGKILL)


if __name__ == '__main__':
    sys.exit(main())
