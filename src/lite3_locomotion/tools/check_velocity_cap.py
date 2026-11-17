"""Verify the M5 overspeed fix on the real nav2_velocity_smoother in an isolated loopback domain.

Root cause (physical M5, 2026-11-16): Humble MPPI logs "Optimizer reset" 2-4 ms after "Received a
goal"; the reset restores the base vx_max (0.10), so a /speed_limit published before the goal is
lost and Nav2 commanded 0.064-0.098 m/s against a 0.05 m/s authorization. The fix sets the
velocity_smoother's dynamic max_velocity (downstream of MPPI, not reset per goal) before each goal
and reads it back. This tool starts the smoother with the generated Lite3 parameters, feeds
0.30 m/s, sets max_velocity to [0.05, 0, 0.2] over the parameter service and checks the output.
No robot. Exit 0 only if every expectation holds.
"""

import os
from pathlib import Path
import secrets
import signal
import subprocess
import sys
import tempfile
import time

ROOT = Path(__file__).resolve().parents[1]
CAP = 0.05


def isolated_env():
    env = os.environ.copy()
    profile = ROOT / 'test' / 'loopback_dds.xml'
    for name in ('ROS_DISCOVERY_SERVER', 'ROS_SUPER_CLIENT', 'FASTDDS_BUILTIN_TRANSPORTS'):
        env.pop(name, None)
    env.update({'ROS_DOMAIN_ID': str(40 + secrets.randbelow(50)), 'ROS_LOCALHOST_ONLY': '1',
                'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
                'FASTRTPS_DEFAULT_PROFILES_FILE': str(profile),
                'FASTDDS_DEFAULT_PROFILES_FILE': str(profile), 'LITE3_CAP_CHECK': '1'})
    return env


def scenario():
    import threading
    import rclpy
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.node import Node
    from geometry_msgs.msg import Twist, Vector3
    from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
    from rcl_interfaces.srv import GetParameters, SetParameters

    rclpy.init()
    node = Node('lite3_cap_check')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)
    out = []
    node.create_subscription(Twist, '/cmd_vel_smoothed',
                             lambda m: out.append((time.monotonic(), m.linear.x)), 100)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()

    def call(client, request):
        if not client.wait_for_service(timeout_sec=10.0):
            return None
        future = client.call_async(request)
        deadline = time.monotonic() + 5.0
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        return future.result() if future.done() else None

    def feed(seconds, vx):
        start = time.monotonic()
        while time.monotonic() - start < seconds:
            pub.publish(Twist(linear=Vector3(x=vx)))
            time.sleep(0.05)
        tail = [x for t, x in out if t >= start + seconds * 0.5]
        return max(tail) if tail else None

    feed(6.0, 0.0)                                     # discovery + lifecycle activation
    before = feed(2.0, 0.30)
    setter = node.create_client(SetParameters, '/velocity_smoother/set_parameters')
    getter = node.create_client(GetParameters, '/velocity_smoother/get_parameters')
    result = call(setter, SetParameters.Request(parameters=[Parameter(
        name='max_velocity', value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                                                  double_array_value=[CAP, 0.0, 0.2]))]))
    accepted = bool(result and result.results[0].successful)
    got = call(getter, GetParameters.Request(names=['max_velocity']))
    readback = list(got.values[0].double_array_value) if got and got.values else []
    after = feed(2.0, 0.30)
    all_after = [x for t, x in out if t >= time.monotonic() - 1.0]
    checks = [
        ('uncapped: 0.30 m/s input passes (ceiling 0.5)', before is not None and abs(before - 0.30) < 1e-6),
        (f'max_velocity [{CAP}, 0, 0.2] accepted by the parameter service', accepted),
        (f'read back {readback}', len(readback) == 3 and abs(readback[0] - CAP) < 1e-9),
        (f'capped: 0.30 m/s input -> output max {after}', after is not None and after <= CAP + 1e-9),
        ('no output above the cap in the last second', bool(all_after) and max(all_after) <= CAP + 1e-9),
    ]
    ok = True
    for name, good in checks:
        ok &= good
        print(f'{"PASS" if good else "FAIL"}  {name}')
    print('RESULT:', 'PASS' if ok else 'FAIL')
    executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


def main():
    if os.environ.get('LITE3_CAP_CHECK') == '1':
        return scenario()
    import yaml
    env = isolated_env()
    nav2 = yaml.safe_load((ROOT / 'params' / 'nav2_lite3.yaml').read_text())
    params = {'velocity_smoother': nav2['velocity_smoother']}
    params['velocity_smoother']['ros__parameters']['use_sim_time'] = False
    handle, path = tempfile.mkstemp(suffix='.yaml')
    with os.fdopen(handle, 'w') as stream:
        yaml.safe_dump(params, stream)
    print(f'Isolated domain {env["ROS_DOMAIN_ID"]}; velocity_smoother params '
          f'{params["velocity_smoother"]["ros__parameters"]}', flush=True)
    procs = [subprocess.Popen(cmd, env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                              start_new_session=True) for cmd in (
        ['ros2', 'run', 'nav2_velocity_smoother', 'velocity_smoother', '--ros-args',
         '--params-file', path],
        ['ros2', 'run', 'nav2_lifecycle_manager', 'lifecycle_manager', '--ros-args',
         '-r', '__node:=lifecycle_manager_vs', '-p', 'autostart:=true',
         '-p', 'node_names:=[velocity_smoother]'])]
    try:
        return subprocess.call([sys.executable, __file__], env=env, timeout=120)
    finally:
        for p in procs:
            try:
                os.killpg(p.pid, signal.SIGINT)
                p.wait(timeout=5)
            except (subprocess.TimeoutExpired, ProcessLookupError):
                os.killpg(p.pid, signal.SIGKILL)
        os.unlink(path)


if __name__ == '__main__':
    sys.exit(main())
