"""Launch the real navigation.launch.py (dry) in an isolated loopback domain and verify the graph:
twist_mux -> /lite3_ops/mux_out -> collision_monitor (active, expected polygon) -> dry output,
and nobody else publishes the output or /cmd_vel. No robot, no live domains."""

import os
from pathlib import Path
import secrets
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[1]
DRY = '/lite3_locomotion/dry_cmd_vel'


def main():
    env = os.environ.copy()
    profile = ROOT / 'test' / 'loopback_dds.xml'
    for name in ('ROS_DISCOVERY_SERVER', 'ROS_SUPER_CLIENT', 'FASTDDS_BUILTIN_TRANSPORTS'):
        env.pop(name, None)
    domain = 40 + secrets.randbelow(50)
    env.update({'ROS_DOMAIN_ID': str(domain), 'ROS_LOCALHOST_ONLY': '1',
                'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
                'FASTRTPS_DEFAULT_PROFILES_FILE': str(profile),
                'FASTDDS_DEFAULT_PROFILES_FILE': str(profile)})
    clearance, speed = sys.argv[1:3] if len(sys.argv) >= 3 else ('0.40', '0.05')
    physical = 'physical' in sys.argv[3:]
    extra = ['mux_output:=/cmd_vel', 'allow_robot_motion:=true'] if physical else []
    env['LITE3_CHAIN_OUTPUT'] = '/cmd_vel' if physical else DRY
    print(f'Isolated domain {domain}; launching navigation.launch.py clearance={clearance} '
          f'max_speed={speed} (dry)', flush=True)
    log = open('/tmp/lite3_launch_chain.log', 'w')
    launch = subprocess.Popen(['ros2', 'launch', 'lite3_locomotion', 'navigation.launch.py',
                               f'clearance:={clearance}', f'max_speed:={speed}'] + extra,
                              env=env, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
    try:
        time.sleep(25)
        probe = subprocess.run([sys.executable, '-c', PROBE, clearance, speed], env=env,
                               capture_output=True, text=True, timeout=60)
        print(probe.stdout + probe.stderr)
        return probe.returncode
    finally:
        for sig, wait in ((signal.SIGINT, 10), (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
            try:
                os.killpg(launch.pid, sig)
                launch.wait(timeout=wait)
                break
            except subprocess.TimeoutExpired:
                continue
            except ProcessLookupError:
                break


PROBE = r'''
import sys, time
import rclpy
from rcl_interfaces.srv import GetParameters
from lifecycle_msgs.srv import GetState
sys.path.insert(0, "%s")
from lite3_locomotion import collision_profile as CP
import os
OUT = os.environ["LITE3_CHAIN_OUTPUT"]
rclpy.init(); n = rclpy.create_node("lite3_chain_probe")
time.sleep(3)
def pubs(t): return sorted(i.node_name for i in n.get_publishers_info_by_topic(t))
def subs(t): return sorted(i.node_name for i in n.get_subscriptions_info_by_topic(t))
def call(cls, srv, req):
    c = n.create_client(cls, srv)
    if not c.wait_for_service(timeout_sec=5): return None
    f = c.call_async(req); rclpy.spin_until_future_complete(n, f, timeout_sec=5); return f.result()
state = call(GetState, "/collision_monitor/get_state", GetState.Request())
params = call(GetParameters, "/collision_monitor/get_parameters",
              GetParameters.Request(names=["PolygonStop.points", "cmd_vel_in_topic", "cmd_vel_out_topic"]))
want = CP.boundary(float(sys.argv[1]), float(sys.argv[2]), CP.Calibration())["boundary_m"]
got = round(max(params.values[0].double_array_value[0::2]) - CP.NOSE_M, 3) if params else None
checks = [
    ("collision_monitor lifecycle", state and state.current_state.label == "active", state and state.current_state.label),
    ("twist_mux -> mux_out", pubs("/lite3_ops/mux_out") == ["twist_mux"], pubs("/lite3_ops/mux_out")),
    ("collision_monitor reads mux_out", "collision_monitor" in subs("/lite3_ops/mux_out"), subs("/lite3_ops/mux_out")),
    ("collision_monitor reads hazards", "collision_monitor" in subs("/lite3_terrain/hazards"), subs("/lite3_terrain/hazards")),
    ("only collision_monitor publishes the output " + OUT, pubs(OUT) == ["collision_monitor"], pubs(OUT)),
    ("/cmd_vel: dry -> nobody, physical -> only collision_monitor",
     pubs("/cmd_vel") == ([] if OUT != "/cmd_vel" else ["collision_monitor"]), pubs("/cmd_vel")),
    ("physical: dead-man lock wired" if OUT == "/cmd_vel" else "dry: no dead-man config",
     ("twist_mux" in subs("/lite3_ops/hold")) == (OUT == "/cmd_vel"), subs("/lite3_ops/hold")),
    ("stop polygon boundary (default calibration)", got == want, f"{got} m (expected {want})"),
]
ok = True
for name, good, detail in checks:
    ok &= bool(good); print(("PASS " if good else "FAIL ") + f"{name}: {detail}")
print("RESULT:", "PASS" if ok else "FAIL"); sys.exit(0 if ok else 1)
''' % (str(ROOT),)

if __name__ == '__main__':
    sys.exit(main())
