"""M10 independent safety watchdog: a separate process with its own ROS node and subscriptions.

It never publishes while the roaming runner is alive (so the runner's exclusive-wiring check stays
clean). Every 0.1 s it evaluates WatchdogLogic (telemetry freshness, tilt, raw LiDAR liveness,
robot-side command speed, leg-odometry geofence, runner heartbeat). On a violation it TRIPS:

  1. SIGINT to the runner (it cancels Nav2, commands zeros above every lock, verifies rest);
  2. if the runner has not stopped within 1.5 s: SIGKILL - the twist_mux dead-man then locks
     every Nav2/test input within 0.5 s;
  3. once the runner is gone (also after a runner crash): zeros on the priority-255 stop input and
     hold=true on the dead-man input for 3 s (--publish-stop, physical sessions only).

It writes <dir>/watchdog.json every 0.1 s; the runner stops roaming if that heartbeat is lost.
"""

import argparse
import json
import os
import signal
import sys
import time

from .roam import ControlChannel, WatchdogLogic

ACTIVE = ('ROAMING', 'PAUSED')


def alive(pid):
    try:
        os.kill(pid, 0)
        return True
    except OSError:
        return False


def publish_stop(robot, seconds=3.0):
    from geometry_msgs.msg import Twist
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool
    from .ros_io import OPS_HOLD_TOPIC, OPS_STOP_TOPIC
    node = robot._nodes[robot.wiring.velocity_domain]
    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                     durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
    stop = node.create_publisher(Twist, OPS_STOP_TOPIC, qos)
    hold = node.create_publisher(Bool, OPS_HOLD_TOPIC, qos)
    deadline = time.monotonic() + 1.0
    while time.monotonic() < deadline and stop.get_subscription_count() < 1:
        time.sleep(0.05)
    end = time.monotonic() + seconds
    while time.monotonic() < end:
        stop.publish(Twist())
        hold.publish(Bool(data=True))
        time.sleep(0.05)


def main(argv=None):
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--runner-pid', type=int, required=True)
    p.add_argument('--dir', required=True)
    p.add_argument('--speed-limit', type=float, required=True)
    p.add_argument('--geofence', type=float, required=True)
    p.add_argument('--heartbeat', type=float, default=2.0)
    p.add_argument('--publish-stop', action='store_true')
    p.add_argument('--direct-telemetry', action='store_true')
    a = p.parse_args(argv)
    from .roam_ros import WatchdogRobot
    from .ros_io import NAV_WIRING, direct_telemetry
    for s in (signal.SIGINT, signal.SIGTERM):
        signal.signal(s, signal.SIG_IGN)            # only the runner's end stops the watchdog
    channel = ControlChannel(a.dir)
    robot = WatchdogRobot(direct_telemetry(NAV_WIRING) if a.direct_telemetry else NAV_WIRING)
    logic = WatchdogLogic(a.speed_limit, a.geofence, heartbeat_s=a.heartbeat)
    started = time.monotonic()
    state = {'pid': os.getpid(), 'ready': False, 'tripped': False, 'reasons': [], 'runner_pid': a.runner_pid}

    def write():
        state['heartbeat'] = time.time()
        tmp = channel.watchdog_path.with_suffix('.tmp')
        tmp.write_text(json.dumps(state))
        tmp.replace(channel.watchdog_path)

    print(f'watchdog started for runner {a.runner_pid}: speed <= {a.speed_limit} m/s, geofence '
          f'{a.geofence} m, heartbeat {a.heartbeat} s, publish stop {a.publish_stop}', flush=True)
    last_runner = {}
    code = 0
    try:
        while True:
            now = time.monotonic()
            runner = channel.read_json(channel.status_path) or {}
            if runner.get('pid') == a.runner_pid:
                last_runner = runner
            if not alive(a.runner_pid):
                if last_runner.get('state') in ACTIVE or last_runner.get('state') == 'READINESS':
                    print('runner died while active: commanding zeros', flush=True)
                    state.update(tripped=True, reasons=['runner process died while active'])
                    write()
                    if a.publish_stop:
                        publish_stop(robot)
                    code = 1
                break
            if last_runner.get('state') in ('STOPPED', 'SAFETY_STOP'):
                break
            ready = robot.telemetry.joints.rate(now) > 20 and now - started > 2.0
            state['ready'] = ready
            if ready and not state['tripped']:
                active = last_runner if last_runner.get('state') in ACTIVE else None
                if last_runner.get('state') == 'READINESS':
                    logic.heartbeat_s, active = 15.0, dict(last_runner, state='ROAMING')
                else:
                    logic.heartbeat_s = a.heartbeat
                reasons = logic.evaluate(robot.telemetry, now, active, time.time())
                for note in logic.notes:
                    print(note, flush=True)
                logic.notes.clear()
                if reasons:
                    state.update(tripped=True, reasons=reasons)
                    write()
                    print(f'TRIP: {reasons}', flush=True)
                    os.kill(a.runner_pid, signal.SIGINT)
                    deadline = time.monotonic() + 1.5
                    while time.monotonic() < deadline and alive(a.runner_pid):
                        st = (channel.read_json(channel.status_path) or {}).get('state')
                        if st in ('STOPPING', 'SAFETY_STOP', 'STOPPED'):
                            break
                        time.sleep(0.05)
                    st = (channel.read_json(channel.status_path) or {}).get('state')
                    if alive(a.runner_pid) and st not in ('STOPPING', 'SAFETY_STOP', 'STOPPED'):
                        print('runner did not stop: SIGKILL (dead-man locks within 0.5 s)', flush=True)
                        os.kill(a.runner_pid, signal.SIGKILL)
                    code = 1
            write()
            time.sleep(0.1)
        # The runner has ended. After a trip, make sure zeros are commanded once it is gone.
        if state['tripped'] and a.publish_stop:
            deadline = time.monotonic() + 20.0          # let the runner finish its own safe stop
            while time.monotonic() < deadline and alive(a.runner_pid):
                write()
                time.sleep(0.1)
            if not alive(a.runner_pid):
                publish_stop(robot)
    finally:
        write()
        robot.close()
    return code


if __name__ == '__main__':
    sys.exit(main())
