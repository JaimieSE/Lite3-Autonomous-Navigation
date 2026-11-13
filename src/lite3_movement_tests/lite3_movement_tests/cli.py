"""movement_test: automated, telemetry-verified Lite3 movement test sequences.

  plan                 show the sequence, limits and sha256 (no ROS)
  authorize-template   write a physical-session authorization to fill in (all false)
  dry-run --sim        full sequence against the simulator in an isolated loopback domain
  dry-run --live-readonly  live robot telemetry checks; creates NO publishers
  physical --authorization FILE   authorized bounded session; no per-step prompts

Exit codes: 0 passed, 1 failed with safe state verified, 2 intervention required,
3 refused before any command (configuration/authorization).
"""

import argparse
import datetime as dt
import json
import os
from pathlib import Path
import secrets
import signal
import subprocess
import sys
import threading
import time

import yaml

from .config import ConfigError, authorization_template, check_authorization, \
    load_authorization, load_sequence
from .report import write_reports
from .runner import MODE_PHYSICAL, MODE_READONLY, MODE_SIM, SequenceRunner

PACKAGE = 'lite3_movement_tests'
EXIT = {'PASSED': 0, 'READONLY_CHECKS_PASSED': 0, 'FAILED_SAFE': 1, 'INTERVENTION_REQUIRED': 2}


def share_dir():
    try:
        from ament_index_python.packages import get_package_share_directory
        return Path(get_package_share_directory(PACKAGE))
    except Exception:  # noqa: BLE001 - running from source
        return Path(__file__).resolve().parents[1]


def default_report_root():
    return Path(os.environ.get('LITE3_MOVEMENT_REPORTS',
                               Path.home() / 'lite3_ws' / 'log' / 'movement_tests'))


def parse(argv):
    parser = argparse.ArgumentParser(prog='movement_test', description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--sequence', default=str(share_dir() / 'config' / 'basic_walk.yaml'))
    parser.add_argument('--report-dir', default=None)
    sub = parser.add_subparsers(dest='command', required=True)
    sub.add_parser('plan')
    tpl = sub.add_parser('authorize-template')
    tpl.add_argument('--hours', type=float, default=2.0)
    tpl.add_argument('--output', required=True)
    dry = sub.add_parser('dry-run')
    group = dry.add_mutually_exclusive_group(required=True)
    group.add_argument('--sim', action='store_true')
    group.add_argument('--live-readonly', action='store_true')
    dry.add_argument('--fault', default='none')
    dry.add_argument('--start', choices=('lying', 'standing'), default='lying')
    phys = sub.add_parser('physical')
    phys.add_argument('--authorization', required=True)
    return parser.parse_args(argv)


def print_plan(seq):
    print(f'Sequence {seq.name}  sha256={seq.sha256}\n  source: {seq.source}')
    for label, steps in (('steps', seq.steps), ('finalize', seq.finalize)):
        print(f'{label}:')
        for s in steps:
            print(f'  - {s.id:<22} {s.type:<20} timeout={s.timeout_s:>5.1f}s '
                  f'{"ACTUATING " if s.actuating else ""}{json.dumps(s.params)}')
    print(f'safety: {seq.safety}\nposture: {seq.posture}\nstationary: {seq.stationary}')


def isolated_env():
    profile = share_dir() / 'config' / 'loopback_dds.xml'
    env = os.environ.copy()
    for name in ('ROS_DISCOVERY_SERVER', 'ROS_SUPER_CLIENT', 'FASTDDS_BUILTIN_TRANSPORTS',
                 'RMW_FASTRTPS_USE_QOS_FROM_XML'):
        env.pop(name, None)
    domain = 40 + secrets.randbelow(50)   # never the live domains 8 or 64
    env.update({'ROS_DOMAIN_ID': str(domain), 'ROS_LOCALHOST_ONLY': '1',
                'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
                'FASTRTPS_DEFAULT_PROFILES_FILE': str(profile),
                'FASTDDS_DEFAULT_PROFILES_FILE': str(profile),
                'LITE3_MT_ISOLATED': '1'})
    return env, domain


def wait_for_telemetry(robot, seconds=15.0):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        now = time.monotonic()
        if all(getattr(robot.telemetry, n).rate(now) > 50 for n in ('joints', 'odom', 'imu')):
            return True
        time.sleep(0.2)
    return False


def finish(report, args, mode):
    root = Path(args.report_dir) if args.report_dir else default_report_root()
    stamp = dt.datetime.now().strftime('%Y%m%d-%H%M%S')
    out = root / f'{stamp}-{mode}'
    paths = write_reports(report, out)
    print(f'\nVERDICT: {report["verdict"]}  ({report["evidence_class"]})')
    for s in report['steps']:
        print(f'  {s["status"]:<13} {s["id"]:<22} {s["summary"]}')
    print(f'Report: {paths["markdown"]}\nData:   {paths["json"]}')
    return EXIT[report['verdict']]


def run_sequence(seq, robot, mode, args, auth=None, cancel=None):
    if cancel is not None:
        handler = lambda *_: cancel.set()  # noqa: E731
        old = {s: signal.signal(s, handler) for s in (signal.SIGINT, signal.SIGTERM)}
    try:
        runner = SequenceRunner(seq, robot, mode, log=lambda m: print(m, flush=True),
                                authorization=auth)
        report = runner.run()
    finally:
        if cancel is not None:
            for s, h in old.items():
                signal.signal(s, h)
    return finish(report, args, mode)


def cmd_sim(seq, args):
    from .sim_model import FAULTS
    if args.fault not in FAULTS:
        raise ConfigError(f'unknown fault {args.fault}; choose from {FAULTS}')
    if os.environ.get('LITE3_MT_ISOLATED') != '1':
        env, domain = isolated_env()
        print(f'Isolated simulation: domain {domain}, loopback-only DDS', flush=True)
        return subprocess.call([sys.executable, '-m', 'lite3_movement_tests.cli'] + sys.argv[1:],
                               env=env)
    from .ros_io import RosRobot, sim_wiring
    domain = int(os.environ['ROS_DOMAIN_ID'])
    sim = subprocess.Popen([sys.executable, '-m', 'lite3_movement_tests.sim_node',
                            '--fault', args.fault, '--start', args.start],
                           start_new_session=True)
    cancel = threading.Event()
    robot = None
    try:
        robot = RosRobot(sim_wiring(domain), allow_commands=True, cancel_flag=cancel)
        if not wait_for_telemetry(robot):
            raise SystemExit('Simulator telemetry did not appear')
        return run_sequence(seq, robot, MODE_SIM, args, cancel=cancel)
    finally:
        if robot is not None:
            robot.close()
        sim.send_signal(signal.SIGINT)
        try:
            sim.wait(timeout=5)
        except subprocess.TimeoutExpired:
            sim.kill()


def cmd_live_readonly(seq, args):
    from .ros_io import LIVE_WIRING, RosRobot
    cancel = threading.Event()
    robot = RosRobot(LIVE_WIRING, allow_commands=False, cancel_flag=cancel)
    try:
        if not wait_for_telemetry(robot):
            print('WARNING: live telemetry not at expected rate; readiness will report it')
        return run_sequence(seq, robot, MODE_READONLY, args, cancel=cancel)
    finally:
        robot.close()


def cmd_physical(seq, args):
    auth = load_authorization(args.authorization)
    problems = check_authorization(auth, seq)
    if os.environ.get('ROS_LOCALHOST_ONLY') == '1' or os.environ.get('LITE3_MT_ISOLATED') == '1':
        problems.append('environment is isolated/localhost-only; cannot reach the robot')
    if problems:
        print('REFUSED - physical session not authorized; nothing was published:')
        for p in problems:
            print(f'  - {p}')
        return 3
    from .ros_io import LIVE_WIRING, RosRobot
    print(f'Authorized session {auth.session} until {auth.valid_until.isoformat()}')
    print_plan(seq)
    print('Starting in 3 s without further prompts. Ctrl+C aborts (zeros are sent if moving).',
          flush=True)
    cancel = threading.Event()
    old = {s: signal.signal(s, lambda *_: cancel.set()) for s in (signal.SIGINT, signal.SIGTERM)}
    try:
        cancel.wait(3.0)
    finally:
        for s, h in old.items():
            signal.signal(s, h)
    if cancel.is_set():
        print('Cancelled before start; nothing was published.')
        return 3
    robot = RosRobot(LIVE_WIRING, allow_commands=True, cancel_flag=cancel)
    try:
        if not wait_for_telemetry(robot):
            print('WARNING: live telemetry not at expected rate; readiness will fail safely')
        return run_sequence(seq, robot, MODE_PHYSICAL, args, auth=auth, cancel=cancel)
    finally:
        robot.close()


def main(argv=None):
    args = parse(sys.argv[1:] if argv is None else argv)
    try:
        seq = load_sequence(args.sequence)
        if args.command == 'plan':
            print_plan(seq)
            return 0
        if args.command == 'authorize-template':
            out = Path(args.output)
            if out.exists():
                raise ConfigError(f'{out} exists; refusing to overwrite an authorization')
            out.write_text('# Set each confirmation to true only after checking it physically.\n'
                           + yaml.safe_dump(authorization_template(seq, args.hours), sort_keys=False))
            print(f'Wrote {out}; edit confirmations, then run: movement_test physical '
                  f'--authorization {out}')
            return 0
        if args.command == 'dry-run':
            return cmd_sim(seq, args) if args.sim else cmd_live_readonly(seq, args)
        return cmd_physical(seq, args)
    except ConfigError as error:
        print(f'REFUSED: {error}', file=sys.stderr)
        return 3


if __name__ == '__main__':
    raise SystemExit(main())
