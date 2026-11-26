"""Robot command logger: who commanded the robot, and what (operator 2026-11-23: the dog stood up with nobody
in group 8 commanding it; another machine was running a fake_lite3 test on domain 64, and nothing recorded it).

Read-only. It only SUBSCRIBES, best effort (a reliable reader could make the bridge's writer wait for us), to the
robot command topics in the system domain (64), where the vendor motion_sender takes them:
  /simple_cmd, /complex_cmd  (transfer_interfaces)  - every posture toggle, gait, AI-mode code, ...
  /cmd_vel, /cmd_vel_corrected (Twist)              - motion start / stop with peak speeds
and watches who PUBLISHES those topics in the system domain and in the group domains (8, 7), where the lab bridge
picks them up. Humble's rclpy does not say which publisher sent a message, so each command record carries the
publishers present at that moment, and every publisher joining or leaving is a record of its own: a foreign
publisher appearing, a toggle arriving and the publisher leaving is the trail.

Hosts come from the DDS GUID (bytes 2-3, Fast DDS host id): this PC is learnt from our own reader, the system
computer from motion_sender's reader. A best-effort reader can miss a message on a lossy link; the publisher
join/leave records do not depend on it.

Records are JSON lines in log/operator/command-log/commands-YYYYMMDD.jsonl. Event logic is pure: unit-tested.
"""

import argparse
import datetime as dt
import json
import math
import os
from pathlib import Path
import signal
import sys
import threading
import time

from .ros_io import (AI_ENTER, AI_EXIT, AI_GAIT_BASIC, AI_OPTIONS_CODE, AXIS_FORWARD, GAITS, HEARTBEAT_CODE,
                     NAVIGATION_MODE, POSTURE_TOGGLE)

SYSTEM_DOMAIN = 64
GROUP_DOMAINS = (8, 7)
SIMPLE_TOPICS = ('/simple_cmd', '/complex_cmd')
TWIST_TOPICS = ('/cmd_vel', '/cmd_vel_corrected')
COMMAND_TOPICS = SIMPLE_TOPICS + TWIST_TOPICS
STREAM_GAP_S = 2.0          # the same code again within this: part of one stream (heartbeat, 20 Hz axis commands)
TWIST_ZERO = 1e-4           # |component| at or below this counts as zero
GRAPH_PERIOD_S = 0.5
SYSTEM_NODES = ('motion_sender', 'motion_receiver')
NODE_PREFIX = 'lite3_command_log'

CODE_NAMES = {POSTURE_TOGGLE: 'posture_toggle (stand/sit)', HEARTBEAT_CODE: 'heartbeat',
              AI_OPTIONS_CODE: 'ai_options', AI_ENTER: 'ai_enter', AI_EXIT: 'ai_exit',
              AI_GAIT_BASIC: 'ai_gait_basic', NAVIGATION_MODE: 'navigation_mode', AXIS_FORWARD: 'axis_forward'}
CODE_NAMES.update({code: f'gait {name}' for name, code in GAITS.items()})
ALWAYS_SINGLE = {POSTURE_TOGGLE}   # never folded into a stream: two toggles in a row are stand AND sit


def code_name(code):
    return CODE_NAMES.get(code & 0xFFFFFFFF, 'unknown')


def host_id(gid):
    """Fast DDS host id (GUID prefix bytes 2-3) as 4 hex digits."""
    return bytes(gid[2:4]).hex() if len(gid) >= 4 else '????'


def participant(gid):
    return bytes(gid[:12]).hex()


class Hosts:
    """Names hosts from what we learn: our own reader's host is this PC, motion_sender's is the system computer."""

    def __init__(self):
        self.names = {}

    def learn(self, host, name):
        self.names.setdefault(host, name)

    def label(self, host):
        return self.names.get(host, f'other:{host}')


def endpoint(info, hosts):
    """Graph TopicEndpointInfo -> plain dict (node, host, QoS)."""
    gid = list(info.endpoint_gid)
    qos = info.qos_profile
    return {'node': info.node_name, 'host': hosts.label(host_id(gid)), 'participant': participant(gid),
            'reliability': qos.reliability.name.lower(), 'durability': qos.durability.name.lower(),
            'key': bytes(gid).hex()}


class CommandLog:
    """Turns command messages and graph snapshots into records. emit(record) receives each one."""

    def __init__(self, emit, system_domain=SYSTEM_DOMAIN):
        self.emit = emit
        self.system_domain = system_domain
        self.streams = {}       # (topic, code) -> {'start', 'last', 'count', 'publishers'}
        self.motion = {}        # topic -> {'start', 'last', 'count', 'peak'}
        self.publishers = {}    # (domain, topic) -> {key: endpoint}

    def _record(self, t, event, **fields):
        rec = {'t': round(t, 3), 'time': dt.datetime.fromtimestamp(t).isoformat(timespec='milliseconds'),
               'event': event}
        rec.update(fields)
        self.emit(rec)

    def _present(self, topic):
        return [{k: v for k, v in e.items() if k != 'key'}
                for e in self.publishers.get((self.system_domain, topic), {}).values()]

    def on_simple(self, t, topic, code, size=0, kind=0, data=None):
        code &= 0xFFFFFFFF
        extra = {} if data is None else {'data': data}
        if code not in ALWAYS_SINGLE:
            s = self.streams.get((topic, code))
            if s is not None and t - s['last'] <= STREAM_GAP_S:
                s['last'], s['count'] = t, s['count'] + 1
                return
            if s is not None:
                self._end_stream(topic, code, s)
            self.streams[(topic, code)] = {'start': t, 'last': t, 'count': 1}
        self._record(t, 'command', topic=topic, code=hex(code), name=code_name(code), size=size, type=kind,
                     publishers=self._present(topic), **extra)

    def _end_stream(self, topic, code, s):
        if s['count'] > 1:
            span = s['last'] - s['start']
            self._record(s['last'], 'command_stream_end', topic=topic, code=hex(code), name=code_name(code),
                         count=s['count'], duration_s=round(span, 2),
                         rate_hz=round((s['count'] - 1) / span, 1) if span > 0 else None)

    def on_twist(self, t, topic, vx, vy, wz):
        moving = max(abs(vx), abs(vy), abs(wz)) > TWIST_ZERO
        m = self.motion.get(topic)
        if moving:
            if m is None:
                self.motion[topic] = m = {'start': t, 'count': 0, 'peak': [0.0, 0.0, 0.0]}
                self._record(t, 'motion_start', topic=topic, vx=round(vx, 3), vy=round(vy, 3), wz=round(wz, 3),
                             publishers=self._present(topic))
            m['last'], m['count'] = t, m['count'] + 1
            m['peak'] = [max(p, abs(v)) for p, v in zip(m['peak'], (vx, vy, wz))]
        elif m is not None:
            self._end_motion(t, topic, 'zero command')

    def _end_motion(self, t, topic, why):
        m = self.motion.pop(topic)
        self._record(t, 'motion_stop', topic=topic, why=why, duration_s=round(t - m['start'], 2),
                     count=m['count'], peak_vx=round(m['peak'][0], 3), peak_vy=round(m['peak'][1], 3),
                     peak_wz=round(m['peak'][2], 3))

    def on_graph(self, t, domain, topic, endpoints):
        """endpoints: current publishers (dicts from endpoint()); records joins and leaves."""
        now = {e['key']: e for e in endpoints}
        before = self.publishers.get((domain, topic))
        self.publishers[(domain, topic)] = now
        if before is None:
            for e in now.values():
                self._record(t, 'publisher_present', domain=domain, topic=topic,
                             **{k: v for k, v in e.items() if k != 'key'})
            return
        for key in now.keys() - before.keys():
            self._record(t, 'publisher_joined', domain=domain, topic=topic,
                         **{k: v for k, v in now[key].items() if k != 'key'})
        for key in before.keys() - now.keys():
            self._record(t, 'publisher_left', domain=domain, topic=topic,
                         **{k: v for k, v in before[key].items() if k != 'key'})

    def tick(self, t):
        """Close streams and motions that went silent (a twist stream that stops without a zero)."""
        for (topic, code), s in list(self.streams.items()):
            if t - s['last'] > STREAM_GAP_S:
                del self.streams[(topic, code)]
                self._end_stream(topic, code, s)
        for topic, m in list(self.motion.items()):
            if t - m['last'] > STREAM_GAP_S:
                self._end_motion(m['last'], topic, 'commands stopped without a zero')

    def close(self, t):
        self.tick(math.inf)


def summary(rec):
    """One human-readable line for a record."""
    e, when = rec['event'], rec['time'][11:23]
    pubs = rec.get('publishers')
    who = '' if pubs is None else ' from ' + (', '.join(f"{p['node']}@{p['host']}" for p in pubs) or 'no publisher')
    if e == 'command':
        return f"{when} {rec['topic']} {rec['code']} {rec['name']}{who}"
    if e == 'command_stream_end':
        return f"{when} {rec['topic']} {rec['name']} stream ended: {rec['count']} msgs, {rec['duration_s']} s"
    if e == 'motion_start':
        return f"{when} {rec['topic']} MOVE vx={rec['vx']} vy={rec['vy']} wz={rec['wz']}{who}"
    if e == 'motion_stop':
        return f"{when} {rec['topic']} stop ({rec['why']}) after {rec['duration_s']} s, peak vx={rec['peak_vx']}"
    if e.startswith('publisher_'):
        return (f"{when} [domain {rec['domain']}] {rec['topic']} {e[10:]}: {rec['node']}@{rec['host']} "
                f"({rec['reliability']}, {rec['durability']})")
    return f"{when} {e} {json.dumps({k: v for k, v in rec.items() if k not in ('t', 'time', 'event')})}"


class JsonlSink:
    """Append records to a per-day file; flush every line so a crash or kill loses nothing."""

    def __init__(self, directory, echo=True):
        self.dir = Path(directory)
        self.dir.mkdir(parents=True, exist_ok=True)
        self.echo = echo
        self.lock = threading.Lock()

    def __call__(self, rec):
        line = json.dumps(rec, separators=(',', ':'))
        with self.lock:
            with open(self.dir / f"commands-{rec['time'][:10].replace('-', '')}.jsonl", 'a') as f:
                f.write(line + '\n')
            if self.echo:
                print(summary(rec), flush=True)


# ------------------------------------------------------------------------------------------------- ROS layer

def _node(domain, name):
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node
    context = Context()
    rclpy.init(args=[], context=context, domain_id=domain)
    return context, Node(name, context=context)


def run(out_dir, group_domains=GROUP_DOMAINS, seconds=None, echo=True, system_domain=SYSTEM_DOMAIN):
    from geometry_msgs.msg import Twist
    from rclpy.executors import MultiThreadedExecutor
    from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
    from transfer_interfaces.msg import MotionComplexCMD, MotionSimpleCMD

    sink = JsonlSink(out_dir, echo=echo)
    log = CommandLog(sink, system_domain)
    lock = threading.Lock()
    hosts = Hosts()
    qos = QoSProfile(depth=200, reliability=ReliabilityPolicy.BEST_EFFORT, history=HistoryPolicy.KEEP_LAST)
    nodes = {d: _node(d, f'{NODE_PREFIX}_d{d}') for d in (system_domain, *group_domains)}
    system = nodes[system_domain][1]

    def simple(topic):
        def cb(msg):
            with lock:
                log.on_simple(time.time(), topic, msg.cmd_code, msg.size, msg.type, getattr(msg, 'data', None))
        return cb

    def twist(topic):
        def cb(msg):
            with lock:
                log.on_twist(time.time(), topic, msg.linear.x, msg.linear.y, msg.angular.z)
        return cb

    system.create_subscription(MotionSimpleCMD, '/simple_cmd', simple('/simple_cmd'), qos)
    system.create_subscription(MotionComplexCMD, '/complex_cmd', simple('/complex_cmd'), qos)
    for topic in TWIST_TOPICS:
        system.create_subscription(Twist, topic, twist(topic), qos)

    def poll():
        t = time.time()
        own = system.get_subscriptions_info_by_topic('/simple_cmd')
        for info in own:
            if info.node_name.startswith(NODE_PREFIX):
                hosts.learn(host_id(list(info.endpoint_gid)), 'this-pc')
            elif info.node_name in SYSTEM_NODES:
                hosts.learn(host_id(list(info.endpoint_gid)), 'system')
        with lock:
            for domain, (_, node) in nodes.items():
                for topic in COMMAND_TOPICS:
                    pubs = [endpoint(i, hosts) for i in node.get_publishers_info_by_topic(topic)]
                    log.on_graph(t, domain, topic, pubs)
            log.tick(t)

    executor = MultiThreadedExecutor(num_threads=2, context=nodes[system_domain][0])
    executor.add_node(system)
    stop = threading.Event()
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stop.set())
    spin = threading.Thread(target=lambda: _spin(executor, stop), daemon=True)
    spin.start()
    with lock:
        sink({'t': time.time(), 'time': dt.datetime.now().isoformat(timespec='milliseconds'),
              'event': 'logger_started', 'pid': os.getpid(), 'domains': [system_domain, *group_domains]})
    time.sleep(2.0)                              # discovery before the first snapshot, so 'present' is complete
    end = math.inf if seconds is None else time.monotonic() + seconds
    while not stop.is_set() and time.monotonic() < end:
        poll()
        stop.wait(GRAPH_PERIOD_S)
    with lock:
        log.close(time.time())
        sink({'t': time.time(), 'time': dt.datetime.now().isoformat(timespec='milliseconds'),
              'event': 'logger_stopped', 'pid': os.getpid()})
    stop.set()
    spin.join(timeout=2.0)                       # no callback may run while the nodes are torn down
    executor.shutdown(timeout_sec=1.0)
    for context, node in nodes.values():
        node.destroy_node()
        context.try_shutdown()


def _spin(executor, stop):
    while not stop.is_set():
        executor.spin_once(timeout_sec=0.1)


def show(out_dir, lines, everything=False):
    """Print the last records; heartbeats and stream ends are hidden unless everything=True."""
    files = sorted(Path(out_dir).glob('commands-*.jsonl'))
    recs = []
    for path in reversed(files):
        with open(path) as f:
            recs = [json.loads(line) for line in f if line.strip()] + recs
        if len(recs) >= lines * 4:
            break
    if not everything:
        recs = [r for r in recs if r.get('name') != 'heartbeat' and r['event'] != 'command_stream_end']
    for rec in recs[-lines:]:
        print(summary(rec))


def main(argv=None):
    default_dir = Path(os.environ.get('LITE3_COMMAND_LOG_DIR',
                                      Path.home() / 'lite3_ws' / 'log' / 'operator' / 'command-log'))
    p = argparse.ArgumentParser(prog='command_log', description=__doc__.split('\n\n')[0])
    sub = p.add_subparsers(dest='cmd', required=True)
    r = sub.add_parser('run', help='log until stopped (read-only)')
    r.add_argument('--out', default=str(default_dir))
    r.add_argument('--group-domains', default=','.join(map(str, GROUP_DOMAINS)),
                   help='group domains whose command publishers are watched (graph only)')
    r.add_argument('--seconds', type=float, default=None)
    r.add_argument('--quiet', action='store_true')
    r.add_argument('--system-domain', type=int, default=SYSTEM_DOMAIN,
                   help='testing only: an isolated domain standing in for the robot (default 64)')
    s = sub.add_parser('show', help='print recent records')
    s.add_argument('-n', type=int, default=40)
    s.add_argument('--all', action='store_true', help='include heartbeats and stream summaries')
    s.add_argument('--out', default=str(default_dir))
    a = p.parse_args(argv)
    if a.cmd == 'show':
        show(a.out, a.n, a.all)
        return 0
    domains = tuple(int(d) for d in a.group_domains.split(',') if d.strip())
    if a.system_domain in domains:
        p.error('the system domain is always logged; list only group domains')
    run(a.out, domains, a.seconds, echo=not a.quiet, system_domain=a.system_domain)
    return 0


if __name__ == '__main__':
    sys.exit(main())
