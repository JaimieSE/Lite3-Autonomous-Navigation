"""Command logger event logic (command_log.py): pure, no ROS."""
import json

from lite3_movement_tests import command_log as C
from lite3_movement_tests.ros_io import GAITS, HEARTBEAT_CODE, POSTURE_TOGGLE


def make():
    recs = []
    return C.CommandLog(recs.append), recs


def pub(node, host, key, durability='volatile'):
    return {'node': node, 'host': host, 'participant': key, 'reliability': 'reliable',
            'durability': durability, 'key': key}


def events(recs, kind=None):
    return [r for r in recs if kind is None or r['event'] == kind]


def test_code_names_and_host_id():
    assert C.code_name(POSTURE_TOGGLE).startswith('posture_toggle')
    assert C.code_name(HEARTBEAT_CODE) == 'heartbeat'
    assert C.code_name(GAITS['rug_hstep']) == 'gait rug_hstep'
    assert C.code_name(0x12345678) == 'unknown'
    assert C.code_name(-1) == 'unknown'                      # int32 on the wire: masked to uint32
    assert C.host_id([1, 15, 0x51, 0x98, 0, 0]) == '5198'


def test_hosts_are_named_from_what_is_learnt():
    h = C.Hosts()
    h.learn('28e0', 'this-pc')
    h.learn('28e0', 'system')                                # first name wins
    assert h.label('28e0') == 'this-pc' and h.label('5198') == 'other:5198'


def test_posture_toggle_is_logged_every_time_with_the_publishers_present():
    log, recs = make()
    log.on_graph(0.0, 64, '/simple_cmd', [pub('_NODE_NAME_UNKNOWN_', 'other:5198', 'a')])
    log.on_simple(1.0, '/simple_cmd', POSTURE_TOGGLE)
    log.on_simple(1.5, '/simple_cmd', POSTURE_TOGGLE)          # stand then sit: both matter
    cmds = events(recs, 'command')
    assert len(cmds) == 2 and all(c['code'] == hex(POSTURE_TOGGLE) for c in cmds)
    assert cmds[0]['publishers'] == [{'node': '_NODE_NAME_UNKNOWN_', 'host': 'other:5198', 'participant': 'a',
                                      'reliability': 'reliable', 'durability': 'volatile'}]


def test_heartbeat_is_one_record_plus_a_stream_summary():
    log, recs = make()
    for i in range(10):
        log.on_simple(i * 0.35, '/simple_cmd', HEARTBEAT_CODE)
    assert len(events(recs, 'command')) == 1
    log.tick(3.15 + C.STREAM_GAP_S + 0.1)
    end, = events(recs, 'command_stream_end')
    assert end['count'] == 10 and end['duration_s'] == 3.15 and end['rate_hz'] == 2.9


def test_a_code_after_a_silence_starts_a_new_record():
    log, recs = make()
    log.on_simple(0.0, '/simple_cmd', GAITS['flat_slow'])
    log.on_simple(10.0, '/simple_cmd', GAITS['flat_slow'])
    assert len(events(recs, 'command')) == 2
    assert not events(recs, 'command_stream_end')            # single messages need no summary


def test_motion_start_and_stop_with_peaks():
    log, recs = make()
    log.on_graph(0.0, 64, '/cmd_vel', [pub('lite3_domain_bridge_64', 'system', 'b')])
    log.on_twist(1.0, '/cmd_vel', 0.0, 0.0, 0.0)              # zeros while idle: nothing
    log.on_twist(1.1, '/cmd_vel', 0.2, 0.0, 0.1)
    log.on_twist(1.2, '/cmd_vel', 0.5, -0.1, 0.0)
    log.on_twist(1.3, '/cmd_vel', 0.0, 0.0, 0.0)
    start, = events(recs, 'motion_start')
    stop, = events(recs, 'motion_stop')
    assert start['vx'] == 0.2 and start['publishers'][0]['node'] == 'lite3_domain_bridge_64'
    assert stop['why'] == 'zero command' and stop['peak_vx'] == 0.5 and stop['peak_vy'] == 0.1
    assert stop['count'] == 2 and stop['duration_s'] == 0.2


def test_motion_that_goes_silent_without_a_zero_is_closed():
    log, recs = make()
    log.on_twist(1.0, '/cmd_vel', 0.3, 0.0, 0.0)
    log.tick(1.0 + C.STREAM_GAP_S + 0.5)
    stop, = events(recs, 'motion_stop')
    assert stop['why'] == 'commands stopped without a zero'


def test_publishers_present_join_and_leave_per_domain():
    log, recs = make()
    bridge = pub('lite3_domain_bridge_64', 'system', 'b', 'transient_local')
    foreign = pub('_NODE_NAME_UNKNOWN_', 'other:5198', 'f')
    log.on_graph(0.0, 64, '/simple_cmd', [bridge])
    log.on_graph(0.0, 8, '/simple_cmd', [])
    log.on_graph(0.5, 64, '/simple_cmd', [bridge, foreign])
    log.on_graph(0.5, 8, '/simple_cmd', [])
    log.on_graph(1.0, 64, '/simple_cmd', [bridge])
    present, = events(recs, 'publisher_present')
    joined, = events(recs, 'publisher_joined')
    left, = events(recs, 'publisher_left')
    assert present['node'] == 'lite3_domain_bridge_64' and present['durability'] == 'transient_local'
    assert joined['host'] == left['host'] == 'other:5198' and joined['domain'] == 64
    assert 'key' not in joined


def test_close_flushes_open_streams_and_motion():
    log, recs = make()
    log.on_simple(0.0, '/simple_cmd', HEARTBEAT_CODE)
    log.on_simple(0.5, '/simple_cmd', HEARTBEAT_CODE)
    log.on_twist(0.5, '/cmd_vel', 0.1, 0.0, 0.0)
    log.close(1.0)
    assert events(recs, 'command_stream_end') and events(recs, 'motion_stop')


def test_sink_writes_daily_jsonl_and_show_hides_heartbeats(tmp_path, capsys):
    sink = C.JsonlSink(tmp_path, echo=False)
    log = C.CommandLog(sink)
    log.on_simple(1795450000.0, '/simple_cmd', HEARTBEAT_CODE)
    log.on_simple(1795450001.0, '/simple_cmd', POSTURE_TOGGLE)
    files = list(tmp_path.glob('commands-*.jsonl'))
    assert len(files) == 1
    lines = [json.loads(line) for line in files[0].read_text().splitlines()]
    assert [r['name'] for r in lines] == ['heartbeat', 'posture_toggle (stand/sit)']
    C.show(tmp_path, 10)
    out = capsys.readouterr().out
    assert 'posture_toggle' in out and 'heartbeat' not in out
    C.show(tmp_path, 10, everything=True)
    assert 'heartbeat' in capsys.readouterr().out


def test_summary_lines_name_the_sender():
    rec = {'t': 1.0, 'time': '2026-11-23T16:00:00.000', 'event': 'command', 'topic': '/simple_cmd',
           'code': hex(POSTURE_TOGGLE), 'name': 'posture_toggle (stand/sit)',
           'publishers': [{'node': 'x', 'host': 'other:5198'}]}
    assert C.summary(rec) == '16:00:00.000 /simple_cmd 0x21010202 posture_toggle (stand/sit) from x@other:5198'
