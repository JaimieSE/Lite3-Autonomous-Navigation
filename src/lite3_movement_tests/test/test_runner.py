"""Runner behavior against the virtual-clock simulator (no ROS, no robot)."""

from pathlib import Path

import pytest

from lite3_movement_tests.config import load_sequence
from lite3_movement_tests.runner import (
    MODE_READONLY, MODE_SIM, SequenceRunner, PASSED, FAILED, ABORTED, NOT_RUN, SKIPPED, PREVIEW)
from lite3_movement_tests.sim_model import FakeRobot, SimLite3
from lite3_movement_tests.telemetry import Telemetry

SEQUENCE = Path(__file__).resolve().parents[1] / 'config' / 'basic_walk.yaml'


def run(fault='none', start='lying', mode=MODE_SIM, healthy=True, can_actuate=True, cancel_at=None):
    robot = FakeRobot(Telemetry(), SimLite3(fault=fault, start=start), healthy=healthy,
                      can_actuate=can_actuate)
    if cancel_at is not None:
        original = robot.wait

        def wait(seconds):
            original(seconds)
            moving = [e[0] for e in robot.sent if e[1] == 'vel' and any(e[2:])]
            if moving and robot.t > moving[0] + cancel_at:
                robot.cancel = True
        robot.wait = wait
    report = SequenceRunner(load_sequence(SEQUENCE), robot, mode, log=lambda *_: None).run()
    return report, robot, {s['id']: s for s in report['steps']}


def test_nominal_sequence_passes_and_returns_to_lying():
    report, robot, steps = run()
    assert report['verdict'] == 'PASSED', report
    assert all(s['status'] == PASSED for s in report['steps'])
    assert [e[1] for e in robot.sent].count('toggle') == 2
    forward = steps['forward_short']['measurements']['measured']['forward_m']
    assert 0.03 <= forward <= 0.15
    assert report['final_state']['posture']['posture'] == 'lying'
    assert not report['state_uncertain']


def test_already_standing_skips_stand_toggle():
    report, robot, steps = run(start='standing')
    assert steps['stand']['status'] == SKIPPED
    assert report['verdict'] == 'PASSED'
    assert [e[1] for e in robot.sent].count('toggle') == 1  # final lie down only


def test_marching_in_place_fails_then_stops_and_lies_down():
    report, robot, steps = run(fault='march_in_place')
    assert steps['forward_short']['status'] == FAILED
    assert 'marching in place' in steps['forward_short']['measurements']['diagnosis']
    assert steps['stop_after_forward']['status'] == NOT_RUN
    assert report['safe_stop']['rest_verified'] is True
    assert steps['lie_down']['status'] == PASSED
    assert report['verdict'] == 'FAILED_SAFE'


def test_ignored_posture_toggle_latches_and_requests_intervention():
    report, robot, steps = run(fault='ignore_posture')
    assert steps['stand']['status'] == FAILED
    assert report['posture_commands_latched_off']
    assert report['verdict'] == 'INTERVENTION_REQUIRED'
    assert steps['lie_down']['status'] == NOT_RUN
    assert [e[1] for e in robot.sent].count('toggle') == 1  # never retried
    assert not any(e[1] == 'vel' and e[2] for e in robot.sent)


def test_slow_stand_beyond_deadline_fails_without_retry():
    report, robot, steps = run(fault='slow_stand')
    assert steps['stand']['status'] == FAILED
    assert [e[1] for e in robot.sent].count('toggle') == 1
    assert report['verdict'] == 'INTERVENTION_REQUIRED'


def test_robot_that_keeps_moving_after_zeros_requires_intervention():
    report, robot, steps = run(fault='no_stop')
    assert steps['forward_short']['status'] in (FAILED, PASSED)
    assert steps['stop_after_forward']['status'] in (FAILED, NOT_RUN)
    assert report['safe_stop'] is None or report['safe_stop']['rest_verified'] is False
    assert report['verdict'] == 'INTERVENTION_REQUIRED'
    assert steps['lie_down']['status'] == NOT_RUN


@pytest.mark.parametrize('fault, reason', [
    ('telemetry_dropout', 'joints telemetry stale'),
    ('tilt', 'tilt'),
    ('remote_takeover', 'operator takeover'),
    ('runaway', 'geofence'),
])
def test_safety_monitor_aborts_motion_and_sends_zeros(fault, reason):
    report, robot, steps = run(fault=fault)
    move = steps['forward_short']
    assert move['status'] == ABORTED and reason in move['summary']
    assert report['verdict'] == 'INTERVENTION_REQUIRED'
    last_nonzero = max(t for t, kind, *v in robot.sent if kind == 'vel' and any(v))
    zeros_after = [e for e in robot.sent if e[0] > last_nonzero and e[1] == 'vel']
    assert len(zeros_after) >= 30 and all(not any(e[2:]) for e in zeros_after)
    assert move['measurements']['commanded_s'] < 2.0
    assert steps['lie_down']['status'] == NOT_RUN


def test_unhealthy_command_path_sends_nothing():
    report, robot, steps = run(healthy=False)
    assert steps['readiness']['status'] == FAILED
    assert robot.sent == []
    assert steps['lie_down']['status'] == NOT_RUN
    assert report['verdict'] == 'FAILED_SAFE' and not report['commands_sent']


def test_cancel_during_motion_aborts_and_zeros():
    report, robot, steps = run(cancel_at=0.5)
    statuses = {s['status'] for s in report['steps']}
    assert ABORTED in statuses
    assert report['verdict'] == 'INTERVENTION_REQUIRED'
    assert robot.sent[-1][1] == 'vel' and robot.sent[-1][2:] == (0.0, 0.0)


def test_readonly_mode_never_actuates_and_previews_rest():
    report, robot, steps = run(mode=MODE_READONLY, can_actuate=False)
    assert robot.sent == []
    assert steps['readiness']['status'] == PASSED
    assert steps['telemetry_contract']['status'] == PASSED
    assert steps['initial_rest']['status'] == PASSED
    assert steps['stand']['status'] == PREVIEW and steps['lie_down']['status'] == PREVIEW
    assert report['verdict'] == 'READONLY_CHECKS_PASSED'


def test_readonly_mode_rejects_actuating_interface():
    robot = FakeRobot(Telemetry(), SimLite3())
    with pytest.raises(ValueError):
        SequenceRunner(load_sequence(SEQUENCE), robot, MODE_READONLY)


def test_physical_mode_requires_authorization():
    robot = FakeRobot(Telemetry(), SimLite3())
    with pytest.raises(ValueError):
        SequenceRunner(load_sequence(SEQUENCE), robot, 'physical')


ENVELOPE = Path(__file__).resolve().parents[1] / 'config' / 'envelope_step1.yaml'


def run_envelope(fault='none'):
    robot = FakeRobot(Telemetry(), SimLite3(fault=fault))
    report = SequenceRunner(load_sequence(ENVELOPE), robot, MODE_SIM, log=lambda *_: None).run()
    return report, robot, {s['id']: s for s in report['steps']}


def test_envelope_nominal_passes_and_silence_stop_is_measured():
    report, robot, steps = run_envelope()
    assert report['verdict'] == 'PASSED', [(s['id'], s['status'], s['summary']) for s in report['steps']]
    silence = steps['silence_stop_005']['measurements']['motion']['silence']
    assert silence['stopped'] and silence['rest_onset_after_s'] < 2.0
    assert steps['turn_left_02']['measurements']['measured']['yaw_change_rad'] > 0.3


def test_envelope_range_misses_continue_only_with_verified_stops():
    report, robot, steps = run_envelope('march_in_place')
    moves = [s for s in report['steps'] if s['type'] == 'velocity_move' and s['id'] != 'silence_stop_005']
    assert all(s['status'] == FAILED for s in moves[:6])          # linear moves do not translate
    assert all(steps['stop_' + s['id']]['status'] == PASSED for s in moves)
    assert len(report['range_failures_continued']) >= 6
    assert report['verdict'] == 'FAILED_SAFE'
    assert steps['lie_down']['status'] == PASSED


def test_envelope_detects_robot_that_keeps_walking_when_commands_stop():
    report, robot, steps = run_envelope('no_silence_stop')
    silence = steps['silence_stop_005']
    assert silence['status'] == FAILED and 'silence' in silence['summary']
    assert silence['measurements']['motion']['silence']['stopped'] is False
    last_nonzero = max(t for t, kind, *v in robot.sent if kind == 'vel' and any(v))
    assert any(kind == 'vel' and not any(v) and t > last_nonzero + 2.9 for t, kind, *v in robot.sent)
    assert report['safe_stop']['rest_verified'] is True
    assert report['verdict'] == 'FAILED_SAFE'


def test_range_failure_does_not_continue_past_unverified_stop():
    report, robot, steps = run_envelope('no_stop')
    assert report['verdict'] == 'INTERVENTION_REQUIRED'
    assert steps['lie_down']['status'] == NOT_RUN


def test_silence_stop_onset_within_timeout_passes_even_if_hold_extends_past_it():
    # Regression for the 2026-11-13 physical run: rest began ~1.6 s after silence, but the
    # 1.5 s hold window could not complete inside a 3 s timeout, so it was wrongly failed.
    report, robot, steps = run_envelope('slow_silence_stop')
    silence = steps['silence_stop_005']
    assert silence['status'] == PASSED, silence['summary']
    onset = silence['measurements']['motion']['silence']['rest_onset_after_s']
    assert 1.4 <= onset <= 3.0
