"""Operator 2026-11-26: SLAM takes no new scans while in AI mode; resumed when AI mode ends."""
from types import SimpleNamespace

from lite3_movement_tests import roam as R


def _session(answers):
    calls, events, logs = [], [], []

    class Robot:
        can_actuate = True

        def slam_toggle_pause(self):
            calls.append('toggle')
            return answers.pop(0)
    s = SimpleNamespace(robot=Robot(), event=lambda name, **kw: events.append((name, kw)), log=logs.append)
    pause = lambda on, why: R.__dict__[ROAM_CLASS]._slam_pause(s, on, why)   # noqa: E731
    return s, pause, calls, events, logs


ROAM_CLASS = next(n for n, v in vars(R).items() if isinstance(v, type) and hasattr(v, '_slam_pause'))


def test_pauses_once_and_resumes_once():
    s, pause, calls, events, _ = _session([{'answered': True, 'status': True}] * 2)
    pause(True, 'AI mode on')
    pause(True, 'AI mode on')                     # already paused: no second toggle
    pause(False, 'AI mode off')
    pause(False, 'AI mode off')                   # already running: no toggle
    assert calls == ['toggle', 'toggle'] and [e[0] for e in events] == ['slam_paused', 'slam_resumed']


def test_an_unanswered_pause_is_never_toggled_back():
    s, pause, calls, events, _ = _session([{'answered': False, 'reason': 'not available'}])
    pause(True, 'AI mode on')
    pause(False, 'AI mode off')                   # we never paused it: toggling would pause SLAM
    assert calls == ['toggle'] and events[0][1]['answered'] is False


def test_an_unanswered_resume_warns_and_is_retried_on_the_next_exit():
    s, pause, calls, events, logs = _session([{'answered': True, 'status': True},
                                              {'answered': False, 'reason': 'no answer'},
                                              {'answered': True, 'status': True}])
    pause(True, 'AI mode on')
    pause(False, 'AI mode off')
    assert logs and 'SLAM may still be paused' in logs[0]
    pause(False, 'session ending')
    assert calls == ['toggle'] * 3 and events[-1][0] == 'slam_resumed' and events[-1][1]['answered']


def test_a_robot_without_the_service_is_left_alone():
    s = SimpleNamespace(robot=SimpleNamespace(can_actuate=True), event=None, log=None)
    R.__dict__[ROAM_CLASS]._slam_pause(s, True, 'AI mode on')   # no slam_toggle_pause: nothing happens
