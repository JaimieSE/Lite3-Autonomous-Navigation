"""Test 4 pass/fail logic on synthetic recordings (no ROS)."""

import importlib.util
from pathlib import Path

import pytest

SPEC = importlib.util.spec_from_file_location(
    'nav_dry_soak', Path(__file__).resolve().parents[1] / 'tools' / 'nav_dry_soak.py')
T4 = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(T4)
TOTAL = 300.0


def scene(t, *, base_obstacle=None, base_step=None, box=True, leak=False, p1_stops=False):
    """What the terrain monitor reports at time t for an operator following the procedure."""
    obstacle, step, stop, reason = base_obstacle, base_step, False, None
    if box and 75 <= t < 120:
        obstacle = 1.5
    if box and 135 <= t < 180:
        obstacle, stop, reason = 0.55, True, 'hazard 0.55 m ahead'
    if box and 195 <= t < 240:
        step = 1.3
    if box and 250 <= t < 280 and int(t) % 6 < 3:
        obstacle = 1.4
    if p1_stops and 20 <= t < 40:
        stop, reason = True, 'hazard within 0.45 m'
    return obstacle, step, stop, reason


def recording(**kwargs):
    leak = kwargs.pop('leak', False)
    robot_cmd = kwargs.pop('robot_cmd', False)
    rec = T4.Recorder()
    t = 0.0
    while t < TOTAL:
        obstacle, step, stop, reason = scene(t, **kwargs)
        rec.add('odom', (t,))
        rec.add('status', (t, {'nearest_obstacle_m': obstacle, 'nearest_step_m': step,
                               'nearest_drop_m': None, 'clear_ahead_m': 1.0}, reason, []))
        rec.add('stop', (t, stop))
        for dt in (0.0, 0.05):
            rec.add('nav_in', (t + dt, 0.1, 0.0))
            if not stop or leak:
                rec.add('dry', (t + dt, 0.1, 0.0, 0.05))
        t = round(t + 0.1, 3)
    if robot_cmd:
        rec.add('robot_cmd', (150.0, 0.1))
    return rec


def results(rec, windows=None):
    verdict, checks = T4.evaluate(rec, TOTAL, aborted=False, windows=windows)
    return verdict, {c['check']: c for c in checks}


def find(checks, prefix):
    return next(c for name, c in checks.items() if name.startswith(prefix))


def test_operator_following_procedure_passes_and_lock_is_proven():
    verdict, checks = results(recording())
    assert verdict == 'PASS', [c for c in checks.values() if c['result'] != 'PASS']
    assert '(proven)' in find(checks, 'LOCK')['check']


def test_static_obstacle_matching_box_distance_is_inconclusive_not_pass():
    verdict, checks = results(recording(base_obstacle=1.5, box=False))
    assert find(checks, 'P2 box')['result'] == 'INCONCLUSIVE'
    assert verdict in ('FAIL', 'INCONCLUSIVE')


def test_no_operator_action_fails_detection_checks():
    verdict, checks = results(recording(box=False))
    assert verdict == 'FAIL'
    assert find(checks, 'P2 box')['result'] == 'FAIL'
    assert find(checks, 'P3 proximity')['result'] == 'FAIL'


def test_commands_passing_mux_during_stop_fail_the_lock():
    verdict, checks = results(recording(leak=True))
    assert find(checks, 'LOCK')['result'] == 'FAIL' and verdict == 'FAIL'


def test_any_robot_side_command_fails():
    verdict, checks = results(recording(robot_cmd=True))
    assert find(checks, 'NON-ACTUATING')['result'] == 'FAIL' and verdict == 'FAIL'


def test_cluttered_surroundings_fail_p1():
    verdict, checks = results(recording(p1_stops=True))
    assert find(checks, 'P1 surroundings')['result'] == 'FAIL'


def test_aborted_run_is_reported_as_aborted():
    verdict, _ = T4.evaluate(recording(), 100.0, aborted=True)
    assert verdict == 'ABORTED'


@pytest.mark.parametrize('episodes, expected', [
    ([(0, True), (1, True), (2, False), (3, True)], [(0, 1), (3, 3)]),
    ([(0, False)], []),
])
def test_stop_episode_segmentation(episodes, expected):
    rec = T4.Recorder()
    for t, v in episodes:
        rec.add('stop', (t, v))
    assert T4.stop_episodes(rec) == expected


def test_single_phase_run_scores_only_that_phase():
    rec = recording()
    verdict, checks = results(rec, {'P3': (120.0, 180.0)})
    names = list(checks)
    assert any(n.startswith('P3 proximity') for n in names)
    assert not any(n.startswith(('P1', 'P2', 'P4', 'P5', 'BASE')) for n in names)
    assert verdict == 'PASS'


def test_base_capture_serves_as_reference_for_p2():
    rec = recording(base_obstacle=None)
    windows = {'BASE': (0.0, 60.0), 'P2': (60.0, 120.0)}
    verdict, checks = results(rec, windows)
    assert find(checks, 'BASE surroundings')['result'] == 'PASS'
    assert find(checks, 'P2 box')['result'] == 'PASS'


def test_manual_windows_use_actual_phase_times():
    # Operator waited 25 s before starting P3; the box is in place 135-180 s.
    rec = recording()
    verdict, checks = results(rec, {'P3': (140.0, 180.0)})
    assert find(checks, 'P3 proximity')['result'] == 'PASS'
