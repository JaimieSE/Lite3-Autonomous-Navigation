"""--arena-holds (operator 2026-11-26, operator/arena_holds.sh): face the next grid cell before walking, hold when a step
turn stops early or the walk leaves the lane, align to the grid after the stairs. The stair sequence is untouched."""
import inspect
import math
import pathlib
import subprocess

import pytest

from lite3_movement_tests import roam as R
from lite3_movement_tests.course_grid import CourseGrid

S = R.RoamSession


class Fake:
    """Just enough of a RoamSession for the --arena-holds methods."""
    _grid_faced = S._grid_faced
    _arena_hold = S._arena_hold
    _grid_axis_yaw = S._grid_axis_yaw
    _end_spin = S._end_spin
    _post_stair_tick = S._post_stair_tick
    _post_stair_problem = S._post_stair_problem
    _post_stair_point = S._post_stair_point
    _post_stair_hold = S._post_stair_hold

    def __init__(self):
        self.course = CourseGrid(0.0, 0.0, 0.0)
        self.cfg = type('C', (), {'arena_holds': True})()
        self.events, self.spins, self.walks, self.paused, self.blocked = [], [], [], [], []
        self.goal, self._grid_face, self._post_stair = None, None, None
        self._arena_hold_until = -math.inf
        self.room, self.invalid = True, lambda x, y: False
        self.stats = {'spins_completed': 0}
        self.look, self.map, self._slam_paused = [], (100.0, None), False
        self.course.block_cell = lambda cell, now, s: self.blocked.append(tuple(cell))
        test = self

        class Tel:
            def current_tilt(self):
                return (0.0, 0.0)

        class Robot:
            telemetry = Tel()

            def costmap_grid(self):
                return (100.0, None)
        self.robot = Robot()

    def event(self, name, **kw):
        self.events.append((name, kw))

    def names(self):
        return [n for n, _ in self.events]

    def _spin_room(self, pose):
        return self.room

    def _start_spin(self, now, pose, angle, reason=''):
        self.spins.append(round(math.degrees(angle)))
        self.goal = {'kind': 'spin', 'angle': angle}

    def _start_walk(self, now, pose, goal):
        self.walks.append(goal)
        self.goal = goal

    def _goal_invalid(self, goal):
        return self.invalid(goal['x'], goal['y'])

    def _pause(self, why):
        self.paused.append(why)

    def _drift_check(self, now, spin):
        pass


LANE_EAST = ((0, 0), (1, 0))


def test_facing_north_an_eastward_goal_turns_90_right_then_walks():
    f = Fake()
    goal = {'x': 1.0, 'y': 0.0, 'yaw': 0.0}
    assert f._grid_faced(10.0, (0.0, 0.0, math.pi / 2), dict(goal), goal, LANE_EAST) is False
    assert f.spins == [-90] and f.goal['grid_face'] and f.goal['fixed_turn']      # turn first, no walk yet
    f.goal = None
    walk = dict(goal)
    assert f._grid_faced(14.0, (0.0, 0.0, math.radians(2.0)), walk, goal, LANE_EAST) is True   # verified: walk
    assert walk['yaw'] == 0.0 and f.names()[-1] == 'grid_faced'


def test_already_facing_walks_straight_without_a_turn():
    f = Fake()
    goal = {'x': 1.0, 'y': 0.0, 'yaw': 0.0}
    assert f._grid_faced(10.0, (0.0, 0.0, math.radians(4.0)), dict(goal), goal, LANE_EAST) is True
    assert f.spins == []


def test_a_cell_behind_takes_two_quarter_turns_only_with_room():
    f = Fake()
    goal = {'x': 1.0, 'y': 0.0, 'yaw': 0.0}
    f._grid_faced(10.0, (0.0, 0.0, math.pi), dict(goal), goal, LANE_EAST)
    assert abs(f.spins[0]) == 90                                               # half of 180, first quarter-turn
    g = Fake()
    g.room = False
    g._grid_faced(10.0, (0.0, 0.0, math.pi), dict(goal), goal, LANE_EAST)
    assert g.spins == [] and g.names()[-1] == 'arena_hold' and 'no room' in g.events[-1][1]['reason']


def test_an_interrupted_turn_holds_and_reconsiders_without_walking():
    f = Fake()
    goal = {'x': 1.0, 'y': 0.0, 'yaw': 0.0}
    f._grid_faced(10.0, (0.0, 0.0, math.pi / 2), dict(goal), goal, LANE_EAST)
    f._end_spin(12.0, 'incomplete', math.radians(39.0))                        # 06:24: stopped at 39 deg
    hold = [kw for n, kw in f.events if n == 'arena_hold']
    assert hold and 'stopped early' in hold[0]['reason'] and f._grid_face is None
    assert f._arena_hold_until == pytest.approx(12.0 + R.ARENA_HOLD_S) and f.blocked == [(1, 0)]
    assert f.walks == []


def test_turns_that_never_reach_the_heading_hold_after_the_limit():
    f = Fake()
    goal = {'x': 1.0, 'y': 0.0, 'yaw': 0.0}
    for i in range(R.ARENA_MAX_TURNS + 1):
        f.goal = None
        f._grid_faced(10.0 + i, (0.0, 0.0, math.pi / 2), dict(goal), goal, LANE_EAST)
    assert len(f.spins) == R.ARENA_MAX_TURNS and f.names()[-1] == 'arena_hold'


def test_a_diagonal_replan_holds_instead_of_walking_on():
    """06:24: Nav2 replanned out of the lane and the walk carried on at an angle (centre_lane_left)."""
    src = inspect.getsource(S._scan_walk)
    strict = src.index("self.event('arena_hold', reason='walk left the lane'")
    lenient = src.index("self.event('centre_lane_left'")
    assert strict < lenient                                    # the arena check comes first, for grid steps only
    assert "'reselect: walk left the lane (arena holds)'" in src        # ends as reselected: no failed destination
    assert 'reselect' in inspect.getsource(S._scan_walk).split("if self.cancel_request:")[1][:900]


def _post_stair(f, place=None, poses_from=97.0):
    f._post_stair = {'t': 99.0, 'since': 99.0, 'fresh_map': True, 'corrections': 0, 'place': place,
                     'poses': [(poses_from + 0.1 * i, *f.pose) for i in range(30)]}


def test_off_centre_and_off_heading_after_the_stairs_walks_to_the_centre_of_the_same_cell():
    f = Fake()
    f.pose = (0.25, 0.10, math.radians(12.0))
    _post_stair(f, place=(-0.9, 0.0))
    f._post_stair_tick(100.0, f.pose)
    walk = f.walks[0]
    assert f.course.cell(walk['x'], walk['y']) == f.course.cell(*f.pose[:2])          # same cell
    assert math.hypot(walk['x'], walk['y']) < 0.05 and walk['yaw'] == 0.0              # its centre, grid heading
    assert walk['centre_lane_skip'] and walk['ends_at'] == 'post_stair'


def test_centred_but_off_heading_after_the_stairs_turns_in_place():
    f = Fake()
    f.pose = (0.03, 0.0, math.radians(-12.0))
    _post_stair(f)
    f._post_stair_tick(100.0, f.pose)
    assert f.spins == [12] and f.goal['fixed_turn'] and f.walks == []


def test_aligned_after_the_stairs_carries_on():
    f = Fake()
    f.pose = (0.05, 0.02, math.radians(2.0))
    _post_stair(f)
    f._post_stair_tick(100.0, f.pose)
    assert f.names()[-1] == 'post_stair_aligned' and f._post_stair is None and not f.paused


def test_a_blocked_landing_or_the_stair_edge_holds_with_the_reason():
    f = Fake()
    f.pose = (0.3, 0.0, 0.0)
    f.invalid = lambda x, y: True                                   # costmap / map not clear anywhere
    _post_stair(f)
    f._post_stair_tick(100.0, f.pose)
    assert f.paused and 'not clear' in f.paused[0]
    g = Fake()
    g.pose = (0.4, 0.0, 0.0)                                        # robot itself 0.55 m clear of the stairs ...
    _post_stair(g, place=(-0.15, 0.0))                              # ... but every point nearer the centre is not
    g._post_stair_tick(100.0, g.pose)
    assert g.paused and 'stair edge' in g.paused[0]


def test_stale_data_after_the_stairs_waits_then_holds():
    f = Fake()
    f.pose = (0.3, 0.0, 0.0)
    f._slam_paused = True
    _post_stair(f)
    f._post_stair_tick(100.0, f.pose)
    assert not f.paused and not f.walks                             # waiting
    f._post_stair_tick(99.0 + R.POST_STAIR_WAIT_MAX_S + 1, f.pose)
    assert f.paused and 'SLAM still paused' in f.paused[0]


def test_a_pose_still_settling_after_slam_resumes_is_not_measured():
    f = Fake()
    f.pose = (0.3, 0.0, 0.0)
    _post_stair(f)
    f._post_stair['poses'] = [(99.5, 0.0, 0.0, 0.0), (99.8, 0.4, 0.0, 0.2)]         # jumped 0.4 m / 11 deg
    f._post_stair_tick(100.0, f.pose)
    assert not f.walks and not f.spins and not f.paused


def test_arena_holds_needs_the_centre_lane_and_the_command_adds_it_to_competition_start():
    script = pathlib.Path(__file__).resolve().parents[3] / 'operator' / 'arena_holds.sh'
    text = script.read_text()
    assert 'competition_start.sh" "$@" --arena-holds' in text
    assert '--arena-holds' not in (script.parent / 'competition_start.sh').read_text()


# _stairs_tick left out: operator 2026-11-26 approved one change there (a failed crossing keeps exploring, no pause).
STAIR_FUNCS = ('_pi_stairs_only_tick', '_pi_new_stairs', '_pi_stairs_handover', '_begin_stairs',
               '_leave_stairs', '_slam_pause')


def test_the_stair_sequence_is_unchanged_from_the_backup():
    """Hard constraint: --arena-holds only watches the stairs; their code is identical to the working backup."""
    repo = pathlib.Path(__file__).resolve().parents[3]
    branch = 'backup/working-stairs-20261126'
    def show(path):
        r = subprocess.run(['git', '-C', str(repo), 'show', f'{branch}:{path}'], capture_output=True, text=True)
        return r.stdout if r.returncode == 0 else None
    pkg = 'src/lite3_movement_tests/lite3_movement_tests/'
    old_stairs, old_roam = show(pkg + 'stairs.py'), show(pkg + 'roam.py')
    if old_stairs is None or old_roam is None:
        pytest.skip('backup branch not available')
    assert (repo / pkg / 'stairs.py').read_text() == old_stairs
    import ast
    def funcs(text):
        tree = ast.parse(text)
        cls = next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == 'RoamSession')
        return {n.name: ast.get_source_segment(text, n) for n in cls.body if isinstance(n, ast.FunctionDef)}
    now, before = funcs((repo / pkg / 'roam.py').read_text()), funcs(old_roam)
    for name in STAIR_FUNCS:
        assert now[name] == before[name], f'{name} changed'
