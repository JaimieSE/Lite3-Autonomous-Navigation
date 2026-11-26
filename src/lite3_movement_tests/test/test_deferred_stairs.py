"""Offline regression tests. No ROS nodes, sensors, or motion clients."""
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch

from lite3_movement_tests import roam


class DeferredStairsTest(unittest.TestCase):
    def setUp(self):
        self.s = s = object.__new__(roam.RoamSession)
        self.events, self.goals, self.climbs, self.holds = [], [], [], []
        s.event = lambda name, **data: self.events.append((name, data))
        s._start_walk = lambda now, pose, goal: self.goals.append(goal)
        s._begin_stairs = lambda *args, **opts: self.climbs.append((args, opts))
        s._pause = self.holds.append
        s._reachable = lambda x, y: True
        s._frontier_goal = lambda now, pose: None
        s._phase = lambda now: 'move_on'
        s.course = NS(cells={(0, 0): {}})
        s.record = NS(breadcrumbs=[(0, 0), (-0.3, 0), (-0.6, 0)])
        s.cfg = NS(pi_advisory=False, max_map_age_s=3)
        s.map = (10, None)
        s.stairs_until = 0
        self.camera = {'obstacle': 'stairs', 'distance_m': 0.6, 'heading_deg': 0}
        self.sample = NS(step_ahead_m=0.6, step_rise_m=0.18)
        s._remember_stairs(2, (0, 0, 0), self.camera, 'detector')
        self.stair = s._pending_stair
        self.stamp = 10.6
        s.runner = NS(_stationary=lambda seconds: (True, {}))
        s.robot = NS(advisory=(10.6, 'stairs'), telemetry=NS(
            terrain=NS(latest=lambda: (self.stamp, self.sample))))

    def test_search_keeps_stairs_deferred(self):
        self.s._phase = lambda now: 'search'
        self.assertFalse(self.s._return_to_stairs(10, (-0.6, 0, 0), self.sample))
        self.assertFalse(self.goals)

    def test_stale_map_does_not_start_return(self):
        self.assertTrue(self.s._return_to_stairs(15, (-0.6, 0, 0), self.sample))
        self.assertIsNone(self.stair['route'])
        self.assertFalse(self.goals)

    def test_breadcrumb_order_and_missed_waypoint(self):
        self.s._return_to_stairs(10, (-0.6, 0, 0), self.sample)
        self.assertAlmostEqual(self.goals[0]['x'], -0.3)
        self.s._return_to_stairs(11, (-0.6, 0, 0), self.sample)
        self.assertIn('not reached', self.holds[0])
        self.assertEqual(len(self.goals), 1)
        self.assertIsNone(self.stair['route'])

    def test_blocked_breadcrumb_holds(self):
        self.s._reachable = lambda x, y: False
        self.s._return_to_stairs(10, (-0.6, 0, 0), self.sample)
        self.assertIn('blocked', self.holds[0])
        self.assertFalse(self.goals)

    def test_plan_shortcut_is_detected(self):
        self.s.goal_sent_at = 9
        self.s.robot.nav_plan = (10, [(0, 0), (0.6, 0), (1.2, 0)])
        self.assertTrue(self.s._plan_crosses_pending_stair())
        self.s.robot.nav_plan = (10, [(0, 0), (0, 0.6), (1.2, 0.6)])
        self.assertFalse(self.s._plan_crosses_pending_stair())
        self.s.robot.nav_plan = (8, [(0, 0), (1.2, 0)])
        self.assertFalse(self.s._plan_crosses_pending_stair())

    def confirmation(self, now, heading=0):
        with patch.object(roam, 'parse_advisory', return_value=self.camera):
            self.s._confirm_deferred_stairs(now, (0, 0, heading), self.sample, self.stair)

    def test_stale_camera_and_heading_disagreement_prevent_handoff(self):
        self.stair['confirm_since'] = 10
        self.s.robot.advisory = (9, 'stairs')
        self.confirmation(10.6)
        self.assertIn('camera frame', self.stair['waiting_for'])
        self.s.robot.advisory = (10.6, 'stairs')
        self.confirmation(10.6, heading=0.5)
        self.assertIn('heading', self.stair['waiting_for'])
        self.assertFalse(self.climbs)

    def test_distance_disagreement_prevents_handoff(self):
        self.stair['confirm_since'] = 10
        self.sample.step_ahead_m = 0.25
        self.confirmation(10.6)
        self.stamp = 10.7
        self.confirmation(10.7)
        self.assertIn('disagree', self.stair['waiting_for'])
        self.assertFalse(self.climbs)

    def test_two_distinct_agreeing_samples_enable_handoff(self):
        self.stair['confirm_since'] = 10
        self.confirmation(10.6)
        self.confirmation(10.65)
        self.assertFalse(self.climbs)
        self.stamp = 10.7
        self.confirmation(10.7)
        self.assertEqual(len(self.climbs), 1)
        self.assertEqual(self.climbs[0][1]['mode'], 'ai')

    def test_negative_terrain_without_camera_does_not_descend(self):
        self.stair['confirm_since'] = 10
        self.sample.step_rise_m = -0.18
        self.s.robot.advisory = None
        self.confirmation(10.6)
        self.confirmation(20.1)
        self.assertIn('camera frame', self.holds[0])
        self.assertFalse(self.climbs)


if __name__ == '__main__':
    unittest.main()
