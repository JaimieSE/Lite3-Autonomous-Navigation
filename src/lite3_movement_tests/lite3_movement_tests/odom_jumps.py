"""Leg-odometry distance without jumps.

Run 26 Nov 04:24: about 2 s after entering AI state, /leg_odom2's position jumped 14.8 m while the robot stood at
the first step (its speed stayed under 0.12 m/s, the map pose was 0.94 m from the start). The watchdog's
leg-odometry geofence tripped on it. Run 03:05 the same day: the stair routine, which measures how far it walked
on the same odometry, judged a step crossed 0.4 s into the climb.

JumpFreeOdom keeps a position that follows every ordinary step of the odometry but drops any step larger than the
robot could move between two readings (MAX_SPEED_M_S times the time between them, plus MARGIN_M). Movement on either
side of a jump still counts, so a geofence or a distance walked stays meaningful.
"""

import math

MAX_SPEED_M_S = 1.0     # documented speed-command maximum (Comm Interface s1.2.12); roam runs at <= 0.5 m/s
MARGIN_M = 0.10         # odometry noise and a late reading on top of that


class JumpFreeOdom:
    def __init__(self, max_speed_m_s=MAX_SPEED_M_S, margin_m=MARGIN_M, origin=None):
        self.max_speed_m_s, self.margin_m = max_speed_m_s, margin_m
        self.last = None              # (t, raw x, raw y)
        self.offset = (0.0, 0.0)      # sum of the dropped jumps, taken off the raw position
        self.origin = origin          # first position unless given
        self.jumps = []               # (t, size m) of every dropped jump

    def update(self, t, x, y):
        """Feed one reading (t in seconds, raw odometry x/y). Returns the size of a jump it dropped, else None."""
        jump = None
        if self.last is None:
            if self.origin is None:
                self.origin = (x, y)
        else:
            lt, lx, ly = self.last
            dx, dy = x - lx, y - ly
            step = math.hypot(dx, dy)
            if step > self.max_speed_m_s * max(0.0, t - lt) + self.margin_m:
                self.offset = (self.offset[0] + dx, self.offset[1] + dy)
                self.jumps.append((t, step))
                jump = step
        self.last = (t, x, y)
        return jump

    def position(self):
        if self.last is None:
            return None
        return self.last[1] - self.offset[0], self.last[2] - self.offset[1]

    def distance(self):
        """Straight-line distance from the first reading, jumps removed (0.0 before any reading)."""
        p = self.position()
        return 0.0 if p is None else math.hypot(p[0] - self.origin[0], p[1] - self.origin[1])
