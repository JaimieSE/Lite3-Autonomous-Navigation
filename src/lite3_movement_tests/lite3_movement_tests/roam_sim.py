"""Software stand-in for M10 (unit tests and --sim rehearsal only; not physics, not evidence).

FakeNavRobot (mux, dead-man, terrain lock, collision_monitor, straight-line "Nav2") plus:
  * a walled room; a SLAM-like 2D map that becomes known within 2.5 m of where the robot has been
    (cells outside the room stay unknown: walls block the view);
  * a global costmap = everything the LiDAR point cloud sees, inflated 0.25 m. "Point-cloud-only"
    obstacles (a table: legs too thin / top too high for the 2D map) are in the costmap and the
    terrain hazards, but not in the 2D map;
  * raw LiDAR (/livox/points) and Fast-LIO (/cloud_registered_body) message sizes at 10 Hz.
"""

import math

from lite3_locomotion.frontiers import Grid

from .sim_model import FakeNavRobot, SimLite3, box_points
from .telemetry import Telemetry

ROOM = (-1.0, 5.0, -2.0, 2.0)          # x0, x1, y0, y1 (walls), robot starts at (0, 0) facing +x
ROAM_FAULTS = ('raw_lidar_blind', 'raw_lidar_stale', 'lio_stale', 'map_frozen')


def wall_points(room=ROOM, step=0.1):
    x0, x1, y0, y1 = room
    pts = []
    n = int(round((x1 - x0) / step))
    for i in range(n + 1):
        x = x0 + i * step
        pts += [(x, y0), (x, y1)]
    n = int(round((y1 - y0) / step))
    for i in range(n + 1):
        y = y0 + i * step
        pts += [(x0, y), (x1, y)]
    return pts


class SimRoamRobot(FakeNavRobot):
    RES = 0.1
    ORIGIN = (-2.0, -3.0)
    SIZE = (80, 60)                      # 8 m x 6 m

    def __init__(self, telemetry, model, nav_fault='none', roam_fault='none', room=ROOM,
                 map_obstacles=(), cloud_only_obstacles=(), **kwargs):
        self.roam_faults = set(roam_fault.split('+')) - {'none'}
        if self.roam_faults - set(ROAM_FAULTS):
            raise ValueError(f'unknown roam fault {roam_fault}')
        self.room = room
        self.map_points = wall_points(room) + list(map_obstacles)
        self.cloud_only = list(cloud_only_obstacles)
        self.known = set()
        self.visited = []
        self.map_item = None
        self.cost_item = None
        self.next_map = 0.0
        self.next_cloud = 0.0
        self.roam_started = None
        super().__init__(telemetry, model, nav_fault=nav_fault,
                         obstacles=self.map_points + self.cloud_only, **kwargs)

    # ---- map / costmap
    def _cell(self, x, y):
        return (math.floor((x - self.ORIGIN[0]) / self.RES), math.floor((y - self.ORIGIN[1]) / self.RES))

    def _reveal(self, x, y, radius=2.5):
        cx, cy = self._cell(x, y)
        r = int(radius / self.RES)
        x0, x1, y0, y1 = self.room
        for j in range(cy - r, cy + r + 1):
            for i in range(cx - r, cx + r + 1):
                if not (0 <= i < self.SIZE[0] and 0 <= j < self.SIZE[1]):
                    continue
                wx = self.ORIGIN[0] + (i + 0.5) * self.RES
                wy = self.ORIGIN[1] + (j + 0.5) * self.RES
                if math.hypot(wx - x, wy - y) <= radius and \
                        x0 - self.RES <= wx <= x1 + self.RES and y0 - self.RES <= wy <= y1 + self.RES:
                    self.known.add((i, j))

    def _build(self):
        w, h = self.SIZE
        occupied = {self._cell(x, y) for x, y in self.map_points}
        data = [-1] * (w * h)
        for i, j in self.known:
            data[j * w + i] = 100 if (i, j) in occupied else 0
        grid = Grid(w, h, self.RES, data, *self.ORIGIN)
        cost = [0] * (w * h)
        infl = int(math.ceil(0.25 / self.RES))
        for x, y in self.map_points + self.cloud_only:
            ci, cj = self._cell(x, y)
            for j in range(cj - infl, cj + infl + 1):
                for i in range(ci - infl, ci + infl + 1):
                    if 0 <= i < w and 0 <= j < h:
                        cost[j * w + i] = 100
        return grid, Grid(w, h, self.RES, cost, *self.ORIGIN)

    def _publish(self, t):
        super()._publish(t)
        if not hasattr(self, 'next_cloud'):
            return
        m = self.model
        if not self.visited or math.hypot(m.x - self.visited[-1][0], m.y - self.visited[-1][1]) > 0.25:
            self.visited.append((m.x, m.y))
            self._reveal(m.x, m.y)
        if t >= self.next_map and not ('map_frozen' in self.roam_faults and self.roam_started
                                       and t > self.roam_started + 5):
            self.next_map = t + 2.0                         # slam_toolbox map_update_interval
            grid, cost = self._build()
            self.map_item, self.cost_item = (t, grid), (t, cost)
        if t >= self.next_cloud:
            self.next_cloud = t + 0.1
            late = self.roam_started is not None and t > self.roam_started + 5
            if not ('raw_lidar_stale' in self.roam_faults and late):
                self.telemetry.raw_cloud.add(t, 900 if 'raw_lidar_blind' in self.roam_faults and late else 360000)
            if not ('lio_stale' in self.roam_faults and late):
                self.telemetry.lio_cloud.add(t, 180000)

    def map_grid(self):
        return self.map_item

    def costmap_grid(self):
        return self.cost_item

    def nav_send_goal(self, x, y, yaw):
        if self.roam_started is None:
            self.roam_started = self.t
        return super().nav_send_goal(x, y, yaw)


def make_sim_robot(nav_fault='none', roam_fault='none', model_fault='none', **kwargs):
    kwargs.setdefault('cm_boundary', 0.20)     # what roam.sh starts navigation with (0.05 m at 0.08 m/s)
    return SimRoamRobot(Telemetry(), SimLite3(start='lying', fault=model_fault), nav_fault=nav_fault,
                        roam_fault=roam_fault, **kwargs)


def table(x, y, size=0.6):
    """A table the 2D SLAM map does not see (only legs/top in the 3D point cloud)."""
    return box_points(x, y, size, step=0.1)
