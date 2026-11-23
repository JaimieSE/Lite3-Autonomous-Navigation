"""Camera coverage: which floor and walls the front camera has actually looked at.

Operator 2026-11-23: missions (object, hazard, two survivors, QR) can be done in any order, so the camera has to
see the whole arena; "walked within 0.7 m" (roam's walked path) is not the same as "seen". This keeps a coarse
grid around the start, marks the cells inside the camera's view (horizontal field of view CAMERA_HFOV_DEG, out
to CAMERA_RANGE_M, stopped by occupied SLAM cells) and answers "how much mapped floor near here is unseen".

Camera: Orbbec on the head, CAMERA_AHEAD_M in front of base_link, looking along the body heading. Field of view
from the depth camera's intrinsics read 2026-11-23 (fx 372.5 px at 640 px wide -> 2 atan(320 / 372.5) = 81 deg).
Range: the stair/curtain detector confirms at ~0.7-1.2 m and YOLO is unreliable past ~3 m at 640 px.
Pure (numpy only): unit-tested.
"""

import math

import numpy as np

CAMERA_HFOV_DEG = 80.0
CAMERA_RANGE_M = 3.0
CAMERA_AHEAD_M = 0.25
COVER_RES_M = 0.10
RAY_STEP_DEG = 2.0


def _slam_array(grid):
    """(H x W int16 array, res, ox, oy) of a lite3_locomotion.frontiers.Grid (origin yaw assumed 0)."""
    return (np.asarray(grid.data, dtype=np.int16).reshape(grid.height, grid.width), grid.resolution,
            grid.origin_x, grid.origin_y)


class CameraCoverage:
    def __init__(self, centre, half_size_m):
        self.res = COVER_RES_M
        self.n = int(math.ceil(2 * half_size_m / self.res))
        self.x0 = centre[0] - half_size_m
        self.y0 = centre[1] - half_size_m
        self.seen = np.zeros((self.n, self.n), dtype=bool)
        self._slam_key, self._slam = None, None

    # ------------------------------------------------------------------ geometry
    def _cells(self, xs, ys):
        i = np.floor((np.asarray(xs) - self.x0) / self.res).astype(int)
        j = np.floor((np.asarray(ys) - self.y0) / self.res).astype(int)
        ok = (i >= 0) & (i < self.n) & (j >= 0) & (j < self.n)
        return i, j, ok

    def _slam_for(self, grid):
        key = (id(grid), grid.width, grid.height)
        if key != self._slam_key:
            self._slam_key, self._slam = key, _slam_array(grid)
        return self._slam

    def is_seen(self, x, y):
        i, j, ok = self._cells([x], [y])
        return bool(ok[0] and self.seen[j[0], i[0]])

    def seen_m2(self):
        return float(self.seen.sum()) * self.res ** 2

    # ------------------------------------------------------------------ update
    def update(self, pose, grid, hfov_deg=CAMERA_HFOV_DEG, range_m=CAMERA_RANGE_M):
        """Mark what the camera sees from pose (x, y, yaw) on the SLAM grid: rays across the field of view,
        each stopping at (and including) its first occupied cell. Unknown cells do not stop a ray: the camera
        sees them even where the 2D LiDAR band has not mapped them."""
        data, res, ox, oy = self._slam_for(grid)
        cx = pose[0] + CAMERA_AHEAD_M * math.cos(pose[2])
        cy = pose[1] + CAMERA_AHEAD_M * math.sin(pose[2])
        half = math.radians(hfov_deg) / 2
        angles = pose[2] + np.arange(-half, half + 1e-9, math.radians(RAY_STEP_DEG))
        dists = np.arange(self.res / 2, range_m, self.res / 2)
        xs = cx + dists[None, :] * np.cos(angles)[:, None]                 # rays x steps
        ys = cy + dists[None, :] * np.sin(angles)[:, None]
        si = np.floor((xs - ox) / res).astype(int)
        sj = np.floor((ys - oy) / res).astype(int)
        inside = (si >= 0) & (si < data.shape[1]) & (sj >= 0) & (sj < data.shape[0])
        occupied = np.zeros_like(inside)
        occupied[inside] = data[sj[inside], si[inside]] > 50
        first = np.where(occupied.any(axis=1), occupied.argmax(axis=1), dists.size - 1)
        visible = np.arange(dists.size)[None, :] <= first[:, None]           # up to and including the wall
        i, j, ok = self._cells(xs[visible], ys[visible])
        self.seen[j[ok], i[ok]] = True

    # ------------------------------------------------------------------ queries
    def unseen_free_m2(self, grid, centre, radius_m, heading=None, half_angle=None):
        """Mapped free floor (SLAM 0..20) within radius_m of centre that the camera has not seen, optionally only
        in the sector heading +- half_angle (radians)."""
        data, res, ox, oy = self._slam_for(grid)
        i0 = max(0, int((centre[0] - radius_m - ox) / res))
        i1 = min(data.shape[1], int((centre[0] + radius_m - ox) / res) + 1)
        j0 = max(0, int((centre[1] - radius_m - oy) / res))
        j1 = min(data.shape[0], int((centre[1] + radius_m - oy) / res) + 1)
        if i0 >= i1 or j0 >= j1:
            return 0.0
        patch = data[j0:j1, i0:i1]
        jj, ii = np.nonzero((patch >= 0) & (patch <= 20))
        wx = ox + (ii + i0 + 0.5) * res
        wy = oy + (jj + j0 + 0.5) * res
        dx, dy = wx - centre[0], wy - centre[1]
        keep = dx * dx + dy * dy <= radius_m * radius_m
        if heading is not None:
            rel = np.arctan2(np.sin(np.arctan2(dy, dx) - heading), np.cos(np.arctan2(dy, dx) - heading))
            keep &= np.abs(rel) <= half_angle
        wx, wy = wx[keep], wy[keep]
        i, j, ok = self._cells(wx, wy)
        unseen = np.ones(wx.size, dtype=bool)
        unseen[ok] = ~self.seen[j[ok], i[ok]]
        return float(unseen.sum()) * res * res
