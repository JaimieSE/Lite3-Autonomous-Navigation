"""Walls with small gaps are still walls (operator 2026-11-24).

"Make sure that walls are detected properly, even if there is a slight gap in it as it may think that it is an open
area that can be cut through." The SLAM map often holds a wall as a line of occupied cells with holes in it (sparse
LiDAR returns, a dark panel, a seam). A hole narrower than the dog can never be walked through, so it is closed:
morphological closing of the occupied cells with a disc of diameter GAP_M; the cells it adds are the gaps.

Never closed: anywhere the dog has walked (breadcrumbs, WALKED_CLEAR_M around them) - if it went through, it is not
a wall. Openings at least GAP_M wide stay open, so real doorways and corridors are untouched.

Uses: roam rejects straight-line goals across a gap, cancels a Nav2 path planned through one, the course grid treats
gap cells as wall, and RViz shows each closed gap as a wireframe box tagged "WALL GAP". Pure (numpy/scipy).
"""

import math

import numpy as np

GAP_M = 0.40                 # holes narrower than this in a wall are closed (Lite3 body ~0.39 m wide)
MIN_WALL_CELLS = 7           # occupied blobs smaller than this (~0.35 m of wall) are speckles, not walls (runs 17:07
                             # and 17:17: 1375 of 1427 blobs were <= 5 cells; closing joined them into fake walls)
MAX_FILL_M = 0.5             # a closed gap is thin (a hole in a wall); a filled blob wider than this both ways is not
WALKED_CLEAR_M = 0.25        # never close a gap within this of the walked path
MAX_BOXES = 30


def _disc(radius_cells):
    r = int(radius_cells)
    yy, xx = np.mgrid[-r:r + 1, -r:r + 1]
    return xx * xx + yy * yy <= r * r + r


def gap_mask(grid, walked=(), gap_m=GAP_M):
    """(bool mask in the SLAM grid's shape of closed-gap cells, (res, origin_x, origin_y))."""
    from scipy import ndimage
    data = np.asarray(grid.data, dtype=np.int16).reshape(grid.height, grid.width)
    res = grid.resolution
    occupied = data > 50
    labels, _ = ndimage.label(occupied, structure=np.ones((3, 3)))
    sizes = np.bincount(labels.ravel())
    keep = sizes >= MIN_WALL_CELLS
    keep[0] = False
    walls = keep[labels]                                   # speckles dropped: only real wall segments are joined
    r = max(1, int(round(gap_m / 2 / res)))
    closed = ndimage.binary_closing(walls, structure=_disc(r), iterations=1)
    gaps = closed & ~occupied
    if gaps.any():                                         # keep thin holes in walls, drop filled blobs
        glab, _ = ndimage.label(gaps, structure=np.ones((3, 3)))
        limit = int(round(MAX_FILL_M / res))
        for k, sl in enumerate(ndimage.find_objects(glab), start=1):
            if sl is not None and (sl[0].stop - sl[0].start) > limit and (sl[1].stop - sl[1].start) > limit:
                gaps[sl] &= glab[sl] != k
    if walked:
        free = np.zeros_like(gaps)
        for x, y in walked:
            i, j = int((x - grid.origin_x) / res), int((y - grid.origin_y) / res)
            if 0 <= i < grid.width and 0 <= j < grid.height:
                free[j, i] = True
        free = ndimage.binary_dilation(free, structure=_disc(max(1, int(round(WALKED_CLEAR_M / res)))))
        gaps &= ~free
    return gaps, (res, grid.origin_x, grid.origin_y)


def crosses(gaps, geo, points, step_m=0.05):
    """True if the polyline through points (x, y) passes over a closed-gap cell."""
    res, gx, gy = geo
    h, w = gaps.shape
    for (ax, ay), (bx, by) in zip(points, points[1:]):
        n = max(1, int(math.hypot(bx - ax, by - ay) / step_m))
        for k in range(n + 1):
            x, y = ax + (bx - ax) * k / n, ay + (by - ay) * k / n
            i, j = int(math.floor((x - gx) / res)), int(math.floor((y - gy) / res))
            if 0 <= i < w and 0 <= j < h and gaps[j, i]:
                return True
    return False


def gap_boxes(gaps, geo, near=None, max_boxes=MAX_BOXES):
    """World bounding boxes (x0, y0, x1, y1) of the closed gaps, nearest first when near=(x, y) is given."""
    from scipy import ndimage
    res, gx, gy = geo
    labels, n = ndimage.label(gaps, structure=np.ones((3, 3)))
    boxes = []
    for sl in ndimage.find_objects(labels):
        if sl is None:
            continue
        j0, j1, i0, i1 = sl[0].start, sl[0].stop, sl[1].start, sl[1].stop
        boxes.append((gx + i0 * res, gy + j0 * res, gx + i1 * res, gy + j1 * res))
    if near is not None:
        boxes.sort(key=lambda b: math.hypot((b[0] + b[2]) / 2 - near[0], (b[1] + b[3]) / 2 - near[1]))
    return boxes[:max_boxes]


# ---------------------------------------------------------------------------------- reachable floor
# Operator 2026-11-24: "troubleshoot why it is setting goals outside of the course". The LiDAR sees over and through
# the course walls, so SLAM maps free floor OUTSIDE the course, and the frontier / patrol / come-back goals went there
# (only the geofence bounded them). The walls are the guide: a goal must lie on floor connected to the dog without
# crossing a wall (small gaps closed). Unknown floor is not connected - the course goes on only through seen floor or
# a frontier next to it, which is why goals may sit up to REACH_SLACK_M beyond the connected floor.
REACH_CLEARANCE_M = 0.0      # (holes narrower than the dog are already closed by gap_mask)
REACH_SLACK_M = 0.5          # a goal may be this far past the connected floor (frontier / unseen ends)


def reachable_mask(grid, start_xy, gaps=None, walked=(), clearance_m=REACH_CLEARANCE_M):
    """Bool mask (SLAM shape) of free floor connected to start_xy without crossing a wall or a closed gap.
    Cells the dog has walked count as connected floor whatever the map says now. None if start is off the map."""
    from scipy import ndimage
    data = np.asarray(grid.data, dtype=np.int16).reshape(grid.height, grid.width)
    res = grid.resolution
    wall = data > 50
    if gaps is not None and gaps.shape == wall.shape:
        wall = wall | gaps
    r = int(round(clearance_m / res))
    near_wall = ndimage.binary_dilation(wall, structure=_disc(r)) if r > 0 else wall
    passable = (data >= 0) & (data <= 20) & ~near_wall
    trail = np.zeros_like(passable)
    for x, y in walked:
        i, j = int((x - grid.origin_x) / res), int((y - grid.origin_y) / res)
        if 0 <= i < grid.width and 0 <= j < grid.height:
            trail[j, i] = True
    if trail.any():
        passable |= ndimage.binary_dilation(trail, structure=_disc(max(1, int(round(0.15 / res))))) & ~wall
    si, sj = int((start_xy[0] - grid.origin_x) / res), int((start_xy[1] - grid.origin_y) / res)
    if not (0 <= si < grid.width and 0 <= sj < grid.height):
        return None
    # 4-connected: floor may not leak diagonally through a one-cell-thick wall
    labels, _ = ndimage.label(passable, structure=np.array([[0, 1, 0], [1, 1, 1], [0, 1, 0]]))
    k = int(round(1.0 / res))                    # LiDAR blind zone round the dog: its floor may start ~0.7 m out
    window = labels[max(0, sj - k):sj + k + 1, max(0, si - k):si + k + 1]
    ids = window[window > 0]
    if ids.size == 0:
        return None
    lab = np.bincount(ids).argmax()
    return labels == lab


def near_mask(mask, geo, x, y, slack_m=REACH_SLACK_M):
    """True if (x, y) is within slack_m of a True cell of mask."""
    res, gx, gy = geo
    k = int(math.ceil(slack_m / res))
    i, j = int(math.floor((x - gx) / res)), int(math.floor((y - gy) / res))
    h, w = mask.shape
    i0, i1, j0, j1 = max(0, i - k), min(w, i + k + 1), max(0, j - k), min(h, j + k + 1)
    if i0 >= i1 or j0 >= j1:
        return False
    return bool(mask[j0:j1, i0:i1].any())


def in_course(grid_data, reach, geo, x, y, slack_m=REACH_SLACK_M):
    """A goal is inside the course: on mapped free floor it must be on the dog's own floor (reach); on still
    unmapped floor (a frontier, a scan-walk end in unseen space) it must be within slack_m of it."""
    res, gx, gy = geo
    i, j = int(math.floor((x - gx) / res)), int(math.floor((y - gy) / res))
    h, w = reach.shape
    if not (0 <= i < w and 0 <= j < h):
        return near_mask(reach, geo, x, y, slack_m)
    v = grid_data[j, i]
    if 0 <= v <= 20:
        return bool(reach[j, i])
    if v < 0:
        return near_mask(reach, geo, x, y, slack_m)
    return False
