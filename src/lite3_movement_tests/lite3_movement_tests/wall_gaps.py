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
    r = max(1, int(round(gap_m / 2 / res)))
    closed = ndimage.binary_closing(occupied, structure=_disc(r), iterations=1)
    gaps = closed & ~occupied
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
