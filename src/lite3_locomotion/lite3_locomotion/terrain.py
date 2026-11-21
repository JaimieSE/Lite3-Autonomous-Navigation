"""Local terrain assessment from the existing Mid-360 cloud (pure numpy, no ROS).

The university pipeline flattens the cloud to /scan using heights 0.1-0.5 m above
base_link, i.e. roughly 0.43-0.83 m above the floor when standing. Steps, curbs,
stairs, low obstacles and drop-offs are therefore invisible to its costmaps.
This module builds a gravity-aligned elevation grid around the robot and labels
each cell. It is perception only: it never commands motion.

Frames: points arrive in livox_frame, are moved to base_link with the static
extrinsic (live /tf_static: x 0.18678, z 0.12498, pitch 15 deg down), then into
a "level" frame with the same origin and heading as base_link but z along
gravity (IMU roll/pitch removed). Heights are reported relative to the floor
under the robot.

Calibrated: the floor reference from leg geometry (Lite3 URDF + joint telemetry;
validated standing 0.327 m / lying 0.081 m). NOT validated: the step/drop limits,
which are placeholders until Lite3 climbing capability is established on the
real course; all limits are parameters.
"""

from dataclasses import dataclass
import math

import numpy as np

UNKNOWN = -1
FREE = 0
STEP = 50
OBSTACLE = 100
DROP = 101          # reported separately; written as 100 in occupancy output
OVERHEAD = 1        # points only above body clearance (ceiling, curtain rail): passable

LABEL_NAMES = {UNKNOWN: 'unknown', FREE: 'free', STEP: 'step', OBSTACLE: 'obstacle',
               DROP: 'drop', OVERHEAD: 'overhead'}


@dataclass(frozen=True)
class TerrainParams:
    resolution_m: float = 0.05
    size_m: float = 8.0                 # square grid centred on the robot (was 6.0: 4.0 m ahead covers
                                        # the 3.0 m max collision boundary + 0.305 m nose)
    floor_tolerance_m: float = 0.04     # |height| within this = floor
    # Rise still walkable (not an obstacle). No vendor guide gives a climbable height (Motion Development
    # Guide s1.2 lists dimensions only). Competition: ~5 cm stairs + rubble; RUG H-Step is the documented
    # high-lift gait (Comm Interface s1.2.5). Set per run: start_navigation.sh --max-step M.
    max_step_m: float = 0.40            # was 0.30 (operator: smaller hazard cloud; /scan covers >= ~0.43 m), 0.18 (course 12:33: stacked/unseen-top lips marked as walls
                                        # boxed the robot in at the start); before that 0.10, 0.12, 0.15; course lips are 0.10-0.15 m (operator 2026-11-21),
                                        # judged as the jump to the lowest neighbouring cell (see build_grid)
    max_drop_m: float = 0.18            # deeper than this = drop-off; was 0.10, 0.15 (stepping off a 0.10-0.15 m
                                        # lip counts as a step down, 2026-11-21)
    body_clearance_m: float = 0.75      # points above this (over floor) are overhead
    min_points_per_cell: int = 2
    # Front edge measured 2026-11-20 (standing, 108 clouds): own-body returns reach x 0.289 at most (head,
    # 0.43-0.50 m up), none ahead of the nose (0.305). 0.32 = +3 cm margin; was 0.40, which hid 9.5 cm
    # ahead of the nose and made a stop line closer than that blind.
    self_filter_x: tuple = (-0.40, 0.32)   # robot body/legs box in base_link (URDF ~0.61 x 0.37 m)
    self_filter_y: tuple = (-0.25, 0.25)
    corridor_half_width_m: float = 0.30  # footprint half width 0.185 + margin
    corridor_length_m: float = 4.0
    near_field_m: tuple = (0.35, 1.2)
    floor_search_tolerance_m: float = 0.12

    def __post_init__(self):
        if not 0.01 <= self.resolution_m <= 0.2 or not 1.0 <= self.size_m <= 20.0:
            raise ValueError('grid resolution/size out of range')
        if not 0 < self.floor_tolerance_m < self.max_step_m < self.body_clearance_m:
            raise ValueError('need floor_tolerance < max_step < body_clearance')
        if self.max_drop_m <= self.floor_tolerance_m:
            raise ValueError('max_drop_m must exceed floor_tolerance_m')


def rot_y(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[c, 0.0, s], [0.0, 1.0, 0.0], [-s, 0.0, c]])


def rot_x(angle):
    c, s = math.cos(angle), math.sin(angle)
    return np.array([[1.0, 0.0, 0.0], [0.0, c, -s], [0.0, s, c]])


def quat_to_matrix(x, y, z, w):
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])


def level_rotation(roll, pitch):
    """Rotation taking base_link vectors into the gravity-aligned, same-heading frame."""
    return rot_y(pitch) @ rot_x(roll)


def to_level(points_lidar, extrinsic_r, extrinsic_t, roll, pitch):
    base = points_lidar @ np.asarray(extrinsic_r).T + np.asarray(extrinsic_t)
    return base, base @ level_rotation(roll, pitch).T


def self_filter(base_points, params):
    x, y = base_points[:, 0], base_points[:, 1]
    body = ((x > params.self_filter_x[0]) & (x < params.self_filter_x[1]) &
            (y > params.self_filter_y[0]) & (y < params.self_filter_y[1]))
    return ~body


def estimate_floor(level_points, leg_reference_z, params):
    """Floor height under the robot in the level frame.

    Reference: -(mean leg extension), from joint telemetry. Refined with the median of
    visible low points within 1.8 m that agree with the reference; otherwise unchanged.
    Returns (floor_z, source).
    """
    if leg_reference_z is None or not math.isfinite(leg_reference_z):
        return None, 'none'
    r = np.hypot(level_points[:, 0], level_points[:, 1])
    near = level_points[(r < 1.8) & (np.abs(level_points[:, 2] - leg_reference_z)
                                      < params.floor_search_tolerance_m)]
    if len(near) >= 50:
        # Low percentile: a raised platform ahead must not pull the floor upward.
        lowest = np.percentile(near[:, 2], 10)
        candidate = float(np.median(near[near[:, 2] <= lowest + params.floor_tolerance_m][:, 2]))
        if abs(candidate - leg_reference_z) < params.floor_search_tolerance_m:
            return candidate, 'lidar_floor_consistent_with_legs'
    return float(leg_reference_z), 'leg_geometry'


@dataclass
class TerrainGrid:
    labels: np.ndarray        # int16 [ny, nx]; row = y index, col = x index
    max_height: np.ndarray    # float32 heights above floor, nan when unknown
    min_height: np.ndarray
    counts: np.ndarray
    origin: tuple             # (x, y) of cell [0, 0] corner in level frame
    resolution: float
    floor_z: float
    floor_source: str

    def cell_centres(self):
        ny, nx = self.labels.shape
        xs = self.origin[0] + (np.arange(nx) + 0.5) * self.resolution
        ys = self.origin[1] + (np.arange(ny) + 0.5) * self.resolution
        return xs, ys


def build_grid(level_points, floor_z, params, floor_source='given'):
    res, size = params.resolution_m, params.size_m
    n = int(round(size / res))
    origin = (-size / 2.0, -size / 2.0)
    labels = np.full((n, n), UNKNOWN, np.int16)
    max_h = np.full((n, n), np.nan, np.float32)
    min_h = np.full((n, n), np.nan, np.float32)
    counts = np.zeros((n, n), np.int32)
    if floor_z is None or len(level_points) == 0:
        return TerrainGrid(labels, max_h, min_h, counts, origin, res, floor_z, floor_source)
    ix = np.floor((level_points[:, 0] - origin[0]) / res).astype(np.int64)
    iy = np.floor((level_points[:, 1] - origin[1]) / res).astype(np.int64)
    h = level_points[:, 2] - floor_z
    keep = (ix >= 0) & (ix < n) & (iy >= 0) & (iy < n)
    ix, iy, h = ix[keep], iy[keep], h[keep]
    flat = iy * n + ix
    cnt = np.bincount(flat, minlength=n * n)
    hi = np.full(n * n, -np.inf)
    lo = np.full(n * n, np.inf)
    # Heights relevant to the body: ignore overhead points when computing the cell top.
    body = h <= params.body_clearance_m
    np.maximum.at(hi, flat[body], h[body])
    np.minimum.at(lo, flat, h)
    over_only = (cnt > 0) & ~np.isfinite(hi)
    hi = np.where(np.isfinite(hi), hi, np.nan)
    lo = np.where(np.isfinite(lo), lo, np.nan)
    counts[:] = cnt.reshape(n, n)
    max_h[:] = hi.reshape(n, n)
    min_h[:] = lo.reshape(n, n)
    seen = counts >= params.min_points_per_cell
    # Course of stacked 0.10-0.15 m lips (operator 2026-11-21): a second tier is 0.25 m above the floor the robot
    # stands on but only one lip above the tier before it. A cell above max_step is an obstacle unless it can be
    # REACHED from the seen floor through cells whose tops differ by <= max_step (flood fill). Walls, poles and
    # objects whose base is hidden stay obstacles: nothing walkable leads up to them. Drops stay absolute.
    reachable = _reachable(np.where(seen, max_h, np.nan), params)
    with np.errstate(invalid='ignore'):
        drop = seen & (min_h < -params.max_drop_m)
        obstacle = seen & (max_h > params.max_step_m) & ~reachable
        step_up = seen & (max_h > params.floor_tolerance_m) & ~obstacle
        step_down = seen & (min_h < -params.floor_tolerance_m) & ~drop & ~obstacle & ~step_up   # lower floor
        step = step_up | step_down
        free = seen & ~obstacle & ~step & ~drop & (np.abs(max_h) <= params.floor_tolerance_m)
    labels[seen & over_only.reshape(n, n)] = OVERHEAD
    labels[free] = FREE
    labels[step] = STEP
    labels[drop] = DROP
    labels[obstacle] = OBSTACLE
    return TerrainGrid(labels, max_h, min_h, counts, origin, res, floor_z, floor_source)


STEP_TOP_CLEAR_M = 0.30   # no obstacle this close behind a step, or it is a wall's base, not a step


def _reachable(top, params, max_iterations=200):
    """Cells reachable from the seen floor (|top| <= floor tolerance) by steps of <= max_step between 8-neighbours
    (top = cell top height above floor, NaN = unseen). Iterative dilation; ~n/2 rounds at most."""
    n0, n1 = top.shape
    with np.errstate(invalid='ignore'):
        reach = np.abs(top) <= params.floor_tolerance_m
        walkable = np.isfinite(top) & (top <= params.body_clearance_m)
    shifts = [(dy, dx) for dy in (-1, 0, 1) for dx in (-1, 0, 1) if dy or dx]
    for _ in range(max_iterations):
        grown = reach.copy()
        for dy, dx in shifts:
            src = (slice(max(0, -dy), n0 - max(0, dy)), slice(max(0, -dx), n1 - max(0, dx)))
            dst = (slice(max(0, dy), n0 - max(0, -dy)), slice(max(0, dx), n1 - max(0, -dx)))
            with np.errstate(invalid='ignore'):
                ok = reach[src] & walkable[dst] & (np.abs(top[dst] - top[src]) <= params.max_step_m)
            grown[dst] |= ok
        if (grown == reach).all():
            break
        reach = grown
    return reach


def corridor_summary(grid, params):
    """Forward corridor along the body heading: what lies directly ahead."""
    xs, ys = grid.cell_centres()
    cols = np.where((xs > 0.0) & (xs <= params.corridor_length_m))[0]
    rows = np.where(np.abs(ys) <= params.corridor_half_width_m)[0]
    first = {}
    for name, code in (('obstacle', OBSTACLE), ('drop', DROP), ('step', STEP)):
        hits = [xs[c] for c in cols if np.any(grid.labels[rows, c] == code)]
        first[f'nearest_{name}_m'] = round(float(min(hits)), 3) if hits else None
    # Signed height of the nearest step (+ up / - down): roam's automatic stair trigger reads it.
    first['nearest_step_rise_m'] = None
    # A step has a walkable top: an obstacle starting just behind it means the cells are the base of a wall seen
    # from above (depth camera, 2026-11-21: a wall 0.5 m ahead read as a 0.13 m "step" and started AI mode).
    wall_base = first['nearest_step_m'] is not None and first['nearest_obstacle_m'] is not None and \
        first['nearest_obstacle_m'] - first['nearest_step_m'] < STEP_TOP_CLEAR_M
    if first['nearest_step_m'] is not None and not wall_base:
        c = int(np.argmin(np.abs(xs - first['nearest_step_m'])))
        cells = rows[grid.labels[rows, c] == STEP]
        up, down = grid.max_height[cells, c], grid.min_height[cells, c]
        with np.errstate(invalid='ignore'):
            top = float(np.nanmax(up)) if np.isfinite(up).any() else 0.0
            low = float(np.nanmin(down)) if np.isfinite(down).any() else 0.0
        first['nearest_step_rise_m'] = round(top if top >= -low else low, 3)
    near_cols = np.where((xs >= params.near_field_m[0]) & (xs <= params.near_field_m[1]))[0]
    near = grid.labels[np.ix_(rows, near_cols)]
    corridor = grid.labels[np.ix_(rows, cols)]
    heights = grid.max_height[np.ix_(rows, cols)]
    with np.errstate(invalid='ignore'):
        step_heights = heights[(corridor == STEP) | (corridor == OBSTACLE)]
    seen_cols = [xs[c] for c in cols if np.mean(grid.labels[rows, c] != UNKNOWN) >= 0.5]
    return {
        **first,
        'near_field_unknown_fraction': round(float(np.mean(near == UNKNOWN)), 3) if near.size else 1.0,
        'first_observed_ahead_m': round(float(min(seen_cols)), 3) if seen_cols else None,
        'max_rise_in_corridor_m': round(float(np.nanmax(step_heights)), 3)
        if step_heights.size and np.isfinite(step_heights).any() else None,
        'clear_ahead_m': _clear_distance(grid, rows, cols[xs[cols] >= params.near_field_m[0]], xs),
    }


def _clear_distance(grid, rows, cols, xs):
    """Distance ahead over which every corridor column is observed and FREE/OVERHEAD."""
    clear = 0.0
    for c in cols:
        column = grid.labels[rows, c]
        if np.all(np.isin(column, (FREE, OVERHEAD))):
            clear = float(xs[c])
        elif np.any(np.isin(column, (OBSTACLE, DROP, STEP))) or np.mean(column == UNKNOWN) > 0.5:
            break
    return round(clear, 3)


def label_counts(grid):
    values, counts = np.unique(grid.labels, return_counts=True)
    return {LABEL_NAMES[int(v)]: int(c) for v, c in zip(values, counts)}


def occupancy_values(grid):
    """nav_msgs/OccupancyGrid data: -1 unknown, 0 free/overhead, 50 step, 100 obstacle/drop."""
    out = np.where(grid.labels == OVERHEAD, FREE, grid.labels)
    out = np.where(out == DROP, OBSTACLE, out)
    return out.astype(np.int8)


def hazard_points(grid):
    """Cell centres (level frame, floor-relative z) of obstacle/drop cells for costmap marking."""
    xs, ys = grid.cell_centres()
    rows, cols = np.where(np.isin(grid.labels, (OBSTACLE, DROP)))
    z = np.where(grid.labels[rows, cols] == DROP, 0.0,
                 np.nan_to_num(grid.max_height[rows, cols], nan=0.0))
    return np.stack([xs[cols], ys[rows], z + grid.floor_z], axis=1) if len(rows) else np.zeros((0, 3))


class Accumulator:
    """Keeps recent clouds in an odometry frame to fill the LiDAR's near-field blind zone.

    The Mid-360 is pitched 15 deg down with a -7 deg lower FOV: the floor closer than
    roughly 0.7-1.1 m ahead and almost everything behind is never in a single scan.
    Stairs being approached therefore leave the view; accumulation keeps them.
    Requires an odometry pose (e.g. university Fast-LIO odom -> base_link).
    """

    def __init__(self, horizon_s=4.0, max_points=120000, voxel_m=0.03):
        self.horizon_s = horizon_s
        self.max_points = max_points
        self.voxel_m = voxel_m
        self.frames = []   # (stamp, points in odom frame)

    def add(self, stamp, base_points, odom_r, odom_t):
        world = base_points @ np.asarray(odom_r).T + np.asarray(odom_t)
        if self.voxel_m:
            keys = np.floor(world / self.voxel_m).astype(np.int64)
            _, first = np.unique(keys, axis=0, return_index=True)
            world = world[np.sort(first)]
        self.frames.append((stamp, world))
        self.frames = [(t, p) for t, p in self.frames if stamp - t <= self.horizon_s]
        while sum(len(p) for _, p in self.frames) > self.max_points and len(self.frames) > 1:
            self.frames.pop(0)

    def base_points(self, odom_r, odom_t):
        if not self.frames:
            return np.zeros((0, 3))
        world = np.concatenate([p for _, p in self.frames])
        r = np.asarray(odom_r)
        return (world - np.asarray(odom_t)) @ r   # inverse rigid transform: R^T (p - t)


@dataclass(frozen=True)
class StopParams:
    """Proximity stop for the twist_mux lock. Forward-only driving (Nav2 vx_min = 0).

    stop_ahead_m: footprint front 0.305 m + 0.20 m nose-to-obstacle stop boundary (operator decision
    2026-11-19, was 0.295 m). Covers the calibrated stop up to 0.08 m/s (reaction ~0.03 + coast ~0.06
    + 5 cm grid margin = 0.14 m, leaving >= 5 cm); at 0.10 m/s about 4 cm would remain.
    body_margin_m: hazard within this distance of the body footprint (0.61 x 0.37 m box) on any side.
    Operator 2026-11-20 (course <= 1.5 m wide): was a 0.40 m circle round the centre, which stopped
    the robot for walls 21 cm beside it. In-place turns are covered by roam's turn-room check and
    Nav2's footprint collision checking, not by this backup lock.
    """
    stop_ahead_m: float = 0.505         # default nose + 0.20 (physical tests); roaming lowers it to 0.405
    body_margin_m: float = 0.05         # 5 cm round the footprint (was a 0.40 m circle; operator 2026-11-20)
    half_width_m: float = 0.235         # body half width 0.185 + 5 cm (was 0.30)
    release_hold_s: float = 1.0


# Runtime-adjustable range of the proximity stop (ROS parameters on terrain_monitor). The lower
# bounds are the validated defaults: a test may make the stop MORE conservative, never less.
NOSE_OFFSET_M = 0.305          # footprint front edge ahead of base_link
STOP_LIMITS = {'stop_ahead_m': (0.355, 3.40),   # floor nose + 0.05 for roaming (operator 2026-11-20);   # was 1.80; 2.95 m nose + 0.305 at 1.0 m/s
               'body_margin_m': (StopParams.body_margin_m, 1.00)}
FOOTPRINT_HALF_LENGTH_M = 0.305   # Nav2 footprint [[0.305, 0.185], ...]
FOOTPRINT_HALF_WIDTH_M = 0.185


def updated_stop_params(params, name, value):
    """New StopParams with one field changed, or ValueError if outside STOP_LIMITS."""
    if name not in STOP_LIMITS:
        raise ValueError(f'{name} is not adjustable')
    low, high = STOP_LIMITS[name]
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) \
            or not low <= value <= high:
        raise ValueError(f'{name}={value} outside [{low}, {high}]; the minimum is the validated '
                         'default and cannot be lowered')
    return StopParams(**{**params.__dict__, name: float(value)})


def proximity_stop(grid, params=StopParams()):
    """Return (stop, reason) from obstacle/drop cells near the robot (level frame)."""
    xs, ys = grid.cell_centres()
    rows, cols = np.where(np.isin(grid.labels, (OBSTACLE, DROP)))
    if not len(rows):
        return False, None
    x, y = xs[cols], ys[rows]
    ahead = (x > 0) & (x <= params.stop_ahead_m) & (np.abs(y) <= params.half_width_m)
    around = (np.abs(x) <= FOOTPRINT_HALF_LENGTH_M + params.body_margin_m) & \
             (np.abs(y) <= FOOTPRINT_HALF_WIDTH_M + params.body_margin_m)
    if ahead.any():
        return True, f'hazard {float(x[ahead].min()):.2f} m ahead'
    if around.any():
        return True, f'hazard within {params.body_margin_m} m of the body'
    return False, None
