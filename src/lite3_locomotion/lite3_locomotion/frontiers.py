"""Bounded 2D frontier selection for visualization only; no ROS or actuation."""

from collections import deque
from dataclasses import dataclass
import math
from typing import Optional, Sequence, Tuple


@dataclass(frozen=True)
class Grid:
    width: int
    height: int
    resolution: float
    data: Sequence[int]
    origin_x: float = 0.0
    origin_y: float = 0.0
    origin_yaw: float = 0.0

    def __post_init__(self):
        if (type(self.width) is not int or type(self.height) is not int or
                self.width <= 0 or self.height <= 0 or self.width * self.height > 1_000_000):
            raise ValueError('Grid must contain 1..1,000,000 cells')
        if not all(math.isfinite(v) for v in (
            self.resolution, self.origin_x, self.origin_y, self.origin_yaw,
        )) or self.resolution <= 0:
            raise ValueError('Grid geometry must be finite with positive resolution')
        if len(self.data) != self.width * self.height or any(
            type(value) is not int or not -1 <= value <= 100 for value in self.data
        ):
            raise ValueError('Occupancy data must match dimensions and use -1..100')

    def contains(self, cell):
        x, y = cell
        return 0 <= x < self.width and 0 <= y < self.height

    def value(self, cell):
        return self.data[cell[1] * self.width + cell[0]]

    def world(self, cell):
        x, y = ((v + 0.5) * self.resolution for v in cell)
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        return self.origin_x + c * x - s * y, self.origin_y + s * x + c * y

    def cell(self, x, y):
        dx, dy = x - self.origin_x, y - self.origin_y
        c, s = math.cos(self.origin_yaw), math.sin(self.origin_yaw)
        return math.floor((c * dx + s * dy) / self.resolution), math.floor((-s * dx + c * dy) / self.resolution)


@dataclass(frozen=True)
class FrontierConfig:
    radius: float = 5.0
    clearance: float = 0.30
    min_cluster_cells: int = 3
    max_search_cells: int = 30_000

    def __post_init__(self):
        if not math.isfinite(self.radius) or not 0 < self.radius <= 10:
            raise ValueError('Preview radius must be in (0, 10] metres')
        if not math.isfinite(self.clearance) or not 0 <= self.clearance <= 1:
            raise ValueError('Clearance must be in [0, 1] metres')
        for name, maximum in (('min_cluster_cells', 1000), ('max_search_cells', 50_000)):
            value = getattr(self, name)
            if type(value) is not int or not 1 <= value <= maximum:
                raise ValueError(f'{name} must be an integer in [1, {maximum}]')


@dataclass(frozen=True)
class Candidate:
    x: float
    y: float
    yaw: float
    cluster_cells: int
    path_distance: float
    score: float


@dataclass(frozen=True)
class ExplorationResult:
    status: str
    candidates: Tuple[Candidate, ...] = ()
    visited_cells: int = 0

    @property
    def selected(self) -> Optional[Candidate]:
        return self.candidates[0] if self.candidates else None


def neighbors(cell):
    x, y = cell
    return ((x - 1, y), (x + 1, y), (x, y - 1), (x, y + 1))


def select_frontiers(grid: Grid, robot_x: float, robot_y: float,
                     config: FrontierConfig = FrontierConfig()) -> ExplorationResult:
    if not math.isfinite(robot_x) or not math.isfinite(robot_y):
        raise ValueError('Robot pose must be finite')
    start = grid.cell(robot_x, robot_y)
    if not grid.contains(start) or not 0 <= grid.value(start) <= 20:
        return ExplorationResult('robot_not_in_known_free_cell')
    radius_cells = math.ceil(config.radius / grid.resolution)
    padding = math.ceil(config.clearance / grid.resolution)
    x0, x1 = max(0, start[0] - radius_cells - padding), min(grid.width, start[0] + radius_cells + padding + 1)
    y0, y1 = max(0, start[1] - radius_cells - padding), min(grid.height, start[1] + radius_cells + padding + 1)
    if (x1 - x0) * (y1 - y0) > 200_000:
        return ExplorationResult('local_grid_limit_exceeded')

    # Summed-area tables make conservative square-clearance checks O(1).
    # BFS avoids known obstacles/uncertain cells; goals additionally avoid unknown.
    stride = x1 - x0 + 1
    obstacles = [0] * (stride * (y1 - y0 + 1))
    unsafe = [0] * len(obstacles)
    for y in range(y0, y1):
        obs_row = unsafe_row = 0
        row = (y - y0 + 1) * stride
        previous = row - stride
        for x in range(x0, x1):
            value = grid.value((x, y))
            obs_row += value > 20
            unsafe_row += value < 0 or value > 20
            index = row + x - x0 + 1
            obstacles[index] = obstacles[previous + x - x0 + 1] + obs_row
            unsafe[index] = unsafe[previous + x - x0 + 1] + unsafe_row

    def clear(cell, table):
        x, y = cell
        left, right = x - padding, x + padding + 1
        bottom, top = y - padding, y + padding + 1
        if left < x0 or right > x1 or bottom < y0 or top > y1:
            return False
        left, right, bottom, top = left - x0, right - x0, bottom - y0, top - y0
        total = (table[top * stride + right] - table[top * stride + left] -
                 table[bottom * stride + right] + table[bottom * stride + left])
        return total == 0

    if not clear(start, obstacles):
        return ExplorationResult('robot_clearance_insufficient')
    queue = deque([start])
    distances = {start: 0}
    frontiers = set()
    while queue:
        cell = queue.popleft()
        for other in neighbors(cell):
            if not grid.contains(other):
                continue  # Map edges alone are not unexplored frontiers.
            value = grid.value(other)
            if value == -1:
                frontiers.add(cell)
            if other in distances or not 0 <= value <= 20:
                continue
            wx, wy = grid.world(other)
            if math.hypot(wx - robot_x, wy - robot_y) > config.radius or not clear(other, obstacles):
                continue
            if len(distances) >= config.max_search_cells:
                return ExplorationResult('search_limit_exceeded', visited_cells=len(distances))
            distances[other] = distances[cell] + 1
            queue.append(other)

    candidates = []
    clusters_processed = 0
    for seed in sorted(frontiers):
        if seed not in frontiers:
            continue
        clusters_processed += 1
        if clusters_processed > 200:
            return ExplorationResult('frontier_cluster_limit_exceeded', visited_cells=len(distances))
        frontiers.remove(seed)
        cluster = [seed]
        queue = deque([seed])
        while queue:
            for other in neighbors(queue.popleft()):
                if other in frontiers:
                    frontiers.remove(other)
                    cluster.append(other)
                    queue.append(other)
        if len(cluster) < config.min_cluster_cells:
            continue
        # Representative belongs to the cluster, even for concave frontiers.
        cx = sum(cell[0] for cell in cluster) / len(cluster)
        cy = sum(cell[1] for cell in cluster) / len(cluster)
        representative = min(cluster, key=lambda cell: ((cell[0] - cx)**2 + (cell[1] - cy)**2, cell))
        reach = math.ceil((config.clearance + 0.5) / grid.resolution)
        choices = []
        for y in range(max(y0, representative[1] - reach), min(y1, representative[1] + reach + 1)):
            for x in range(max(x0, representative[0] - reach), min(x1, representative[0] + reach + 1)):
                cell = (x, y)
                if cell in distances and clear(cell, unsafe):
                    choices.append(cell)
        if not choices:
            continue
        goal = min(choices, key=lambda cell: (
            (cell[0] - representative[0])**2 + (cell[1] - representative[1])**2,
            distances[cell], cell,
        ))
        gx, gy = grid.world(goal)
        fx, fy = grid.world(representative)
        distance = distances[goal] * grid.resolution
        candidates.append(Candidate(gx, gy, math.atan2(fy - gy, fx - gx), len(cluster), distance,
                                    len(cluster) * grid.resolution / (1 + distance)))
    candidates.sort(key=lambda item: (-item.score, item.path_distance, item.x, item.y))
    return ExplorationResult('frontiers_found' if candidates else 'no_frontiers_in_preview_area',
                             tuple(candidates[:50]), len(distances))
