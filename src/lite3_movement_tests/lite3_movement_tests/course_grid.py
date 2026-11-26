"""Course grid: roam's MEMORY of which parts of the course still look unexplored.

Operator 2026-11-24: "this grid search should only be for the dog to store it as somewhat of a memory, acting as
an indicator that tells the dog: this grid seems unexplored, lets get back there as soon as we can". So the grid
never drives the robot. It keeps, per cell, three DISTINCT facts and offers "come back" points to the one goal
evaluator (goal_selection.py), which weighs them against every other candidate.

  mapped   fraction of the cell's SLAM cells that are known (free or occupied)
  visited  the walked path (breadcrumbs) passed through the cell
  seen     fraction of the cell's mapped free floor the front camera has looked at (coverage.CameraCoverage)

  connections to the 4 neighbours: open / unknown / blocked, from the SLAM map (three lanes across the shared edge,
  after ~/grid_roam_nav2/preview.py Explorer.crossing by a teammate, re-implemented here on numpy), plus failure
  memory: a failed walk across a connection blocks it for a while (temporary) or longer (persistent).

  unfinished  reachable (inside the walls), enough mapped free floor, not temporarily blocked, and not INSPECTED yet.
              Operator 2026-11-25: mapped is never inspected, and reaching the centre does not complete a cell either -
              inspected = walked through AND the camera covered enough of its floor (SEEN_INSPECTED) AND no mission
              evidence pending there (mark_pending / resolve). Two inspection visits that still leave the camera short
              count as inspected (the floor is hidden by blocks). An unfinished cell is the "let's get back there" flag.
  walls       how many of the 4 sides are blocked (a wall, small gaps closed). Operator 2026-11-25: "if it sees 3 walls
              around the grid, it will process it as a room"; operator 2026-11-26: 2 walls (ROOM_WALLS) - a room is
              read with scene understanding (rooms_read).
              Cells with 2+ walls are worth walking into to see whether they are a dead end (stronger pull).

Alignment is automatic (origin = start pose, +X = start heading; nothing about the course is hard-coded) and can be
overridden per run. The grid belongs to one map session: a map reset clears it. Pure (numpy): unit-tested.
"""

import math

import numpy as np

CELL_M = 1.0
HALF_SIZE_M = 12.0                    # cells kept within this of the origin (geofence max 10 m + margin)
MIN_FREE_M2 = 0.20                    # less mapped free floor than this: not a place to go
VISIT_REACH_M = 0.20                  # walked through = the dog's centre came within this of the cell (its body,
                                       # 0.61 x 0.37 m, was then inside it): a cell along a wall has only ~0.5 m of
                                       # floor and its come-back point sat 0.45 m from the wall, out of reach (sim)
SEEN_DONE = 0.70                      # camera has seen this much of the free floor: looked at
SEEN_INSPECTED = 0.50                 # inspected = walked through + the camera saw at least this much of the floor
INSPECT_VISITS = 2                    # ... or this many inspection visits (floor hidden behind blocks)
CENTRE_TOL_M = 0.15                   # a goal point is "the middle of the cell" within this of the centre
ROOM_WALLS = 2                        # operator 2026-11-26: 2 walls (was 3, operator 2026-11-25)
WALL_SIDE_COVER = 0.80                # a side is a wall when the map is occupied along >= 80 % of it (not a prop)
WALL_BAND_M = 0.12                    # ... within this of the side line
OPEN_NEIGHBOUR_MAPPED = 0.50          # a neighbour mapped less than this behind a passable edge: still to explore
LANE_OFFSETS = (-0.25, 0.0, 0.25)     # x cell_m, lanes across a shared edge
LANE_HALF_M = 0.20                    # each lane checked this far either side of the edge
LANE_BODY_M = 0.15                    # lateral half-width checked along a lane (body half width)
RETURN_CLEARANCE_M = 0.35             # a "come back" point needs free floor this far around it (was 0.50: on
                                       # the cluttered course hardly any point qualified; Nav2 checks the fit)
RETURN_ROOMY_M = 0.55                 # first choice: a point this clear (a cell along a wall: 0.55 m out from it,
                                       # still inside VISIT_REACH_M of the cell; 0.35 m from the wall stalled the sim)
RETURN_REACH_M = 0.25                 # ... and may lie this far outside the cell (a viewpoint, not a centre)
TEMP_BLOCK_S, TEMP_MAX = 10.0, 3      # same limits as exploration_record (temporary / persistent failures)
PERSIST_BLOCK_S, PERSIST_MAX = 120.0, 2
MAX_TAGS = 5

FREE_MAX, OCC_MIN = 20, 50            # SLAM: 0..20 free, > 50 occupied, -1 unknown


def _slam(grid):
    return (np.asarray(grid.data, dtype=np.int16).reshape(grid.height, grid.width), grid.resolution,
            grid.origin_x, grid.origin_y)


class CourseGrid:
    def __init__(self, origin_x, origin_y, heading, cell_m=CELL_M, half_size_m=HALF_SIZE_M, session=None):
        if not all(math.isfinite(v) for v in (origin_x, origin_y, heading, cell_m)) or not 0.3 <= cell_m <= 3.0:
            raise ValueError('course grid alignment must be finite with a 0.3-3 m cell')
        self.ox, self.oy, self.heading, self.cell_m = float(origin_x), float(origin_y), float(heading), float(cell_m)
        self.c, self.s = math.cos(heading), math.sin(heading)
        self.n = int(math.ceil(half_size_m / cell_m))
        self.session = session
        self.reset(session)

    # ------------------------------------------------------------------ geometry
    def reset(self, session=None):
        self.session = session if session is not None else self.session
        self.cells = {}                   # (i, j) -> dict(mapped, free_m2, seen, visited, reachable, walls)
        self.pending = {}                 # cell -> reason: mission evidence there still to resolve
        self.rooms_read = {}              # room cell -> scene result (or why none)
        self.captured = {}                # pocket cell -> camera capture result (operator 2026-11-25, item 7)
        self.features = {}                # cell -> 'stairs' | 'curtain' detected there (RViz: grey)
        self.cell_blocks = {}             # cell -> until: temporarily not a place to go (failed there)
        self.inspect_visits = {}          # cell -> inspection visits (a "come back" goal reached there)
        self.reopened = []                # connections that turned from blocked to open in the last update
        self.visited = set()
        self.edges = {}                   # frozenset({a, b}) -> 'open' | 'unknown' | 'blocked'
        self.edge_failures = {}           # frozenset({a, b}) -> dict(category, attempts, until)
        self._key = None

    def cell(self, x, y):
        dx, dy = x - self.ox, y - self.oy
        return (math.floor((self.c * dx + self.s * dy) / self.cell_m + 0.5),
                math.floor((-self.s * dx + self.c * dy) / self.cell_m + 0.5))

    def centre(self, i, j):
        u, v = i * self.cell_m, j * self.cell_m
        return self.ox + self.c * u - self.s * v, self.oy + self.s * u + self.c * v

    def _walked(self, breadcrumbs, reach=VISIT_REACH_M):
        """Cells the dog's body was in: its centre within reach of the cell (each breadcrumb marks up to 4 cells)."""
        out = set()
        for x, y in breadcrumbs:
            dx, dy = x - self.ox, y - self.oy
            u, v = (self.c * dx + self.s * dy) / self.cell_m, (-self.s * dx + self.c * dy) / self.cell_m
            r = reach / self.cell_m
            for i in {math.floor(u - r + 0.5), math.floor(u + 0.5), math.floor(u + r + 0.5)}:
                for j in {math.floor(v - r + 0.5), math.floor(v + 0.5), math.floor(v + r + 0.5)}:
                    out.add((i, j))
        return out

    def local(self, x, y, i, j):
        """(u, v) of a world point in cell (i, j)'s own frame, centre at 0."""
        dx, dy = x - self.ox, y - self.oy
        return self.c * dx + self.s * dy - i * self.cell_m, -self.s * dx + self.c * dy - j * self.cell_m

    def _world(self, i, j, u, v):
        return self.centre(i, j)[0] + self.c * u - self.s * v, self.centre(i, j)[1] + self.s * u + self.c * v

    # ------------------------------------------------------------------ update
    def update(self, grid, coverage=None, breadcrumbs=(), now=0.0, walls=None, reach=None):
        """Recompute mapped / seen / visited per cell and the connection states from the current SLAM map.
        walls: optional bool mask (SLAM shape) of extra wall cells - small gaps in a wall closed (wall_gaps.py).
        reach: optional bool mask of the floor connected to the dog inside the walls; cells without any of it are
        outside the course (seen over / through a wall) and never "come back" targets."""
        data, res, gx, gy = _slam(grid)
        if walls is not None and walls.shape == data.shape:
            data = np.where(walls, np.int16(100), data)
        h, w = data.shape
        jj, ii = np.nonzero(data >= 0)                                        # known SLAM cells only
        if ii.size == 0:
            self.cells, self.edges = {}, {}
            return
        wx, wy = gx + (ii + 0.5) * res, gy + (jj + 0.5) * res
        dx, dy = wx - self.ox, wy - self.oy
        ci = np.floor((self.c * dx + self.s * dy) / self.cell_m + 0.5).astype(int)
        cj = np.floor((-self.s * dx + self.c * dy) / self.cell_m + 0.5).astype(int)
        keep = (np.abs(ci) <= self.n) & (np.abs(cj) <= self.n)
        ci, cj, wx, wy = ci[keep], cj[keep], wx[keep], wy[keep]
        vals = data[jj[keep], ii[keep]]
        inside = reach[jj[keep], ii[keep]] if reach is not None and reach.shape == data.shape else None
        free = vals <= FREE_MAX
        span = 2 * self.n + 1
        reach_n = None
        key = (ci + self.n) * span + (cj + self.n)
        known_n = np.bincount(key, minlength=span * span)
        free_n = np.bincount(key, weights=free, minlength=span * span)
        seen_n = np.zeros(span * span)
        if inside is not None:
            reach_n = np.bincount(key, weights=inside, minlength=span * span)
        if coverage is not None and free.any():
            fi, fj, ok = coverage._cells(wx[free], wy[free])
            seen = np.zeros(fi.size, dtype=bool)
            seen[ok] = coverage.seen[fj[ok], fi[ok]]
            seen_n = np.bincount(key[free], weights=seen, minlength=span * span)
        per_cell = (self.cell_m / res) ** 2
        self.visited = self._walked(breadcrumbs)
        cells = {}
        for k in np.nonzero(known_n)[0]:
            c = (int(k // span - self.n), int(k % span - self.n))
            f = float(free_n[k])
            cells[c] = {'mapped': min(1.0, float(known_n[k]) / per_cell), 'free_m2': f * res * res,
                        'seen': float(seen_n[k]) / f if f else 0.0, 'visited': c in self.visited,
                        'reachable': True if reach_n is None else reach_n[k] * res * res >= MIN_FREE_M2}
        for c in self.visited:
            cells.setdefault(c, {'mapped': 0.0, 'free_m2': 0.0, 'seen': 0.0, 'visited': True, 'reachable': True})
        self.cells = cells
        self._data, self._geo = data, (res, gx, gy)
        pairs = [(c, n) for c in cells for n in ((c[0] + 1, c[1]), (c[0], c[1] + 1))]
        before = self.edges
        self.edges = {frozenset(p): st for p, st in zip(pairs, self._edge_states(pairs))}
        self.reopened = [k for k, st in self.edges.items() if st == 'open' and before.get(k) == 'blocked']
        for c, info in cells.items():
            info['walls'] = sum(1 for n in self.neighbours(c) if self.edges.get(frozenset((c, n))) == 'blocked'
                                and self.side_cover(c, n) >= WALL_SIDE_COVER)

    def side_cover(self, a, b):
        """Fraction of the shared side of cells a and b along which the map holds an occupied cell within
        WALL_BAND_M of the side line. A wall panel runs along most of the side; a prop (the kitchen set, a block
        pile) usually covers only part of it. Operator 2026-11-25: props must not make "rooms"."""
        du, dv = b[0] - a[0], b[1] - a[1]
        n_pts, hits = 20, 0
        band = np.linspace(-WALL_BAND_M, WALL_BAND_M, 5)
        for k in range(n_pts):
            t = (k + 0.5) / n_pts - 0.5                       # along the side, -0.5 .. 0.5 cell
            for off in band:
                u = du * (0.5 * self.cell_m + off) - dv * (t * self.cell_m)
                v = dv * (0.5 * self.cell_m + off) + du * (t * self.cell_m)
                if self._value(*self._world(a[0], a[1], u, v)) > OCC_MIN:
                    hits += 1
                    break
        return hits / n_pts

    @staticmethod
    def neighbours(c):
        return [(c[0] + 1, c[1]), (c[0] - 1, c[1]), (c[0], c[1] + 1), (c[0], c[1] - 1)]

    def _value(self, x, y):
        res, gx, gy = self._geo
        i, j = int(math.floor((x - gx) / res)), int(math.floor((y - gy) / res))
        if 0 <= i < self._data.shape[1] and 0 <= j < self._data.shape[0]:
            return int(self._data[j, i])
        return -1

    def _edge_states(self, pairs):
        """_edge_state for many connections at once (numpy; ~10x faster on a 40 m map, runs 2026-11-24)."""
        if not pairs:
            return []
        res, gx, gy = self._geo
        data = self._data
        steps = max(2, int(2 * LANE_HALF_M / (res / 2)))
        ts = -LANE_HALF_M + 2 * LANE_HALF_M * np.arange(steps + 1) / steps
        sides = np.array([-LANE_BODY_M, 0.0, LANE_BODY_M])
        offs = np.array(LANE_OFFSETS) * self.cell_m
        a = np.array([p[0] for p in pairs], dtype=float)
        d = np.array([(p[1][0] - p[0][0], p[1][1] - p[0][1]) for p in pairs], dtype=float)
        along = 0.5 * self.cell_m + ts                                    # (S,)
        across = offs[:, None] + sides[None, :]                           # (L, W)
        du, dv = d[:, 0][:, None, None, None], d[:, 1][:, None, None, None]
        al = along[None, None, :, None]
        ac = across[None, :, None, :]
        u = du * al - dv * ac + a[:, 0][:, None, None, None] * self.cell_m
        v = dv * al + du * ac + a[:, 1][:, None, None, None] * self.cell_m
        x = self.ox + self.c * u - self.s * v
        y = self.oy + self.s * u + self.c * v
        i = np.floor((x - gx) / res).astype(int)
        j = np.floor((y - gy) / res).astype(int)
        inside = (i >= 0) & (i < data.shape[1]) & (j >= 0) & (j < data.shape[0])
        vals = np.full(i.shape, -1, dtype=np.int16)
        vals[inside] = data[j[inside], i[inside]]
        blocked = (vals > OCC_MIN).any(axis=(2, 3))                       # per edge, per lane
        unknown = (vals < 0).any(axis=(2, 3))
        open_lane = ~blocked & ~unknown
        unk_lane = ~blocked & unknown
        return ['open' if o.any() else 'unknown' if k.any() else 'blocked' for o, k in zip(open_lane, unk_lane)]

    def _edge_state(self, a, b):
        """Three lanes across the shared edge (teammate's Explorer.crossing): open if one lane is free floor for
        the body width; unknown if none is open but one has unmapped cells; blocked otherwise."""
        du, dv = b[0] - a[0], b[1] - a[1]
        res = self._geo[0]
        steps = max(2, int(2 * LANE_HALF_M / (res / 2)))
        states = []
        for off in LANE_OFFSETS:
            state = 'open'
            for k in range(steps + 1):
                t = -LANE_HALF_M + 2 * LANE_HALF_M * k / steps
                for side in (-LANE_BODY_M, 0.0, LANE_BODY_M):
                    u = du * (0.5 * self.cell_m + t) - dv * (off * self.cell_m + side)
                    v = dv * (0.5 * self.cell_m + t) + du * (off * self.cell_m + side)
                    val = self._value(*self._world(a[0], a[1], u, v))
                    if val > OCC_MIN:
                        state = 'blocked'
                        break
                    if val < 0:
                        state = 'unknown'
                if state == 'blocked':
                    break
            if state == 'open':
                return 'open'
            states.append(state)
        return 'unknown' if 'unknown' in states else 'blocked'

    # ------------------------------------------------------------------ failures (connection memory)
    def note_failure(self, now, from_xy, to_xy, category):
        """A walk from from_xy toward to_xy failed: block the first connection on the way (adjacent cells only)."""
        a, b = self.cell(*from_xy[:2]), self.cell(*to_xy[:2])
        if a == b:
            return None
        if abs(a[0] - b[0]) + abs(a[1] - b[1]) != 1:                       # step to the neighbour toward b
            u, v = self.local(to_xy[0], to_xy[1], a[0], a[1])
            ang = math.atan2(v, u)
            b = (a[0] + round(math.cos(ang)), a[1]) if abs(math.cos(ang)) >= abs(math.sin(ang)) else \
                (a[0], a[1] + round(math.sin(ang)))
        key = frozenset((a, b))
        f = self.edge_failures.get(key) or {'category': category, 'attempts': 0, 'until': -math.inf}
        f['attempts'] += 1
        f['category'] = category
        # Only walls block for good; terrain / temporary trouble only for a while (longer each time).
        f['until'] = now + (PERSIST_BLOCK_S if category == 'persistent' else TEMP_BLOCK_S * min(f['attempts'], 6))
        self.edge_failures[key] = f
        return key

    def connection(self, a, b, now):
        f = self.edge_failures.get(frozenset((a, b)))
        if f is not None:
            if now < f['until'] or (f['category'] == 'persistent' and f['attempts'] >= PERSIST_MAX):
                return 'blocked'
        return self.edges.get(frozenset((a, b)), 'unknown')

    # ------------------------------------------------------------------ memory queries
    def inspected(self, c):
        """Walked through, camera coverage enough (or INSPECT_VISITS tries), no mission evidence pending."""
        info = self.cells.get(c)
        if info is None or not info['visited'] or c in self.pending:
            return False
        return info['seen'] >= SEEN_INSPECTED or self.inspect_visits.get(c, 0) >= INSPECT_VISITS

    def unfinished(self, c, now=0.0):
        info = self.cells.get(c)
        if info is None or info['free_m2'] < MIN_FREE_M2 or not info.get('reachable', True):
            return False                                  # (unreachable: outside the course walls)
        if self.cell_blocks.get(c, -math.inf) > now:
            return False                                  # failed there: not for a while
        # Operator 2026-11-26: the dog never steps into a room (dead end). A room is looked at from the cell in front
        # of it (room_viewpoint) by the room goals, so it is never a come-back target of its own.
        if c in self.rooms_read or self.is_room(c):
            return False
        # Operator 2026-11-24 (grid runs 17:32 / 17:41): seen from a distance is not explored; 2026-11-25: nor is
        # reaching the centre - the camera must have covered it and pending mission evidence be resolved.
        return not self.inspected(c)

    def note_inspection(self, c):
        self.inspect_visits[c] = self.inspect_visits.get(c, 0) + 1

    def block_cell(self, c, now, seconds):
        self.cell_blocks[c] = now + seconds

    def mark_pending(self, c, reason):
        self.pending[c] = reason

    def resolve(self, c):
        self.pending.pop(c, None)

    # ------------------------------------------------------------------ rooms (operator 2026-11-25)
    def is_room(self, c):
        """Walls along ROOM_WALLS (2) sides or more, enough floor, inside the course, and NOT a cell the dog has been in (its start cell
        included): it already saw that one from inside (operator 2026-11-25, Arena B 18:04 SGT)."""
        info = self.cells.get(c)
        return info is not None and info.get('walls', 0) >= ROOM_WALLS and info['free_m2'] >= MIN_FREE_M2 and \
            info.get('reachable', True) and not info['visited'] and not self.is_path(c)

    def wall_sides(self, c):
        """The neighbours across a wall side of c (blocked and covered, as counted in 'walls')."""
        return [n for n in self.neighbours(c) if self.edges.get(frozenset((c, n))) == 'blocked'
                and self.side_cover(c, n) >= WALL_SIDE_COVER]

    def is_path(self, c):
        """Operator 2026-11-26: a corridor - walls on exactly two OPPOSITE sides, open at both ends - is a path the
        dog walks through, never a room (with ROOM_WALLS = 2 every corridor cell would otherwise be a 'room')."""
        sides = self.wall_sides(c) if c in self.cells else []
        if not (len(sides) == 2 and sides[0][0] + sides[1][0] == 2 * c[0] and sides[0][1] + sides[1][1] == 2 * c[1]):
            return False
        # Operator 2026-11-26: a wall or obstacle in front of either open end (any blocked side, even a prop that
        # covers only part of it) makes it a dead end - a room, not a path.
        ends = [n for n in self.neighbours(c) if n not in sides]
        return not any(self.edges.get(frozenset((c, n))) == 'blocked' for n in ends)

    def mark_feature(self, c, kind):
        """A cell where stairs or a curtain was detected (shown grey in RViz)."""
        self.features[c] = kind

    def rooms(self):
        return [c for c in self.cells if self.is_room(c)]

    def rooms_unread(self, now=0.0):
        return [c for c in self.rooms() if c not in self.rooms_read and self.cell_blocks.get(c, -math.inf) <= now]

    def room_viewpoint(self, c, ok=None):
        """(x, y, yaw): the middle of the cell in front of the room's open side, facing into the room; None if none."""
        for n in self.neighbours(c):
            if self.edges.get(frozenset((c, n))) == 'blocked':
                continue
            p = self.return_point(n, ok=ok)
            if p is not None:
                cx, cy = self.centre(*c)
                return p[0], p[1], math.atan2(cy - p[1], cx - p[0])
        return None

    def mark_room_read(self, c, result):
        self.rooms_read[c] = result

    # ------------------------------------------------------------------ pockets (operator 2026-11-25)
    def is_pocket(self, c):
        """Walls on 2+ sides (a corner, a dead end, a room): the camera is used when the dog walks into one."""
        info = self.cells.get(c)
        return info is not None and info.get('walls', 0) >= 2 and info['free_m2'] >= MIN_FREE_M2 and \
            info.get('reachable', True)

    def mark_captured(self, c, result):
        self.captured[c] = result
        if self.is_room(c):
            self.rooms_read.setdefault(c, result)

    def unfinished_cells(self, now=0.0):
        return [c for c in self.cells if self.unfinished(c, now)]

    def pull(self, x, y, now=0.0):
        """Memory term for a goal at (x, y): 1 for an unvisited cell the camera has not seen, down to 0.5 for one it
        has (still to be walked through); 0 for a visited or unreachable cell."""
        c = self.cell(x, y)
        if not self.unfinished(c, now):
            return 0.0
        pull = 1.0 - 0.5 * self.cells[c]['seen']
        if self.cells[c].get('walls', 0) >= 2:            # a wall pattern: walk in and see if it is a dead end
            pull = max(pull, 0.9)
        return pull

    def revisit(self, x, y):
        info = self.cells.get(self.cell(x, y))
        return 1.0 if info and info['visited'] and info['seen'] >= SEEN_DONE else 0.0

    def return_point(self, c, ok=None):
        """A free point in cell c with RETURN_CLEARANCE_M of free floor around it, nearest the cell centre: off
        the centre when the centre is blocked. ok(x, y): the caller's own check (roam: costmap below the inscribed
        cost). None if the cell holds no such point."""
        if not hasattr(self, '_data'):
            return None
        res = self._geo[0]
        step = max(res, 0.1)
        half = 0.5 * self.cell_m + RETURN_REACH_M
        offsets = np.arange(-half, half + 1e-9, step)
        pts = sorted(((u * u + v * v, u, v) for u in offsets for v in offsets))
        # Operator 2026-11-25: goals in the middle of the cells. The centre (within CENTRE_TOL_M) first; only when it
        # is blocked a roomy point, then any point with RETURN_CLEARANCE_M.
        for clear, limit in ((RETURN_CLEARANCE_M, CENTRE_TOL_M), (RETURN_ROOMY_M, None), (RETURN_CLEARANCE_M, None)):
            ring = [(r * math.cos(a), r * math.sin(a)) for r, n in ((clear, 12), (clear * 0.6, 8), (clear * 0.3, 6))
                    for a in np.linspace(0, 2 * math.pi, n, endpoint=False)]
            for d2, u, v in pts:
                if limit is not None and d2 > limit * limit:
                    break
                x, y = self._world(c[0], c[1], u, v)
                if not 0 <= self._value(x, y) <= FREE_MAX:
                    continue
                if all(0 <= self._value(x + rx, y + ry) <= FREE_MAX for rx, ry in ring) and (ok is None or ok(x, y)):
                    return x, y
        return None

    def centre_point(self, c, ok=None):
        """The cell's middle (within CENTRE_TOL_M) if free with RETURN_CLEARANCE_M round it, else None."""
        if not hasattr(self, '_data'):
            return None
        step = max(self._geo[0], 0.05)
        offs = np.arange(-CENTRE_TOL_M, CENTRE_TOL_M + 1e-9, step)
        ring = [(r * math.cos(a), r * math.sin(a)) for r, n in ((RETURN_CLEARANCE_M, 12), (RETURN_CLEARANCE_M * 0.5, 8))
                for a in np.linspace(0, 2 * math.pi, n, endpoint=False)]
        for _, u, v in sorted((u * u + v * v, u, v) for u in offs for v in offs):
            if u * u + v * v > CENTRE_TOL_M ** 2:
                break
            x, y = self._world(c[0], c[1], u, v)
            if 0 <= self._value(x, y) <= FREE_MAX and all(0 <= self._value(x + rx, y + ry) <= FREE_MAX
                                                             for rx, ry in ring) and (ok is None or ok(x, y)):
                return x, y
        return None

    def summary(self, now=0.0):
        unfinished = self.unfinished_cells(now)
        return {'cell_m': self.cell_m, 'origin': [round(self.ox, 2), round(self.oy, 2)],
                'heading_deg': round(math.degrees(self.heading), 1), 'cells_mapped': len(self.cells),
                'cells_visited': len(self.visited), 'cells_unfinished': len(unfinished),
                'unfinished': [list(c) for c in sorted(unfinished)][:40],
                'rooms': [list(c) for c in sorted(self.rooms())][:20],
                'rooms_read': {f'{c[0]},{c[1]}': r for c, r in list(self.rooms_read.items())[:20]},
                'pending': {f'{c[0]},{c[1]}': r for c, r in list(self.pending.items())[:20]},
                'captured': {f'{c[0]},{c[1]}': r for c, r in list(self.captured.items())[:30]},
                'blocked_connections': sum(1 for k in self.edge_failures if self.connection(*tuple(k), now) == 'blocked')}

    # ------------------------------------------------------------------ RViz
    def markers(self, now=0.0, robot_xy=None):
        """Marker specs (exploration_record._m format): flat squares per cell, one marker per class. Points are in
        the grid's own frame (marker pose = grid origin and heading), so squares line up with the grid."""
        from .exploration_record import _m
        # Operator 2026-11-26 colours: yellow = unexplored, blue = room seen and scanned, green = a path (explored, not
        # a room), grey = stairs / curtain detected there; cells that are only mapped are left almost clear.
        groups = {'grid_unexplored': [], 'grid_room_scanned': [], 'grid_path': [], 'grid_feature': [],
                  'grid_mapped': []}
        outline = []
        size, cm = self.cell_m * 0.94, self.cell_m
        for c, info in self.cells.items():
            if info['mapped'] < 0.05 and not info['visited'] and c not in self.features:
                continue
            u, v = c[0] * cm, c[1] * cm
            if c in self.features:
                groups['grid_feature'].append((u, v, 0.0))
            elif c in self.rooms_read:
                groups['grid_room_scanned'].append((u, v, 0.0))
            elif self.unfinished(c, now) or self.is_room(c):
                groups['grid_unexplored'].append((u, v, 0.0))
            elif self.inspected(c) or info['visited']:
                groups['grid_path'].append((u, v, 0.0))
            else:
                groups['grid_mapped'].append((u, v, 0.0))
            if info['visited']:
                h = size / 2
                corners = [(u + su * h, v + sv * h) for su, sv in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
                for k in range(4):
                    outline += [(*corners[k], 0.03), (*corners[(k + 1) % 4], 0.03)]
        colours = {'grid_unexplored': ((1.0, 0.9, 0.1), 0.35), 'grid_room_scanned': ((0.15, 0.45, 1.0), 0.4),
                   'grid_path': ((0.15, 0.75, 0.25), 0.3), 'grid_feature': ((0.55, 0.55, 0.55), 0.55),
                   'grid_mapped': ((1.0, 1.0, 1.0), 0.06)}
        out = []
        for ns, pts in groups.items():
            if pts:
                rgb, alpha = colours[ns]
                out.append(_m('grid', len(out), 'cube_list', self.ox, self.oy, 0.0, self.heading,
                              (size, size, 0.02), rgb, alpha, points=pts))
        if outline:
            out.append(_m('grid', len(out), 'line_list', self.ox, self.oy, 0.0, self.heading, (0.02, 0, 0),
                          (0.2, 0.45, 0.95), 0.8, points=outline))
        crosses = []
        for key in self.edge_failures:
            a, b = tuple(key)
            if self.connection(a, b, now) != 'blocked':
                continue
            mu, mv, r = (a[0] + b[0]) * cm / 2, (a[1] + b[1]) * cm / 2, 0.12
            crosses += [(mu - r, mv - r, 0.05), (mu + r, mv + r, 0.05), (mu - r, mv + r, 0.05), (mu + r, mv - r, 0.05)]
        if crosses:
            out.append(_m('grid', len(out), 'line_list', self.ox, self.oy, 0.0, self.heading, (0.04, 0, 0),
                          (0.9, 0.1, 0.1), points=crosses))
        unfinished = self.unfinished_cells(now)
        if robot_xy is not None:
            unfinished.sort(key=lambda c: math.dist(self.centre(*c), robot_xy))
        for n, c in enumerate(unfinished[:MAX_TAGS]):
            x, y = self.centre(*c)
            out.append(_m('grid_tags', n, 'text', x, y, 0.15, scale=(0, 0, 0.07), alpha=0.9, text='come back'))
        for n, c in enumerate(self.rooms()[:10]):                  # operator 2026-11-25: rooms and whether read
            x, y = self.centre(*c)
            read = c in self.rooms_read
            out.append(_m('grid_rooms', n, 'text', x, y - 0.18, 0.2, scale=(0, 0, 0.09),
                          rgb=(0.3, 0.9, 0.3) if read else (1.0, 0.85, 0.1), text='ROOM read' if read else 'ROOM'))
        tags = []                                                 # why each blocked connection is blocked
        for key, f in self.edge_failures.items():
            a, b = tuple(key)
            if self.connection(a, b, now) == 'blocked':
                left = max(0.0, f['until'] - now)
                tags.append((a, b, f"failed {f['attempts']}x" + (f' {left:.0f}s' if math.isfinite(left)
                                                                  and left > 0 else '')))
        for n, (a, b, text) in enumerate(tags[:8]):
            x, y = self._world(a[0], a[1], (b[0] - a[0]) * cm / 2, (b[1] - a[1]) * cm / 2)
            out.append(_m('grid_blocked', n, 'text', x, y, 0.2, scale=(0, 0, 0.07), rgb=(1.0, 0.3, 0.3), text=text))
        return out
