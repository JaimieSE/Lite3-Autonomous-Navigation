"""Shared exploration record: one state that drives roam's goal selection/recovery AND the RViz overlay.

Operator 2026-11-23. Pure (no ROS): unit-tested; roam owns one ExplorationRecord per session and the ROS
layer only converts build_markers() output into a visualization_msgs/MarkerArray.

What it records
  destination  current goal + action + decision reason (why this goal was chosen)
  breadcrumbs  the travelled path (bounded) - shared with scan-walk's coverage novelty and patrol
  junctions    decision points with >= 2 open directions; each branch UNVISITED / ACTIVE / INSPECTED / BLOCKED /
               DEFERRED. A branch becomes INSPECTED only with a usable-camera confirmation received while it was
               ACTIVE; walked without one it becomes DEFERRED (explored for now, revisitable when nothing else)
  failures     failed destinations: reason, category (persistent obstacle vs temporary telemetry/navigation
               problem), attempt count, retry condition
  inspections  pending mission inspections (request id, mission ids, evidence ref, viewing pose, priority), same
               five states; INSPECTED only on a usable-camera confirmation, else DEFERRED on time-out
  session      the navigation/map session the records belong to; a map reset invalidates positions

Interfaces for the separately developed usable-camera module (std_msgs/String JSON, domain 8)
  /exploration/inspection_requests   {"request_id": str, "mission_ids": [str], "evidence_ref": str,
                                      "viewing_pose": {"x": m, "y": m, "yaw": rad}  (map frame),
                                      "priority": number (higher first, default 0), "map_session": str (optional)}
  /exploration/camera_confirmations  {"usable": bool, "request_id": str (optional), "reason": str (optional)}
       with request_id: confirms (usable) or rejects that inspection while it is ACTIVE at its viewpoint;
       without: a usable frame from the current activity (the ACTIVE branch, or the ACTIVE inspection).
"""

import math
from collections import deque

STATES = ('UNVISITED', 'ACTIVE', 'INSPECTED', 'BLOCKED', 'DEFERRED')
EXHAUSTED = ('INSPECTED', 'BLOCKED')

BREADCRUMB_SPACING_M = 0.3
BREADCRUMB_MAX = 800                 # 240 m of path; oldest dropped
JUNCTION_MERGE_M = 0.8               # decision points closer than this are the same junction
JUNCTION_MIN_SPREAD_DEG = 60.0       # >= 2 open directions at least this far apart make a junction
BRANCH_MATCH_DEG = 30.0              # a heading within this of a branch belongs to it
BRANCH_NEAR_M = 0.8                  # branch rules apply within this of the junction
MAX_JUNCTIONS = 60
DEFERRED_FACTOR = 0.5                # score weight of a DEFERRED branch

FAIL_KEY_M = 0.4                     # destinations closer than this are the same destination
TEMP_RETRY_AFTER_S = 10.0            # temporary failure: retry after this ...
TEMP_MAX_ATTEMPTS = 3                # ... at most this many attempts
PERSIST_AVOID_S = 120.0              # persistent failure: avoid this long ...
PERSIST_MAX_ATTEMPTS = 2             # ... and for good after this many
# Operator 2026-11-24: "difficulty crossing the terrain should not indicate inaccessible as the walls are blatantly
# left there as a proper guide". Terrain trouble (standoff, terrain hold, no progress, Nav2 abort) is 'terrain':
# retried after a growing wait, never given up. Spots on or next to the walked path are reachable by definition.
TERRAIN_RETRY_S = 20.0               # terrain failure: retry after this, doubling per attempt ...
TERRAIN_RETRY_MAX_S = 160.0          # ... up to this; never for good
WALKED_NEAR_M = 0.5                  # a destination this close to the walked path has been reached before
MAX_FAILURES = 200

INSPECTION_ARRIVE_M = 0.35           # at the viewpoint
INSPECTION_YAW_TOL_DEG = 15.0        # facing the viewing yaw
INSPECTION_WAIT_S = 8.0              # wait this long at the viewpoint for a usable-camera confirmation
INSPECTION_DEFER_RETRY_S = 60.0       # a DEFERRED inspection gets one more look, not before this
INSPECTION_MAX_ATTEMPTS = 2          # navigation attempts before BLOCKED
INSPECTION_DUP_M = 0.3               # same missions + viewpoint closer than this = duplicate request
MAX_INSPECTIONS = 50

MAP_RESET_FRACTION = 0.5             # known map area below this fraction of its peak ...
MAP_RESET_MIN_PEAK_M2 = 4.0          # ... (once the peak is meaningful) = the map was reset

TEMPORARY_HINTS = ('rejected', 'cancellation', 'stale', 'telemetry', 'link', 'status_', 'did not acknowledge')
WALL_HINTS = ('wall',)                # only walls make a destination inaccessible
TERRAIN_HINTS = ('obstacle', 'standoff', 'no progress', 'aborted', 'walk ended after', 'timeout', 'hold', 'terrain',
                 'stuck', 'spin stalled')


def classify_failure(reason):
    """'persistent' (a wall: avoid), 'temporary' (telemetry, link, navigation server hiccups: retry soon) or
    'terrain' (uneven ground made it hard: retry after a growing wait, never given up). Unknown reasons count as
    persistent (the safe side for goal selection)."""
    text = str(reason).lower()
    if any(h in text for h in WALL_HINTS):
        return 'persistent'
    if any(h in text for h in TEMPORARY_HINTS):
        return 'temporary'
    if any(h in text for h in TERRAIN_HINTS):
        return 'terrain'
    return 'persistent'


def terrain_wait(attempts):
    return min(TERRAIN_RETRY_MAX_S, TERRAIN_RETRY_S * 2 ** max(0, attempts - 1))


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class ExplorationRecord:
    def __init__(self, session, now=0.0):
        self.session = str(session)
        self.started = now
        self.revision = 0                       # bumped on every change (overlay republishes on change)
        self.breadcrumbs = deque(maxlen=BREADCRUMB_MAX)
        self._reset_positions()
        self.inspections = {}                   # request_id -> dict
        self.map_peak_m2 = 0.0
        self.resets = []                        # [(t, reason)]

    # ---------------------------------------------------------------- bookkeeping
    def _reset_positions(self):
        self.destination = None                 # {kind, x, y, yaw, action, reason, t}
        self.breadcrumbs.clear()
        self.junctions = []                     # [{id, x, y, branches: [{id, heading, state, reason}]}]
        self.next_junction = 1
        self.active_branch = None               # (junction id, branch id)
        self.branch_views = 0                   # usable views while the active branch is walked
        self.failures = {}                      # key -> {x, y, reason, category, attempts, last, retry}
        self.active_inspection = None

    def _changed(self):
        self.revision += 1

    # ---------------------------------------------------------------- session / map
    def observe_map(self, known_m2, now, session=None):
        """Called with each new SLAM map: a different navigation session or a map that lost most of its area
        (slam_toolbox reset) invalidates every position-based record. Returns the reset reason or None."""
        reason = None
        if session is not None and str(session) != self.session:
            reason = f'navigation session changed ({self.session} -> {session})'
            self.session = str(session)
        elif self.map_peak_m2 >= MAP_RESET_MIN_PEAK_M2 and known_m2 < MAP_RESET_FRACTION * self.map_peak_m2:
            reason = f'map reset ({known_m2:.1f} m2 < {MAP_RESET_FRACTION:.0%} of {self.map_peak_m2:.1f} m2)'
        if reason:
            self.reset(now, reason)
            self.map_peak_m2 = known_m2
            return reason
        self.map_peak_m2 = max(self.map_peak_m2, known_m2)
        return None

    def reset(self, now, reason):
        """Positions are meaningless in a new map: drop path, junctions, failures; pending inspections keep their
        request but go DEFERRED until the requester re-sends a viewing pose for this map."""
        self._reset_positions()
        for insp in self.inspections.values():
            if insp['state'] in ('UNVISITED', 'ACTIVE'):
                insp.update(state='DEFERRED', reason=f'invalidated: {reason}', valid=False)
        self.resets.append((now, reason))
        self.resets = self.resets[-10:]
        self._changed()

    # ---------------------------------------------------------------- destination / path
    def set_destination(self, now, goal, action, reason):
        self.destination = {'kind': goal.get('kind', 'walk') if goal else None,
                            'x': goal['x'] if goal else None, 'y': goal['y'] if goal else None,
                            'yaw': goal.get('yaw') if goal else None, 'action': action, 'reason': reason, 't': now}
        self._changed()

    def clear_destination(self, now, reason):
        if self.destination is not None:
            self.destination = {'kind': None, 'x': None, 'y': None, 'yaw': None, 'action': 'idle',
                                'reason': reason, 't': now}
            self._changed()

    def add_breadcrumb(self, x, y):
        if not self.breadcrumbs or math.hypot(x - self.breadcrumbs[-1][0], y - self.breadcrumbs[-1][1]) >= \
                BREADCRUMB_SPACING_M:
            self.breadcrumbs.append((x, y))
            self._changed()
            return True
        return False

    # ---------------------------------------------------------------- junctions / branches
    def junction_at(self, x, y):
        best = None
        for j in self.junctions:
            d = math.hypot(x - j['x'], y - j['y'])
            if d < JUNCTION_MERGE_M and (best is None or d < best[0]):
                best = (d, j)
        return best[1] if best else None

    def note_decision(self, pose, open_headings):
        """At a decision point: >= 2 open directions at least JUNCTION_MIN_SPREAD_DEG apart make (or extend) a
        junction; new directions become UNVISITED branches. Returns the junction or None."""
        heads = sorted(set(round(math.degrees(_wrap(h)) / 15) * 15 for h in open_headings))
        if len(heads) < 2:
            return self.junction_at(pose[0], pose[1])
        spread = max(abs(_wrap(math.radians(a - b))) for a in heads for b in heads)
        j = self.junction_at(pose[0], pose[1])
        if j is None:
            if math.degrees(spread) < JUNCTION_MIN_SPREAD_DEG:
                return None
            j = {'id': self.next_junction, 'x': pose[0], 'y': pose[1], 'branches': []}
            self.next_junction += 1
            self.junctions.append(j)
            if len(self.junctions) > MAX_JUNCTIONS:            # oldest first; never the active one
                drop = next((k for k in self.junctions
                             if not self.active_branch or k['id'] != self.active_branch[0]), None)
                if drop is not None:
                    self.junctions.remove(drop)
        for a in heads:
            h = math.radians(a)
            if not any(abs(_wrap(h - b['heading'])) <= math.radians(BRANCH_MATCH_DEG) for b in j['branches']):
                j['branches'].append({'id': len(j['branches']) + 1, 'heading': h, 'state': 'UNVISITED',
                                      'reason': 'open direction'})
        self._changed()
        return j

    def _branch(self, x, y, heading):
        j = self.junction_at(x, y)
        if j is None or math.hypot(x - j['x'], y - j['y']) > BRANCH_NEAR_M:
            return None, None
        best = None
        for b in j['branches']:
            d = abs(_wrap(heading - b['heading']))
            if d <= math.radians(BRANCH_MATCH_DEG) and (best is None or d < best[0]):
                best = (d, b)
        return j, (best[1] if best else None)

    def branch_factor(self, pose, heading):
        """Goal-selection weight for leaving pose along heading: 0 for an exhausted branch (INSPECTED/BLOCKED),
        DEFERRED_FACTOR for DEFERRED, 1 otherwise (UNVISITED, ACTIVE, or not at a junction)."""
        _, b = self._branch(pose[0], pose[1], heading)
        if b is None:
            return 1.0
        if b['state'] in EXHAUSTED:
            return 0.0
        return DEFERRED_FACTOR if b['state'] == 'DEFERRED' else 1.0

    def start_branch(self, pose, heading):
        j, b = self._branch(pose[0], pose[1], heading)
        self.end_branch('superseded', None) if self.active_branch else None
        if b is None:
            return None
        b.update(state='ACTIVE', reason='walking it')
        self.active_branch = (j['id'], b['id'])
        self.branch_views = 0
        self._changed()
        return b

    def _active(self):
        if not self.active_branch:
            return None
        j = next((k for k in self.junctions if k['id'] == self.active_branch[0]), None)
        return None if j is None else next((b for b in j['branches'] if b['id'] == self.active_branch[1]), None)

    def end_branch(self, outcome, category):
        """outcome 'walked': INSPECTED with a usable view while ACTIVE, else DEFERRED; 'failed': BLOCKED when the
        failure is persistent, back to UNVISITED when temporary; anything else: UNVISITED."""
        b = self._active()
        self.active_branch = None
        if b is None:
            return None
        if outcome == 'walked':
            b.update(state='INSPECTED', reason=f'{self.branch_views} usable view(s)') if self.branch_views \
                else b.update(state='DEFERRED', reason='walked, no camera confirmation')
        elif outcome == 'failed' and category == 'persistent':
            b.update(state='BLOCKED', reason='persistent obstacle')
        else:
            b.update(state='UNVISITED', reason=f'{outcome}: retry')
        self._changed()
        return b['state']

    # ---------------------------------------------------------------- failed destinations
    def _fkey(self, x, y):
        return round(x / FAIL_KEY_M), round(y / FAIL_KEY_M)

    def fail(self, now, x, y, reason, category=None):
        category = category or classify_failure(reason)
        if category == 'persistent' and 'wall' not in str(reason).lower() and self.walked_near(x, y):
            category = 'terrain'                         # been there: reachable, only hard going
        k = self._fkey(x, y)
        f = self.failures.get(k) or {'x': x, 'y': y, 'attempts': 0}
        f.update(reason=str(reason), category=category, last=now, attempts=f['attempts'] + 1)
        if category == 'terrain':
            f['retry'] = f'retry after {terrain_wait(f["attempts"]):.0f} s'
        elif category == 'temporary':
            f['retry'] = (f'retry after {TEMP_RETRY_AFTER_S:.0f} s' if f['attempts'] < TEMP_MAX_ATTEMPTS
                          else f'given up after {f["attempts"]} attempts')
        else:
            f['retry'] = (f'avoid {PERSIST_AVOID_S:.0f} s' if f['attempts'] < PERSIST_MAX_ATTEMPTS
                          else f'avoided for this session ({f["attempts"]} attempts)')
        self.failures[k] = f
        if len(self.failures) > MAX_FAILURES:
            oldest = min(self.failures, key=lambda key: self.failures[key]['last'])
            del self.failures[oldest]
        self._changed()
        return f

    def walked_near(self, x, y, radius=WALKED_NEAR_M):
        return any(math.hypot(x - bx, y - by) <= radius for bx, by in self.breadcrumbs)

    def succeeded(self, x, y):
        if self.failures.pop(self._fkey(x, y), None) is not None:
            self._changed()

    def avoided(self, x, y, now):
        """True while a destination near (x, y) may not be retried (see fail()'s retry condition)."""
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                k = self._fkey(x, y)
                f = self.failures.get((k[0] + dx, k[1] + dy))
                if f is None or math.hypot(x - f['x'], y - f['y']) > FAIL_KEY_M:
                    continue
                wall = f['category'] == 'persistent' and 'wall' in f['reason'].lower()
                if f['category'] == 'terrain' or (not wall and self.walked_near(f['x'], f['y'])):
                    if now - f['last'] < terrain_wait(f['attempts']):
                        return True                    # walked there before: only a growing wait, never for good
                elif f['category'] == 'temporary':
                    if f['attempts'] >= TEMP_MAX_ATTEMPTS or now - f['last'] < TEMP_RETRY_AFTER_S:
                        return True
                elif f['attempts'] >= PERSIST_MAX_ATTEMPTS or now - f['last'] < PERSIST_AVOID_S:
                    return True
        return False

    # ---------------------------------------------------------------- inspections
    def add_inspection(self, now, request):
        """Validate and store a request. Returns ('added' | 'duplicate' | 'updated' | 'rejected: why', id)."""
        if not isinstance(request, dict):
            return 'rejected: not an object', None
        rid = request.get('request_id')
        missions = request.get('mission_ids') or []
        pose = request.get('viewing_pose') or {}
        if not isinstance(rid, str) or not rid:
            return 'rejected: request_id missing', None
        if not isinstance(missions, list) or not all(isinstance(m, str) and m for m in missions):
            return 'rejected: mission_ids must be a list of strings', rid
        try:
            x, y, yaw = float(pose['x']), float(pose['y']), float(pose.get('yaw', 0.0))
            priority = float(request.get('priority', 0.0))
        except (KeyError, TypeError, ValueError):
            return 'rejected: viewing_pose needs numeric x, y (yaw optional)', rid
        if not all(math.isfinite(v) for v in (x, y, yaw, priority)):
            return 'rejected: non-finite viewing pose or priority', rid
        ms = request.get('map_session')
        if ms is not None and str(ms) != self.session:
            return f'rejected: for map session {ms}, not {self.session}', rid
        if rid in self.inspections:
            old = self.inspections[rid]
            if old['state'] in EXHAUSTED or (old['x'], old['y'], old['yaw'], old['priority']) == (x, y, yaw, priority):
                return 'duplicate', rid
            if old['state'] == 'ACTIVE':
                old['priority'] = priority             # do not move the robot's target mid-inspection
            else:
                old.update(x=x, y=y, yaw=yaw, priority=priority, state='UNVISITED', valid=True,
                           reason='updated request', attempts=0)
            self._changed()
            return 'updated', rid
        for other in self.inspections.values():
            if other['state'] not in EXHAUSTED and sorted(other['mission_ids']) == sorted(missions) and \
                    math.hypot(other['x'] - x, other['y'] - y) < INSPECTION_DUP_M:
                return 'duplicate', other['id']
        if len(self.inspections) >= MAX_INSPECTIONS:
            done = [k for k, v in self.inspections.items() if v['state'] in EXHAUSTED]
            if not done:
                return 'rejected: too many pending inspections', rid
            del self.inspections[min(done, key=lambda k: self.inspections[k]['t'])]
        self.inspections[rid] = {'id': rid, 'number': self._next_number(), 'mission_ids': list(missions),
                                 'evidence_ref': str(request.get('evidence_ref', '')), 'x': x, 'y': y, 'yaw': yaw,
                                 'priority': priority, 'state': 'UNVISITED', 'reason': 'pending', 'attempts': 0,
                                 'valid': True, 't': now, 'arrived': None}
        self._changed()
        return 'added', rid

    def _next_number(self):
        return 1 + max([v['number'] for v in self.inspections.values()] or [0])

    def next_inspection(self, pose, now, reachable):
        """Highest-priority UNVISITED inspection whose viewpoint reachable(x, y) accepts and is not avoided; ties go
        to the nearest. DEFERRED ones only when no UNVISITED one is reachable."""
        for states in (('UNVISITED',), ('DEFERRED',)):
            cands = [v for v in self.inspections.values() if v['state'] in states and v['valid']
                     and not self.avoided(v['x'], v['y'], now) and reachable(v['x'], v['y'])]
            if states == ('DEFERRED',):                     # one more look, later (exploration resumes first)
                cands = [v for v in cands if v.get('deferrals', 0) < 2
                         and now - v.get('finished', -math.inf) >= INSPECTION_DEFER_RETRY_S]
            if cands:
                return max(cands, key=lambda v: (v['priority'], -math.hypot(v['x'] - pose[0], v['y'] - pose[1])))
        return None

    def activate_inspection(self, now, rid):
        insp = self.inspections[rid]
        insp.update(state='ACTIVE', reason='going to viewpoint', attempts=insp['attempts'] + 1, arrived=None,
                    started=now)
        self.active_inspection = rid
        self._changed()
        return insp

    def arrived_at_viewpoint(self, now):
        insp = self.inspections.get(self.active_inspection)
        if insp is not None and insp['arrived'] is None:
            insp.update(arrived=now, reason='looking: waiting for a usable-camera confirmation')
            self._changed()
        return insp

    def finish_inspection(self, now, state, reason):
        """End the ACTIVE inspection: INSPECTED (confirmed), DEFERRED (timed out / camera said unusable) or
        UNVISITED / BLOCKED after a navigation failure (BLOCKED once INSPECTION_MAX_ATTEMPTS are used)."""
        insp = self.inspections.get(self.active_inspection)
        self.active_inspection = None
        if insp is None:
            return None
        if state == 'UNVISITED' and insp['attempts'] >= INSPECTION_MAX_ATTEMPTS:
            state, reason = 'BLOCKED', f'{reason}; {insp["attempts"]} attempts'
        if state == 'DEFERRED':
            insp['deferrals'] = insp.get('deferrals', 0) + 1
        insp.update(state=state, reason=reason, finished=now)
        self._changed()
        return insp

    # ---------------------------------------------------------------- camera confirmations
    def confirm(self, now, msg):
        """A usable-camera message. Returns what it affected: 'inspection', 'branch' or None (ignored)."""
        if not isinstance(msg, dict) or not isinstance(msg.get('usable'), bool):
            return None
        rid = msg.get('request_id')
        insp = self.inspections.get(self.active_inspection) if self.active_inspection else None
        if rid is not None and (insp is None or rid != insp['id']):
            return None                                    # not the inspection under way: ignore, never guess
        if insp is not None and insp['arrived'] is not None:
            if msg['usable']:
                self.finish_inspection(now, 'INSPECTED', 'usable-camera confirmation')
            else:
                self.finish_inspection(now, 'DEFERRED', f'camera: not usable ({msg.get("reason", "no reason")})')
            return 'inspection'
        if rid is None and msg['usable'] and self.active_branch:
            self.branch_views += 1
            return 'branch'
        return None

    # ---------------------------------------------------------------- summary
    def summary(self):
        count = {s: 0 for s in STATES}
        for j in self.junctions:
            for b in j['branches']:
                count[b['state']] += 1
        insp = {s: sum(1 for v in self.inspections.values() if v['state'] == s) for s in STATES}
        return {'session': self.session, 'breadcrumbs': len(self.breadcrumbs), 'junctions': len(self.junctions),
                'branches': count, 'failures': len(self.failures),
                'failures_persistent': sum(1 for f in self.failures.values() if f['category'] == 'persistent'),
                'inspections': insp, 'map_resets': [r for _, r in self.resets], 'destination': self.destination}


# ===================================================================================== RViz overlay
# Shape-first (operator 2026-11-23: "instead of words, a bounding box or something"). Text labels are optional
# (build_markers(..., labels=True); roam: LITE3_OVERLAY_LABELS=1) and always in their own *_labels namespaces.
COLORS = {'UNVISITED': (0.62, 0.62, 0.62), 'ACTIVE': (0.10, 0.45, 0.95), 'INSPECTED': (0.15, 0.70, 0.25),
          'BLOCKED': (0.85, 0.15, 0.15), 'DEFERRED': (0.95, 0.65, 0.10)}
DEST_RGB = (0.95, 0.25, 0.85)
SHORT = {'UNVISITED': 'U', 'ACTIVE': 'A', 'INSPECTED': 'I', 'BLOCKED': 'X', 'DEFERRED': 'D'}
MAX_FAILURE_MARKERS = 40
MARKER_LABEL_SIZE = 0.14
# Compact tags (operator 2026-11-23: "annotate ... make sure it doesn't take up too much space"): one or two short
# words, 7 cm tall, just above the shape, all in the 'tags' namespace (untick it in RViz to hide them).
TAG_SIZE = 0.07
TAG_RGB = (1.0, 1.0, 1.0)
TAG_BRANCH_JUNCTIONS = 2          # branch tags only at the newest junctions; older ones keep just "J<n>"
DEST_TAG = {'spin': 'TURN', 'look': 'LOOK'}
BRANCH_TAG = {'UNVISITED': 'open', 'ACTIVE': 'going', 'INSPECTED': 'seen', 'DEFERRED': 'walked', 'BLOCKED': 'blocked'}
INSPECTION_TAG = {'UNVISITED': 'to check', 'ACTIVE': 'checking', 'INSPECTED': 'seen', 'DEFERRED': 'later',
                  'BLOCKED': 'blocked'}
LINE_W = 0.03


def _m(ns, mid, kind, x=0.0, y=0.0, z=0.0, yaw=0.0, scale=(0.1, 0.1, 0.1), rgb=(1, 1, 1), alpha=1.0, text='',
       points=()):
    return {'ns': ns, 'id': mid, 'type': kind, 'x': x, 'y': y, 'z': z, 'yaw': yaw, 'scale': scale,
            'color': (*rgb, alpha), 'text': text, 'points': list(points)}


def _box_edges(cx, cy, z0, sx, sy, sz, yaw=0.0):
    """12 edges of a box (line_list point pairs), centred on (cx, cy), from height z0 up sz, rotated by yaw."""
    c, s = math.cos(yaw), math.sin(yaw)
    corners = []
    for dx, dy in ((-1, -1), (1, -1), (1, 1), (-1, 1)):
        x, y = dx * sx / 2, dy * sy / 2
        corners.append((cx + c * x - s * y, cy + s * x + c * y))
    pts = []
    for z in (z0, z0 + sz):                                   # bottom and top rectangles
        for k in range(4):
            a, b = corners[k], corners[(k + 1) % 4]
            pts += [(a[0], a[1], z), (b[0], b[1], z)]
    for a in corners:                                         # verticals
        pts += [(a[0], a[1], z0), (a[0], a[1], z0 + sz)]
    return pts


def _cross(cx, cy, z, half):
    return [(cx - half, cy - half, z), (cx + half, cy + half, z), (cx - half, cy + half, z), (cx + half, cy - half, z)]


def _tag(n, x, y, z, text):
    return _m('tags', n, 'text', x, y, z, scale=(0, 0, TAG_SIZE), rgb=TAG_RGB, alpha=0.9, text=text)


def build_tags(rec):
    """Short words for every shape build_markers draws (labels='tags', the roam default)."""
    out = []
    d = rec.destination
    if d and d.get('x') is not None:
        word = (d.get('reason') or '').split(':')[0].strip()[:12]
        head = DEST_TAG.get(d.get('kind'), 'GOAL')
        out.append(_tag(len(out), d['x'], d['y'], 0.5, f'{head} {word}'.strip()))
    newest = {j['id'] for j in rec.junctions[-TAG_BRANCH_JUNCTIONS:]}
    for j in rec.junctions:
        out.append(_tag(len(out), j['x'], j['y'], 0.2, f'J{j["id"]}'))
        for b in j['branches']:
            if j['id'] in newest or b['state'] in ('ACTIVE', 'BLOCKED'):
                ex, ey = j['x'] + 0.72 * math.cos(b['heading']), j['y'] + 0.72 * math.sin(b['heading'])
                out.append(_tag(len(out), ex, ey, 0.2, BRANCH_TAG[b['state']]))
    for f in sorted(rec.failures.values(), key=lambda f: -f['last'])[:MAX_FAILURE_MARKERS]:
        word = 'AVOID' if f['category'] == 'persistent' else 'RETRY'
        out.append(_tag(len(out), f['x'], f['y'], 0.32 + 0.09 * min(f['attempts'], 3) + 0.05,
                        f'{word} x{f["attempts"]}'))
    for v in sorted(rec.inspections.values(), key=lambda v: v['number']):
        if v['valid']:
            what = ','.join(v['mission_ids']) or f'#{v["number"]}'
            out.append(_tag(len(out), v['x'], v['y'], 0.68, f'{what} {INSPECTION_TAG.get(v["state"], v["state"])}'))
    return out


def build_markers(rec, labels='tags'):
    """Pure marker specs for the RViz overlay (the ROS layer converts them and sends DELETEALL first, so anything not
    in this list disappears). Shape carries the meaning, colour repeats it. labels: 'tags' (default) adds one or two
    short words per shape; True adds the long explanations instead; False draws shapes only."""
    if labels == 'tags':
        return build_markers(rec, labels=False) + build_tags(rec)
    out = []
    d = rec.destination
    if d and d.get('x') is not None:
        kind = d.get('kind')
        if kind == 'spin':                                    # turning on the spot: flat disc round the robot
            out.append(_m('destination', 0, 'cylinder', d['x'], d['y'], 0.01, scale=(0.9, 0.9, 0.02), rgb=DEST_RGB,
                          alpha=0.35))
        elif kind == 'look':                                  # inspecting: translucent solid box
            out.append(_m('destination', 0, 'cube', d['x'], d['y'], 0.25, d.get('yaw') or 0.0, (0.45, 0.45, 0.5),
                          DEST_RGB, alpha=0.35))
        else:                                                 # walking to it: wireframe box
            out.append(_m('destination', 0, 'line_list', scale=(LINE_W, 0, 0), rgb=DEST_RGB,
                          points=_box_edges(d['x'], d['y'], 0.0, 0.45, 0.45, 0.4, d.get('yaw') or 0.0)))
        out.append(_m('destination', 1, 'arrow', d['x'], d['y'], 0.05, d.get('yaw') or 0.0, (0.45, 0.07, 0.07), DEST_RGB))
        if labels:
            out.append(_m('destination_labels', 0, 'text', d['x'], d['y'], 0.65, text=f'{d["action"]}: {d["reason"]}',
                          scale=(0, 0, MARKER_LABEL_SIZE)))
    elif d and labels:
        out.append(_m('destination_labels', 0, 'text', 0.0, 0.0, 0.8, text=f'{d["action"]}: {d["reason"]}',
                      scale=(0, 0, MARKER_LABEL_SIZE)))
    if len(rec.breadcrumbs) >= 2:
        out.append(_m('breadcrumbs', 0, 'line_strip', scale=(0.04, 0, 0), rgb=(0.2, 0.8, 0.9), alpha=0.8,
                      points=[(x, y, 0.02) for x, y in rec.breadcrumbs]))
    for j in rec.junctions:
        out.append(_m('junctions', j['id'], 'cube', j['x'], j['y'], 0.06, scale=(0.16, 0.16, 0.12), rgb=(1, 1, 1)))
        if labels:
            out.append(_m('junction_labels', j['id'], 'text', j['x'], j['y'], 0.35, text=f'J{j["id"]}',
                          scale=(0, 0, MARKER_LABEL_SIZE)))
        for b in j['branches']:
            mid, st = j['id'] * 100 + b['id'], b['state']
            shaft = {'ACTIVE': 0.08, 'UNVISITED': 0.04}.get(st, 0.05)
            out.append(_m('branches', mid, 'arrow', j['x'], j['y'], 0.04, b['heading'], (0.6, shaft, shaft), COLORS[st]))
            ex, ey = j['x'] + 0.72 * math.cos(b['heading']), j['y'] + 0.72 * math.sin(b['heading'])
            if st == 'INSPECTED':                             # done: solid cube at the end
                out.append(_m('branch_ends', mid, 'cube', ex, ey, 0.08, b['heading'], (0.16, 0.16, 0.16), COLORS[st]))
            elif st == 'DEFERRED':                            # walked, unconfirmed: hollow box
                out.append(_m('branch_ends', mid, 'line_list', scale=(LINE_W, 0, 0), rgb=COLORS[st],
                              points=_box_edges(ex, ey, 0.0, 0.2, 0.2, 0.2, b['heading'])))
            elif st == 'BLOCKED':                             # blocked: X
                out.append(_m('branch_ends', mid, 'line_list', scale=(0.05, 0, 0), rgb=COLORS[st],
                              points=_cross(ex, ey, 0.06, 0.12)))
            if labels:
                out.append(_m('branch_labels', mid, 'text', ex, ey, 0.3, text=f'{SHORT[st]} {st}', scale=(0, 0, 0.1)))
    fails = sorted(rec.failures.values(), key=lambda f: -f['last'])[:MAX_FAILURE_MARKERS]
    for n, f in enumerate(fails):
        rgb = (0.9, 0.1, 0.1) if f['category'] == 'persistent' else (0.95, 0.65, 0.1)
        out.append(_m('failures', n, 'line_list', scale=(LINE_W, 0, 0), rgb=rgb,
                      points=_box_edges(f['x'], f['y'], 0.0, 0.32, 0.32, 0.25) + _cross(f['x'], f['y'], 0.25, 0.16)))
        for k in range(min(f['attempts'], 3)):                # attempts: stacked cubes on top (1-3)
            out.append(_m('failure_attempts', n * 10 + k, 'cube', f['x'], f['y'], 0.32 + 0.09 * k,
                          scale=(0.07, 0.07, 0.07), rgb=rgb))
        if labels:
            out.append(_m('failure_labels', n, 'text', f['x'], f['y'], 0.6, scale=(0, 0, 0.1),
                          text=f'x{f["attempts"]} {f["category"]}: {f["reason"][:40]}'))
    for v in sorted(rec.inspections.values(), key=lambda v: v['number']):
        if not v['valid']:
            continue
        rgb = COLORS[v['state']]
        out.append(_m('inspections', v['number'], 'line_list', scale=(LINE_W, 0, 0), rgb=rgb,
                      points=_box_edges(v['x'], v['y'], 0.0, 0.5, 0.5, 0.6, v['yaw'])))
        if v['state'] == 'INSPECTED':                         # confirmed: filled
            out.append(_m('inspection_fill', v['number'], 'cube', v['x'], v['y'], 0.3, v['yaw'], (0.5, 0.5, 0.6),
                          rgb, alpha=0.3))
        out.append(_m('inspection_heading', v['number'], 'arrow', v['x'], v['y'], 0.3, v['yaw'], (0.45, 0.06, 0.06),
                      rgb))
        if labels:
            out.append(_m('inspection_labels', v['number'], 'text', v['x'], v['y'], 0.8,
                          scale=(0, 0, MARKER_LABEL_SIZE),
                          text=f'#{v["number"]} {",".join(v["mission_ids"]) or "-"} {v["state"]}'))
    return out
