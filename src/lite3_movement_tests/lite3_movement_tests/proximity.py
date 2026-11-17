"""M4 sensor-driven proximity stop: obstacle tracking, states and outcomes (pure Python, no ROS).

The tracker consumes one terrain-monitor scan at a time: the nose distance of the nearest
obstacle in the forward corridor (``None`` = nothing) and the robot's forward travel from leg
odometry. It never commands anything; the M4 step uses its events to decide when to abort.

    none --(detect_scans consistent scans inside the detection range)--> confirmed
    confirmed --(lost_scans scans without it while it must still be visible)--> 'lost'
    confirmed --(2 consecutive scans inconsistent with the robot's own travel)--> 'unexpected'

Consistency: a static obstacle gets closer by exactly the distance the robot walks. Leg
odometry under-reads forward travel by up to ~15 % (config.ODOMETRY_TRUST), so the obstacle may
be up to (1/trust - 1) x travel closer than predicted, plus ``tolerance_m`` either way (terrain
grid 5 cm, point noise). A jump between consecutive scans beyond ``tolerance_m`` (something
stepping into the corridor, the object being moved) is unexpected as well.
"""

READY = 'READY'
WALKING = 'WALKING'
DETECTED = 'OBSTACLE_DETECTED'
STOPPING = 'STOPPING'
VERIFIED = 'STOP_VERIFIED'
STATES = (READY, WALKING, DETECTED, STOPPING, VERIFIED)

# Outcome codes: only STOPPED_FOR_OBSTACLE is a PASS.
OUTCOMES = {
    'STOPPED_FOR_OBSTACLE': 'PASS: the collision_monitor stopped the robot for the tracked obstacle, '
                            'unsafe commands were suppressed in time, the robot stopped and the '
                            'clearance held',
    'STOP_CRITERIA_FAILED': 'the collision_monitor stopped the robot for the obstacle, but at least one '
                            'PASS criterion (response, stop time, clearance, latency) was missed',
    'TIME_LIMIT': 'the movement timer ended the walk; no protective stop for an obstacle happened '
                  '(not a pass)',
    'TRAVEL_LIMIT': 'the travel limit ended the walk; no protective stop for an obstacle happened '
                    '(not a pass)',
    'BACKUP_LOCK_STOP': 'the terrain backup lock stopped the robot before the collision_monitor did; '
                        'the validated primary stop did not act first (not a pass)',
    'UNEXPLAINED_STOP': 'commands stopped reaching the robot without a proximity stop (dead-man '
                        'timeout or command path); not a pass',
    'OPERATOR_STOP': 'the operator stopped the test (Ctrl+C, abort script or handheld remote); '
                     'not a pass',
    'SAFETY_ABORT': 'an independent safeguard stopped the test (stale sensors, lost tracking, '
                    'unexpected motion, speed, geofence, ...); not a pass',
}
NOT_PASS_OUTCOMES = set(OUTCOMES) - {'STOPPED_FOR_OBSTACLE'}


def classify_abort(reason):
    """Outcome code for a safety abort reason."""
    text = (reason or '').lower()
    if 'operator cancelled' in text or 'handheld remote' in text:
        return 'OPERATOR_STOP'
    return 'SAFETY_ABORT'


class ObstacleTracker:
    """Nearest forward obstacle, confirmed over several scans and tracked against odometry."""

    def __init__(self, detection_range_m, confirm_scans, tolerance_m, lost_scans,
                 odometry_trust=0.8, check_consistency=True):
        self.range = detection_range_m
        self.confirm_scans = confirm_scans
        self.tol = tolerance_m
        self.lost_scans = lost_scans
        self.slack = max(0.0, 1.0 / odometry_trust - 1.0)
        self.check_consistency = check_consistency
        self.candidate = []          # [(t, d, s)] consecutive consistent in-range scans
        self.confirmed = None        # {'t', 'd', 's'} at confirmation
        self.last = None             # (t, d, s) last in-range measurement after confirmation
        self.misses = 0
        self.anomalies = 0
        self.scans = 0
        self.max_residual = 0.0
        self.note = None             # explanation of the last 'lost' / 'unexpected' event

    @property
    def is_confirmed(self):
        return self.confirmed is not None

    def predicted(self, s):
        """Nose distance a static obstacle should have after forward travel ``s``."""
        if self.confirmed is None:
            return None
        return self.confirmed['d'] - (s - self.confirmed['s'])

    def update(self, t, d, s):
        """One scan. Returns 'confirmed', 'lost', 'unexpected' or None."""
        self.scans += 1
        seen = d is not None and d <= self.range
        if self.confirmed is None:
            if not seen:
                self.candidate = []
                return None
            if self.candidate:
                _, d0, s0 = self.candidate[-1]
                if abs((d - d0) + (s - s0)) > self.tol:
                    self.candidate = []
            self.candidate.append((t, d, s))
            if len(self.candidate) >= self.confirm_scans:
                self.confirmed = {'t': t, 'd': d, 's': s}
                self.last = (t, d, s)
                return 'confirmed'
            return None
        expected = self.predicted(s)
        if not seen:
            # Only a miss if the obstacle should clearly still be inside the detection region.
            if expected < self.range - 2 * self.tol:
                self.misses += 1
                if self.misses >= self.lost_scans:
                    self.note = (f'obstacle expected {expected:.2f} m ahead but not seen in '
                                 f'{self.misses} consecutive scans')
                    return 'lost'
            return None
        self.misses = 0
        if not self.check_consistency:
            self.last = (t, d, s)
            return None
        _, d_prev, s_prev = self.last
        jump = (d - d_prev) + (s - s_prev)           # 0 for a static obstacle
        travelled = max(0.0, s - self.confirmed['s'])
        low = expected - self.tol - self.slack * travelled
        high = expected + self.tol
        residual = d - expected
        self.max_residual = max(self.max_residual, abs(residual))
        inconsistent = abs(jump) > self.tol or not low <= d <= high
        self.last = (t, d, s)
        if inconsistent:
            self.anomalies += 1
            if self.anomalies >= 2:
                self.note = (f'obstacle measured {d:.2f} m, expected {expected:.2f} m from the robot\'s '
                             f'own travel (allowed {low:.2f}-{high:.2f} m; scan-to-scan jump '
                             f'{jump:+.2f} m): moving object or a new object in the path')
                return 'unexpected'
        else:
            self.anomalies = 0
        return None

    def as_dict(self):
        return {'confirmed': self.confirmed, 'scans': self.scans,
                'last': None if self.last is None else [round(v, 3) for v in self.last],
                'max_residual_m': round(self.max_residual, 3), 'note': self.note}
