"""Sequential movement-test runner with telemetry-verified success criteria.

The runner never treats a published command as success. Every step passes only
when independent telemetry satisfies its measurable criteria within its time
limit. Any failure stops the sequence; if commands were sent, a zero-velocity
stop is issued and rest is verified. Posture commands are latched off after any
posture failure, and an unverified state ends the session with an intervention
request instead of further commands.
"""

from dataclasses import dataclass, field
import datetime as dt
import math

from .config import HARD_MAX_SESSION_S, HARD_MAX_SPEED_M_S, HARD_MAX_YAW_RATE_RAD_S
from .geometry import LYING, STANDING, assess_posture, validate_joint_sample
from .telemetry import assess_stationary, displacement, joint_activity

PASSED = 'PASSED'
FAILED = 'FAILED'
SKIPPED = 'SKIPPED'
ABORTED = 'ABORTED'
NOT_RUN = 'NOT_RUN'
PREVIEW = 'PREVIEW_ONLY'

MODE_SIM = 'simulation'
MODE_READONLY = 'live-readonly'
MODE_PHYSICAL = 'physical'


class SafetyAbort(Exception):
    def __init__(self, reason, detail=None):
        super().__init__(reason)
        self.detail = detail      # the step's own measurements, kept in the report


@dataclass
class StepResult:
    id: str
    type: str
    status: str
    summary: str
    actuating: bool = False
    started_s: float = None
    duration_s: float = None
    measurements: dict = field(default_factory=dict)
    trace: list = field(default_factory=list)

    def as_dict(self):
        return {k: v for k, v in self.__dict__.items() if not (k == 'trace' and not v)}


def _r(value, digits=4):
    return None if value is None or not math.isfinite(value) else round(value, digits)


from .nav_steps import NavSteps  # noqa: E402


class SequenceRunner(NavSteps):
    def __init__(self, sequence, robot, mode, log=print, authorization=None):
        self.seq = sequence
        self.robot = robot
        self.mode = mode
        self.log = log
        self.auth = authorization
        self.commands_sent = False
        self.posture_latched_off = False
        self.uncertain = False
        self.events = []
        self.range_failures = []
        self.origin = None
        self.start = None
        self._last_health = (-math.inf, True, {})
        self.nav_active_until = -math.inf      # Nav2 may command the mux only inside nav_goal
        self.nav_anchor = None
        self.geofence = None                   # (map x, y, radius) during nav_goal
        self.last_deadman_feed = None
        if mode == MODE_PHYSICAL and authorization is None:
            raise ValueError('physical mode requires a checked authorization')
        if mode == MODE_READONLY and robot.can_actuate:
            raise ValueError('read-only mode must use an interface without publishers')

    # ------------------------------------------------------------------ helpers
    def _event(self, kind, **detail):
        self.events.append({'t_s': _r(self.robot.now() - self.start, 3), 'event': kind, **detail})
        self.log(f'[{kind}] {detail}')

    def _posture(self, window_s=1.0):
        now = self.robot.now()
        window = [v.positions for _, v in self.robot.telemetry.joints.window(now, window_s)]
        return assess_posture(window, self.robot.telemetry.current_tilt(), self.seq.posture)

    def _stationary(self, window_s):
        return assess_stationary(self.robot.telemetry, self.robot.now(), window_s,
                                 self.seq.stationary)

    def _health(self, force=False):
        now = self.robot.now()
        if force or now - self._last_health[0] >= 0.5:
            ok, detail = self.robot.command_path_health()
            self._last_health = (now, ok, detail)
        return self._last_health[1], self._last_health[2]

    def _safety_violation(self):
        """Conditions that stop any motion immediately."""
        t, req, lim = self.robot.telemetry, self.seq.telemetry, self.seq.safety
        now = self.robot.now()
        if self.robot.cancelled():
            return 'operator cancelled (signal)'
        if now - self.start > getattr(self, 'session_limit_s', HARD_MAX_SESSION_S):
            return 'session time limit exceeded'
        for name in ('joints', 'odom', 'imu'):
            if getattr(t, name).age(now) > req.max_age_s:
                return f'{name} telemetry stale'
        tilt = t.current_tilt()
        if tilt is None or max(abs(tilt[0]), abs(tilt[1])) > lim.max_tilt_rad:
            return 'body tilt beyond limit or IMU orientation invalid'
        handle = t.handle.latest()
        # While roam drives the robot with AI-state axis commands (stairs), the robot reports those axes on
        # /handle_state too: 2026-11-21 its own first axis command read as "remote input". STOP on the remote
        # acts on the robot directly and is unaffected.
        own_axes = now < getattr(self, 'own_axis_until', -math.inf)
        if handle and not own_axes and now - handle[0] < 1.0 and max(
                abs(handle[1].vx), abs(handle[1].vy), abs(handle[1].wz)) > req.max_handle_axis:
            return 'handheld remote input detected (operator takeover)'
        odom = t.odom.latest()
        if self.origin is not None and odom is not None and math.hypot(
                odom[1].x - self.origin.x, odom[1].y - self.origin.y) > lim.geofence_radius_m:
            return 'geofence radius exceeded'
        ok, detail = self._health()
        if not ok:
            return f'command path unhealthy: {detail.get("problems")}'
        # Independent check of what actually reaches the motion_sender (domain 64).
        v_cap = self.auth.max_speed_m_s if self.auth else HARD_MAX_SPEED_M_S
        w_cap = self.auth.max_yaw_rate_rad_s if self.auth else HARD_MAX_YAW_RATE_RAD_S
        for _, c in t.system_cmd.window(now, 0.5):
            if abs(c.vx) > v_cap + 0.005 or abs(c.vy) > 0.005 or abs(c.wz) > w_cap + 0.005:
                return (f'robot-side command beyond authorized limits (vx {c.vx:+.3f}, vy {c.vy:+.3f}, '
                        f'wz {c.wz:+.3f}; caps {v_cap}, {w_cap})')
        if now > self.nav_active_until and any(not c.is_zero for _, c in t.nav_cmd.window(now, 0.5)):
            return 'Nav2 is commanding motion outside a nav_goal step (another goal source?)'
        return self._nav_violation(now)

    def _check(self, feed=True):
        """Abort on any violation; otherwise (if ``feed``) hold the dead-man released."""
        reason = self._safety_violation()
        if reason:
            raise SafetyAbort(reason)
        if feed and hasattr(self.robot, 'feed_deadman'):
            self.robot.feed_deadman()
            self.last_deadman_feed = self.robot.now()

    def _stop_command(self):
        """One zero command on the stop path (mux safety_stop input above the locks if present)."""
        if hasattr(self.robot, 'send_stop'):
            self.robot.send_stop()
        else:
            self.robot.send_velocity(0.0, 0.0)

    def _wait_until(self, step, predicate, poll=0.05, actuating=False, keepalive=None, feed=True):
        """Poll ``predicate`` until it returns (True, detail) or the step times out."""
        deadline = self.robot.now() + step.timeout_s
        detail = {}
        while self.robot.now() < deadline:
            if actuating:
                self._check(feed=feed)
            elif self.robot.cancelled():
                raise SafetyAbort('operator cancelled (signal)')
            if keepalive:
                keepalive()
            ok, detail = predicate()
            if ok:
                return True, detail
            self.robot.wait(poll)
        return False, detail

    def _send(self, vx, wz):
        self.commands_sent = True
        self.robot.send_velocity(vx, wz)

    def _zero_burst(self, seconds):
        """Best-effort zeros; never raises so a stop always gets attempted in full."""
        end = self.robot.now() + seconds
        errors = 0
        while True:
            try:
                self._stop_command()
            except Exception:  # noqa: BLE001 - keep trying zeros during a stop
                errors += 1
            if self.robot.now() >= end:
                break
            self.robot.wait(self.seq.safety.command_period_s)
        return errors

    def _odom(self):
        item = self.robot.telemetry.odom.latest()
        return None if item is None else item[1]

    def _trace(self, since):
        """Downsampled (~20 Hz) telemetry record for the report."""
        t = self.robot.telemetry
        rows, last = [], -math.inf
        joints = t.joints.since(since)
        imus = t.imu.since(since)
        prev_joints = None
        for stamp, o in t.odom.since(since):
            if stamp - last < 0.05:
                continue
            last = stamp
            row = {'t': _r(stamp - self.start, 3), 'x': _r(o.x), 'y': _r(o.y), 'yaw': _r(o.yaw),
                   'vx': _r(o.vx), 'vy': _r(o.vy), 'wz': _r(o.wz)}
            imu = [v for ts, v in imus if abs(ts - stamp) < 0.01]
            if imu:
                row['gyro'] = _r(max(abs(g) for g in imu[-1].gyro), 3)
            near = [v for ts, v in joints if abs(ts - stamp) < 0.01]
            if near and prev_joints is not None:
                row['jd'] = _r(max(abs(a - b) for a, b in zip(near[-1].positions, prev_joints)), 4)
            if near:
                prev_joints = near[-1].positions
                assessment = assess_posture([near[-1].positions], (0.0, 0.0), self.seq.posture)
                row['ext'] = [_r(e, 3) for e in assessment.extensions_m.values()]
            rows.append(row)
        return rows

    # ------------------------------------------------------------------- steps
    def step_readiness(self, step):
        req = self.seq.telemetry

        def ready():
            now = self.robot.now()
            t = self.robot.telemetry
            problems = []
            detail = {'rates_hz': {}, 'ages_s': {}}
            for name in ('joints', 'odom', 'imu'):
                stream = getattr(t, name)
                rate, age = stream.rate(now), stream.age(now)
                detail['rates_hz'][name] = _r(rate, 1)
                detail['ages_s'][name] = _r(age, 3)
                if rate < req.min_rate_hz or age > req.max_age_s:
                    problems.append(f'{name} rate/freshness insufficient')
            alive = t.alive_snapshot()
            detail['sensor_alive'] = {k: v[1] for k, v in alive.items()}
            for name in req.required_sensor_alive:
                stamp_value = alive.get(name)
                if stamp_value is None or now - stamp_value[0] > 3.0 or not stamp_value[1]:
                    problems.append(f'sensor_status/{name}_isalive not fresh and true')
            handles = t.handle.window(now, step.params['quiet_s'])
            if any(max(abs(h.vx), abs(h.vy), abs(h.wz)) > req.max_handle_axis for _, h in handles):
                problems.append('handheld remote axes not neutral')
            foreign = [c for _, c in t.system_cmd.window(now, step.params['quiet_s']) if not c.is_zero]
            if foreign and not self.commands_sent:
                problems.append('nonzero velocity commands observed from another source')
            ok, health = self._health(force=True)
            detail['command_path'] = health
            if not ok:
                problems.append('command path unhealthy: ' + '; '.join(health.get('problems', [])))
            tilt = t.current_tilt()
            if tilt is None or max(map(abs, tilt)) > self.seq.safety.max_tilt_rad:
                problems.append('IMU tilt invalid or beyond limit')
            detail['problems'] = problems
            return not problems, detail

        ok, detail = self._wait_until(step, ready, poll=0.2)
        return (PASSED if ok else FAILED,
                'telemetry, sensor status, remote and command path ready' if ok else
                f'not ready: {detail.get("problems")}', detail)

    def step_telemetry_validation(self, step):
        req = self.seq.telemetry
        window_s = step.params['window_s']

        def valid():
            now = self.robot.now()
            t = self.robot.telemetry
            problems = []
            joints = t.joints.window(now, window_s)
            odom = t.odom.window(now, window_s)
            imu = t.imu.window(now, window_s)
            if min(len(joints), len(odom), len(imu)) < req.min_rate_hz * window_s * 0.8:
                problems.append('insufficient samples in validation window')
            joint_issues = sorted({p for _, j in joints for p in validate_joint_sample(j.names, j.positions)})
            problems += joint_issues
            if any(not all(math.isfinite(v) for v in (o.x, o.y, o.yaw, o.vx, o.vy, o.wz))
                   for _, o in odom):
                problems.append('non-finite odometry')
            if any(not i.valid for _, i in imu):
                problems.append('invalid IMU orientation quaternion')
            norms = [math.sqrt(sum(a * a for a in i.acc)) for _, i in imu]
            if norms and not req.min_accel_norm <= min(norms) <= max(norms) <= req.max_accel_norm:
                problems.append('IMU acceleration norm not consistent with gravity at rest')
            posture = self._posture()
            detail = {'samples': [len(joints), len(odom), len(imu)],
                      'accel_norm_range': [_r(min(norms), 3), _r(max(norms), 3)] if norms else None,
                      'posture': posture.as_dict(), 'problems': problems}
            return not problems, detail

        ok, detail = self._wait_until(step, valid, poll=0.25)
        return (PASSED if ok else FAILED,
                'joint/odometry/IMU contract and ranges valid' if ok else f'invalid: {detail.get("problems")}',
                detail)

    def step_stationary_check(self, step):
        ok, detail = self._wait_until(step, lambda: self._stationary(step.params['hold_s']))
        return (PASSED if ok else FAILED,
                'robot at rest by joints, odometry and IMU' if ok else
                f'rest not verified: {detail.get("reasons")}', detail)

    def step_posture_check(self, step):
        expect, hold = step.params['expect'], step.params['hold_s']

        def check():
            posture = self._posture(hold)
            still, rest = self._stationary(hold)
            return (posture.confident and posture.posture == expect and still,
                    {'posture': posture.as_dict(), 'stationary': rest})

        ok, detail = self._wait_until(step, check)
        return (PASSED if ok else FAILED,
                f'verified {expect} and at rest' if ok else
                f'{expect} not verified: {detail.get("posture", {}).get("posture")} '
                f'{detail.get("posture", {}).get("reasons")}', detail)

    def step_posture_transition(self, step):
        target = step.params['target']
        source = STANDING if target == LYING else LYING
        if self.posture_latched_off:
            return FAILED, 'posture commands latched off after an earlier posture failure', {}
        before = self._posture(1.0)
        still, rest = self._stationary(1.0)
        pre = {'before': before.as_dict(), 'stationary': rest}
        if before.confident and before.posture == target and still:
            return SKIPPED, f'already verified {target}; toggle not sent', pre
        if not (before.confident and before.posture == source and still):
            # Unverified posture is only "uncertain" if this session already commanded the robot.
            self.uncertain = self.uncertain or self.commands_sent
            return FAILED, (f'precondition: need verified {source} at rest before a toggle, got '
                            f'{before.posture} {before.reasons}'), pre
        self._check()
        if not self.robot.can_actuate:
            return PREVIEW, f'precondition verified ({source}, at rest); would send one toggle', pre
        sent_at = self.robot.now()
        self.origin = None
        self.commands_sent = True
        # Any failure after this point leaves posture uncertain until telemetry proves otherwise.
        self.posture_latched_off = True
        delivery = self.robot.send_posture_toggle()
        pre['delivery'] = delivery
        self._event('posture_toggle_sent', target=target, delivery=delivery)
        if not delivery.get('published'):
            self.uncertain = not delivery.get('definitely_not_sent', False)
            return FAILED, f'posture toggle not delivered: {delivery}', pre
        left_source = [False]

        def reached():
            p = self._posture(step.params['hold_s'])
            instant = self._posture(0.2)
            if instant.posture != source:
                left_source[0] = True
            still, rest = self._stationary(step.params['hold_s'])
            return (left_source[0] and p.confident and p.posture == target and still,
                    {'after': p.as_dict(), 'stationary': rest, 'left_source': left_source[0]})

        try:
            ok, detail = self._wait_until(step, reached, actuating=True, feed=False)
        except SafetyAbort:
            self.uncertain = True
            raise
        detail.update(pre)
        detail['transition_s'] = _r(self.robot.now() - sent_at, 2)
        result_trace = self._trace(sent_at - 0.5)
        if ok:
            self.posture_latched_off = False
            return PASSED, f'{target} verified by joint geometry, IMU and rest', detail, result_trace
        self.uncertain = True
        return FAILED, (f'{target} not verified within {step.timeout_s}s '
                        f'(left {source}: {left_source[0]}); posture commands latched off'), detail, result_trace

    def step_velocity_move(self, step):
        p = step.params
        lim = self.seq.safety
        posture = self._posture(1.0)
        still, rest = self._stationary(1.0)
        pre = {'posture': posture.as_dict(), 'stationary': rest}
        if not (posture.confident and posture.posture == STANDING and still):
            return FAILED, f'precondition: verified standing at rest required, got {posture.posture}', pre
        if self._nav_capable():
            sample = self._terrain()
            pre['proximity_stop'] = None if sample is None else sample.stop
            if sample is None or sample.stop:
                return FAILED, ('precondition: proximity stop active or terrain status stale; clear '
                                'the area (nothing within 0.45 m around or in the stop zone ahead)'), pre
        if not self.robot.can_actuate:
            return PREVIEW, f'would command vx={p["vx"]} wz={p["wz"]} for {p["duration_s"]}s', pre
        start_odom = self._odom()
        # Geofence is anchored per move: leg-odometry pose is frozen while lying and may
        # jump when the estimator resumes, so a session-start anchor would be misleading.
        self.origin = start_odom
        self._check()
        t0 = self.robot.now()
        motion = {'reason': 'duration_complete'}
        forward_start = forward_end = None
        try:
            # Zero warmup through the same publisher, then the bounded constant command.
            while self.robot.now() - t0 < p['warmup_s']:
                self._check()
                self._send(0.0, 0.0)
                self.robot.wait(lim.command_period_s)
            forward_start = last = self.robot.now()
            delivered = False
            while True:
                self._check()
                now = self.robot.now()
                if now - last > 3 * lim.command_period_s:
                    raise SafetyAbort('command scheduler gap')
                if now - forward_start >= p['duration_s']:
                    break
                if not delivered:
                    delivered = any(not c.is_zero for _, c in
                                    self.robot.telemetry.system_cmd.since(forward_start))
                    if not delivered and now - forward_start > p['delivery_deadline_s']:
                        raise SafetyAbort('nonzero command not observed at motion_sender domain')
                self._send(p['vx'], p['wz'])
                last = now
                self.robot.wait(min(lim.command_period_s,
                                    max(0.0, forward_start + p['duration_s'] - self.robot.now())))
            forward_end = self.robot.now()
            if p['end_mode'] == 'silence':
                # Stop publishing entirely and watch whether the robot comes to rest by itself
                # (what happens if the upstream planner/mux dies). Zeros follow in `finally`.
                # Criterion: rest must BEGIN within silence_timeout_s; confirming it needs a
                # further hold window, so observation continues up to timeout + hold.
                hold = self.seq.safety.stop_hold_s
                while True:
                    self._check()
                    still, rest = self._stationary(hold)
                    elapsed = self.robot.now() - forward_end
                    onset = max(0.0, elapsed - hold)
                    if still and onset <= p['silence_timeout_s']:
                        motion['silence'] = {'stopped': True,
                                             'rest_onset_after_s': _r(onset, 2),
                                             'verified_after_s': _r(elapsed, 2)}
                        break
                    if elapsed >= p['silence_timeout_s'] + hold:
                        motion['silence'] = {'stopped': False, 'waited_s': _r(elapsed, 2),
                                             'reasons': rest.get('reasons')}
                        motion['reason'] = 'no_stop_on_command_silence'
                        break
                    self.robot.wait(lim.command_period_s)
        except SafetyAbort as error:
            motion['reason'] = f'aborted: {error}'
            forward_end = self.robot.now()
        finally:
            errors = self._zero_burst(max(lim.zero_seconds, p['settle_s']))
            motion['zero_send_errors'] = errors
        end_odom = self._odom()
        measured = displacement(start_odom, end_odom) if start_odom and end_odom else {}
        activity = joint_activity(self.robot.telemetry, forward_start or t0, forward_end or t0)
        odom_during = [o for ts, o in self.robot.telemetry.odom.since(forward_start or t0)
                       if ts <= (forward_end or t0) + 0.5]
        observed = self.robot.telemetry.system_cmd.since(t0)
        detail = {
            **pre, 'motion': motion,
            'commanded': {'vx': p['vx'], 'wz': p['wz'], 'duration_s': p['duration_s']},
            'commanded_s': _r((forward_end or t0) - (forward_start or t0), 3),
            'measured': {k: _r(v) for k, v in measured.items()},
            'peak_planar_speed_m_s': _r(max((math.hypot(o.vx, o.vy) for o in odom_during), default=0.0)),
            'peak_yaw_rate_rad_s': _r(max((abs(o.wz) for o in odom_during), default=0.0)),
            'joint_activity_rad': _r(activity),
            'system_domain_observed': {'nonzero': sum(not c.is_zero for _, c in observed),
                                       'zero': sum(c.is_zero for _, c in observed)},
            'expect': {k: list(v) for k, v in p['expect'].items()},
        }
        trace = self._trace(t0)
        if motion['reason'] == 'no_stop_on_command_silence':
            detail['diagnosis'] = ('robot kept moving after commands stopped; fallback zeros sent. '
                                   'A crashed planner/mux would leave it walking.')
            return FAILED, 'no rest within silence timeout (fallback zeros sent)', detail, trace
        if motion['reason'] != 'duration_complete':
            return ABORTED, motion['reason'], detail, trace
        failures = [f'{k}={measured.get(k, float("nan")):.3f} outside {list(v)}'
                    for k, v in p['expect'].items()
                    if not (k in measured and v[0] <= measured[k] <= v[1])]
        if failures:
            if activity is not None and activity > 0.2 and measured.get('distance_m', 0) < 0.02:
                detail['diagnosis'] = 'stepping without translation (marching in place)'
            elif activity is not None and activity < 0.05:
                detail['diagnosis'] = 'no locomotion response in joint telemetry'
            detail['range_failure_only'] = True
            return FAILED, '; '.join(failures), detail, trace
        return PASSED, 'odometry displacement within expected ranges', detail, trace

    def step_stop_verify(self, step):
        if not self.robot.can_actuate:
            return PREVIEW, 'would send zeros and verify rest', {}
        hold = step.params['hold_s']
        t0 = self.robot.now()
        state = {'last_send': -math.inf}

        def keepalive():
            if self.robot.now() - state['last_send'] >= self.seq.safety.command_period_s:
                self.commands_sent = True
                self._stop_command()
                state['last_send'] = self.robot.now()

        def at_rest():
            still, rest = self._stationary(hold)
            return still and self.robot.now() - t0 >= step.params['zero_min_s'], rest

        try:
            ok, detail = self._wait_until(step, at_rest, poll=self.seq.safety.command_period_s,
                                          actuating=True, keepalive=keepalive, feed=False)
        finally:
            self._zero_burst(0.2)
        detail['verified_after_s'] = _r(self.robot.now() - t0, 2)
        trace = self._trace(t0)
        if ok:
            return PASSED, 'zero commanded and rest verified', detail, trace
        self.uncertain = True
        return FAILED, f'rest not verified within {step.timeout_s}s: {detail.get("reasons")}', detail, trace

    # --------------------------------------------------------------- execution
    def _run_step(self, step):
        handler = getattr(self, 'step_' + step.type)
        started = self.robot.now()
        self.log(f'--> {step.id} ({step.type})')
        self.partial = {}       # steps put their preconditions here so an abort keeps them
        try:
            out = handler(step)
        except SafetyAbort as error:
            if self.commands_sent:
                self.uncertain = self.uncertain or step.actuating
            context = self._abort_context(started, str(error))
            if error.detail:
                context = {**error.detail, **context}
            out = (ABORTED, f'safety stop: {error}', context, self._trace(started))
        except Exception as error:  # noqa: BLE001 - convert to a recorded failure, then stop safely
            self.uncertain = self.uncertain or self.commands_sent
            out = (FAILED, f'internal error: {type(error).__name__}: {error}',
                   self._abort_context(started, repr(error)), self._trace(started))
        status, summary, detail = out[:3]
        result = StepResult(step.id, step.type, status, summary, step.actuating,
                            _r(started - self.start, 3), _r(self.robot.now() - started, 3),
                            detail, out[3] if len(out) > 3 else [])
        self.log(f'<-- {step.id}: {status} - {summary}')
        return result

    def _abort_context(self, started, reason):
        """Everything the buffers hold for the aborted step: what the lock saw, what reached the
        robot and when (seconds after step start), plus the step's recorded preconditions."""
        t = self.robot.telemetry
        rel = lambda ts: _r(ts - started, 3)  # noqa: E731
        context = {'abort_reason': reason, 'abort_at_s': rel(self.robot.now()),
                   'preconditions': self.partial,
                   'robot_commands': [[rel(ts), _r(c.vx), _r(c.vy), _r(c.wz)]
                                      for ts, c in t.system_cmd.since(started - 1.0)],
                   'nav2_commands': [[rel(ts), _r(c.vx), _r(c.wz)] for ts, c in t.nav_cmd.since(started - 1.0)],
                   'terrain_status': [[rel(ts), v.stop, v.reason, _r(v.obstacle_ahead_m), v.stop_ahead_m,
                                       list(v.problems)] for ts, v in t.terrain.since(started - 3.0)],
                   'terrain_stop_topic': [[rel(ts), v] for ts, v in t.terrain_stop.since(started - 3.0)],
                   'clearance_m': [[rel(ts), _r(v)] for ts, v in t.clearance.since(started - 1.0)],
                   'last_dead_man_feed_s': None if self.last_deadman_feed is None
                   else rel(self.last_deadman_feed)}
        return context

    def safe_stop(self):
        """Zeros, then verify rest. Never sends posture commands."""
        self._event('safe_stop_begin')
        self._cancel_navigation()
        errors = self._zero_burst(self.seq.safety.zero_seconds)
        deadline = self.robot.now() + self.seq.safety.stop_timeout_s
        verified, detail = False, {}
        while self.robot.now() < deadline:
            verified, detail = self._stationary(self.seq.safety.stop_hold_s)
            if verified:
                break
            self._zero_burst(self.seq.safety.command_period_s)
        self._event('safe_stop_end', rest_verified=verified, zero_send_errors=errors,
                    reasons=detail.get('reasons'))
        return verified, detail

    def run(self):
        self.start = self.robot.now()
        started_utc = dt.datetime.now(dt.timezone.utc)
        self.origin = None
        results = []
        failed = None
        for index, step in enumerate(self.seq.steps):
            if self.mode == MODE_READONLY and step.actuating:
                # Evaluate the first actuating step's preconditions without any publisher.
                first = self._run_step(step)
                results.append(first)
                if first.status not in (PREVIEW, SKIPPED):
                    failed = failed or first
                results += [StepResult(s.id, s.type, PREVIEW, 'read-only: not executed', s.actuating)
                            for s in self.seq.steps[index + 1:]]
                break
            result = self._run_step(step)
            results.append(result)
            if self.mode == MODE_READONLY and result.status == FAILED:
                # Read-only readiness report: keep evaluating the non-moving checks so every
                # blocker is listed at once (nothing is published in this mode).
                failed = failed or result
                continue
            if (result.status == FAILED and step.type == 'velocity_move'
                    and step.params.get('continue_on_range_failure')
                    and result.measurements.get('range_failure_only')):
                # Characterization: a measured-range miss with zeros already sent is recorded,
                # and the sequence continues only if the following stop_verify proves rest.
                self.range_failures.append(step.id)
                self._event('range_failure_continue', step=step.id)
                continue
            if result.status not in (PASSED, SKIPPED):
                failed = result
                results += [StepResult(s.id, s.type, NOT_RUN, f'not run: {step.id} {result.status}',
                                       s.actuating) for s in self.seq.steps[index + 1:]]
                break
        stop = None
        if failed is not None and failed.status == ABORTED and self.commands_sent:
            # A safety abort (takeover, tilt, stale data, geofence...) always needs a human.
            self.uncertain = True
        if failed is not None and self.commands_sent:
            verified, detail = self.safe_stop()
            stop = {'rest_verified': verified, **detail}
            if not verified:
                self.uncertain = True
        for step in self.seq.finalize:
            if self.mode == MODE_READONLY:
                results.append(StepResult(step.id, step.type, PREVIEW, 'read-only: not executed', True))
            elif self.uncertain:
                results.append(StepResult(step.id, step.type, NOT_RUN,
                                          'robot state uncertain: operator intervention required',
                                          step.actuating))
            elif not self.commands_sent:
                results.append(StepResult(step.id, step.type, NOT_RUN,
                                          'no commands were sent; robot left as found',
                                          step.actuating))
            else:
                result = self._run_step(step)
                results.append(result)
                if result.status not in (PASSED, SKIPPED) and self.commands_sent:
                    verified, detail = self.safe_stop()
                    if not verified:
                        self.uncertain = True
        final_posture = self._posture(1.0)
        final_still, final_rest = self._stationary(1.0)
        if self.commands_sent and not (final_posture.confident and final_still):
            self.uncertain = True
        counts = {}
        for r in results:
            counts[r.status] = counts.get(r.status, 0) + 1
        all_ok = (failed is None and not self.range_failures and
                  all(r.status in (PASSED, SKIPPED, PREVIEW) for r in results))
        verdict = ('INTERVENTION_REQUIRED' if self.uncertain else
                   'PASSED' if all_ok and self.mode != MODE_READONLY else
                   'READONLY_CHECKS_PASSED' if all_ok else 'FAILED_SAFE')
        return {
            'sequence': self.seq.name, 'sequence_sha256': self.seq.sha256,
            'sequence_source': self.seq.source, 'mode': self.mode,
            'evidence_class': {MODE_SIM: 'software simulation (no robot)',
                               MODE_READONLY: 'live robot telemetry, no commands published',
                               MODE_PHYSICAL: 'physical robot telemetry'}[self.mode],
            'authorization': None if self.auth is None else {
                'session': self.auth.session, 'valid_until': self.auth.valid_until.isoformat(),
                'source': self.auth.source},
            'started_utc': started_utc.isoformat(timespec='seconds'),
            'duration_s': _r(self.robot.now() - self.start, 2),
            'verdict': verdict, 'counts': counts, 'range_failures_continued': self.range_failures,
            'commands_sent': self.commands_sent,
            'posture_commands_latched_off': self.posture_latched_off,
            'state_uncertain': self.uncertain,
            'safe_stop': stop,
            'final_state': {'posture': final_posture.as_dict(), 'stationary': final_still,
                            'rest_detail': final_rest},
            'steps': [r.as_dict() for r in results],
            'events': self.events,
            'posture_criteria': {**self.seq.posture.__dict__,
                                 'basis': 'Lite3 URDF leg lengths + live joint telemetry; lying '
                                          'band confirmed on live robot 2026-11-13, standing band '
                                          'confirmed only by a recorded physical stand transition'},
        }
