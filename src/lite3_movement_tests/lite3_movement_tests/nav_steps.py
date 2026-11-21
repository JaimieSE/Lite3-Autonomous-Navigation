"""Navigation-mode steps (sequence ``interface: nav``), mixed into SequenceRunner.

Command path in this mode (all existing components; nothing here talks to the robot directly):

    Nav2 -> /nav_cmd_vel (10) --+
    runner test moves -> /lite3_ops/test_cmd_vel (20) --+-> twist_mux -> /cmd_vel -> domain bridge
    runner zeros -> /lite3_ops/stop_cmd_vel (255) ------+      (locks: terrain_stop, operator_hold)
                                                           -> motion_sender (domain 64) -> UDP

The operator_hold dead-man lock blocks priorities < 50 unless the runner publishes
hold=false; the runner only does so inside ``_check()`` after every safety check passed,
so a stalled or dead runner locks the robot out within 0.5 s.

Every step passes only on telemetry: robot-side commands observed in domain 64, leg
odometry, joint posture, IMU, the terrain monitor's own status and hazard cloud, and
Nav2's map pose. A published command or a Nav2 "SUCCEEDED" alone is never success.
"""

import math

from .config import NAV2_MAX_YAW_RATE_RAD_S, NOSE_OFFSET_M
from .geometry import STANDING
from .m4_steps import ProximityStopSteps
from .telemetry import assess_locomotion_stopped, displacement, rest_timeline, settle_summary

try:
    from lite3_locomotion import collision_profile as CP
except ImportError:      # running from the source tree without the workspace install sourced
    import sys
    from pathlib import Path
    sys.path.append(str(Path(__file__).resolve().parents[2] / 'lite3_locomotion'))
    from lite3_locomotion import collision_profile as CP

FOOTPRINT_HALF = (0.305, 0.185)    # params/nav2_lite3.yaml footprint


def footprint_clearance(points_xy, half=FOOTPRINT_HALF):
    """Smallest distance from base_link points (x, y) to the footprint rectangle; 0 inside."""
    best = math.inf
    for x, y in points_xy:
        dx, dy = max(abs(x) - half[0], 0.0), max(abs(y) - half[1], 0.0)
        best = min(best, math.hypot(dx, dy))
    return best


def compose(anchor, forward, left, yaw):
    """Pose (x, y, yaw) offset from ``anchor`` in its own frame."""
    x, y, a = anchor
    return (x + forward * math.cos(a) - left * math.sin(a),
            y + forward * math.sin(a) + left * math.cos(a), _wrap(a + yaw))


def cross_track(a, b, p):
    """Signed distance of p from the line a->b (left positive)."""
    dx, dy = b[0] - a[0], b[1] - a[1]
    length = math.hypot(dx, dy)
    if length < 1e-6:
        return math.hypot(p[0] - a[0], p[1] - a[1])
    return (dx * (p[1] - a[1]) - dy * (p[0] - a[0])) / length


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _r(value, digits=3):
    return None if value is None or not math.isfinite(value) else round(value, digits)


class NavSteps(ProximityStopSteps):
    """Requires a robot with the navigation interface (ros_io.NavRobot or sim_model.FakeNavRobot)."""

    PASSED, FAILED, PREVIEW = 'PASSED', 'FAILED', 'PREVIEW_ONLY'

    # ------------------------------------------------------------ helpers
    def _nav_capable(self):
        return hasattr(self.robot, 'nav_send_goal')

    def _terrain(self, max_age=0.5):
        item = self.robot.telemetry.terrain.latest()
        if item is None or self.robot.now() - item[0] > max_age:
            return None
        return item[1]

    def _nose_distance(self, sample):
        if sample is None or sample.obstacle_ahead_m is None:
            return None
        return sample.obstacle_ahead_m - NOSE_OFFSET_M

    def _nav_violation(self, now):
        if not self._nav_capable():
            return None
        if self.nav_active_until >= now:
            pose = self.robot.map_pose()
            if self.geofence and pose and math.hypot(pose[0] - self.geofence[0],
                                                     pose[1] - self.geofence[1]) > self.geofence[2]:
                return 'navigation geofence exceeded (map pose)'
        return None

    def _cancel_navigation(self):
        if self._nav_capable() and self.robot.can_actuate:
            try:
                self.robot.nav_cancel()
            except Exception:  # noqa: BLE001 - stop continues regardless
                pass

    def _standing_precondition(self, require_still=True):
        posture = self._posture(1.0)
        still, rest = self._stationary(1.0)
        pre = {'posture': posture.as_dict(), 'stationary': rest}
        ok = posture.confident and posture.posture == STANDING and (still or not require_still)
        return ok, pre

    def _robot_nonzero_since(self, start):
        return [(t, c) for t, c in self.robot.telemetry.system_cmd.since(start) if not c.is_zero]

    # -------------------------------------------------------------- steps
    def step_nav_readiness(self, step):
        """Nav2, terrain monitor, LiDAR rate and the physical mux wiring, without commanding."""
        p = step.params
        want_ahead = p['stop_distance_m'] + NOSE_OFFSET_M
        applied = {}
        if self.robot.can_actuate:
            applied = self.robot.set_stop_distance(want_ahead)

        def ready():
            now = self.robot.now()
            t = self.robot.telemetry
            detail = self.robot.nav_ready()
            problems = list(detail.get('problems', []))
            terrain_hz = len(t.terrain.window(now, p['window_s'])) / p['window_s']
            lidar_hz = len(t.lidar_odom.window(now, p['window_s'])) / p['window_s']
            detail.update({'terrain_hz': _r(terrain_hz, 1), 'lidar_odometry_hz': _r(lidar_hz, 1)})
            if terrain_hz < p['min_lidar_hz']:
                problems.append(f'terrain monitor {terrain_hz:.1f} Hz < {p["min_lidar_hz"]} Hz '
                                '(LiDAR link degraded?)')
            if lidar_hz < p['min_lidar_hz']:
                problems.append(f'LiDAR odometry {lidar_hz:.1f} Hz < {p["min_lidar_hz"]} Hz')
            lags = sorted(v for _, v in t.lidar_odom.window(now, p['window_s']) if v is not None)
            if lags:
                lag = lags[min(len(lags) - 1, int(0.95 * len(lags)))]
                detail['lidar_odometry_lag_p95_s'] = _r(lag)
                if lag > p['max_odom_lag_s']:
                    problems.append(f'LiDAR odometry {lag:.2f} s behind real time (> {p["max_odom_lag_s"]} s): '
                                    'SLAM pose and TF are stale. Usually the robot->PC link is congested '
                                    '(the sensor_relay log shows the domain-64 /livox/imu and cloud rates; close '
                                    'RViz/ros2 CLI subscribers of /livox/* in domain 64; check the robot link)')
            sample = self._terrain()
            if sample is None:
                problems.append('terrain status stale')
            else:
                if sample.problems:
                    problems.append(f'terrain inputs invalid: {list(sample.problems)}')
                detail['stop_ahead_active_m'] = sample.stop_ahead_m
                detail['proximity_stop_now'] = sample.stop
                if sample.stop_ahead_m is None:
                    problems.append('terrain monitor does not report its stop distance (an older build '
                                    'is running: operator/build.sh, then restart navigation)')
                elif abs(sample.stop_ahead_m - want_ahead) > 0.011:
                    problems.append(f'proximity stop distance is {sample.stop_ahead_m} m from centre, '
                                    f'requested {want_ahead:.3f} m')
            if t.clearance.age(now) > 0.5:
                problems.append('hazard cloud stale')
            if p['cm_boundary_m'] > 0:
                problems += self._boundary_problems(p, detail)
            detail['problems'] = problems
            return not problems, detail

        ok, detail = self._wait_until(step, ready, poll=0.25)
        detail['stop_distance_request'] = applied
        return (self.PASSED if ok else self.FAILED,
                f'Nav2, terrain monitor ({p["stop_distance_m"]:.2f} m stop), LiDAR and mux wiring ready'
                if ok else f'navigation not ready: {detail.get("problems")}', detail)

    def _boundary_problems(self, p, detail):
        """Running collision_monitor boundary vs what the LIVE sensing delay requires."""
        try:
            cal, measured = self._live_calibration({'path_latency_s': 0.10, 'coast_m': p['coast_m'],
                                                    'coast_speed_m_s': p['coast_speed_m_s'],
                                                    'sensor_margin_m': CP.DEFAULTS['sensor_margin_m']})
        except ValueError as error:          # e.g. sensing_age_s 2.66 s is outside the model's 0-2 s range
            detail['collision_boundary'] = {'error': str(error)}
            return [f'LIVE LiDAR sensing delay is beyond what the stopping boundary can cover ({error}): the '
                    'LiDAR data is seconds late. Check the robot link / perception host load (sensor_relay '
                    'log "relay timing" shows the cloud age on arrival)']
        try:
            tier = CP.tiers(p['cm_clearance_m'], p['cm_max_speed_m_s'], cal)
            need = tier['stop']
        except ValueError as error:
            detail['collision_boundary'] = {'live': measured, 'error': str(error)}
            return [f'the stopping boundary for {p["cm_max_speed_m_s"]} m/s with {p["cm_clearance_m"]} m '
                    f'clearance cannot be met with the LIVE sensing delay ({measured.get("sensing_age_p95_s")} s): '
                    f'{error}. Restart navigation with a lower --max-speed (or a smaller --clearance); at the '
                    'edge of the speed range normal delay jitter is enough to push the boundary past its cap']
        actual = self.robot.collision_boundary()
        detail['collision_boundary'] = {'running_m': actual, 'expected_m': p['cm_boundary_m'],
                                        'required_now': need, 'live': measured}
        if actual is None:
            return ['collision_monitor stop polygon not readable (not running/active?)']
        problems = []
        if abs(actual - p['cm_boundary_m']) > 0.006:
            problems.append(f'collision_monitor boundary {actual} m != started with {p["cm_boundary_m"]} m')
        if actual + 1e-9 < need['required_m']:
            problems.append(f'collision stop boundary {actual} m < {need["required_m"]} m required NOW '
                            f'(live sensing delay {measured.get("sensing_age_p95_s")} s): restart navigation '
                            'so the boundary is recomputed')
        if tier['approach']:        # above 0.10 m/s the approach action brings the robot to <= 0.10 m/s
            running = self.robot.collision_approach_time()
            approach_need = tier['approach']['time_before_collision_s']
            detail['collision_boundary'].update(approach_running_s=running,
                                                approach_required_now_s=approach_need)
            if running is None:
                problems.append(f'collision_monitor approach action missing: the stack must have been started '
                                f'with --max-speed {p["cm_max_speed_m_s"]} (restart navigation)')
            elif running + 1e-9 < approach_need:
                problems.append(f'collision approach time {running} s < {approach_need} s required NOW '
                                f'(live sensing delay {measured.get("sensing_age_p95_s")} s): restart navigation')
        return problems

    def step_zero_delivery(self, step):
        """Zeros through the mux stop input must reach motion_sender; then idle must be silent."""
        p = step.params
        if not self.robot.can_actuate:
            return self.PREVIEW, 'would send zeros on the stop input and verify delivery', {}
        self._check(feed=False)
        # Zeros published before DDS matches the new stop publisher are silently dropped, so wait for
        # the path to be discovered first (dead-man NOT fed meanwhile: the robot stays locked).
        match_start = self.robot.now()
        path = self.robot.stop_path_status() if hasattr(self.robot, 'stop_path_status') else {'problems': []}
        while path['problems'] and self.robot.now() - match_start < p['match_timeout_s']:
            self._check(feed=False)
            self.robot.wait(0.05)
            path = self.robot.stop_path_status()
        path['waited_s'] = _r(self.robot.now() - match_start)
        if path['problems']:
            return (self.FAILED, f'stop path not connected after {p["match_timeout_s"]} s (no zeros sent): '
                    f'{path["problems"]}', {'stop_path': path, 'problems': path['problems']})
        # Warm-up: our own domain-64 observer must also be matched to the bridge before anything can be
        # measured, and Humble rclpy cannot report a subscription's matched publishers. So send zeros
        # until the first one is SEEN at motion_sender (live 2026-11-18: path ready at 0.45 s, observer
        # first saw a zero 0.86 s later), then measure a full burst. Zeros on the stop input are harmless.
        self.commands_sent = True
        warm_start = self.robot.now()
        while not self.robot.telemetry.system_cmd.since(warm_start):
            if self.robot.now() - warm_start >= p['match_timeout_s']:
                path['warmup_s'] = _r(self.robot.now() - warm_start)
                return (self.FAILED, f'no zero observed at motion_sender within {p["match_timeout_s"]} s of '
                        'warm-up (stop path, domain bridge or our domain-64 observer)',
                        {'stop_path': path, 'problems': ['zeros never observed at motion_sender']})
            self._check(feed=False)
            self._stop_command()
            self.robot.wait(self.seq.safety.command_period_s)
        path['warmup_s'] = _r(self.robot.now() - warm_start)
        self.robot.wait(self.seq.safety.command_period_s)
        t0 = self.robot.now()
        while self.robot.now() - t0 < p['duration_s']:
            self._check(feed=False)
            self._stop_command()
            self.robot.wait(self.seq.safety.command_period_s)
        t1 = self.robot.now()
        self.robot.wait(0.3)                      # let the 0.25 s stop-input timeout lapse
        idle_start = self.robot.now()
        while self.robot.now() - idle_start < p['idle_s']:
            self._check(feed=False)               # dead-man NOT fed: everything must stay locked
            self.robot.wait(0.05)
        observed = self.robot.telemetry.system_cmd.since(t0)
        during = [(t, c) for t, c in observed if t <= t1 + 0.3]
        idle = [(t, c) for t, c in observed if t >= idle_start]
        # While locked only the stop input can pass, and only zeros. A zero shortly after the burst is
        # one of ours arriving late (bridge/executor lag); a non-zero, or a zero long after, is a breach.
        idle_breach = [(t, c) for t, c in idle if not c.is_zero or t - t1 > p['late_zero_s']]
        first = during[0][0] - t0 if during else None
        detail = {'stop_path': path, 'zeros_sent_s': _r(t1 - t0), 'observed_at_motion_sender': len(during),
                  'first_after_s': _r(first), 'nonzero_observed': sum(not c.is_zero for _, c in during),
                  'messages_while_idle_locked': len(idle_breach),
                  'late_zero_stragglers': len(idle) - len(idle_breach)}
        problems = []
        if not during or first > p['deadline_s']:
            problems.append(f'zeros not observed at motion_sender within {p["deadline_s"]} s')
        if detail['nonzero_observed']:
            problems.append('non-zero command observed during a zero-only test')
        if len(during) < p['duration_s'] * 10:
            problems.append(f'only {len(during)} of ~{p["duration_s"] * 20:.0f} zeros arrived')
        if idle_breach:
            problems.append(f'{len(idle_breach)} commands reached motion_sender while idle and locked '
                            f'(non-zero, or a zero > {p["late_zero_s"]} s after the burst)')
        detail['problems'] = problems
        return ((self.PASSED, 'stop path delivers zeros to motion_sender; idle mux silent', detail)
                if not problems else (self.FAILED, '; '.join(problems), detail))

    def _guarded_move(self, step, *, feed_until=math.inf, on_tick=None, expect_delivery=True):
        """Bounded constant forward command on the mux test input. Returns timing detail.

        ``feed_until``: seconds after motion start to keep the dead-man released.
        ``on_tick(elapsed)`` may raise SafetyAbort or return False to end the command early.
        """
        p = step.params
        lim = self.seq.safety
        t0 = self.robot.now()
        info = {'released_dead_man_until_s': None if feed_until == math.inf else feed_until,
                'reason': 'duration_complete'}
        last = t0
        while True:
            elapsed = self.robot.now() - t0
            self._check(feed=elapsed < feed_until)
            if self.robot.now() - last > 3 * lim.command_period_s:
                raise self._abort('command scheduler gap')
            if elapsed >= p['duration_s']:
                break
            if expect_delivery and elapsed > p['delivery_deadline_s'] and \
                    not self._robot_nonzero_since(t0):
                raise self._abort('non-zero command not observed at motion_sender (mux/bridge path)')
            if on_tick is not None and on_tick(elapsed) is False:
                info['reason'] = 'ended_early'
                break
            self._send(p['vx'], 0.0)
            last = self.robot.now()
            self.robot.wait(lim.command_period_s)
        info['commanded_s'] = _r(self.robot.now() - t0)
        first = self._robot_nonzero_since(t0)
        # Measured command latency: first publish -> first arrival at motion_sender (domain 64).
        info['first_delivery_s'] = _r(first[0][0] - t0, 4) if first else None
        return t0, info

    def _abort(self, reason):
        from .runner import SafetyAbort
        return SafetyAbort(reason)

    def _stop_diagnostics(self, t0, release, until, hold, nonzero, last_feed, start_odom):
        """When each stop-related signal settled, relative to the dead-man release (seconds)."""
        crit = self.seq.stationary
        rows = rest_timeline(self.robot.telemetry, t0, until, hold, crit, origin=release)
        settle = settle_summary(rows, crit, hold)
        after = [r for r in rows if r['t'] >= 0]

        def last_over(key, limit):
            over = [r['t'] for r in after if r.get(key) is not None and r[key] > limit]
            return over[-1] if over else None

        times = [t for t, _ in nonzero]
        odom = [(ts, o) for ts, o in self.robot.telemetry.odom.since(t0) if ts <= until]
        fwd = []
        if start_odom is not None:
            c, s_ = math.cos(start_odom.yaw), math.sin(start_odom.yaw)
            fwd = [(ts - release, (o.x - start_odom.x) * c + (o.y - start_odom.y) * s_) for ts, o in odom]
        at_release = min(fwd, key=lambda p: abs(p[0]))[1] if fwd else None
        coast = None
        if times and fwd:
            cut = times[-1] - release
            at_cut = min(fwd, key=lambda q: abs(q[0] - cut))[1]
            coast = max(v for tt, v in fwd if tt >= cut) - at_cut
        disturbances = [v['last_exceeded_s'] for k, v in settle.items()
                        if isinstance(v, dict) and v['last_exceeded_s'] is not None]
        return {
            'release_s_after_motion_start': _r(release - t0),
            'last_dead_man_feed_s': None if last_feed is None else _r(last_feed - release),
            'robot_commands_s': {'first_nonzero': _r(times[0] - release) if times else None,
                                 'last_nonzero': _r(times[-1] - release) if times else None},
            'phases_after_release_s': {
                'commands_cut': _r(times[-1] - release) if times else None,
                'translation_ended': last_over('max_planar_speed_m_s', crit.max_planar_speed_m_s),
                'joint_motion_ended': last_over('joint_range_rad', crit.max_joint_range_rad),
                'yaw_motion_ended': last_over('max_yaw_rate_rad_s', crit.max_yaw_rate_rad_s),
                'last_disturbance': max(disturbances) if disturbances else None,
                'rest_onset': settle['rest_onset_s'],
            },
            'coast_after_cut_m': _r(coast),
            'forward_m': {'at_release': _r(at_release), 'max': _r(max((v for _, v in fwd), default=None)),
                          'end': _r(fwd[-1][1]) if fwd else None},
            'settle': settle,
            'timeline': rows,
        }

    def step_deadman_stop(self, step):
        """Drive slowly, then stop feeding the dead-man while still publishing: the mux must cut it."""
        p = step.params
        ok, pre = self._standing_precondition()
        if not ok:
            return self.FAILED, 'precondition: verified standing at rest required', pre
        sample = self._terrain()
        if sample is None or sample.stop:
            return self.FAILED, 'precondition: terrain stop active or stale; clear the area', pre
        if not self.robot.can_actuate:
            return self.PREVIEW, (f'would command vx={p["vx"]} and drop the dead-man after '
                                  f'{p["release_after_s"]} s'), pre
        start = self._odom()
        self.origin = start
        t0, info = None, {}
        hold = self.seq.safety.stop_hold_s
        rest = {'locomotion_stopped': False, 'settled': False}
        try:
            t0, info = self._guarded_move(step, feed_until=p['release_after_s'])
            release = t0 + p['release_after_s']
            # Observe with no command at all. Two measured stages, both confirmed by a full hold
            # window and judged on their ONSET:
            #   1. locomotion stopped: no translation, no foot lifts      -> by stop_within_s
            #   2. fully settled: every rest criterion incl. posture settle -> by settle_within_s
            while True:
                self._check(feed=False)
                elapsed = self.robot.now() - release
                if not rest['locomotion_stopped']:
                    stopped, sdetail = assess_locomotion_stopped(
                        self.robot.telemetry, self.robot.now(), hold, self.seq.locomotion_stop)
                    if stopped:
                        onset = max(0.0, elapsed - hold)
                        rest.update(locomotion_stopped=True, stop_onset_after_release_s=_r(onset),
                                    stop_pose=self._odom_at(release + onset))
                    elif elapsed >= p['stop_within_s'] + hold:
                        rest['stop_reasons'] = sdetail.get('reasons')
                        break                   # still walking: end the observation, zeros follow
                still, detail = self._stationary(hold)
                if still and rest['locomotion_stopped']:
                    rest.update(settled=True, settle_onset_after_release_s=_r(max(0.0, elapsed - hold)))
                    break
                if elapsed >= p['settle_within_s'] + hold:
                    rest['settle_reasons'] = detail.get('reasons')
                    break
                self.robot.wait(self.seq.safety.command_period_s)
        finally:
            observed_until = self.robot.now()
            last_feed = self.last_deadman_feed
            settled_pose = self._odom()
            info['zero_send_errors'] = self._zero_burst(self.seq.safety.zero_seconds)
        release = t0 + p['release_after_s']
        nonzero = self._robot_nonzero_since(t0)
        diagnostics = self._stop_diagnostics(t0, release, observed_until, hold, nonzero, last_feed,
                                             start)
        diagnostics['observation'] = {
            'commands_published_until_s': _r(t0 + info.get('commanded_s', 0.0) - release),
            'observed_until_s': _r(observed_until - release),
            'zero_burst_s': self.seq.safety.zero_seconds,
            'why_step_lasts_this_long': 'command phase (dead-man released part-way, test keeps '
                                        'publishing) + observation until full rest is confirmed '
                                        '(at most settle_within_s + hold_s after release) + closing '
                                        'zero burst'}
        settle_motion = {}
        stop_pose = rest.pop('stop_pose', None)
        if rest['locomotion_stopped'] and stop_pose is not None and settled_pose is not None:
            d = displacement(stop_pose, settled_pose)
            settle_motion = {'yaw_change_rad': _r(abs(d['yaw_change_rad']), 4),
                             'position_change_m': _r(d['distance_m'], 4)}
        rest['settle_motion'] = settle_motion
        before = [t for t, _ in nonzero if t <= release]
        last = max((t for t, _ in nonzero), default=None)
        cutoff = None if last is None else last - release
        moved = displacement(start, self._odom()) if start else {}
        detail = {**pre, 'motion': info, 'rest': rest, 'diagnostics': diagnostics,
                  'criteria': {k: p[k] for k in ('cutoff_within_s', 'stop_within_s', 'settle_within_s',
                                                 'max_settle_yaw_rad', 'max_settle_shift_m')},
                  'robot_nonzero_before_release': len(before),
                  'robot_nonzero_after_release': len(nonzero) - len(before),
                  'cutoff_after_release_s': _r(cutoff),
                  'measured': {k: _r(v) for k, v in moved.items()}}
        phases = diagnostics['phases_after_release_s']
        phases['locomotion_stopped_onset'] = rest.get('stop_onset_after_release_s')
        problems = []
        if not before:
            problems.append('no motion command reached the robot before the dead-man release')
        if cutoff is None or cutoff > p['cutoff_within_s']:
            problems.append(f'commands kept reaching the robot {cutoff} s after the dead-man release '
                            f'(limit {p["cutoff_within_s"]} s)')
        if not rest['locomotion_stopped'] or rest['stop_onset_after_release_s'] > p['stop_within_s']:
            problems.append(f'robot did not stop moving within {p["stop_within_s"]} s of the release '
                            f'(onset {rest.get("stop_onset_after_release_s")} s; translation ended '
                            f'{phases["translation_ended"]} s; still failing: {rest.get("stop_reasons")})')
        if not rest['settled'] or rest['settle_onset_after_release_s'] > p['settle_within_s']:
            late = [k for k, v in diagnostics['settle'].items()
                    if isinstance(v, dict) and v['last_exceeded_s'] is not None
                    and v['last_exceeded_s'] > p['settle_within_s']]
            problems.append(f'robot not fully settled within {p["settle_within_s"]} s of the release '
                            f'(last disturbance {phases["last_disturbance"]} s'
                            f'{" from " + ", ".join(late) if late else ""})')
        if settle_motion and settle_motion['yaw_change_rad'] > p['max_settle_yaw_rad']:
            problems.append(f'heading changed {settle_motion["yaw_change_rad"]} rad after stopping '
                            f'(limit {p["max_settle_yaw_rad"]})')
        if settle_motion and settle_motion['position_change_m'] > p['max_settle_shift_m']:
            problems.append(f'position shifted {settle_motion["position_change_m"]} m after stopping '
                            f'(limit {p["max_settle_shift_m"]})')
        if moved.get('forward_m', 0.0) < 0.02:
            problems.append('robot did not move before the release (cannot prove the stop)')
        if problems:
            self.uncertain = self.uncertain or not rest['settled']
            return self.FAILED, '; '.join(problems), detail, self._trace(t0)
        return (self.PASSED, f'dead-man release cut robot commands in {cutoff:.2f} s; stopped moving '
                f'{rest["stop_onset_after_release_s"]} s, fully settled '
                f'{rest["settle_onset_after_release_s"]} s after release (settle: '
                f'{settle_motion.get("yaw_change_rad")} rad, {settle_motion.get("position_change_m")} m)',
                detail, self._trace(t0))

    def _odom_at(self, stamp):
        items = self.robot.telemetry.odom.window(stamp + 0.05, 0.1)
        return min(items, key=lambda i: abs(i[0] - stamp))[1] if items else self._odom()

    # ------------------------------------------------------------ object detection
    def _detection_stats(self, window_s):
        """Stability of what the terrain monitor reports, over the last ``window_s`` seconds."""
        now = self.robot.now()
        samples = self.robot.telemetry.terrain.window(now, window_s)
        bools = self.robot.telemetry.terrain_stop.window(now, window_s)
        stamps = [now - window_s] + [ts for ts, _ in samples] + [now]
        dist = [self._nose_distance(v) for _, v in samples]
        seen = [d for d in dist if d is not None]
        proc = sorted(v.processing_s for _, v in samples if v.processing_s is not None)
        n = max(1, len(samples))
        return {
            'samples': len(samples), 'rate_hz': _r(len(samples) / window_s, 1),
            'max_gap_s': _r(max(b - a for a, b in zip(stamps, stamps[1:]))),
            'detected_ahead_fraction': _r(len(seen) / n),
            'stop_fraction': _r(sum(v.stop for _, v in samples) / n),
            'stop_ahead_fraction': _r(sum(v.stop and 'ahead' in (v.reason or '') for _, v in samples) / n),
            'stop_topic_fraction': _r(sum(v for _, v in bools) / max(1, len(bools))),
            'invalid_input_fraction': _r(sum(bool(v.problems) for _, v in samples) / n),
            'reasons': sorted({str(v.reason) for _, v in samples}),
            'nose_distance_m': {'min': _r(min(seen)), 'median': _r(sorted(seen)[len(seen) // 2]),
                                'max': _r(max(seen))} if seen else None,
            'processing_s_median': _r(proc[len(proc) // 2]) if proc else None,
        }

    def _detection_problems(self, stats, expect, distance, min_fraction, max_gap, max_spread):
        problems = []
        if stats['samples'] < 3:
            return ['terrain status not received']
        if stats['max_gap_s'] > max_gap:
            problems.append(f'terrain status gap {stats["max_gap_s"]} s > {max_gap} s (LiDAR link)')
        if stats['invalid_input_fraction'] > 0:
            problems.append('terrain monitor inputs invalid in part of the window')
        if stats['detected_ahead_fraction'] < min_fraction:
            problems.append(f'object detected ahead in only {stats["detected_ahead_fraction"]:.0%} of '
                            f'scans (need {min_fraction:.0%})')
        nd = stats['nose_distance_m']
        if nd and not distance[0] <= nd['median'] <= distance[1]:
            problems.append(f'object {nd["median"]} m from the nose; must be {distance[0]:.2f}-'
                            f'{distance[1]:.2f} m')
        if nd and nd['max'] - nd['min'] > max_spread:
            problems.append(f'object distance unstable ({nd["min"]}-{nd["max"]} m)')
        if expect == 'stop_zone':
            if stats['stop_ahead_fraction'] < min_fraction:
                problems.append(f'proximity stop held for the object ahead in only '
                                f'{stats["stop_ahead_fraction"]:.0%} of scans (reasons {stats["reasons"]})')
            if stats['stop_topic_fraction'] < 1.0:
                problems.append('lock input /lite3_terrain/stop was false at least once')
        return problems

    def step_detection_check(self, step):
        """No commands: is the placed object seen steadily, at the right distance, for the right reason?"""
        p = step.params

        def check():
            stats = self._detection_stats(p['window_s'])
            problems = self._detection_problems(stats, p['expect'], p['distance_m'], p['min_fraction'],
                                                p['max_gap_s'], p['max_spread_m'])
            return not problems, {**stats, 'problems': problems,
                                  'criteria': {k: p[k] for k in ('expect', 'distance_m', 'min_fraction',
                                                                 'max_gap_s', 'max_spread_m')}}

        self.robot.wait(min(p['window_s'], 1.0))
        ok, detail = self._wait_until(step, check, poll=0.25)
        nd = detail.get('nose_distance_m') or {}
        return (self.PASSED if ok else self.FAILED,
                (f'object seen in {detail["detected_ahead_fraction"]:.0%} of {detail["samples"]} scans at '
                 f'{nd.get("median")} m from the nose (spread {_r((nd.get("max") or 0) - (nd.get("min") or 0))} '
                 f'm); max gap {detail["max_gap_s"]} s' if ok else
                 f'detection not reliable: {detail.get("problems")}'), detail)

    def _lock_engaged_now(self, max_age=0.3):
        """Both the status and the actual lock input say: stopped for an object ahead, freshly."""
        now = self.robot.now()
        s = self.robot.telemetry.terrain.latest()
        b = self.robot.telemetry.terrain_stop.latest()
        if s is None or now - s[0] > max_age:
            return False, 'terrain status stale'
        if b is None or now - b[0] > max_age:
            return False, 'lock input /lite3_terrain/stop stale'
        if not (s[1].stop and b[1]):
            return False, f'proximity stop released (status reason {s[1].reason!r})'
        if 'ahead' not in (s[1].reason or ''):
            return False, f'proximity stop no longer caused by the object ahead ({s[1].reason!r})'
        return True, None

    def step_blocked_move(self, step):
        """With an object inside the stop zone, a forward command must not reach the robot.

        Each command is published only after re-confirming that the lock is engaged by the object
        ahead; if perception releases the lock the test stops commanding at once (aborts), so a
        perception dropout can no longer be turned into motion by the test itself."""
        p = step.params
        ok, pre = self._standing_precondition()
        self.partial = pre
        if not ok:
            return self.FAILED, 'precondition: verified standing at rest required', pre
        engaged, why = self._lock_engaged_now()
        sample = self._terrain()
        pre['terrain'] = None if sample is None else {'stop': sample.stop, 'reason': sample.reason,
                                                     'nose_distance_m': _r(self._nose_distance(sample))}
        if not engaged:
            return self.FAILED, f'precondition: lock not engaged by the object ahead ({why})', pre
        if not self.robot.can_actuate:
            return self.PREVIEW, f'would command vx={p["vx"]} for {p["duration_s"]} s into the lock', pre
        start = self._odom()
        self.origin = start
        stop_samples = []
        t_start = [self.robot.now()]

        def tick(_):
            if self._robot_nonzero_since(t_start[0]):
                raise self._abort('a forward command passed the proximity lock to the robot')
            engaged, why = self._lock_engaged_now()
            stop_samples.append(engaged)
            if not engaged:
                raise self._abort(f'lock released during the test, command withheld: {why}')
            return True

        info = {}
        try:
            t0, info = self._guarded_move(step, on_tick=tick, expect_delivery=False)
        finally:
            info['zero_send_errors'] = self._zero_burst(self.seq.safety.zero_seconds)
        moved = displacement(start, self._odom()) if start else {}
        passed = self._robot_nonzero_since(t_start[0])
        detail = {**pre, 'motion': info, 'robot_nonzero': len(passed),
                  'stop_active_fraction': _r(sum(stop_samples) / max(1, len(stop_samples))),
                  'measured': {k: _r(v) for k, v in moved.items()}}
        problems = []
        if passed:
            problems.append(f'{len(passed)} forward commands reached the robot')
        if moved.get('distance_m', 0.0) > p['max_displacement_m']:
            problems.append(f'robot moved {moved["distance_m"]:.3f} m (> {p["max_displacement_m"]})')
        if problems:
            return self.FAILED, '; '.join(problems), detail, self._trace(t_start[0])
        return (self.PASSED, f'forward command blocked for {info.get("commanded_s")} s; robot did not '
                'move', detail, self._trace(t_start[0]))

    def _live_calibration(self, p):
        """Stopping-boundary inputs: sensing age and processing measured NOW, coast from M2."""
        now = self.robot.now()
        ages = sorted(v for _, v in self.robot.telemetry.hazard_age.window(now, 3.0))
        proc = sorted(v.processing_s for _, v in self.robot.telemetry.terrain.window(now, 3.0)
                      if v.processing_s is not None)
        sensing = ages[min(len(ages) - 1, int(0.95 * len(ages)))] if ages else None
        cal = CP.Calibration(sensing_age_s=max(0.0, sensing if sensing is not None else 0.5),
                             processing_s=proc[len(proc) // 2] if proc else 0.05,
                             path_latency_s=p['path_latency_s'], coast_m=max(0.0, p['coast_m']),
                             coast_speed_m_s=p['coast_speed_m_s'], sensor_margin_m=p['sensor_margin_m'])
        return cal, {'sensing_age_p95_s': _r(sensing), 'samples': len(ages)}

    def step_nav_goal(self, step):
        """Nav2 drives to a goal relative to the sequence anchor; judged on telemetry."""
        p = step.params
        index = self.seq.steps.index(step) if step in self.seq.steps else 0
        chained = index > 0 and self.seq.steps[index - 1].type == 'nav_goal'
        ok, pre = self._standing_precondition(require_still=not chained)
        if not ok and not chained:
            return self.FAILED, 'precondition: verified standing at rest required', pre
        pose = self.robot.map_pose()
        if pose is None:
            return self.FAILED, 'no map -> base_link pose (SLAM/localization not running?)', pre
        if self.nav_anchor is None:
            self.nav_anchor = pose
        goal = compose(self.nav_anchor, p['forward_m'], p['left_m'], math.radians(p['yaw_deg']))
        pre.update({'anchor': [_r(v) for v in self.nav_anchor], 'goal': [_r(v) for v in goal],
                    'start_pose': [_r(v) for v in pose]})
        if not self.robot.can_actuate:
            return self.PREVIEW, (f'would send Nav2 goal {math.hypot(goal[0] - pose[0], goal[1] - pose[1]):.2f} m '
                                  f'away at <= {p["max_speed"]} m/s'), pre
        # M5 overspeed root cause (2026-11-16): Humble MPPI resets its optimizer when a goal
        # arrives ("Optimizer reset" 2-4 ms after "Received a goal") and that reset restores the
        # base vx_max, discarding a /speed_limit published before the goal. The hard ceiling is
        # therefore enforced by the velocity_smoother (downstream of MPPI, not reset per goal) via
        # its dynamic max_velocity parameter, read back before the goal is sent.
        cap = self.robot.set_nav_velocity_cap(p['max_speed'], NAV2_MAX_YAW_RATE_RAD_S)
        pre['nav_velocity_cap'] = cap
        if not cap.get('ok'):
            return self.FAILED, (f'Nav2 velocity cap {p["max_speed"]} m/s not applied and verified at the '
                                 f'velocity_smoother ({cap.get("reason")}); no goal sent'), pre
        self.geofence = (self.nav_anchor[0], self.nav_anchor[1], p['geofence_m'])
        start_odom = self._odom()
        self.origin = None                      # leg-odometry geofence replaced by the map geofence
        t0 = self.robot.now()
        self.nav_active_until = math.inf
        self.robot.set_speed_limit(p['max_speed'])
        outcome, track, clear_min, stop_time = None, [], math.inf, 0.0
        recoveries = 0
        last = shown = t0
        try:
            self._check()
            if not self.robot.nav_send_goal(*goal):
                return self.FAILED, 'Nav2 rejected the goal', pre
            self._event('nav_goal_sent', goal=[_r(v) for v in goal])
            self.robot.set_speed_limit(p['max_speed'])     # after the goal-start optimizer reset
            limit_sent = self.robot.now()
            deadline = t0 + step.timeout_s
            while True:
                self._check()
                now = self.robot.now()
                if now - limit_sent >= 1.0:                 # keep MPPI planning at the limit
                    self.robot.set_speed_limit(p['max_speed'])
                    limit_sent = now
                for _, c in self.robot.telemetry.system_cmd.window(now, 0.3):
                    if c.vx < -0.005:
                        raise self._abort('reverse command reached the robot (no rear sensing)')
                if self.robot.telemetry.clearance.age(now) > 0.5:
                    raise self._abort('hazard cloud stale: obstacle clearance unknown')
                clearance = self.robot.telemetry.clearance.latest()[1]
                clear_min = min(clear_min, clearance)
                if clearance < p['abort_clearance_m']:
                    raise self._abort(f'obstacle {clearance:.2f} m from the footprint '
                                      f'(abort below {p["abort_clearance_m"]} m)')
                sample = self._terrain()
                if sample is not None and sample.stop:
                    stop_time += now - last
                last = now
                pose = self.robot.map_pose()
                if pose:
                    if not track or math.hypot(pose[0] - track[-1][1], pose[1] - track[-1][2]) > 0.02:
                        track.append((now - t0, *pose))
                status = self.robot.nav_status()
                recoveries = max(recoveries, status.get('recoveries') or 0)
                if now - shown >= 1.0 and pose:
                    shown = now
                    cmd = self.robot.telemetry.system_cmd.latest()
                    cmd_txt = (f'vx {cmd[1].vx:+.2f} wz {cmd[1].wz:+.2f}'
                               if cmd and now - cmd[0] < 0.5 else 'none (locked/idle)')
                    self.log(f'    [{now - t0:5.1f}s] to goal {math.hypot(goal[0] - pose[0], goal[1] - pose[1]):.2f} m | '
                             f'clearance {clearance:.2f} m | {"STOP " + str(sample.reason) if sample and sample.stop else "clear"} | '
                             f'robot cmd {cmd_txt} | recoveries {recoveries}')
                if status.get('result'):
                    outcome = status['result']
                    break
                if now >= deadline:
                    outcome = 'TIMEOUT'
                    break
                self.robot.wait(0.05)
        finally:
            if outcome != 'SUCCEEDED':
                self._cancel_navigation()
            self.nav_active_until = self.robot.now() + 1.5   # smoother ramp-down grace
            self.geofence = None
            if index + 1 >= len(self.seq.steps) or self.seq.steps[index + 1].type != 'nav_goal' \
                    or outcome != 'SUCCEEDED':
                self.robot.set_speed_limit(0.0)
        end = self.robot.map_pose()
        end_odom = self._odom()
        err = math.hypot(goal[0] - end[0], goal[1] - end[1]) if end else None
        yaw_err = abs(math.degrees(_wrap(goal[2] - end[2]))) if end else None
        map_moved = math.hypot(end[0] - pre['start_pose'][0], end[1] - pre['start_pose'][1]) if end else None
        odom_moved = displacement(start_odom, end_odom).get('distance_m') if start_odom and end_odom else None
        lateral = max((abs(cross_track(self.nav_anchor if not chained else pre['start_pose'], goal,
                                       (x, y))) for _, x, y, _ in track), default=0.0)
        robot_cmds = self.robot.telemetry.system_cmd.since(t0)
        cm_stops = [e[1] for _, e in self.robot.telemetry.cm_events.since(t0) if e[0] == 'stop']
        detail = {
            **pre, 'nav2_result': outcome, 'duration_s': _r(self.robot.now() - t0, 1),
            'final_pose': [_r(v) for v in end] if end else None,
            'goal_error_m': _r(err), 'yaw_error_deg': _r(yaw_err, 1),
            'distance_map_m': _r(map_moved), 'distance_leg_odometry_m': _r(odom_moved),
            'max_cross_track_m': _r(lateral), 'min_footprint_clearance_m': _r(clear_min),
            'proximity_stop_s': _r(stop_time, 1), 'recoveries': recoveries,
            'collision_monitor_stops': len(cm_stops),
            'robot_side_commands': len(robot_cmds),
            'peak_robot_cmd_vx': _r(max((c.vx for _, c in robot_cmds), default=0.0)),
            'peak_robot_cmd_wz': _r(max((abs(c.wz) for _, c in robot_cmds), default=0.0)),
            'track': [[_r(v, 2) for v in row] for row in track[::max(1, len(track) // 200)]],
            'criteria': {k: p[k] for k in ('expect', 'xy_tolerance_m', 'yaw_tolerance_deg',
                                           'min_clearance_m', 'min_lateral_m', 'max_speed',
                                           'max_cm_stops')},
        }
        problems = []
        if clear_min < p['min_clearance_m']:
            problems.append(f'min footprint clearance {clear_min:.2f} m < {p["min_clearance_m"]} m')
        if p['max_cm_stops'] >= 0 and len(cm_stops) > p['max_cm_stops']:
            problems.append(f'collision_monitor had to stop the robot {len(cm_stops)} time(s): Nav2 did '
                            'not plan around the obstacle')
        if p['expect'] == 'reached':
            if outcome != 'SUCCEEDED':
                problems.append(f'Nav2 result {outcome}')
            if err is None or err > p['xy_tolerance_m']:
                problems.append(f'goal error {_r(err)} m > {p["xy_tolerance_m"]} m')
            if yaw_err is None or yaw_err > p['yaw_tolerance_deg']:
                problems.append(f'heading error {_r(yaw_err, 1)} deg > {p["yaw_tolerance_deg"]} deg')
            if map_moved is not None and odom_moved is not None and \
                    abs(map_moved - odom_moved) > 0.3 * map_moved + 0.15:
                problems.append(f'map displacement {map_moved:.2f} m disagrees with leg odometry '
                                f'{odom_moved:.2f} m (localization or slip)')
            if lateral < p['min_lateral_m']:
                problems.append(f'max path deviation {lateral:.2f} m < {p["min_lateral_m"]} m '
                                '(obstacle not avoided?)')
        else:
            if err is not None and err <= p['xy_tolerance_m']:
                problems.append('goal was reached although the path should be blocked')
        detail['problems'] = problems
        if problems:
            return self.FAILED, '; '.join(problems), detail, self._trace(t0)
        clear_txt = f'{clear_min:.2f} m' if math.isfinite(clear_min) else 'n/a (no hazards seen)'
        if p['expect'] == 'reached':
            summary = (f'reached goal (error {err:.2f} m, {yaw_err:.0f} deg) in {detail["duration_s"]} s; '
                       f'min clearance {clear_txt}')
        else:
            summary = (f'blocked goal handled safely: Nav2 {outcome}, {recoveries} recoveries, '
                       f'min clearance {clear_txt}')
        return self.PASSED, summary, detail, self._trace(t0)
