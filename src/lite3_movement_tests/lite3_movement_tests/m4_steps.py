"""M4: continuous forward walk with sensor-driven proximity stopping (mixed into SequenceRunner).

    READY -> WALKING -> OBSTACLE_DETECTED -> STOPPING -> STOP_VERIFIED

No goal and no predefined obstacle position. The robot walks straight ahead at a bounded low
speed while fresh LiDAR data says the path is clear, and must be stopped by the protective stop
when an obstacle (wherever it appears in the validated detection region) reaches the stopping
boundary.

Command path (all existing components; nothing here talks to the robot directly):

    runner forward command -> /lite3_ops/test_cmd_vel (prio 20) -> twist_mux
        locks: operator_hold (dead-man, fed only while every runner check passes)
               terrain_stop  (perception health + BACKUP proximity stop, 0.30 m)
    -> /lite3_ops/mux_out -> nav2_collision_monitor STOP polygon (the validated boundary)
    -> /cmd_vel -> lite3_domain_bridge_8 -> motion_sender

* The PROTECTIVE STOP is Nav2's maintained collision_monitor. Its boundary was computed at
  navigation start from the requested clearance, the speed it must cover and measured latency
  and coast; this step re-checks it against the LIVE sensing delay and the M2 coast measured on
  this navigation run before moving, and refuses if it no longer suffices.
* The terrain lock is the independent BACKUP and the stale-perception guard (Humble's
  collision_monitor ignores stale sources).
* This runner is a third, independent layer: it tracks the obstacle against leg odometry, checks
  freshness, speed, motion, geofence and travel/time limits on every tick, verifies that the
  stop suppresses forward commands at the robot side within the response limit, and aborts
  (zeros on the priority-255 stop input + dead-man released) on any violation.

PASS needs evidence, not timing: the robot moved, the obstacle was detected and tracked, the
collision_monitor zeroed forward commands BECAUSE of that obstacle while the runner was still
publishing them, the robot physically stopped, and the clearance held. Timer, travel limit,
backup lock, operator or safeguard stops are separate, explained outcomes.
"""

import math

from .config import HARD_MIN_APPROACH_CLEARANCE_M, MIN_PRIMARY_SEPARATION_M, ODOMETRY_TRUST
from .geometry import validate_joint_sample
from .proximity import (DETECTED, OUTCOMES, READY, STOPPING, VERIFIED, WALKING, ObstacleTracker,
                        classify_abort)
from .telemetry import assess_locomotion_stopped

try:
    from lite3_locomotion import collision_profile as CP
except ImportError:      # running from the source tree without the workspace install sourced
    import sys
    from pathlib import Path
    sys.path.append(str(Path(__file__).resolve().parents[2] / 'lite3_locomotion'))
    from lite3_locomotion import collision_profile as CP


def _r(value, digits=3):
    return None if value is None or not math.isfinite(value) else round(value, digits)


def _wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class ProximityStopSteps:
    """Requires the navigation robot interface (ros_io.NavRobot or sim_model.FakeNavRobot)."""

    # ------------------------------------------------------------------ READY
    def _m4_readiness(self, p, require_standing=True):
        """Every precondition for the walk; returns (problems, detail). Publishes nothing."""
        problems, detail = [], {}
        ok, standing = self._standing_precondition()
        detail.update(standing)
        if not ok:
            (problems if require_standing else detail.setdefault('notes', [])).append(
                'robot must be verified standing at rest')
        cal, live = self._live_calibration(p)
        bound = None
        try:
            bound = CP.boundary(p['min_clearance_m'], p['vx'], cal)
        except ValueError as error:
            problems.append(f'stopping boundary cannot be computed: {error}')
        cm = self.robot.collision_boundary()
        now = self.robot.now()
        t = self.robot.telemetry
        window = t.terrain.window(now, 2.0)
        stamps = [now - 2.0] + [ts for ts, _ in window] + [now]
        lidar_hz = len(window) / 2.0
        recent = [self._nose_distance(v) for _, v in t.terrain.window(now, 1.0)]
        blocking = sorted(d for d in recent if d is not None and d < p['initial_clear_m'])
        sample = self._terrain(max_age=p['max_sensor_age_s'])
        detail.update({
            'live_measurements': live, 'boundary_required': bound, 'collision_monitor_boundary_m': cm,
            'backup_stop_distance_m': p['backup_stop_distance_m'], 'lidar_hz': _r(lidar_hz, 1),
            'max_status_gap_s': _r(max(b - a for a, b in zip(stamps, stamps[1:]))),
            'initial_clear_m': p['initial_clear_m'],
            'nearest_ahead_now_m': _r(self._nose_distance(sample)) if sample else None,
            'detection_range_m': p['detection_range_m']})
        if p['coast_m'] < 0:
            problems.append('no measured coast distance: M2 must pass first on this navigation run')
        if cm is None:
            problems.append('collision_monitor stop polygon not readable (is it running and active?)')
        else:
            if abs(cm - p['stop_distance_m']) > 0.006:
                problems.append(f'collision_monitor boundary {cm} m != expected trigger distance '
                                f'{p["stop_distance_m"]} m')
            if bound and cm + 1e-9 < bound['required_m']:
                problems.append(
                    f'collision_monitor boundary {cm} m is below the {bound["required_m"]} m needed for '
                    f'{p["min_clearance_m"]} m clearance at {p["vx"]} m/s with the measured latency '
                    f'({bound["latency_s"]} s) and coast ({bound["coast_m"]} m): restart navigation '
                    'with a larger --clearance, or choose a smaller --min-clearance / --approach-speed')
            need = p['backup_stop_distance_m'] + MIN_PRIMARY_SEPARATION_M
            if cm + 1e-9 < need:
                problems.append(
                    f'collision_monitor boundary {cm} m is within {MIN_PRIMARY_SEPARATION_M} m of the '
                    f'{p["backup_stop_distance_m"]} m backup lock: the primary stop cannot be told apart '
                    f'from the backup. Restart navigation with --clearance >= '
                    f'{suggest_clearance(cm, need, self._nav_clearance)} (boundary >= {need:.2f} m)')
            if p['initial_clear_m'] < cm + 0.05 - 1e-9:
                problems.append(f'initial clear distance {p["initial_clear_m"]} m must exceed the '
                                f'boundary {cm} m by >= 0.05 m')
        if sample is None:
            problems.append(f'terrain status stale (> {p["max_sensor_age_s"]} s) or missing')
        else:
            if sample.problems:
                problems.append(f'terrain inputs invalid: {list(sample.problems)}')
            if sample.stop:
                problems.append(f'proximity stop already active ({sample.reason}); clear the area')
        if lidar_hz < p['min_lidar_hz']:
            problems.append(f'LiDAR/terrain rate {lidar_hz:.1f} Hz < {p["min_lidar_hz"]} Hz')
        if detail['max_status_gap_s'] is not None and detail['max_status_gap_s'] > p['max_sensor_age_s']:
            problems.append(f'terrain status gap {detail["max_status_gap_s"]} s > {p["max_sensor_age_s"]} s')
        if t.clearance.age(now) > 0.5:
            problems.append('hazard cloud stale')
        if blocking:
            problems.append(f'path not clear: obstacle {blocking[0]:.2f} m ahead (< initial clear distance '
                            f'{p["initial_clear_m"]} m); move it further away or out of the path')
        detail['problems'] = problems
        return problems, detail

    @property
    def _nav_clearance(self):
        return getattr(self, 'nav_clearance_hint', None)

    # ------------------------------------------------------------------ monitors
    def _m4_sensor_fault(self, p, now, walking_s):
        """Sensor/telemetry validity that must hold on EVERY tick of the walk."""
        t = self.robot.telemetry
        item = t.terrain.latest()
        if item is None or now - item[0] > p['max_sensor_age_s']:
            age = None if item is None else now - item[0]
            return f'LiDAR/terrain status stale (age {_r(age)} s > {p["max_sensor_age_s"]} s)'
        if item[1].problems:
            return f'terrain inputs invalid: {list(item[1].problems)}'
        if t.clearance.age(now) > 0.5:
            return 'hazard cloud stale (> 0.5 s)'
        lock = t.terrain_stop.latest()
        if lock is None or now - lock[0] > p['max_sensor_age_s']:
            return 'proximity-lock input /lite3_terrain/stop stale'
        if walking_s > 1.0:
            hz = len(t.terrain.window(now, 1.0))
            if hz < p['min_lidar_hz'] - 0.5:
                return f'LiDAR/terrain rate {hz} Hz < {p["min_lidar_hz"]} Hz'
        imu = t.imu.latest()
        if imu is None or not imu[1].valid:
            return 'invalid IMU orientation'
        joints = t.joints.latest()
        if joints is None:
            return 'joint telemetry missing'
        bad = validate_joint_sample(joints[1].names, joints[1].positions)
        if bad:
            return f'invalid joint telemetry: {bad}'
        return None

    def _m4_stop_trigger(self, since, quiet_after):
        """Earliest evidence that something stopped the forward commands: (time, layer) or None.

        collision_monitor: its own 'stop' decision (rosout) or robot-side ZEROS while its input
        (twist_mux output) was still forward - only the collision_monitor turns forward into zero.
        terrain_backup_lock: the lock input went true. command_path_silent: nothing reaches the
        robot while the runner publishes and no proximity layer engaged (dead-man/command path)."""
        t = self.robot.telemetry
        cm = [ts for ts, e in t.cm_events.since(since) if e[0] == 'stop']
        zeroing = [ts for ts, c in t.system_cmd.since(since) if c.is_zero and any(
            not m.is_zero for _, m in t.mux_out.window(ts, 0.1))]
        primary = min(cm[:1] + zeroing[:1], default=None)
        backup = next((ts for ts, v in t.terrain_stop.since(since) if v), None)
        candidates = []
        if primary is not None:
            candidates.append((primary, 'collision_monitor'))
        if backup is not None:
            candidates.append((backup, 'terrain_backup_lock'))
        if candidates:
            first = min(candidates)
            if first[1] == 'terrain_backup_lock' and primary is not None and primary <= backup + 0.05:
                return primary, 'collision_monitor'
            return first
        now = self.robot.now()
        if quiet_after is not None and now > quiet_after and not t.system_cmd.window(now, 0.3):
            return now - 0.3, 'command_path_silent'
        return None

    def _m4_line(self, now, t0, st, p, fwd, tracker, d):
        t = self.robot.telemetry
        odom = self._odom()
        cmd = t.system_cmd.latest()
        item = t.terrain.latest()
        lock = t.terrain_stop.latest()
        cm_stop = any(e[0] == 'stop' for _, e in t.cm_events.since(st['t_walk'] or t0))
        obstacle = ('none in range' if d is None or d > p['detection_range_m'] else
                    f'{d:.2f} m ({"tracked" if tracker.is_confirmed else "candidate"})')
        left_m = max(0.0, p['max_travel_m'] - fwd / ODOMETRY_TRUST)
        self.log(f'    [{now - t0:5.1f}s] {st["state"]:<17} | speed {odom.vx if odom else 0.0:+.3f} m/s | '
                 f'travelled {fwd:.2f} m (limit left {left_m:.2f} m) | time left '
                 f'{max(0.0, p["duration_s"] - (now - t0)):4.1f} s | obstacle {obstacle} | '
                 f'collision_monitor {"STOP" if cm_stop else "clear"} | backup lock '
                 f'{"STOP" if lock and lock[1] else "clear"} | LiDAR '
                 f'{len(t.terrain.window(now, 1.0))} Hz, age {_r(now - item[0], 2) if item else "-"} s | '
                 f'robot cmd {"vx %+.3f" % cmd[1].vx if cmd and now - cmd[0] < 0.5 else "none"}')

    # ------------------------------------------------------------------ the walk
    def step_approach_stop(self, step):
        p = step.params
        problems, pre = self._m4_readiness(p, require_standing=self.robot.can_actuate)
        pre['state'] = READY
        self.partial = pre
        if problems:
            return self.FAILED, 'not READY: ' + '; '.join(problems), pre
        if not self.robot.can_actuate:
            return self.PREVIEW, (f'READY: would walk forward at {p["vx"]} m/s for at most {p["max_travel_m"]} m '
                                  f'/ {p["duration_s"]} s; collision_monitor stop at '
                                  f'{pre["collision_monitor_boundary_m"]} m (needs >= '
                                  f'{pre["boundary_required"]["required_m"]} m); backup lock '
                                  f'{p["backup_stop_distance_m"]} m'), pre
        cm = pre['collision_monitor_boundary_m']
        coast = pre['boundary_required']['coast_m']
        tel = self.robot.telemetry
        period = self.seq.safety.command_period_s
        start = self._odom()
        self.origin = start                      # odometry geofence (seq safety radius)
        map0 = self.robot.map_pose()
        c, s_ = math.cos(start.yaw), math.sin(start.yaw)

        def travel():
            o = self._odom()
            dx, dy = o.x - start.x, o.y - start.y
            return dx * c + dy * s_, -dx * s_ + dy * c, _wrap(o.yaw - start.yaw), o

        tracker = ObstacleTracker(p['detection_range_m'], p['detect_scans'], p['track_tolerance_m'],
                                  p['lost_track_scans'], ODOMETRY_TRUST)
        st = {'state': WALKING, 't_walk': None, 't_det': None, 'd_det': None, 's_det': None,
              't_cross': None, 't_stop': None, 'layer': None, 'end': None, 'min_nose': math.inf,
              'sends': 0, 'sends_after_stop': 0, 'scan_stamp': None, 'fast_since': None,
              'lock_misses': 0, 'proof_cut': None, 'transitions': [], 'map_travel': None,
              'first_delivery': None, 'anchor': None, 'scans': [], 'certain_since': None}
        t0 = self.robot.now()

        def enter(state, **info):
            st['state'] = state
            st['transitions'].append({'state': state, 't_s': _r(self.robot.now() - t0), **info})
            self.log(f'    -> {state} {info if info else ""}')

        # Pre-seed the tracker with the scans taken while standing still: an obstacle that is
        # already visible is tracked from the first step, so a LiDAR that later goes blind (while
        # still reporting "nothing ahead") is caught as lost tracking, and the odometry budget to
        # that obstacle applies from the start.
        for stamp, sample in tel.terrain.window(t0, 1.5):
            if tracker.update(stamp, self._nose_distance(sample), 0.0) == 'confirmed':
                st.update(t_det=stamp, d_det=tracker.confirmed['d'], s_det=0.0,
                          anchor=(tracker.confirmed['d'], 0.0))
        pre['obstacle_tracked_at_start_m'] = _r(st['d_det'])
        st['transitions'].append({'state': WALKING, 't_s': 0.0})
        if st['d_det'] is not None:
            enter(DETECTED, distance_m=_r(st['d_det']), travelled_m=0.0, at_start=True)
        last_send = last_line = t0
        aborted = None
        d = None
        try:
            self._check()
            while True:
                now = self.robot.now()
                self._check()                   # dead-man fed only after every runner check passes
                if now - last_send > 3 * period:
                    raise self._abort('command scheduler gap: the dead-man would lapse')
                elapsed = now - t0
                fwd, lat, dyaw, odom = travel()
                fault = self._m4_sensor_fault(p, now, elapsed)
                if fault:
                    raise self._abort(fault)
                # ---- new LiDAR scan -> tracker, boundary crossing, lock health, hard floor
                item = tel.terrain.latest()
                if item[0] != st['scan_stamp']:
                    st['scan_stamp'] = item[0]
                    d = self._nose_distance(item[1])
                    if d is not None:
                        st['min_nose'] = min(st['min_nose'], d)
                    event = tracker.update(item[0], d, fwd)
                    if event is None and tracker.is_confirmed and tracker.anomalies == 0 and \
                            d is not None and d <= p['detection_range_m']:
                        st['anchor'] = (d, fwd)         # last consistent measurement of the obstacle
                    if event == 'confirmed':
                        st.update(t_det=item[0], d_det=d, s_det=fwd, anchor=(d, fwd))
                        if st['state'] == WALKING:
                            enter(DETECTED, distance_m=_r(d), travelled_m=_r(fwd))
                    elif event == 'lost':
                        raise self._abort(f'lost obstacle tracking: {tracker.note}')
                    elif event == 'unexpected':
                        raise self._abort(f'unexpected obstacle measurement: {tracker.note}')
                    if d is not None and d < HARD_MIN_APPROACH_CLEARANCE_M:
                        raise self._abort(f'obstacle {d:.2f} m from the nose (hard floor '
                                          f'{HARD_MIN_APPROACH_CLEARANCE_M} m)')
                    if d is not None and d <= cm and st['t_cross'] is None:
                        st['t_cross'] = item[0]
                    st['scans'].append((item[0], d))
                    # The terrain grid is 5 cm: an obstacle resting ON the polygon edge reads
                    # cm - 0.05 / cm / cm + 0.05 and the collision_monitor may release and re-stop
                    # (seen physically 2026-11-17 12:57). "Certainly inside" = more than one grid
                    # cell (the boundary's sensor margin) inside the boundary.
                    if d is not None and d <= cm - p['sensor_margin_m'] + 1e-9:
                        st['certain_since'] = st['certain_since'] or item[0]
                    else:
                        st['certain_since'] = None
                    lock = tel.terrain_stop.latest()
                    if d is not None and d <= p['backup_stop_distance_m'] - p['sensor_margin_m'] \
                            and not lock[1]:
                        st['lock_misses'] += 1
                        if st['lock_misses'] >= 2:
                            raise self._abort(f'proximity-lock failure: obstacle {d:.2f} m ahead, inside '
                                              f'the {p["backup_stop_distance_m"]} m backup stop, lock '
                                              'input false')
                    else:
                        st['lock_misses'] = 0
                # ---- independent of further LiDAR: odometry budget from the LAST consistent
                # measurement of the tracked obstacle (leg odometry may under-read by ~15 %).
                if st['anchor'] is not None:
                    d_a, s_a = st['anchor']
                    budget = s_a + ODOMETRY_TRUST * (d_a - HARD_MIN_APPROACH_CLEARANCE_M - coast)
                    if fwd > budget:
                        raise self._abort(f'leg-odometry travel {fwd:.3f} m exceeds the {budget:.3f} m '
                                          f'budget to the obstacle last measured {d_a:.2f} m ahead '
                                          '(independent of further LiDAR data)')
                # ---- unexpected motion
                if abs(lat) > p['max_lateral_m']:
                    raise self._abort(f'unexpected motion: lateral drift {lat:+.3f} m '
                                      f'(limit {p["max_lateral_m"]} m)')
                if abs(dyaw) > p['max_yaw_change_rad']:
                    raise self._abort(f'unexpected motion: heading change {dyaw:+.3f} rad '
                                      f'(limit {p["max_yaw_change_rad"]} rad)')
                if fwd < -0.05:
                    raise self._abort(f'unexpected motion: moved {fwd:+.3f} m backwards')
                fast = max(2.0 * p['vx'], p['vx'] + 0.05)
                if odom.vx > fast:
                    st['fast_since'] = st['fast_since'] or now
                    if now - st['fast_since'] > 0.5:
                        raise self._abort(f'unexpected motion: odometry speed {odom.vx:.3f} m/s > '
                                          f'{fast:.3f} m/s for 0.5 s')
                else:
                    st['fast_since'] = None
                # ---- independent travel cross-check: SLAM map pose
                pose = self.robot.map_pose()
                if map0 is not None and pose is not None:
                    st['map_travel'] = math.hypot(pose[0] - map0[0], pose[1] - map0[1])
                # ---- delivery: forward commands must reach motion_sender
                if st['t_walk'] is not None and now - st['t_walk'] > 0.5 and \
                        not self._robot_nonzero_since(st['t_walk']):
                    raise self._abort('forward command not observed at motion_sender (mux/bridge path)')
                # ---- did a layer stop the commands?
                if st['t_walk'] is not None and st['first_delivery'] is None:
                    st['first_delivery'] = next((ts for ts, cmd in tel.system_cmd.since(st['t_walk'])
                                                 if not cmd.is_zero), None)
                if st['t_stop'] is None and st['t_walk'] is not None:
                    trigger = self._m4_stop_trigger(st['t_walk'], None if st['first_delivery'] is None
                                                    else st['first_delivery'] + 0.3)
                    if trigger:
                        st['t_stop'], st['layer'] = trigger
                        enter(STOPPING, layer=st['layer'], nose_distance_m=_r(d))
                # ---- third layer: obstacle inside the boundary, forward still reaching the robot
                if st['certain_since'] is not None and now > st['certain_since'] + p['response_limit_s']:
                    late = [ts for ts, cmd in tel.system_cmd.since(st['certain_since'] + p['response_limit_s'])
                            if not cmd.is_zero]
                    if late:
                        raise self._abort(f'protective stop failed: obstacle more than one grid cell '
                                          f'inside the {cm} m boundary and forward commands still reached '
                                          f'the robot {late[-1] - st["certain_since"]:.2f} s later')
                # ---- end of the walk
                if st['t_stop'] is None:
                    if elapsed >= p['duration_s']:
                        st['end'] = 'TIME_LIMIT'
                        break
                    if fwd >= p['max_travel_m'] * ODOMETRY_TRUST or (
                            st['map_travel'] is not None and st['map_travel'] >= p['max_travel_m']):
                        st['end'] = 'TRAVEL_LIMIT'
                        break
                else:
                    if st['layer'] != 'collision_monitor':
                        st['end'] = 'STOPPED'
                        break
                    if now - st['t_stop'] >= p['suppression_proof_s']:
                        st['end'] = 'STOPPED'
                        break
                    # Keep proving suppression ONLY while the obstacle is freshly seen inside the
                    # boundary (as M3 re-confirms its lock before every command).
                    # Judge only scans taken AFTER the stop: the collision_monitor reacts to its
                    # hazard cloud slightly before the status scan arrives (seen physically).
                    post = [dd for tt, dd in st['scans'] if tt > st['t_stop']]
                    if (post and (post[-1] is None or post[-1] > cm + 1e-9)) or \
                            now - st['scan_stamp'] > p['max_sensor_age_s']:
                        st['proof_cut'] = _r(now - st['t_stop'])
                        st['end'] = 'STOPPED'
                        break
                if now - last_line >= 1.0:
                    last_line = now
                    self._m4_line(now, t0, st, p, fwd, tracker, d)
                self._send(p['vx'], 0.0)
                st['sends'] += 1
                if st['t_walk'] is None:
                    st['t_walk'] = now
                if st['t_stop'] is not None:
                    st['sends_after_stop'] += 1
                last_send = self.robot.now()
                self.robot.wait(period)
        except Exception as error:        # noqa: BLE001 - SafetyAbort or anything else: stop first
            aborted = error
        command_end = self.robot.now()
        # ---- STOPPING: actively commanded zeros (priority 255) while locomotion stop is verified
        t_ref = st['t_stop'] if st['t_stop'] is not None else command_end
        hold = self.seq.safety.stop_hold_s
        onset = None
        deadline = t_ref + p['stop_within_s'] + hold
        while True:
            try:
                self._stop_command()
            except Exception:  # noqa: BLE001 - keep observing, the runner's safe stop follows
                pass
            now = self.robot.now()
            stopped, sdetail = assess_locomotion_stopped(tel, now, hold, self.seq.locomotion_stop)
            if stopped:
                onset = max(t_ref, now - hold)
                break
            if now >= max(deadline, command_end + hold) or self.robot.cancelled():
                break
            self.robot.wait(period)
        if onset is not None and aborted is None and st['state'] == STOPPING:
            enter(VERIFIED, stopped_moving_after_s=_r(onset - t_ref))
        self._zero_burst(0.5)
        detail = self._m4_evaluate(p, pre, st, tracker, t0, command_end, t_ref, onset, start, aborted)
        if aborted is not None:
            from .runner import SafetyAbort
            reason = str(aborted) if isinstance(aborted, SafetyAbort) else \
                f'internal error {type(aborted).__name__}: {aborted}'
            raise SafetyAbort(f'{detail["outcome"]}: {reason}', detail=detail)
        status = self.PASSED if detail['outcome'] == 'STOPPED_FOR_OBSTACLE' else self.FAILED
        if status == self.FAILED and onset is None:
            self.uncertain = True
        return status, detail['summary'], detail, self._trace(t0)

    # ------------------------------------------------------------------ evaluation
    def _m4_evaluate(self, p, pre, st, tracker, t0, command_end, t_ref, onset, start, aborted):
        tel = self.robot.telemetry
        cm = pre['collision_monitor_boundary_m']
        c, s_ = math.cos(start.yaw), math.sin(start.yaw)

        def fwd_at(stamp):
            items = tel.odom.since(t0)
            if not items:
                return None
            o = min(items, key=lambda i: abs(i[0] - stamp))[1]
            return (o.x - start.x) * c + (o.y - start.y) * s_

        def nose_at(stamp):
            items = [(ts, self._nose_distance(v)) for ts, v in tel.terrain.since(t0) if ts <= stamp + 1e-9]
            items = [(ts, v) for ts, v in items if v is not None]
            return items[-1][1] if items else None

        end = self.robot.now()
        final = [self._nose_distance(v) for _, v in tel.terrain.window(end, 0.5)]
        final = [v for v in final if v is not None]
        final_d = min(final) if final else None
        travelled = fwd_at(end)
        nonzero = self._robot_nonzero_since(st['t_walk'] or t0)
        first_delivery = nonzero[0][0] if nonzero else None
        latency = (first_delivery - st['t_walk']) if first_delivery and st['t_walk'] else None
        response = None
        if st['t_cross'] is not None:
            after = [ts for ts, _ in nonzero if ts >= st['t_cross']]
            response = max(0.0, (after[-1] - st['t_cross']) if after else 0.0)
        chatter = []
        if st['t_stop'] is not None:
            for ts, _ in nonzero:
                if ts <= st['t_stop'] + 0.1:          # in flight when the stop began
                    continue
                prior = [dd for tt, dd in st['scans'] if tt <= ts]
                if prior and prior[-1] is not None and prior[-1] <= cm + 1e-9:
                    chatter.append(ts)
        zeros_while_forward = [ts for ts, cmd in tel.system_cmd.since(st['t_stop'] or end)
                               if cmd.is_zero and any(not m.is_zero for _, m in tel.mux_out.window(ts, 0.1))]
        cm_reasons = sorted({e[1] for _, e in tel.cm_events.since(st['t_walk'] or t0) if e[0] == 'stop'})
        moved_before_stop = fwd_at(st['t_stop']) if st['t_stop'] else travelled
        stop_distance = None
        if st['t_stop'] is not None and onset is not None:
            a, b = fwd_at(st['t_stop']), fwd_at(onset)
            stop_distance = None if a is None or b is None else b - a
        valid_response = min(p['response_limit_s'], p['path_latency_s'] + 0.05)
        detail = {
            **pre, 'states': st['transitions'], 'final_state': st['state'],
            'parameters': {k: p[k] for k in sorted(p)},
            'stop_layer': st['layer'], 'collision_monitor_reasons': cm_reasons,
            'walk_end': st['end'], 'proof_cut_short_after_s': st['proof_cut'],
            'tracking': tracker.as_dict(),
            'obstacle_detected_at_m': _r(st['d_det']),
            'obstacle_detected_after_s': _r(st['t_det'] - t0) if st['t_det'] else None,
            'boundary_crossed_after_s': _r(st['t_cross'] - t0) if st['t_cross'] else None,
            'stop_after_s': _r(st['t_stop'] - t0) if st['t_stop'] else None,
            'nose_distance_at_stop_m': _r(nose_at(st['t_stop'])) if st['t_stop'] else None,
            'forward_commands_published': st['sends'],
            'forward_commands_after_stop': st['sends_after_stop'],
            'robot_zeros_while_input_forward': len(zeros_while_forward),
            'forward_at_robot_after_stop_inside_boundary': len(chatter),
            'response_s': _r(response), 'response_limit_s': _r(valid_response),
            'command_latency_s': _r(latency), 'assumed_path_latency_s': p['path_latency_s'],
            'stopped_moving_after_s': _r(onset - t_ref) if onset is not None else None,
            'stopping_distance_m': _r(stop_distance),
            'distance_travelled_m': _r(travelled), 'travelled_before_stop_m': _r(moved_before_stop),
            'map_travel_m': _r(st['map_travel']),
            'min_nose_distance_m': _r(st['min_nose']), 'final_nose_distance_m': _r(final_d),
            'walk_duration_s': _r(command_end - t0),
        }
        criteria = []

        def need(ok, text):
            criteria.append({'ok': bool(ok), 'criterion': text})

        if aborted is not None:
            outcome = classify_abort(str(aborted))
        elif st['end'] in ('TIME_LIMIT', 'TRAVEL_LIMIT'):
            outcome = st['end']
        elif st['layer'] == 'terrain_backup_lock':
            outcome = 'BACKUP_LOCK_STOP'
        elif st['layer'] == 'command_path_silent':
            outcome = 'UNEXPLAINED_STOP'
        else:
            d_stop = detail['nose_distance_at_stop_m']
            need((moved_before_stop or 0.0) >= 0.05,
                 f'robot moved forward >= 0.05 m before the stop ({_r(moved_before_stop)} m)')
            need(st['t_det'] is not None and st['t_det'] <= st['t_stop'],
                 f'obstacle detected and tracked during the approach (at {_r(st["d_det"])} m)')
            need(d_stop is not None and d_stop <= cm + p['sensor_margin_m'],
                 f'stop caused by the tracked obstacle at the boundary (obstacle {d_stop} m at the stop, '
                 f'boundary {cm} m + {p["sensor_margin_m"]} m)')
            need(not any('PolygonStop' not in r for r in cm_reasons),
                 f'collision_monitor stop reason is the PolygonStop polygon ({cm_reasons or "log not received"})')
            need(st['sends_after_stop'] >= 5 and st['proof_cut'] is None and len(zeros_while_forward) >= 3,
                 f'forward commands kept being published after the stop for {p["suppression_proof_s"]} s and '
                 f'were zeroed at the robot ({st["sends_after_stop"]} published, '
                 f'{len(zeros_while_forward)} robot-side zeros'
                 + (f'; proof cut short after {st["proof_cut"]} s' if st['proof_cut'] is not None else '') + ')')
            need(not chatter,
                 'no forward command reached the robot after the stop while the obstacle read inside the '
                 f'boundary ({len(chatter)} did: the collision_monitor released with the obstacle on its '
                 'polygon edge - 5 cm terrain grid)' if chatter else
                 'no forward command reached the robot after the stop while the obstacle read inside the boundary')
            need(response is not None and response <= valid_response,
                 f'unsafe commands suppressed within {valid_response:.2f} s of the obstacle crossing the '
                 f'boundary (measured {_r(response)} s)')
            need(latency is not None and latency <= p['path_latency_s'],
                 f'command latency {_r(latency)} s <= {p["path_latency_s"]} s assumed by the stopping '
                 'boundary')
            need(onset is not None and onset - t_ref <= p['stop_within_s'],
                 f'robot stopped moving within {p["stop_within_s"]} s of the stop '
                 f'({detail["stopped_moving_after_s"]} s)')
            need(st['min_nose'] >= p['min_clearance_m'] and final_d is not None and
                 final_d >= p['min_clearance_m'],
                 f'minimum and final clearance >= {p["min_clearance_m"]} m (min {_r(st["min_nose"])} m, '
                 f'final {_r(final_d)} m)')
            outcome = 'STOPPED_FOR_OBSTACLE' if all(x['ok'] for x in criteria) else 'STOP_CRITERIA_FAILED'
        detail['criteria'] = criteria
        detail['outcome'] = outcome
        detail['outcome_explanation'] = OUTCOMES[outcome]
        failed = [x['criterion'] for x in criteria if not x['ok']]
        if outcome == 'STOPPED_FOR_OBSTACLE':
            summary = (f'STOPPED_FOR_OBSTACLE: obstacle detected at {_r(st["d_det"], 2)} m, '
                       f'collision_monitor stopped the robot at {detail["nose_distance_at_stop_m"]} m after '
                       f'{_r(moved_before_stop, 2)} m; forward commands suppressed in {_r(response, 2)} s; '
                       f'stopped moving {detail["stopped_moving_after_s"]} s later after '
                       f'{_r(stop_distance, 3)} m; final clearance {_r(final_d, 2)} m (min '
                       f'{_r(st["min_nose"], 2)} m, required >= {p["min_clearance_m"]} m)')
        elif outcome in ('TIME_LIMIT', 'TRAVEL_LIMIT'):
            seen = (f'obstacle tracked at {_r(tracker.last[1], 2)} m but never reached the {cm} m boundary'
                    if tracker.is_confirmed else 'no obstacle detected in the '
                    f'{p["detection_range_m"]} m detection region')
            summary = (f'{outcome}: {OUTCOMES[outcome]}. Walked {_r(travelled, 2)} m in '
                       f'{detail["walk_duration_s"]} s; {seen}')
        elif outcome == 'STOP_CRITERIA_FAILED':
            summary = f'{outcome}: ' + '; '.join(failed)
        elif aborted is not None:
            summary = f'{outcome}: {aborted}'
        else:
            summary = f'{outcome}: {OUTCOMES[outcome]}'
        detail['summary'] = summary
        return detail

    # ------------------------------------------------------------------ dry shadow
    def step_proximity_watch(self, step):
        """No motion, nothing published: run the M4 detection/stop logic on live sensing while an
        object is moved toward the standing robot. Reports where the obstacle is confirmed, when
        the collision_monitor boundary would trigger, whether the backup lock engages and the
        sensor health, plus whether a physical M4 would currently be allowed."""
        p = step.params
        blockers, ready = self._m4_readiness(p, require_standing=False)
        tel = self.robot.telemetry
        cm = ready['collision_monitor_boundary_m'] or p['stop_distance_m']
        tracker = ObstacleTracker(p['detection_range_m'], p['detect_scans'], p['track_tolerance_m'],
                                  p['lost_track_scans'], ODOMETRY_TRUST, check_consistency=False)
        t0 = self.robot.now()
        info = {'t_det': None, 'd_det': None, 't_cross': None, 't_lock': None, 'faults': [],
                'lock_failures': 0, 'scan': None, 'min_nose': math.inf}
        last_line = t0
        held = None
        self.log('    SHADOW (dry): the robot does not move and nothing is published. Move an object '
                 'slowly toward the robot along its heading until it is inside the boundary.')
        while self.robot.now() - t0 < p['window_s']:
            if self.robot.cancelled():
                from .runner import SafetyAbort
                raise SafetyAbort('operator cancelled (signal)')
            now = self.robot.now()
            fault = self._m4_sensor_fault(p, now, now - t0)
            if fault and (not info['faults'] or info['faults'][-1][1] != fault):
                info['faults'].append((_r(now - t0), fault))
            item = tel.terrain.latest()
            d = None
            if item is not None and item[0] != info['scan']:
                info['scan'] = item[0]
                d = self._nose_distance(item[1])
                if d is not None:
                    info['min_nose'] = min(info['min_nose'], d)
                if tracker.update(item[0], d, 0.0) == 'confirmed':
                    info.update(t_det=item[0], d_det=d)
                    self.log(f'    -> {DETECTED} at {d:.2f} m')
                if d is not None and tracker.is_confirmed and d <= cm and info['t_cross'] is None:
                    info['t_cross'] = item[0]
                    self.log(f'    -> would STOP: obstacle {d:.2f} m <= boundary {cm} m '
                             '(collision_monitor trigger)')
                lock = tel.terrain_stop.latest()
                if lock and lock[1] and info['t_cross'] is not None and info['t_lock'] is None:
                    info['t_lock'] = lock[0]
                if d is not None and d <= p['backup_stop_distance_m'] - p['sensor_margin_m'] and \
                        lock and not lock[1]:
                    info['lock_failures'] += 1
            if info['t_cross'] is not None:
                held = held or now
                if now - held >= 1.0:
                    break
            if now - last_line >= 1.0:
                last_line = now
                lock = tel.terrain_stop.latest()
                shown = tracker.last[1] if tracker.last else None
                self.log(f'    [{now - t0:5.1f}s] shadow | obstacle '
                         f'{"none in range" if shown is None else "%.2f m" % shown} '
                         f'({"tracked" if tracker.is_confirmed else "not confirmed"}) | would stop at '
                         f'{cm} m | backup lock {"STOP" if lock and lock[1] else "clear"} | LiDAR '
                         f'{len(tel.terrain.window(now, 1.0))} Hz | sensors '
                         f'{"OK" if not fault else fault}')
            self.robot.wait(0.05)
        problems = []
        if info['t_det'] is None:
            problems.append('no obstacle was confirmed in the detection region')
        elif info['t_cross'] is None:
            problems.append(f'the obstacle never reached the {cm} m trigger distance '
                            f'(closest {_r(info["min_nose"])} m)')
        if info['faults']:
            problems.append(f'sensor faults during the watch: {info["faults"]}')
        if info['lock_failures'] >= 2:
            problems.append('proximity-lock failure: obstacle inside the backup stop but lock input false')
        detail = {'physical_m4_blockers': blockers, 'readiness': ready, 'tracking': tracker.as_dict(),
                  'detected_at_m': _r(info['d_det']),
                  'detected_after_s': _r(info['t_det'] - t0) if info['t_det'] else None,
                  'would_stop_after_s': _r(info['t_cross'] - t0) if info['t_cross'] else None,
                  'boundary_m': cm, 'min_nose_distance_m': _r(info['min_nose']),
                  'backup_lock_after_trigger_s': _r(info['t_lock'] - info['t_cross'])
                  if info['t_lock'] and info['t_cross'] else None,
                  'sensor_faults': info['faults'], 'problems': problems}
        if problems:
            return self.FAILED, 'shadow: ' + '; '.join(problems), detail
        return (self.PASSED, f'shadow: obstacle confirmed at {_r(info["d_det"], 2)} m, would trigger the '
                f'protective stop at {cm} m after {detail["would_stop_after_s"]} s; sensors healthy'
                + (f'; physical M4 blockers now: {blockers}' if blockers else '; physical M4 READY'),
                detail)


def suggest_clearance(boundary, need, current):
    """Navigation --clearance that raises the boundary to at least ``need`` (same calibration)."""
    base = current if current is not None else 0.15
    return round(base + math.ceil((need - boundary) * 100 - 1e-9) / 100, 2)
