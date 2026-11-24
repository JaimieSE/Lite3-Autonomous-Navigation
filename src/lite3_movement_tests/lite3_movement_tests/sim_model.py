"""Simplified Lite3 stand-in for software validation only. Not a physics model.

It reproduces the observable telemetry contract of the deployed bridge
(12 named joints, /leg_odom2, /imu/data at 200 Hz; odometry pose frozen while
lying) and the posture *toggle* semantics of ``0x21010202``. Faults let tests
prove that the runner fails safely; simulated results never count as physical
evidence.
"""

import math
import random

from .geometry import JOINT_NAMES
from .telemetry import TwistSample

# Mean of a live read-only sample on 2026-11-13 while the robot lay down.
LYING_POSE = (0.4497, 1.1513, -2.7500, -0.4456, 1.1638, -2.7427,
              0.4465, 1.1387, -2.7465, -0.4384, 1.1237, -2.7425)
# Nominal stand geometry (|knee| 1.65 rad -> 0.278 m hip-to-foot); simulation only.
STANDING_POSE = (0.05, 0.85, -1.65, -0.05, 0.85, -1.65,
                 0.05, 0.85, -1.65, -0.05, 0.85, -1.65)

FAULTS = ('none', 'march_in_place', 'ignore_posture', 'no_stop', 'telemetry_dropout',
          'tilt', 'remote_takeover', 'slow_stand', 'runaway', 'no_silence_stop', 'slow_silence_stop',
          'settle_twitch', 'settle_twitch_large')
# settle_twitch reproduces the physical M2 signature of 2026-11-16: after command silence the robot
# keeps its last command ~1.2 s (stepping ends ~1.7 s after silence), then ~1.1 s after stepping
# ends the body makes a short settling motion (yaw ~0.08 rad/s, small joint shift).


class SimLite3:
    TRANSITION_S = 2.0
    COMMAND_TIMEOUT_S = 0.5

    def __init__(self, fault='none', start='lying', seed=1):
        if fault not in FAULTS:
            raise ValueError(f'unknown fault {fault}')
        self.fault = fault
        self.rng = random.Random(seed)
        self.blend = 1.0 if start == 'standing' else 0.0   # 0 lying, 1 standing
        self.target = self.blend
        self.transition_rate = 1.0 / (self.TRANSITION_S * (8 if fault == 'slow_stand' else 1))
        self.cmd = (0.0, 0.0)
        self.cmd_time = -math.inf
        self.last_nonzero = (0.0, 0.0)
        self.last_nonzero_time = -math.inf
        self.v = [0.0, 0.0]          # forward, yaw rate
        self.x = self.y = self.yaw = 0.0
        self.gait_until = -math.inf
        self.gait_start = None
        self.t = 0.0
        self.toggles = 0
        self.velocity_messages = 0
        self.twitch_at = None
        self.twitch_for = None

    # Commands -----------------------------------------------------------
    def command_velocity(self, t, vx, wz):
        self.velocity_messages += 1
        self.cmd, self.cmd_time = (vx, wz), t
        if vx or wz:
            self.last_nonzero, self.last_nonzero_time = (vx, wz), t

    def command_toggle(self, t):
        self.toggles += 1
        if self.fault == 'ignore_posture' or self.target != self.blend:
            return
        if abs(self.v[0]) > 0.01 or abs(self.v[1]) > 0.01:
            return  # firmware-like refusal while moving (assumed)
        self.target = 0.0 if self.blend >= 1.0 else 1.0

    # Dynamics -----------------------------------------------------------
    def effective_command(self, t):
        if self.fault == 'no_stop' and t - self.last_nonzero_time < 30.0:
            return self.last_nonzero
        if self.fault == 'slow_silence_stop':   # like the real robot: keeps going ~1.6 s
            return self.cmd if t - self.cmd_time < 1.6 else (0.0, 0.0)
        if self.fault.startswith('settle_twitch'):
            return self.cmd if t - self.cmd_time < 1.2 else (0.0, 0.0)
        if self.fault == 'no_silence_stop':
            return self.cmd   # holds the last command forever when commands stop arriving
        return self.cmd if t - self.cmd_time < self.COMMAND_TIMEOUT_S else (0.0, 0.0)

    def advance(self, t):
        dt = max(0.0, t - self.t)
        self.t = t
        if self.blend != self.target:
            step = self.transition_rate * dt
            self.blend = min(self.target, self.blend + step) if self.target > self.blend else \
                max(self.target, self.blend - step)
        vx, wz = self.effective_command(t) if self.blend >= 1.0 else (0.0, 0.0)
        if vx or wz:
            if self.gait_start is None:
                self.gait_start = t
            self.gait_until = t + 0.5
        elif t > self.gait_until:
            self.gait_start = None
        if self.fault.startswith('settle_twitch') and not self.gait_active and self.gait_start is None \
                and self.gait_until > -math.inf and self.twitch_for != self.gait_until:
            self.twitch_for, self.twitch_at = self.gait_until, self.gait_until + 1.1
        scale = {'march_in_place': 0.0, 'runaway': 20.0}.get(self.fault, 1.0)
        alpha = min(1.0, dt / 0.3)
        self.v[0] += alpha * (scale * vx - self.v[0])
        self.v[1] += alpha * (scale * wz - self.v[1])
        if self.twitching:
            self.yaw += self.twitch_rate * dt
        if self.blend >= 1.0:   # the real leg odometry froze while lying
            self.x += self.v[0] * math.cos(self.yaw) * dt
            self.y += self.v[0] * math.sin(self.yaw) * dt
            self.yaw = math.atan2(math.sin(self.yaw + self.v[1] * dt), math.cos(self.yaw + self.v[1] * dt))

    @property
    def twitch_rate(self):
        return 0.4 if self.fault == 'settle_twitch_large' else 0.08

    @property
    def twitching(self):
        return self.twitch_at is not None and self.twitch_at <= self.t < self.twitch_at + 0.25

    @property
    def gait_active(self):
        return self.t <= self.gait_until

    # Telemetry ----------------------------------------------------------
    def joints(self):
        if self.fault == 'telemetry_dropout' and self.gait_start is not None and \
                self.t - self.gait_start > 0.5:
            return None
        pose = [a + (b - a) * self.blend for a, b in zip(LYING_POSE, STANDING_POSE)]
        if self.gait_active:
            phase = math.sin(2 * math.pi * 2.0 * self.t)
            for leg in range(4):
                sign = 1 if leg in (0, 3) else -1
                pose[leg * 3 + 1] += 0.15 * sign * phase
                pose[leg * 3 + 2] -= 0.12 * max(0.0, sign * phase)
        if self.twitching:
            pose[1] += 0.06
        return JOINT_NAMES, [p + self.rng.uniform(-4e-4, 4e-4) for p in pose]

    def odom(self):
        noise = lambda: self.rng.uniform(-0.008, 0.008)  # noqa: E731
        stepping = 0.03 if self.gait_active else 0.0
        return (self.x, self.y, 0.3227, self.yaw,
                self.v[0] + noise() + stepping * math.sin(9 * self.t), noise(),
                self.v[1] + noise() + (self.twitch_rate if self.twitching else 0.0))

    def imu(self):
        roll = 0.45 if self.fault == 'tilt' and self.gait_start is not None and \
            self.t - self.gait_start > 0.8 else 0.0005
        pitch = 0.003
        cy, sy = math.cos(self.yaw / 2), math.sin(self.yaw / 2)
        cr, sr = math.cos(roll / 2), math.sin(roll / 2)
        cp, sp = math.cos(pitch / 2), math.sin(pitch / 2)
        quat = (sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy,
                cr * cp * sy - sr * sp * cy, cr * cp * cy + sr * sp * sy)
        gyro_noise = 0.004 + (0.08 if self.gait_active else 0.0)
        gyro = tuple(self.rng.uniform(-gyro_noise, gyro_noise) for _ in range(3))
        acc = (self.rng.uniform(-0.06, 0.06), self.rng.uniform(-0.05, 0.05),
               9.76 + self.rng.uniform(-0.04, 0.04))
        return quat, gyro, acc

    def handle(self):
        if self.fault == 'remote_takeover' and self.gait_start is not None and \
                self.t - self.gait_start > 0.5:
            return (0.6, 0.0, 0.0)
        return (0.0, 0.0, 0.0)


class FakeRobot:
    """Virtual-clock robot interface used by unit tests (no ROS)."""

    can_actuate = True

    def __init__(self, telemetry, model, rate_hz=200.0, healthy=True, can_actuate=True):
        self.telemetry = telemetry
        self.model = model
        self.t = 100.0
        self.dt = 1.0 / rate_hz
        self.healthy = healthy
        self.can_actuate = can_actuate
        self.cancel = False
        self.sent = []
        self._publish(self.t)
        self.wait(2.5)

    def _publish(self, t):
        m = self.model
        m.advance(t)
        joints = m.joints()
        if joints:
            self.telemetry.on_joint(t, *joints)
        x, y, z, yaw, vx, vy, wz = m.odom()
        self.telemetry.on_odom(t, x, y, z, 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2), vx, vy, wz)
        quat, gyro, acc = m.imu()
        self.telemetry.on_imu(t, *quat, gyro, acc)
        self.telemetry.on_handle(t, *m.handle())
        for name in ('imu', 'joint', 'odom2', 'odom', 'ultrasound'):
            self.telemetry.on_sensor_alive(t, name, True)

    def now(self):
        return self.t

    def wait(self, seconds):
        end = self.t + max(seconds, 0.0)
        while self.t + self.dt <= end + 1e-12:
            self.t += self.dt
            self._publish(self.t)
        self.t = end

    def send_velocity(self, vx, wz):
        if not self.can_actuate:
            raise RuntimeError('read-only interface')
        self.sent.append((self.t, 'vel', vx, wz))
        self.model.command_velocity(self.t, vx, wz)
        self.telemetry.on_system_cmd(self.t, vx, 0.0, wz)

    def send_posture_toggle(self):
        if not self.can_actuate:
            raise RuntimeError('read-only interface')
        self.sent.append((self.t, 'toggle'))
        self.model.command_toggle(self.t)
        return {'published': True, 'matched_subscribers': 1, 'observed_in_system_domain': True}

    def send_simple_command(self, code, value=0):
        if not self.can_actuate:
            raise RuntimeError('read-only interface')
        self.sent.append((self.t, 'simple', code, value))
        return {'published': True, 'matched_subscribers': 1, 'observed_in_system_domain': True}

    def send_axis(self, code, value):
        self.sent.append((self.t, 'axis', code, value))
        if code == 0x21010130:                    # simulated: axis forward walks like a speed command
            self.model.command_velocity(self.t, 0.05 if abs(value) > 6553 else 0.0, 0.0)

    def send_gait(self, name):
        if not self.can_actuate:
            raise RuntimeError('read-only interface')
        self.sent.append((self.t, 'gait', name))
        return {'published': True, 'matched_subscribers': 1, 'observed_in_system_domain': True}

    def command_path_health(self):
        return self.healthy, {'problems': [] if self.healthy else ['competing publisher']}

    def cancelled(self):
        return self.cancel


# ---------------------------------------------------------------------------------------------
# Navigation-mode stand-in (unit tests only). Models the twist_mux priorities/locks of
# params/twist_mux_physical.yaml, the dead-man, a straight-line "Nav2" and the terrain
# monitor's proximity stop over point obstacles. Not a planner and not physics.
NAV_FAULTS = ('none', 'mux_leak', 'deadman_ignored', 'nav_reverse', 'nav_overspeed', 'rogue_goal',
              'lock_late', 'hazard_stale', 'detection_dropout', 'blind_after_start', 'cm_disabled',
              'cm_missing', 'cap_unverified', 'terrain_stale_walk', 'lidar_slow_walk',
              'imu_invalid_walk', 'cm_slow', 'deadman_lapse_walk', 'cm_edge_chatter',
              'stop_unmatched', 'odom_lag')
# Humble MPPI is modelled as it behaves on the robot: a goal resets the optimizer, so a
# /speed_limit published BEFORE the goal is lost (M5 overspeed 2026-11-16); the velocity_smoother
# max_velocity (set_nav_velocity_cap) clamps Nav2's output regardless.
# M4 faults: terrain_stale_walk (terrain status stops 2 s into the walk), lidar_slow_walk (5 Hz
# from 2 s), imu_invalid_walk (orientation invalid from 2 s), cm_slow (collision_monitor zeroes
# only 0.6 s after the obstacle enters its polygon), deadman_lapse_walk (the mux dead-man lock
# engages 3 s into the walk although the runner keeps feeding it), cm_edge_chatter (physical
# 2026-11-17 12:57: obstacle resting on the polygon edge; the collision_monitor releases for 0.2 s,
# 0.35 s after it stopped, then stops again).
PRIORITY = {'stop': 255, 'test': 20, 'nav': 10}
TIMEOUT = {'stop': 0.25, 'test': 0.5, 'nav': 0.5}


def box_points(x, y, size=0.3, step=0.05):
    """Points on the outline of a square obstacle centred at (x, y) in the world frame."""
    half, pts = size / 2, []
    n = int(size / step) + 1
    for i in range(n):
        a = -half + i * step
        pts += [(x - half, y + a), (x + half, y + a), (x + a, y - half), (x + a, y + half)]
    return pts


class FakeNavRobot(FakeRobot):
    def __init__(self, telemetry, model, nav_fault='none', obstacles=(), cm_boundary=0.37,
                 cm_approach_s=None, **kwargs):
        self.faults = set(nav_fault.split('+'))
        if self.faults - set(NAV_FAULTS):
            raise ValueError(f'unknown nav fault {nav_fault}')
        self.nav_fault = nav_fault
        self.cm_boundary = cm_boundary            # collision_monitor stop polygon, nose distance
        self.cm_approach_s = cm_approach_s        # optional approach action (stacks above 0.10 m/s)
        self.cm_stopping = False
        self.obstacles = list(obstacles)
        self.inputs = {}
        self.last_feed = -math.inf
        self.stop_ahead = 0.505
        self.body_margin = 0.05                   # terrain.StopParams.body_margin_m
        self.last_hazard = -math.inf
        self.terrain_stop = False
        self.speed_limit = 0.0
        self.smoother_cap = 0.10                  # velocity_smoother max_velocity[0]
        self.obstacle_velocity = (0.0, 0.0)       # world m/s, applied after the first test command
        self.obstacle_delay = 0.0
        self.late_obstacles = []                  # [(seconds after first test command, points)]
        self.obstacle_motion_from = None          # override: obstacle motion clock start (shadow)
        self.cm_inside_since = None
        self.goal = None
        self.goal_sent = False
        self.first_test_cmd = None
        self.goal_state = {'result': None, 'recoveries': 0, 'blocked_since': None}
        self.next_nav = self.next_terrain = 0.0
        self.mux_log = []
        super().__init__(telemetry, model, **kwargs)

    # ---- simulated graph ----
    def _locked(self):
        hold = 'deadman_ignored' not in self.faults and self.t - self.last_feed > 0.5
        hold = hold or self._walk_fault('deadman_lapse_walk', after=3.0)
        return hold          # (terrain proximity lock removed 2026-11-24, as on the robot)

    def _mux(self, source, vx, wz):
        self.inputs[source] = (self.t, vx, wz)
        live = [(PRIORITY[s], s) for s, (ts, _, _) in self.inputs.items() if self.t - ts <= TIMEOUT[s]]
        if max(live)[1] != source or (PRIORITY[source] < 50 and self._locked()):
            return
        self.telemetry.mux_out.add(self.t, TwistSample(vx, 0.0, wz))
        # nav2_collision_monitor (Humble): STOP polygon over the hazard points -> zeros.
        if 'cm_disabled' not in self.faults:
            front = 0.305 + self.cm_boundary
            inside = any(-0.355 <= x <= front and abs(y) <= 0.235 for x, y in self._base_points())
            if inside and 'cm_slow' in self.faults:
                self.cm_inside_since = self.cm_inside_since or self.t
                inside = self.t - self.cm_inside_since >= 0.6
            elif not inside:
                self.cm_inside_since = None
            if inside and 'cm_edge_chatter' in self.faults:
                self.cm_first_stop = getattr(self, 'cm_first_stop', None) or self.t
                if 0.35 <= self.t - self.cm_first_stop < 0.55:
                    inside = False
            if inside != self.cm_stopping:
                self.cm_stopping = inside
                self.telemetry.cm_events.add(self.t, ('stop', 'Robot to stop due to PolygonStop polygon')
                                             if inside else ('continue', 'Robot to continue normal operation'))
            if inside:
                vx, wz = 0.0, 0.0
            elif self.cm_approach_s and vx > 0:
                # Humble approach: footprint simulated along the command; forward motion is scaled so the
                # predicted collision is no sooner than time_before_collision. Turning in place is free.
                ahead = [x - 0.305 for x, y in self._base_points() if x > 0.305 and abs(y) <= 0.185]
                if ahead and min(ahead) / vx < self.cm_approach_s:
                    vx = max(0.0, min(ahead)) / self.cm_approach_s
        self.mux_log.append((self.t, source, vx, wz))
        self.model.command_velocity(self.t, vx, wz)
        self.telemetry.on_system_cmd(self.t, vx, 0.0, wz)

    def _base_points(self):
        if 'detection_dropout' in self.faults and self.first_test_cmd is not None and \
                self.first_test_cmd + 0.3 <= self.t < self.first_test_cmd + 1.8:
            return []                       # object not detected for 1.5 s (> 1 s release hold)
        if 'blind_after_start' in self.faults and self.first_test_cmd is not None:
            return []                       # LiDAR detection fails completely once motion starts
        m = self.model
        c, s = math.cos(m.yaw), math.sin(m.yaw)
        obstacles = list(self.obstacles)
        t_ref = self.obstacle_motion_from if self.obstacle_motion_from is not None else \
            self.first_test_cmd
        if t_ref is not None and self.t >= t_ref:
            dt = max(0.0, self.t - t_ref - self.obstacle_delay)
            ox, oy = self.obstacle_velocity[0] * dt, self.obstacle_velocity[1] * dt
            obstacles = [(px + ox, py + oy) for px, py in obstacles]
            for after, pts in self.late_obstacles:
                if self.t - t_ref >= after:
                    obstacles += list(pts)
        return [((px - m.x) * c + (py - m.y) * s, -(px - m.x) * s + (py - m.y) * c)
                for px, py in obstacles]

    def _walk_fault(self, name, after=2.0):
        return name in self.faults and self.first_test_cmd is not None and \
            self.t - self.first_test_cmd >= after

    def _publish(self, t):
        super()._publish(t)
        if not hasattr(self, 'next_terrain'):
            return
        if self._walk_fault('imu_invalid_walk'):
            self.telemetry.on_imu(t, 0.0, 0.0, 0.0, 0.0, (0.0, 0.0, 0.0), (0.0, 0.0, 9.8))
        if t >= self.next_terrain:
            self.next_terrain = t + (0.2 if self._walk_fault('lidar_slow_walk') else 0.1)
            if self._walk_fault('terrain_stale_walk'):
                return
            pts = self._base_points()
            ahead = [x for x, y in pts if 0 < x <= 3.0 and abs(y) <= 0.235]   # terrain corridor
            nearest = min(ahead) if ahead else None
            trigger = self.stop_ahead - (0.15 if 'lock_late' in self.faults else 0.0)
            hazard = (nearest is not None and nearest <= trigger) or any(
                abs(x) <= 0.305 + self.body_margin and abs(y) <= 0.185 + self.body_margin for x, y in pts)
            if hazard:
                self.last_hazard = t
            self.terrain_stop = t - self.last_hazard < 1.0
            # The straight-line "Nav2" below does not plan round obstacles: it stops for one straight ahead (the real
            # Nav2 avoids it in its costmap). Walls beside the body no longer stop it (terrain lock removed 2026-11-24).
            self.nav_blocked = nearest is not None and nearest <= trigger
            reason = None
            if hazard:
                reason = (f'hazard {nearest:.2f} m ahead' if nearest is not None and nearest <= trigger
                          else f'hazard within {self.body_margin} m of the body')
            self.telemetry.on_terrain(t, self.terrain_stop, reason if hazard else
                                      ('release hold' if self.terrain_stop else None),
                                      nearest, self.stop_ahead, (), 0.03)
            self.telemetry.terrain_stop.add(t, self.terrain_stop)
            if not ('hazard_stale' in self.faults and self.goal_sent):
                from .nav_steps import footprint_clearance
                self.telemetry.on_clearance(t, footprint_clearance(pts))
            self.telemetry.lidar_odom.add(t, 1.5 if 'odom_lag' in self.faults else 0.15)
            self.telemetry.hazard_age.add(t, getattr(self, 'hazard_age_s', 0.3))   # tests may raise it
        if t >= self.next_nav:
            self.next_nav = t + 0.05
            self._nav_tick()

    def _spin_tick(self):
        """behavior_server Spin: constant 0.2 rad/s in place; progress is the yaw the body really
        turned (a lock or the collision_monitor zeroing the output stalls it, as on the robot)."""
        sp, m = self.spin, self.model
        turned = math.atan2(math.sin(m.yaw - sp['last_yaw']), math.cos(m.yaw - sp['last_yaw']))
        sp['last_yaw'] = m.yaw
        sp['done'] += abs(turned)
        self.goal_state['spun_rad'] = sp['done']
        if sp['done'] >= abs(sp['target']) - 0.02:
            self.goal_state['result'] = 'SUCCEEDED'
            self.spin = None
            self._mux('nav', 0.0, 0.0)
            return
        if self.t - sp['start'] > sp['allowance']:
            self.goal_state['result'] = 'ABORTED'
            self.spin = None
            return
        wz = math.copysign(0.2, sp['target'])
        self.telemetry.on_nav_cmd(self.t, 0.0, 0.0, wz)
        self._mux('nav', 0.0, wz)

    def _nav_tick(self):
        if getattr(self, 'spin', None):
            self._spin_tick()
            return
        if 'rogue_goal' in self.faults and self.model.blend >= 1.0 and self.goal is None \
                and self.t > 110:
            self.telemetry.on_nav_cmd(self.t, 0.05, 0.0, 0.0)
            self._mux('nav', 0.05, 0.0)
            return
        if self.goal is None or self.goal_state['result']:
            return
        m, (gx, gy, gyaw) = self.model, self.goal
        dist = math.hypot(gx - m.x, gy - m.y)
        bearing_err = math.atan2(math.sin(math.atan2(gy - m.y, gx - m.x) - m.yaw),
                                 math.cos(math.atan2(gy - m.y, gx - m.x) - m.yaw))
        if getattr(self, 'nav_blocked', False) and dist >= 0.10 and abs(bearing_err) <= 0.5:   # driving into it
            blocked = self.goal_state['blocked_since'] = self.goal_state['blocked_since'] or self.t
            self.goal_state['recoveries'] = int((self.t - blocked) // 4)
            if self.t - blocked > 10.0:
                self.goal_state['result'] = 'ABORTED'
                self.goal = None
                return
        else:
            self.goal_state['blocked_since'] = None
        cap = 0.15 if 'nav_overspeed' in self.faults else \
            min(self.smoother_cap, self.speed_limit or 0.10)
        if dist < 0.10:
            err = math.atan2(math.sin(gyaw - m.yaw), math.cos(gyaw - m.yaw))
            if abs(err) < 0.1:
                self.goal_state['result'] = 'SUCCEEDED'
                self.goal = None
                return
            vx, wz = 0.0, max(-0.2, min(0.2, err))
        else:
            bearing = math.atan2(gy - m.y, gx - m.x)
            err = math.atan2(math.sin(bearing - m.yaw), math.cos(bearing - m.yaw))
            wz = max(-0.2, min(0.2, 1.5 * err))
            vx = 0.0 if abs(err) > 0.5 else min(cap, 0.5 * dist)
        wz = max(-getattr(self, 'smoother_wz', 0.2), min(getattr(self, 'smoother_wz', 0.2), wz))
        if 'nav_reverse' in self.faults and dist < 0.6:
            vx = -0.05
        self.telemetry.on_nav_cmd(self.t, vx, 0.0, wz)
        self._mux('nav', vx, wz)

    # ---- navigation interface ----
    def send_velocity(self, vx, wz):
        self.sent.append((self.t, 'vel', vx, wz))
        if (vx or wz) and self.first_test_cmd is None:
            self.first_test_cmd = self.t
        self._mux('test', vx, wz)

    def send_stop(self):
        self.sent.append((self.t, 'stop'))
        self._mux('stop', 0.0, 0.0)

    def stop_path_status(self):
        if 'stop_unmatched' in self.faults:
            return {'stop_input_subscribers': 0, 'hold_subscribers': 1,
                    'robot_cmd_publishers': ['/lite3_domain_bridge_64'],
                    'problems': ['twist_mux not matched on /lite3_ops/stop_cmd_vel']}
        return {'stop_input_subscribers': 1, 'hold_subscribers': 1,
                'robot_cmd_publishers': ['/lite3_domain_bridge_64'], 'problems': []}

    def feed_deadman(self):
        self.last_feed = self.t

    def set_speed_limit(self, speed):
        self.speed_limit = speed
        return True

    def set_nav_velocity_cap(self, vx, wz):
        if 'cap_unverified' in self.faults:
            return {'ok': False, 'reason': 'velocity_smoother parameter service unavailable'}
        self.smoother_cap = vx
        self.smoother_wz = wz
        return {'ok': True, 'requested': [vx, 0.0, wz], 'max_velocity': [vx, 0.0, wz]}

    def set_stop_distance(self, ahead_m):
        if ahead_m < 0.355:
            return {'ok': False, 'reason': 'below floor'}
        self.stop_ahead = ahead_m
        return {'ok': True}

    def nav_send_goal(self, x, y, yaw):
        self.speed_limit = 0.0                  # Humble MPPI optimizer reset at goal start
        self.goal = (x, y, yaw)
        self.goal_sent = True
        self.goal_state = {'result': None, 'recoveries': 0, 'blocked_since': None}
        return True

    def nav_spin(self, angle_rad, time_allowance_s):
        self.goal = None
        self.spin = {'target': float(angle_rad), 'done': 0.0, 'last_yaw': self.model.yaw, 'start': self.t,
                     'allowance': float(time_allowance_s)}
        self.spins_sent = getattr(self, 'spins_sent', 0) + 1
        self.goal_state = {'result': None, 'recoveries': 0, 'blocked_since': None, 'spun_rad': 0.0}
        return True

    def nav_status(self):
        return {'active': self.goal is not None or bool(getattr(self, 'spin', None)),
                'result': self.goal_state['result'], 'recoveries': self.goal_state['recoveries'],
                'spun_rad': self.goal_state.get('spun_rad')}

    def clear_costmaps(self):
        self.costmap_clears = getattr(self, 'costmap_clears', 0) + 1
        return ['/global_costmap/clear_entirely_global_costmap', '/local_costmap/clear_entirely_local_costmap']

    def nav_cancel(self, timeout_s=3.0):
        if self.goal is not None or getattr(self, 'spin', None):
            self.goal = None
            self.spin = None
            self.goal_state['result'] = 'CANCELED'
        return True

    def map_pose(self):
        return self.model.x, self.model.y, self.model.yaw

    def nav_ready(self):
        return {'problems': []}

    def collision_boundary(self, polygon='PolygonStop'):
        if 'cm_missing' in self.faults:
            return None
        return self.cm_boundary if polygon == 'PolygonStop' else None

    def collision_approach_time(self):
        return None if 'cm_missing' in self.faults else self.cm_approach_s
