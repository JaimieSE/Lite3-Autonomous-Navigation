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

# Mean of a live read-only sample on 2026-11-13 while the robot lay down.
LYING_POSE = (0.4497, 1.1513, -2.7500, -0.4456, 1.1638, -2.7427,
              0.4465, 1.1387, -2.7465, -0.4384, 1.1237, -2.7425)
# Nominal stand geometry (|knee| 1.65 rad -> 0.278 m hip-to-foot); simulation only.
STANDING_POSE = (0.05, 0.85, -1.65, -0.05, 0.85, -1.65,
                 0.05, 0.85, -1.65, -0.05, 0.85, -1.65)

FAULTS = ('none', 'march_in_place', 'ignore_posture', 'no_stop', 'telemetry_dropout',
          'tilt', 'remote_takeover', 'slow_stand', 'runaway', 'no_silence_stop')


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
        scale = {'march_in_place': 0.0, 'runaway': 20.0}.get(self.fault, 1.0)
        alpha = min(1.0, dt / 0.3)
        self.v[0] += alpha * (scale * vx - self.v[0])
        self.v[1] += alpha * (scale * wz - self.v[1])
        if self.blend >= 1.0:   # the real leg odometry froze while lying
            self.x += self.v[0] * math.cos(self.yaw) * dt
            self.y += self.v[0] * math.sin(self.yaw) * dt
            self.yaw = math.atan2(math.sin(self.yaw + self.v[1] * dt), math.cos(self.yaw + self.v[1] * dt))

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
        return JOINT_NAMES, [p + self.rng.uniform(-4e-4, 4e-4) for p in pose]

    def odom(self):
        noise = lambda: self.rng.uniform(-0.008, 0.008)  # noqa: E731
        stepping = 0.03 if self.gait_active else 0.0
        return (self.x, self.y, 0.3227, self.yaw,
                self.v[0] + noise() + stepping * math.sin(9 * self.t), noise(), self.v[1] + noise())

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

    def command_path_health(self):
        return self.healthy, {'problems': [] if self.healthy else ['competing publisher']}

    def cancelled(self):
        return self.cancel
