"""Receipt-time telemetry buffers and motion assessments (no ROS imports).

Bridge header stamps use the robot host's steady clock, which is not comparable
with this computer's clock, so freshness and windows use local receipt time.
"""

from collections import deque
from dataclasses import dataclass
import math
import threading

from .geometry import quaternion_tilt, quaternion_yaw


@dataclass(frozen=True)
class JointSample:
    names: tuple
    positions: tuple


@dataclass(frozen=True)
class OdomSample:
    x: float
    y: float
    z: float
    yaw: float
    vx: float
    vy: float
    wz: float


@dataclass(frozen=True)
class ImuSample:
    roll: float
    pitch: float
    valid: bool
    gyro: tuple
    acc: tuple


@dataclass(frozen=True)
class TwistSample:
    vx: float
    vy: float
    wz: float

    @property
    def is_zero(self):
        return self.vx == 0.0 and self.vy == 0.0 and self.wz == 0.0


class Stream:
    """Bounded, lock-protected (receipt_time, value) history."""

    def __init__(self, seconds=30.0, rate_hint=250):
        self._items = deque(maxlen=int(seconds * rate_hint))
        self._lock = threading.Lock()
        self.total = 0

    def add(self, stamp, value):
        with self._lock:
            self._items.append((stamp, value))
            self.total += 1

    def latest(self):
        with self._lock:
            return self._items[-1] if self._items else None

    def window(self, now, seconds):
        with self._lock:
            return [(t, v) for t, v in self._items if now - seconds <= t <= now]

    def since(self, start):
        with self._lock:
            return [(t, v) for t, v in self._items if t >= start]

    def age(self, now):
        item = self.latest()
        return math.inf if item is None else now - item[0]

    def rate(self, now, seconds=1.0):
        return len(self.window(now, seconds)) / seconds


class Telemetry:
    """Holds the streams used for verification. Filled by ROS callbacks or the simulator."""

    STREAMS = ('joints', 'odom', 'imu', 'handle', 'system_cmd')

    def __init__(self):
        self.joints = Stream()
        self.odom = Stream()
        self.imu = Stream()
        self.handle = Stream(rate_hint=250)
        self.system_cmd = Stream(rate_hint=100)  # /cmd_vel observed at the motion_sender side
        self.sensor_alive = {}
        self._alive_lock = threading.Lock()

    # Conversion helpers used by both ROS callbacks and the simulator.
    def on_joint(self, stamp, names, positions):
        self.joints.add(stamp, JointSample(tuple(names), tuple(float(p) for p in positions)))

    def on_odom(self, stamp, px, py, pz, qx, qy, qz, qw, vx, vy, wz):
        self.odom.add(stamp, OdomSample(px, py, pz, quaternion_yaw(qx, qy, qz, qw), vx, vy, wz))

    def on_imu(self, stamp, qx, qy, qz, qw, gyro, acc):
        tilt = quaternion_tilt(qx, qy, qz, qw)
        self.imu.add(stamp, ImuSample(
            tilt[0] if tilt else math.nan, tilt[1] if tilt else math.nan,
            tilt is not None, tuple(gyro), tuple(acc)))

    def on_handle(self, stamp, vx, vy, wz):
        self.handle.add(stamp, TwistSample(vx, vy, wz))

    def on_system_cmd(self, stamp, vx, vy, wz):
        self.system_cmd.add(stamp, TwistSample(vx, vy, wz))

    def on_sensor_alive(self, stamp, name, value):
        with self._alive_lock:
            self.sensor_alive[name] = (stamp, bool(value))

    def alive_snapshot(self):
        with self._alive_lock:
            return dict(self.sensor_alive)

    def current_tilt(self):
        item = self.imu.latest()
        if item is None or not item[1].valid:
            return None
        return item[1].roll, item[1].pitch


@dataclass(frozen=True)
class StationaryCriteria:
    max_planar_speed_m_s: float = 0.04   # rest noise observed <= 0.015
    max_yaw_rate_rad_s: float = 0.05
    max_joint_range_rad: float = 0.05    # rest noise observed < 0.001
    max_position_change_m: float = 0.02
    max_gyro_rad_s: float = 0.15


def assess_stationary(telemetry, now, window_s, criteria=StationaryCriteria()):
    """True only when every independent signal in the window indicates rest."""
    joints = [v.positions for _, v in telemetry.joints.window(now, window_s)]
    odom = [v for _, v in telemetry.odom.window(now, window_s)]
    imu = [v for _, v in telemetry.imu.window(now, window_s)]
    reasons = []
    detail = {}
    min_samples = max(3, int(window_s * 20))
    if len(joints) < min_samples or len(odom) < min_samples or len(imu) < min_samples:
        return False, {'reasons': ['insufficient telemetry in window'],
                       'samples': [len(joints), len(odom), len(imu)]}
    joint_range = max(max(s[i] for s in joints) - min(s[i] for s in joints)
                      for i in range(len(joints[0])))
    speed = max(math.hypot(o.vx, o.vy) for o in odom)
    yaw_rate = max(abs(o.wz) for o in odom)
    moved = math.hypot(odom[-1].x - odom[0].x, odom[-1].y - odom[0].y)
    gyro = max(max(abs(g) for g in s.gyro) for s in imu)
    detail.update(joint_range_rad=joint_range, max_planar_speed_m_s=speed,
                  max_yaw_rate_rad_s=yaw_rate, position_change_m=moved, max_gyro_rad_s=gyro)
    if joint_range > criteria.max_joint_range_rad:
        reasons.append('joints moving (stepping)')
    if speed > criteria.max_planar_speed_m_s:
        reasons.append('odometry speed above rest threshold')
    if yaw_rate > criteria.max_yaw_rate_rad_s:
        reasons.append('odometry yaw rate above rest threshold')
    if moved > criteria.max_position_change_m:
        reasons.append('odometry position changing')
    if gyro > criteria.max_gyro_rad_s:
        reasons.append('IMU angular rate above rest threshold')
    detail = {k: round(v, 5) for k, v in detail.items()}
    detail['reasons'] = reasons
    return not reasons, detail


def wrap_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def displacement(start, end):
    """Displacement of ``end`` relative to ``start`` expressed in the start heading frame."""
    dx, dy = end.x - start.x, end.y - start.y
    c, s = math.cos(start.yaw), math.sin(start.yaw)
    return {
        'forward_m': c * dx + s * dy,
        'lateral_m': -s * dx + c * dy,
        'distance_m': math.hypot(dx, dy),
        'yaw_change_rad': wrap_angle(end.yaw - start.yaw),
    }


def joint_activity(telemetry, start, end):
    """Largest joint excursion between two receipt times; large values indicate stepping."""
    samples = [v.positions for t, v in telemetry.joints.since(start) if t <= end]
    if len(samples) < 2:
        return None
    return max(max(s[i] for s in samples) - min(s[i] for s in samples)
               for i in range(len(samples[0])))
