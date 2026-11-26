"""Receipt-time telemetry buffers and motion assessments (no ROS imports).

Bridge header stamps use the robot host's steady clock, which is not comparable
with this computer's clock, so freshness and windows use local receipt time.
"""

from collections import deque
from dataclasses import dataclass
import math
import threading

from .geometry import KNEE_INDEX, leg_extension, quaternion_tilt, quaternion_yaw


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


@dataclass(frozen=True)
class TerrainSample:
    """terrain_monitor status (perception only). Distances from base_link centre."""
    stop: bool
    reason: str
    obstacle_ahead_m: float     # nearest obstacle in the forward corridor, or None
    stop_ahead_m: float         # active proximity-stop distance, or None if not reported
    problems: tuple
    processing_s: float = None  # terrain_monitor processing time for this cloud


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

    STREAMS = ('joints', 'odom', 'imu', 'handle', 'system_cmd', 'terrain', 'clearance', 'nav_cmd',
               'lidar_odom', 'terrain_stop', 'mux_out', 'cm_events', 'hazard_age', 'raw_cloud',
               'lio_cloud')

    def __init__(self):
        self.joints = Stream()
        self.odom = Stream()
        self.imu = Stream()
        self.handle = Stream(rate_hint=250)
        self.system_cmd = Stream(rate_hint=100)  # /cmd_vel observed at the motion_sender side
        # Navigation-mode streams (empty in direct mode).
        self.terrain = Stream(rate_hint=20)       # /lite3_terrain/status
        self.clearance = Stream(rate_hint=20)     # footprint clearance from /lite3_terrain/hazards
        self.nav_cmd = Stream(rate_hint=100)      # Nav2 output /nav_cmd_vel (mux input)
        self.lidar_odom = Stream(rate_hint=20)    # Fast-LIO /odom receipt times
        self.terrain_stop = Stream(rate_hint=20)  # /lite3_terrain/stop Bool: the mux lock's input
        self.mux_out = Stream(rate_hint=100)      # twist_mux output = collision_monitor input
        self.cm_events = Stream(rate_hint=5)      # collision_monitor decisions ('stop', polygon text)
        self.hazard_age = Stream(rate_hint=20)    # local receipt time - hazard cloud header stamp
        self.raw_cloud = Stream(rate_hint=20)     # /livox/points receipt -> serialized bytes (M10)
        self.lio_cloud = Stream(rate_hint=20)     # /cloud_registered_body receipt -> bytes: Fast-LIO alive (M10)
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

    def on_terrain(self, stamp, stop, reason, obstacle_ahead_m, stop_ahead_m, problems,
                   processing_s=None):
        self.terrain.add(stamp, TerrainSample(bool(stop), reason, obstacle_ahead_m, stop_ahead_m,
                                              tuple(problems or ()), processing_s))

    def on_clearance(self, stamp, value):
        self.clearance.add(stamp, value)

    def on_nav_cmd(self, stamp, vx, vy, wz):
        self.nav_cmd.add(stamp, TwistSample(vx, vy, wz))

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


def odometry_frozen(odom):
    """Vendor leg-odometry freeze (measured live 2026-11-16, robot lying): the x/y position is
    bit-identical and the planar speed magnitude is bit-identical (a latched velocity vector,
    re-rotated by the noisy orientation, e.g. 0.0681 m/s) - the estimator is not integrating.
    Real translation changes the position, so this cannot hide motion."""
    first = odom[0]
    speed = math.hypot(first.vx, first.vy)
    return len(odom) >= 3 and all(o.x == first.x and o.y == first.y and
                                  abs(math.hypot(o.vx, o.vy) - speed) <= 1e-9 for o in odom)


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
    # Speed only is ignored when the odometry is provably frozen (see odometry_frozen) AND the
    # independent joint and gyro signals show rest. Yaw rate and every other criterion still apply.
    latched = (joint_range <= criteria.max_joint_range_rad and gyro <= criteria.max_gyro_rad_s
               and odometry_frozen(odom))
    detail['odometry_frozen'] = latched
    if joint_range > criteria.max_joint_range_rad:
        reasons.append('joints moving (stepping)')
    if speed > criteria.max_planar_speed_m_s and not latched:
        reasons.append('odometry speed above rest threshold')
    if yaw_rate > criteria.max_yaw_rate_rad_s:
        reasons.append('odometry yaw rate above rest threshold')
    if moved > criteria.max_position_change_m:
        reasons.append('odometry position changing')
    if gyro > criteria.max_gyro_rad_s:
        reasons.append('IMU angular rate above rest threshold')
    detail = {k: (round(v, 5) if isinstance(v, float) else v) for k, v in detail.items()}
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


CRITERIA_KEYS = (   # (metric, StationaryCriteria limit, reason text)
    ('max_planar_speed_m_s', 'max_planar_speed_m_s', 'odometry speed'),
    ('max_yaw_rate_rad_s', 'max_yaw_rate_rad_s', 'odometry yaw rate'),
    ('joint_range_rad', 'max_joint_range_rad', 'joint motion'),
    ('max_gyro_rad_s', 'max_gyro_rad_s', 'IMU angular rate'),
)


def rest_timeline(telemetry, start, end, hold_s, criteria=StationaryCriteria(), step_s=0.1,
                  instant_s=0.2, origin=None):
    """Retrospective rest diagnosis from buffered telemetry (no effect on the decision itself).

    Every ``step_s`` it records the instantaneous value of each rest metric (over ``instant_s``)
    and whether the full ``hold_s`` window ending there satisfied every criterion. Times are
    seconds relative to ``origin`` (default ``start``).
    """
    origin = start if origin is None else origin
    rows = []
    count = int((end - start) / step_s + 1e-9)
    stamps = [start + step_s * (i + 1) for i in range(count)]
    if not stamps or end - stamps[-1] > 1e-6:
        stamps.append(end)                       # always include the decision instant
    for t in stamps:
        _, inst = assess_stationary(telemetry, t, instant_s, criteria)
        still, window = assess_stationary(telemetry, t, hold_s, criteria)
        stopped, loco = assess_locomotion_stopped(telemetry, t, hold_s)
        _, loco_inst = assess_locomotion_stopped(telemetry, t, instant_s)
        row = {'t': round(t - origin, 2), 'window_rest': still,
               'window_reasons': window.get('reasons', []), 'window_stopped': stopped,
               'leg_extension_range_m': loco_inst.get('leg_extension_range_m')}
        for key, _, _ in CRITERIA_KEYS:
            row[key] = inst.get(key)
        rows.append(row)
    return rows


def settle_summary(rows, criteria=StationaryCriteria(), hold_s=1.5):
    """Per criterion: last time its instantaneous value exceeded the rest limit (None = never)."""
    summary = {}
    for key, limit_name, label in CRITERIA_KEYS:
        limit = getattr(criteria, limit_name)
        over = [r['t'] for r in rows if r.get(key) is not None and r[key] > limit]
        summary[label] = {'limit': limit, 'last_exceeded_s': over[-1] if over else None,
                          'peak': max((r[key] for r in rows if r.get(key) is not None), default=None)}
    first_rest = next((r['t'] for r in rows if r['window_rest']), None)
    summary['first_verified_rest_s'] = first_rest
    summary['rest_onset_s'] = None if first_rest is None else round(first_rest - hold_s, 2)
    return summary


@dataclass(frozen=True)
class LocomotionStopCriteria:
    """"Stopped moving": no translation and no foot lifts. Thresholds from physical data
    (2026-11-16 M2 traces): stepping changes leg extension by 0.05-0.06 m per step, the
    post-gait posture settle by <= 0.006 m; at-rest odometry speed noise <= 0.015 m/s."""
    max_planar_speed_m_s: float = 0.04
    max_position_change_m: float = 0.02
    max_leg_extension_range_m: float = 0.02


def assess_locomotion_stopped(telemetry, now, window_s, criteria=LocomotionStopCriteria()):
    """True when, over the whole window, the robot neither translates nor lifts its feet.

    Unlike assess_stationary this tolerates the small whole-body posture settle that follows
    the end of the gait; that is judged separately (full rest + settle bounds)."""
    joints = [v.positions for _, v in telemetry.joints.window(now, window_s)]
    odom = [v for _, v in telemetry.odom.window(now, window_s)]
    min_samples = max(3, int(window_s * 20))
    if len(joints) < min_samples or len(odom) < min_samples:
        return False, {'reasons': ['insufficient telemetry in window'],
                       'samples': [len(joints), len(odom)]}
    ext_range = max(max(leg_extension(j[k]) for j in joints) - min(leg_extension(j[k]) for j in joints)
                    for k in KNEE_INDEX)
    speed = max(math.hypot(o.vx, o.vy) for o in odom)
    moved = math.hypot(odom[-1].x - odom[0].x, odom[-1].y - odom[0].y)
    reasons = []
    latched = ext_range <= criteria.max_leg_extension_range_m and odometry_frozen(odom)
    if speed > criteria.max_planar_speed_m_s and not latched:
        reasons.append('odometry speed above stop threshold')
    if moved > criteria.max_position_change_m:
        reasons.append('odometry position changing')
    if ext_range > criteria.max_leg_extension_range_m:
        reasons.append('feet lifting (stepping)')
    return not reasons, {'max_planar_speed_m_s': round(speed, 5), 'position_change_m': round(moved, 5),
                         'leg_extension_range_m': round(ext_range, 5), 'reasons': reasons}
