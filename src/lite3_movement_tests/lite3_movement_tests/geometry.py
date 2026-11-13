"""Posture assessment from the joint and IMU telemetry the existing bridge already publishes.

The vendor firmware's RobotState packet carries ``robot_basic_state`` and
``robot_motion_state``, but the deployed ``motion_receiver`` (vendor
Jetson2Motion.cpp) does not publish them to ROS. No posture/mode topic exists in
the inspected graph, so posture is inferred here from measured geometry only.

Criterion: per-leg hip-to-foot distance from the measured knee angle and the
Lite3 URDF link lengths (thigh 0.200 m, shank 0.21012 m). It is independent of
the bridge's sign convention because only |knee| is used. Live lying telemetry
(2026-11-13: |knee| ~= 2.75 rad on all legs) gives ~0.08 m, i.e. body on the
ground. Standing geometry is a physical prediction until the first authorized
stand transition records it; reports label it accordingly.
"""

from dataclasses import dataclass, field
import math

THIGH_M = 0.200    # Lite3.urdf: FL_Knee_joint origin z = -0.20
SHANK_M = 0.21012  # Lite3.urdf: FL_Ankle origin z = -0.21012
KNEE_LIMITS_RAD = (0.524, 2.792)  # URDF magnitude limits
HIPX_LIMIT_RAD = 0.523

# Exact order published by motion_receiver (vendor Jetson2Motion.cpp).
JOINT_NAMES = (
    'LF_Joint', 'LF_Joint_1', 'LF_Joint_2',
    'RF_Joint', 'RF_Joint_1', 'RF_Joint_2',
    'LB_Joint', 'LB_Joint_1', 'LB_Joint_2',
    'RB_Joint', 'RB_Joint_1', 'RB_Joint_2',
)
LEGS = ('LF', 'RF', 'LB', 'RB')
KNEE_INDEX = (2, 5, 8, 11)
HIPX_INDEX = (0, 3, 6, 9)

LYING = 'lying'
STANDING = 'standing'
TRANSITION = 'transition'
UNKNOWN = 'unknown'


def leg_extension(knee_rad):
    """Hip-to-foot distance for a knee bend magnitude (0 = straight leg)."""
    bend = abs(knee_rad)
    return math.sqrt(THIGH_M ** 2 + SHANK_M ** 2 + 2 * THIGH_M * SHANK_M * math.cos(bend))


def quaternion_tilt(x, y, z, w):
    """Return (roll, pitch) in radians, or None for a non-unit/invalid quaternion."""
    values = (x, y, z, w)
    if not all(math.isfinite(v) for v in values):
        return None
    norm = math.sqrt(sum(v * v for v in values))
    if abs(norm - 1.0) > 0.05:
        return None
    x, y, z, w = (v / norm for v in values)
    roll = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2 * (w * y - z * x))))
    return roll, pitch


def quaternion_yaw(x, y, z, w):
    return math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))


@dataclass(frozen=True)
class PostureCriteria:
    lying_max_extension_m: float = 0.14
    standing_min_extension_m: float = 0.22
    max_leg_spread_m: float = 0.06
    max_tilt_rad: float = 0.20
    settled_joint_range_rad: float = 0.05

    def __post_init__(self):
        if not (0.05 < self.lying_max_extension_m < self.standing_min_extension_m
                < THIGH_M + SHANK_M):
            raise ValueError('Posture extension thresholds must be ordered and physical')
        if not 0 < self.max_leg_spread_m < 0.2:
            raise ValueError('max_leg_spread_m out of range')
        if not 0 < self.max_tilt_rad < 0.6 or not 0 < self.settled_joint_range_rad < 0.3:
            raise ValueError('Tilt/settle thresholds out of range')


@dataclass
class PostureAssessment:
    posture: str
    settled: bool
    extensions_m: dict = field(default_factory=dict)
    tilt_rad: tuple = None
    joint_range_rad: float = None
    reasons: list = field(default_factory=list)

    @property
    def confident(self):
        return self.posture in (LYING, STANDING) and self.settled and not self.reasons

    def as_dict(self):
        return {
            'posture': self.posture, 'settled': self.settled, 'confident': self.confident,
            'extensions_m': {k: round(v, 4) for k, v in self.extensions_m.items()},
            'tilt_rad': None if self.tilt_rad is None else [round(v, 4) for v in self.tilt_rad],
            'joint_range_rad': None if self.joint_range_rad is None else round(self.joint_range_rad, 4),
            'reasons': list(self.reasons),
        }


def validate_joint_sample(names, positions):
    """Return a list of problems; empty means the sample matches the bridge contract."""
    problems = []
    if tuple(names) != JOINT_NAMES:
        problems.append('joint names/order differ from motion_receiver contract')
    if len(positions) != 12:
        problems.append(f'expected 12 joint positions, got {len(positions)}')
        return problems
    if not all(math.isfinite(p) for p in positions):
        problems.append('non-finite joint position')
        return problems
    for leg, k in zip(LEGS, KNEE_INDEX):
        lo, hi = KNEE_LIMITS_RAD
        if not lo - 0.15 <= abs(positions[k]) <= hi + 0.15:
            problems.append(f'{leg} knee |{positions[k]:.3f}| outside URDF range')
    for leg, h in zip(LEGS, HIPX_INDEX):
        if abs(positions[h]) > HIPX_LIMIT_RAD + 0.15:
            problems.append(f'{leg} hip abduction {positions[h]:.3f} outside URDF range')
    return problems


def assess_posture(joint_window, tilt, criteria=PostureCriteria()):
    """Classify from a window of 12-joint position lists plus current IMU (roll, pitch).

    Settled means every joint varied less than ``settled_joint_range_rad`` over the window.
    Anything outside both bands, disagreeing legs or excessive tilt is not confident.
    """
    if not joint_window:
        return PostureAssessment(UNKNOWN, False, reasons=['no joint samples in window'])
    latest = joint_window[-1]
    problems = validate_joint_sample(JOINT_NAMES, latest)
    if problems:
        return PostureAssessment(UNKNOWN, False, reasons=problems)
    joint_range = max(max(s[i] for s in joint_window) - min(s[i] for s in joint_window)
                      for i in range(12))
    settled = joint_range <= criteria.settled_joint_range_rad
    ext = {leg: leg_extension(latest[k]) for leg, k in zip(LEGS, KNEE_INDEX)}
    values = list(ext.values())
    reasons = []
    if max(values) <= criteria.lying_max_extension_m:
        posture = LYING
    elif min(values) >= criteria.standing_min_extension_m:
        posture = STANDING
    else:
        posture = TRANSITION
    if posture == STANDING and max(values) - min(values) > criteria.max_leg_spread_m:
        reasons.append('leg extensions disagree beyond spread limit')
    if tilt is None:
        reasons.append('IMU orientation invalid or unavailable')
    elif max(abs(tilt[0]), abs(tilt[1])) > criteria.max_tilt_rad:
        reasons.append('body tilt exceeds limit')
    if not settled:
        reasons.append('joints still moving')
    return PostureAssessment(posture, settled, ext, tilt, joint_range, reasons)
