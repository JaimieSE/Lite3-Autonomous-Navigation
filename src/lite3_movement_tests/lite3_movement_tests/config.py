"""Sequence and physical-session authorization loading with hard, code-level limits."""

from dataclasses import dataclass, field, fields
import datetime as dt
import hashlib
import math
from pathlib import Path

import yaml

from .geometry import LYING, STANDING, PostureCriteria
from .telemetry import LocomotionStopCriteria, StationaryCriteria

# Ceilings no configuration or authorization can raise.
HARD_MAX_SPEED_M_S = 0.5             # raised from 0.10 on 2026-11-17 by the operator's decision. Validated
#   (coast, latency, backup lock, Nav2 tuning) only up to 0.10 m/s: everything above is opt-in per
#   session (start_navigation.sh --max-speed) and needs the backup lock moved out (see roam.py,
#   physical_suite.py) and a coast re-measured at that speed.
HARD_MAX_YAW_RATE_RAD_S = 0.30
HARD_MAX_MOVE_S = 3.0
HARD_MAX_POSTURE_TIMEOUT_S = 20.0
HARD_MAX_STEP_TIMEOUT_S = 30.0
HARD_MAX_SESSION_S = 600.0
HARD_MAX_AUTH_WINDOW = dt.timedelta(hours=4)
# Navigation-mode ceilings (interface: nav).
HARD_MAX_NAV_DISTANCE_M = 6.0         # one nav_goal, from the sequence anchor
HARD_MAX_NAV_GEOFENCE_M = 10.0
HARD_MAX_NAV_STEP_S = 240.0
HARD_MAX_APPROACH_SPEED_M_S = 0.05    # deliberate approach toward an obstacle
HARD_MIN_APPROACH_CLEARANCE_M = 0.10  # runner aborts if an obstacle gets closer to the nose
# M4 sensor-driven walk (approach_stop). Travel is bounded independently by leg odometry, the
# SLAM pose, the odometry geofence and (once an obstacle is tracked) an obstacle-relative budget.
HARD_MAX_APPROACH_S = 60.0            # M4 movement duration ceiling
HARD_MAX_APPROACH_TRAVEL_M = 1.8      # M4 travel ceiling (geofence = travel + 0.2 <= 2.0 m)
HARD_MAX_DETECTION_RANGE_M = 2.5      # validated region: terrain corridor 3.0 m from centre, +-0.30 m
HARD_MAX_SENSOR_AGE_S = 0.5           # = terrain_stop lock timeout; freshness may only be tightened
HARD_MIN_LIDAR_HZ = 8.0               # readiness minimum; may only be raised
HARD_MAX_RESPONSE_S = 0.5             # obstacle inside boundary -> no forward command at the robot
MIN_PRIMARY_SEPARATION_M = 0.05       # collision_monitor boundary vs backup lock: one grid cell
HARD_MAX_M4_STEP_S = 130.0            # step timeout ceiling for approach_stop / proximity_watch
ODOMETRY_TRUST = 0.8                  # leg odometry under-reads forward travel by up to ~15%
HARD_MIN_STOP_DISTANCE_M = 0.30       # proximity stop from the nose; validated default
HARD_MIN_ABORT_CLEARANCE_M = 0.05     # footprint clearance abort floor during navigation
NAV2_MAX_YAW_RATE_RAD_S = 0.20       # params/nav2_lite3.yaml MPPI + smoother cap
NOSE_OFFSET_M = 0.305                 # footprint front edge ahead of base_link (Nav2 footprint)

STEP_TYPES = {
    'detection_check',
    'readiness', 'telemetry_validation', 'stationary_check', 'posture_check',
    'posture_transition', 'velocity_move', 'stop_verify',
    'nav_readiness', 'zero_delivery', 'deadman_stop', 'blocked_move', 'approach_stop', 'nav_goal',
    'proximity_watch',
}
NAV_ONLY_TYPES = {'detection_check', 'nav_readiness', 'zero_delivery', 'deadman_stop', 'blocked_move',
                  'approach_stop', 'nav_goal', 'proximity_watch'}
ACTUATING_TYPES = {'posture_transition', 'velocity_move', 'stop_verify', 'zero_delivery',
                   'deadman_stop', 'blocked_move', 'approach_stop', 'nav_goal'}
# Bounded direct moves through the mux test input; each must be followed by stop_verify.
GUARDED_MOVE_TYPES = {'velocity_move', 'deadman_stop', 'blocked_move', 'approach_stop'}

REQUIRED_CONFIRMATIONS = (
    'area_clear_radius_1m',
    'emergency_stop_in_hand_and_tested',
    'exclusive_robot_control',
    'navigation_mode_selected',
    'operator_supervising_entire_session',
    'accept_no_lidar_collision_gate',
    'accept_unverified_firmware_command_timeout',
)
# Navigation sessions run with the LiDAR proximity lock and Nav2 costmaps active, so the
# "no LiDAR collision gate" acceptance is replaced by a per-test scene confirmation.
NAV_REQUIRED_CONFIRMATIONS = tuple(c for c in REQUIRED_CONFIRMATIONS
                                   if c != 'accept_no_lidar_collision_gate') + (
    'test_area_prepared_as_instructed',
)


def required_confirmations(sequence):
    return NAV_REQUIRED_CONFIRMATIONS if sequence.interface == 'nav' else REQUIRED_CONFIRMATIONS


class ConfigError(ValueError):
    pass


def _finite(value, name, low, high, *, low_inclusive=True):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ConfigError(f'{name} must be a finite number')
    if value > high or value < low or (not low_inclusive and value == low):
        raise ConfigError(f'{name}={value} outside [{low}, {high}]')
    return float(value)


def _range(value, name, low, high):
    if not isinstance(value, (list, tuple)) or len(value) != 2:
        raise ConfigError(f'{name} must be [min, max]')
    a = _finite(value[0], name + '[0]', low, high)
    b = _finite(value[1], name + '[1]', low, high)
    if a > b:
        raise ConfigError(f'{name} min exceeds max')
    return (a, b)


def _m4_params(p, take, step_id, kind):
    """M4 (sensor-driven walk) parameters. Every value is range-checked against the validated
    envelope; values outside it are refused, never clamped. Travel and duration are independent
    of speed and of the stopping boundary: raising them cannot raise the speed or move the stop."""
    def num(key, default, low, high):
        return _finite(take(key, default), f'{step_id}.{key}', low, high)

    if kind == 'proximity_watch':       # no motion: vx only feeds the boundary re-check
        p['vx'] = _finite(take('vx', 0.05), f'{step_id}.vx', 0.0, HARD_MAX_APPROACH_SPEED_M_S,
                          low_inclusive=False)
        p['duration_s'] = num('duration_s', 40.0, 5.0, HARD_MAX_APPROACH_S)
    p['max_travel_m'] = num('max_travel_m', 1.5, 0.2, HARD_MAX_APPROACH_TRAVEL_M)
    # The RUNNING collision_monitor stop boundary (nose distance); verified live before moving.
    p['stop_distance_m'] = num('stop_distance_m', 0.37, HARD_MIN_STOP_DISTANCE_M, 1.5)
    p['backup_stop_distance_m'] = num('backup_stop_distance_m', HARD_MIN_STOP_DISTANCE_M,
                                      HARD_MIN_STOP_DISTANCE_M, 1.5)
    p['min_clearance_m'] = num('min_clearance_m', 0.15, HARD_MIN_APPROACH_CLEARANCE_M, 1.0)
    p['detection_range_m'] = num('detection_range_m', 2.0, p['stop_distance_m'] + 0.10,
                                 HARD_MAX_DETECTION_RANGE_M)
    p['initial_clear_m'] = num('initial_clear_m', min(p['detection_range_m'], p['stop_distance_m'] + 0.15),
                               p['stop_distance_m'] + 0.05, p['detection_range_m'])
    p['max_sensor_age_s'] = num('max_sensor_age_s', 0.3, 0.1, HARD_MAX_SENSOR_AGE_S)
    p['min_lidar_hz'] = num('min_lidar_hz', HARD_MIN_LIDAR_HZ, HARD_MIN_LIDAR_HZ, 20.0)
    p['detect_scans'] = num('detect_scans', 3, 2, 10)
    p['track_tolerance_m'] = num('track_tolerance_m', 0.15, 0.05, 0.25)
    p['lost_track_scans'] = num('lost_track_scans', 3, 1, 5)
    p['suppression_proof_s'] = num('suppression_proof_s', 1.0, 0.5, 1.5)
    p['response_limit_s'] = num('response_limit_s', 0.3, 0.1, HARD_MAX_RESPONSE_S)
    p['stop_within_s'] = num('stop_within_s', 3.0, 1.0, 3.0)
    p['max_lateral_m'] = num('max_lateral_m', 0.15, 0.05, 0.30)
    p['max_yaw_change_rad'] = num('max_yaw_change_rad', 0.25, 0.05, 0.35)
    # Stopping-boundary inputs (lite3_locomotion.collision_profile). coast_m < 0 = unmeasured.
    p['coast_m'] = num('coast_m', -1.0, -1.0, 0.2)
    p['coast_speed_m_s'] = num('coast_speed_m_s', 0.05, 0.02, HARD_MAX_SPEED_M_S)
    p['path_latency_s'] = num('path_latency_s', 0.15, 0.05, 0.5)
    p['sensor_margin_m'] = num('sensor_margin_m', 0.05, 0.05, 0.3)
    for key in ('detect_scans', 'lost_track_scans'):
        if p[key] != int(p[key]):
            raise ConfigError(f'{step_id}.{key} must be a whole number of scans')
        p[key] = int(p[key])


@dataclass(frozen=True)
class Step:
    id: str
    type: str
    params: dict
    timeout_s: float

    @property
    def actuating(self):
        return self.type in ACTUATING_TYPES


def _parse_step(raw, index):
    if not isinstance(raw, dict):
        raise ConfigError(f'step {index} must be a mapping')
    raw = dict(raw)
    step_id = raw.pop('id', None)
    kind = raw.pop('type', None)
    if not isinstance(step_id, str) or not step_id:
        raise ConfigError(f'step {index} needs an id')
    if kind not in STEP_TYPES:
        raise ConfigError(f'step {step_id}: unknown type {kind!r}')
    p = {}
    allowed = {'timeout_s'}

    def take(key, default=None):
        allowed.add(key)
        return raw.get(key, default)

    timeout = take('timeout_s', 10.0)
    if kind in ('stationary_check', 'posture_check', 'posture_transition', 'stop_verify'):
        p['hold_s'] = _finite(take('hold_s', 1.5), f'{step_id}.hold_s', 0.5, 5.0)
    if kind == 'readiness':
        p['quiet_s'] = _finite(take('quiet_s', 2.0), f'{step_id}.quiet_s', 0.5, 10.0)
    if kind == 'telemetry_validation':
        p['window_s'] = _finite(take('window_s', 2.0), f'{step_id}.window_s', 0.5, 10.0)
    if kind == 'posture_check':
        p['expect'] = take('expect')
        if p['expect'] not in (LYING, STANDING):
            raise ConfigError(f'{step_id}.expect must be lying or standing')
    if kind == 'posture_transition':
        p['target'] = take('target')
        if p['target'] not in (LYING, STANDING):
            raise ConfigError(f'{step_id}.target must be lying or standing')
        timeout = _finite(timeout, f'{step_id}.timeout_s', 3.0, HARD_MAX_POSTURE_TIMEOUT_S)
    if kind == 'velocity_move':
        p['vx'] = _finite(take('vx', 0.0), f'{step_id}.vx', -HARD_MAX_SPEED_M_S, HARD_MAX_SPEED_M_S)
        p['wz'] = _finite(take('wz', 0.0), f'{step_id}.wz', -HARD_MAX_YAW_RATE_RAD_S,
                          HARD_MAX_YAW_RATE_RAD_S)
        if p['vx'] == 0.0 and p['wz'] == 0.0:
            raise ConfigError(f'{step_id}: velocity_move needs nonzero vx or wz')
        p['duration_s'] = _finite(take('duration_s'), f'{step_id}.duration_s', 0.2, HARD_MAX_MOVE_S)
        p['warmup_s'] = _finite(take('warmup_s', 0.5), f'{step_id}.warmup_s', 0.2, 2.0)
        p['settle_s'] = _finite(take('settle_s', 1.0), f'{step_id}.settle_s', 0.0, 3.0)
        p['end_mode'] = take('end_mode', 'zeros')
        if p['end_mode'] not in ('zeros', 'silence'):
            raise ConfigError(f'{step_id}.end_mode must be zeros or silence')
        p['silence_timeout_s'] = _finite(take('silence_timeout_s', 3.0),
                                         f'{step_id}.silence_timeout_s', 1.0, 5.0)
        p['continue_on_range_failure'] = take('continue_on_range_failure', False) is True
        p['delivery_deadline_s'] = _finite(take('delivery_deadline_s', 0.5),
                                           f'{step_id}.delivery_deadline_s', 0.1, 1.0)
        expect = take('expect') or {}
        if not isinstance(expect, dict) or not expect:
            raise ConfigError(f'{step_id}.expect must give measurable success ranges')
        p['expect'] = {}
        for key, bound in (('forward_m', 1.0), ('lateral_m', 1.0), ('yaw_change_rad', math.pi)):
            if key in expect:
                p['expect'][key] = _range(expect[key], f'{step_id}.expect.{key}', -bound, bound)
        if set(expect) - set(p['expect']):
            raise ConfigError(f'{step_id}.expect has unknown keys {set(expect) - set(p["expect"])}')
        # Timeout covers warmup, motion and settling plus margin; not user-extendable.
        timeout = (p['warmup_s'] + p['duration_s'] + p['settle_s'] + 2.0 +
                   (p['silence_timeout_s'] if p['end_mode'] == 'silence' else 0.0))
    if kind == 'stop_verify':
        p['zero_min_s'] = _finite(take('zero_min_s', 1.0), f'{step_id}.zero_min_s', 0.5, 5.0)
    if kind == 'nav_readiness':
        p['window_s'] = _finite(take('window_s', 3.0), f'{step_id}.window_s', 1.0, 10.0)
        # Collision stop boundary re-check (0 = boundary unknown: not checked, e.g. simulation).
        p['cm_boundary_m'] = _finite(take('cm_boundary_m', 0.0), f'{step_id}.cm_boundary_m', 0.0, 1.5)
        p['cm_clearance_m'] = _finite(take('cm_clearance_m', 0.15), f'{step_id}.cm_clearance_m', 0.05, 1.0)
        p['cm_max_speed_m_s'] = _finite(take('cm_max_speed_m_s', 0.10), f'{step_id}.cm_max_speed_m_s',
                                        0.01, HARD_MAX_SPEED_M_S)
        p['coast_m'] = _finite(take('coast_m', 0.05), f'{step_id}.coast_m', 0.0, 0.2)
        p['coast_speed_m_s'] = _finite(take('coast_speed_m_s', 0.05), f'{step_id}.coast_speed_m_s',
                                       0.02, HARD_MAX_SPEED_M_S)
        p['min_lidar_hz'] = _finite(take('min_lidar_hz', 8.0), f'{step_id}.min_lidar_hz', 5.0, 20.0)
        p['stop_distance_m'] = _finite(take('stop_distance_m', HARD_MIN_STOP_DISTANCE_M),
                                       f'{step_id}.stop_distance_m', HARD_MIN_STOP_DISTANCE_M, 1.5)
    if kind == 'zero_delivery':
        p['duration_s'] = _finite(take('duration_s', 1.0), f'{step_id}.duration_s', 0.5, 3.0)
        p['deadline_s'] = _finite(take('deadline_s', 0.5), f'{step_id}.deadline_s', 0.1, 1.0)
        p['idle_s'] = _finite(take('idle_s', 1.0), f'{step_id}.idle_s', 0.5, 3.0)
        timeout = p['duration_s'] + p['idle_s'] + 1.0
    if kind in ('deadman_stop', 'blocked_move', 'approach_stop'):
        cap = HARD_MAX_APPROACH_SPEED_M_S if kind != 'deadman_stop' else HARD_MAX_SPEED_M_S
        p['vx'] = _finite(take('vx', 0.05), f'{step_id}.vx', 0.0, cap, low_inclusive=False)
        if kind == 'approach_stop':       # M4 movement duration (the walk ends at this timer)
            p['duration_s'] = _finite(take('duration_s', 40.0), f'{step_id}.duration_s', 5.0,
                                      HARD_MAX_APPROACH_S)
        else:
            p['duration_s'] = _finite(take('duration_s', 2.0), f'{step_id}.duration_s', 0.5,
                                      HARD_MAX_MOVE_S)
        p['delivery_deadline_s'] = 0.5
    if kind == 'deadman_stop':
        p['release_after_s'] = _finite(take('release_after_s', 1.0), f'{step_id}.release_after_s',
                                       0.5, p['duration_s'] - 0.5)
        p['cutoff_within_s'] = _finite(take('cutoff_within_s', 0.8), f'{step_id}.cutoff_within_s',
                                       0.3, 1.5)
        # Stage 1: locomotion stopped (no translation, no foot lifts) - the safety limit.
        p['stop_within_s'] = _finite(take('stop_within_s', 3.0), f'{step_id}.stop_within_s', 1.0, 3.0)
        # Stage 2: fully settled (every rest criterion), with bounded motion during the settle.
        p['settle_within_s'] = _finite(take('settle_within_s', 5.0), f'{step_id}.settle_within_s',
                                       p['stop_within_s'], 6.0)
        p['max_settle_yaw_rad'] = _finite(take('max_settle_yaw_rad', 0.05),
                                          f'{step_id}.max_settle_yaw_rad', 0.0, 0.1)
        p['max_settle_shift_m'] = _finite(take('max_settle_shift_m', 0.02),
                                          f'{step_id}.max_settle_shift_m', 0.0, 0.05)
        timeout = p['duration_s'] + p['settle_within_s'] + 6.0
    if kind == 'blocked_move':
        p['max_displacement_m'] = _finite(take('max_displacement_m', 0.03),
                                          f'{step_id}.max_displacement_m', 0.01, 0.10)
        timeout = p['duration_s'] + 3.0
    if kind in ('approach_stop', 'proximity_watch'):
        _m4_params(p, take, step_id, kind)
        if kind == 'approach_stop':
            timeout = (p['duration_s'] + p['suppression_proof_s'] + p['stop_within_s'] + 1.5 + 12.0)
        else:
            p['window_s'] = _finite(take('window_s', 60.0), f'{step_id}.window_s', 5.0, 120.0)
            timeout = p['window_s'] + 5.0
    if kind == 'detection_check':
        p['expect'] = take('expect', 'ahead')
        if p['expect'] not in ('ahead', 'stop_zone'):
            raise ConfigError(f'{step_id}.expect must be ahead or stop_zone')
        p['distance_m'] = _range(take('distance_m', [0.10, 3.0]), f'{step_id}.distance_m', 0.0, 5.0)
        p['window_s'] = _finite(take('window_s', 3.0), f'{step_id}.window_s', 1.0, 10.0)
        p['min_fraction'] = _finite(take('min_fraction', 0.95), f'{step_id}.min_fraction', 0.8, 1.0)
        p['max_gap_s'] = _finite(take('max_gap_s', 0.3), f'{step_id}.max_gap_s', 0.1, 1.0)
        p['max_spread_m'] = _finite(take('max_spread_m', 0.05), f'{step_id}.max_spread_m', 0.02, 0.3)
        timeout = max(timeout, p['window_s'] + 2.0)
    if kind == 'nav_goal':
        p['forward_m'] = _finite(take('forward_m', 0.0), f'{step_id}.forward_m',
                                 -HARD_MAX_NAV_DISTANCE_M, HARD_MAX_NAV_DISTANCE_M)
        p['left_m'] = _finite(take('left_m', 0.0), f'{step_id}.left_m',
                              -HARD_MAX_NAV_DISTANCE_M, HARD_MAX_NAV_DISTANCE_M)
        if math.hypot(p['forward_m'], p['left_m']) > HARD_MAX_NAV_DISTANCE_M:
            raise ConfigError(f'{step_id}: goal beyond {HARD_MAX_NAV_DISTANCE_M} m')
        p['yaw_deg'] = _finite(take('yaw_deg', 0.0), f'{step_id}.yaw_deg', -180.0, 180.0)
        p['max_speed'] = _finite(take('max_speed', 0.05), f'{step_id}.max_speed', 0.02,
                                 HARD_MAX_SPEED_M_S)
        p['xy_tolerance_m'] = _finite(take('xy_tolerance_m', 0.25), f'{step_id}.xy_tolerance_m',
                                      0.05, 1.0)
        p['yaw_tolerance_deg'] = _finite(take('yaw_tolerance_deg', 30.0),
                                         f'{step_id}.yaw_tolerance_deg', 5.0, 180.0)
        p['expect'] = take('expect', 'reached')
        if p['expect'] not in ('reached', 'not_reached'):
            raise ConfigError(f'{step_id}.expect must be reached or not_reached')
        p['min_clearance_m'] = _finite(take('min_clearance_m', 0.10), f'{step_id}.min_clearance_m',
                                       0.0, 1.0)
        p['abort_clearance_m'] = _finite(take('abort_clearance_m', 0.08),
                                         f'{step_id}.abort_clearance_m', HARD_MIN_ABORT_CLEARANCE_M,
                                         0.5)
        p['min_lateral_m'] = _finite(take('min_lateral_m', 0.0), f'{step_id}.min_lateral_m', 0.0, 2.0)
        # -1: any number of collision_monitor stops allowed; 0: must go around without one (M7).
        p['max_cm_stops'] = int(_finite(take('max_cm_stops', -1), f'{step_id}.max_cm_stops', -1, 100))
        p['geofence_m'] = _finite(take('geofence_m', math.hypot(p['forward_m'], p['left_m']) + 1.0),
                                  f'{step_id}.geofence_m', 0.5, HARD_MAX_NAV_GEOFENCE_M)
        timeout = _finite(timeout, f'{step_id}.timeout_s', 10.0, HARD_MAX_NAV_STEP_S)
    timeout = _finite(timeout, f'{step_id}.timeout_s', 0.5,
                      HARD_MAX_NAV_STEP_S if kind == 'nav_goal' else
                      HARD_MAX_M4_STEP_S if kind in ('approach_stop', 'proximity_watch') else
                      HARD_MAX_STEP_TIMEOUT_S)
    unknown = set(raw) - allowed
    if unknown:
        raise ConfigError(f'step {step_id}: unknown keys {sorted(unknown)}')
    return Step(step_id, kind, p, timeout)


def _dataclass_from(cls, raw, name):
    raw = raw or {}
    known = {f.name for f in fields(cls)}
    unknown = set(raw) - known
    if unknown:
        raise ConfigError(f'{name}: unknown keys {sorted(unknown)}')
    try:
        return cls(**raw)
    except (TypeError, ValueError) as error:
        raise ConfigError(f'{name}: {error}') from error


@dataclass(frozen=True)
class TelemetryRequirements:
    min_rate_hz: float = 50.0
    max_age_s: float = 0.25
    required_sensor_alive: tuple = ('imu', 'joint', 'odom2')
    max_handle_axis: float = 0.05
    min_accel_norm: float = 8.8
    max_accel_norm: float = 10.8


@dataclass(frozen=True)
class SafetyLimits:
    geofence_radius_m: float = 0.5
    max_tilt_rad: float = 0.35
    command_period_s: float = 0.05
    zero_seconds: float = 2.0
    stop_timeout_s: float = 6.0
    stop_hold_s: float = 1.5

    def __post_init__(self):
        _finite(self.geofence_radius_m, 'geofence_radius_m', 0.05, 2.0)
        _finite(self.max_tilt_rad, 'max_tilt_rad', 0.05, 0.6)
        _finite(self.command_period_s, 'command_period_s', 0.02, 0.1)
        _finite(self.zero_seconds, 'zero_seconds', 1.0, 5.0)
        _finite(self.stop_timeout_s, 'stop_timeout_s', 2.0, 15.0)
        _finite(self.stop_hold_s, 'stop_hold_s', 0.5, 5.0)


@dataclass(frozen=True)
class Sequence:
    name: str
    steps: tuple
    finalize: tuple
    telemetry: TelemetryRequirements = TelemetryRequirements()
    safety: SafetyLimits = SafetyLimits()
    posture: PostureCriteria = PostureCriteria()
    stationary: StationaryCriteria = StationaryCriteria()
    locomotion_stop: LocomotionStopCriteria = LocomotionStopCriteria()
    sha256: str = ''
    source: str = ''
    interface: str = 'direct'     # direct: runner publishes /cmd_vel; nav: via twist_mux (physical)

    @property
    def all_steps(self):
        return self.steps + self.finalize


def load_sequence(path):
    path = Path(path)
    data = path.read_bytes()
    raw = yaml.safe_load(data)
    if not isinstance(raw, dict):
        raise ConfigError('sequence file must be a mapping')
    known = {'name', 'description', 'steps', 'finalize', 'telemetry', 'safety', 'posture',
             'stationary', 'interface'}
    if set(raw) - known:
        raise ConfigError(f'unknown sequence keys {sorted(set(raw) - known)}')
    steps = tuple(_parse_step(s, i) for i, s in enumerate(raw.get('steps') or []))
    finalize = tuple(_parse_step(s, i) for i, s in enumerate(raw.get('finalize') or []))
    if not steps:
        raise ConfigError('sequence has no steps')
    ids = [s.id for s in steps + finalize]
    if len(ids) != len(set(ids)):
        raise ConfigError('step ids must be unique')
    if steps[0].type != 'readiness':
        raise ConfigError('first step must be readiness')
    if any(s.type == 'velocity_move' for s in finalize):
        raise ConfigError('finalize may not contain velocity_move')
    interface = raw.get('interface', 'direct')
    if interface not in ('direct', 'nav'):
        raise ConfigError('interface must be direct or nav')
    nav_steps = [s.id for s in steps + finalize if s.type in NAV_ONLY_TYPES]
    if nav_steps and interface != 'nav':
        raise ConfigError(f'steps {nav_steps} need interface: nav')
    if any(s.type in GUARDED_MOVE_TYPES | {'nav_goal'} for s in finalize):
        raise ConfigError('finalize may not contain moves')
    if interface == 'nav' and (len(steps) < 2 or steps[1].type != 'nav_readiness'):
        raise ConfigError('interface nav: second step must be nav_readiness')
    for i, step in enumerate(steps):
        nxt = steps[i + 1] if i + 1 < len(steps) else None
        if step.type in GUARDED_MOVE_TYPES and (nxt is None or nxt.type != 'stop_verify'):
            raise ConfigError(f'{step.id}: every {step.type} must be followed by stop_verify')
        if step.type == 'nav_goal' and (nxt is None or nxt.type not in ('nav_goal', 'stop_verify')):
            raise ConfigError(f'{step.id}: a nav_goal chain must end with stop_verify')
    telemetry = _dataclass_from(TelemetryRequirements, raw.get('telemetry'), 'telemetry')
    telemetry = TelemetryRequirements(**{**telemetry.__dict__, 'required_sensor_alive':
                                         tuple(telemetry.required_sensor_alive)})
    sequence = Sequence(
        name=str(raw.get('name') or path.stem), steps=steps, finalize=finalize,
        telemetry=telemetry,
        safety=_dataclass_from(SafetyLimits, raw.get('safety'), 'safety'),
        posture=_dataclass_from(PostureCriteria, raw.get('posture'), 'posture'),
        stationary=_dataclass_from(StationaryCriteria, raw.get('stationary'), 'stationary'),
        sha256=hashlib.sha256(data).hexdigest(), source=str(path), interface=interface,
    )
    budget = sum(s.timeout_s for s in sequence.all_steps)
    if budget > HARD_MAX_SESSION_S:
        raise ConfigError(f'sequence time budget {budget:.0f}s exceeds {HARD_MAX_SESSION_S:.0f}s')
    return sequence


@dataclass(frozen=True)
class Authorization:
    session: str
    sequence_sha256: str
    valid_until: dt.datetime
    max_speed_m_s: float
    max_yaw_rate_rad_s: float
    max_move_duration_s: float
    allow_posture_commands: bool
    confirmations: dict = field(default_factory=dict)
    max_nav_distance_m: float = 0.0       # 0: navigation goals not authorized
    max_approach_duration_s: float = 0.0  # approach_stop (M4) movement duration
    max_approach_travel_m: float = 0.0    # approach_stop (M4) travel; 0 = M4 not authorized
    source: str = ''


def load_authorization(path):
    raw = yaml.safe_load(Path(path).read_text())
    if not isinstance(raw, dict):
        raise ConfigError('authorization must be a mapping')
    try:
        limits = raw['limits']
        valid_until = raw['valid_until']
        if isinstance(valid_until, str):
            valid_until = dt.datetime.fromisoformat(valid_until)
        if not isinstance(valid_until, dt.datetime) or valid_until.tzinfo is None:
            raise ConfigError('valid_until must be an ISO-8601 time with a UTC offset')
        return Authorization(
            session=str(raw['session']),
            sequence_sha256=str(raw['sequence_sha256']),
            valid_until=valid_until,
            max_speed_m_s=_finite(limits['max_speed_m_s'], 'max_speed_m_s', 0.0, HARD_MAX_SPEED_M_S),
            max_yaw_rate_rad_s=_finite(limits['max_yaw_rate_rad_s'], 'max_yaw_rate_rad_s', 0.0,
                                       HARD_MAX_YAW_RATE_RAD_S),
            max_move_duration_s=_finite(limits['max_move_duration_s'], 'max_move_duration_s',
                                        0.0, HARD_MAX_MOVE_S),
            allow_posture_commands=limits['allow_posture_commands'] is True,
            max_nav_distance_m=_finite(limits.get('max_nav_distance_m', 0.0), 'max_nav_distance_m',
                                       0.0, HARD_MAX_NAV_DISTANCE_M),
            max_approach_duration_s=_finite(limits.get('max_approach_duration_s', 0.0),
                                            'max_approach_duration_s', 0.0, HARD_MAX_APPROACH_S),
            max_approach_travel_m=_finite(limits.get('max_approach_travel_m', 0.0),
                                          'max_approach_travel_m', 0.0, HARD_MAX_APPROACH_TRAVEL_M),
            confirmations=dict(raw.get('operator_confirmations') or {}),
            source=str(path),
        )
    except KeyError as error:
        raise ConfigError(f'authorization missing {error}') from error


def check_authorization(auth, sequence, now=None):
    """Return a list of reasons the session is NOT authorized; empty means authorized."""
    now = now or dt.datetime.now(dt.timezone.utc)
    problems = []
    if auth.sequence_sha256 != sequence.sha256:
        problems.append('sequence file changed since authorization (sha256 mismatch)')
    if auth.valid_until <= now:
        problems.append('authorization expired')
    elif auth.valid_until - now > HARD_MAX_AUTH_WINDOW:
        problems.append('authorization validity window exceeds 4 hours')
    for name in required_confirmations(sequence):
        if auth.confirmations.get(name) is not True:
            problems.append(f'operator confirmation missing: {name}')
    for step in sequence.all_steps:
        if step.type == 'posture_transition' and not auth.allow_posture_commands:
            problems.append(f'{step.id}: posture commands not authorized')
        if step.type == 'velocity_move':
            if abs(step.params['vx']) > auth.max_speed_m_s:
                problems.append(f'{step.id}: speed exceeds authorized maximum')
            if abs(step.params['wz']) > auth.max_yaw_rate_rad_s:
                problems.append(f'{step.id}: yaw rate exceeds authorized maximum')
            if step.params['duration_s'] > auth.max_move_duration_s:
                problems.append(f'{step.id}: duration exceeds authorized maximum')
        if step.type in ('deadman_stop', 'blocked_move', 'approach_stop'):
            if step.params['vx'] > auth.max_speed_m_s:
                problems.append(f'{step.id}: speed exceeds authorized maximum')
            limit = (auth.max_approach_duration_s if step.type == 'approach_stop'
                     else auth.max_move_duration_s)
            if step.params['duration_s'] > limit:
                problems.append(f'{step.id}: duration exceeds authorized maximum')
            if step.type == 'approach_stop' and step.params['max_travel_m'] > auth.max_approach_travel_m:
                problems.append(f'{step.id}: travel exceeds authorized maximum')
        if step.type == 'nav_goal':
            if step.params['max_speed'] > auth.max_speed_m_s:
                problems.append(f'{step.id}: navigation speed exceeds authorized maximum')
            if math.hypot(step.params['forward_m'], step.params['left_m']) > auth.max_nav_distance_m:
                problems.append(f'{step.id}: goal distance exceeds authorized maximum')
    return problems


def authorization_template(sequence, hours=2.0):
    until = (dt.datetime.now(dt.timezone.utc) + dt.timedelta(hours=min(hours, 4.0))).replace(
        microsecond=0)
    moves = [s for s in sequence.all_steps if s.type == 'velocity_move']
    guarded = [s for s in sequence.all_steps if s.type in ('deadman_stop', 'blocked_move')]
    approach = [s for s in sequence.all_steps if s.type == 'approach_stop']
    goals = [s for s in sequence.all_steps if s.type == 'nav_goal']
    return {
        'session': f'{sequence.name}-{until:%Y%m%dT%H%M}',
        'sequence_sha256': sequence.sha256,
        'valid_until': until.isoformat(),
        'limits': {
            'max_speed_m_s': max([abs(s.params['vx']) for s in moves + guarded + approach] +
                                 [s.params['max_speed'] for s in goals] or [0.0]),
            'max_yaw_rate_rad_s': max([abs(s.params['wz']) for s in moves] +
                                      ([NAV2_MAX_YAW_RATE_RAD_S] if goals else []) or [0.0]),
            'max_move_duration_s': max([s.params['duration_s'] for s in moves + guarded] or [0.0]),
            'max_nav_distance_m': max([math.hypot(s.params['forward_m'], s.params['left_m'])
                                       for s in goals] or [0.0]),
            'max_approach_duration_s': max([s.params['duration_s'] for s in approach] or [0.0]),
            'max_approach_travel_m': max([s.params['max_travel_m'] for s in approach] or [0.0]),
            'allow_posture_commands': any(s.type == 'posture_transition'
                                          for s in sequence.all_steps),
        },
        'operator_confirmations': {name: False for name in required_confirmations(sequence)},
    }
