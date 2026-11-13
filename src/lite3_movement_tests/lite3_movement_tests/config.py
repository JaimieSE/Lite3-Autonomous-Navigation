"""Sequence and physical-session authorization loading with hard, code-level limits."""

from dataclasses import dataclass, field, fields
import datetime as dt
import hashlib
import math
from pathlib import Path

import yaml

from .geometry import LYING, STANDING, PostureCriteria
from .telemetry import StationaryCriteria

# Ceilings no configuration or authorization can raise.
HARD_MAX_SPEED_M_S = 0.10
HARD_MAX_YAW_RATE_RAD_S = 0.30
HARD_MAX_MOVE_S = 3.0
HARD_MAX_POSTURE_TIMEOUT_S = 20.0
HARD_MAX_STEP_TIMEOUT_S = 30.0
HARD_MAX_SESSION_S = 600.0
HARD_MAX_AUTH_WINDOW = dt.timedelta(hours=4)

STEP_TYPES = {
    'readiness', 'telemetry_validation', 'stationary_check', 'posture_check',
    'posture_transition', 'velocity_move', 'stop_verify',
}
ACTUATING_TYPES = {'posture_transition', 'velocity_move', 'stop_verify'}

REQUIRED_CONFIRMATIONS = (
    'area_clear_radius_1m',
    'emergency_stop_in_hand_and_tested',
    'exclusive_robot_control',
    'navigation_mode_selected',
    'operator_supervising_entire_session',
    'accept_no_lidar_collision_gate',
    'accept_unverified_firmware_command_timeout',
)


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
        timeout = p['warmup_s'] + p['duration_s'] + p['settle_s'] + 2.0
    if kind == 'stop_verify':
        p['zero_min_s'] = _finite(take('zero_min_s', 1.0), f'{step_id}.zero_min_s', 0.5, 5.0)
    timeout = _finite(timeout, f'{step_id}.timeout_s', 0.5, HARD_MAX_STEP_TIMEOUT_S)
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
    sha256: str = ''
    source: str = ''

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
             'stationary'}
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
    for i, step in enumerate(steps):
        if step.type == 'velocity_move':
            nxt = steps[i + 1] if i + 1 < len(steps) else None
            if nxt is None or nxt.type != 'stop_verify':
                raise ConfigError(f'{step.id}: every velocity_move must be followed by stop_verify')
    telemetry = _dataclass_from(TelemetryRequirements, raw.get('telemetry'), 'telemetry')
    telemetry = TelemetryRequirements(**{**telemetry.__dict__, 'required_sensor_alive':
                                         tuple(telemetry.required_sensor_alive)})
    sequence = Sequence(
        name=str(raw.get('name') or path.stem), steps=steps, finalize=finalize,
        telemetry=telemetry,
        safety=_dataclass_from(SafetyLimits, raw.get('safety'), 'safety'),
        posture=_dataclass_from(PostureCriteria, raw.get('posture'), 'posture'),
        stationary=_dataclass_from(StationaryCriteria, raw.get('stationary'), 'stationary'),
        sha256=hashlib.sha256(data).hexdigest(), source=str(path),
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
    for name in REQUIRED_CONFIRMATIONS:
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
    return problems


def authorization_template(sequence, hours=2.0):
    until = (dt.datetime.now(dt.timezone.utc) + dt.timedelta(hours=min(hours, 4.0))).replace(
        microsecond=0)
    moves = [s for s in sequence.all_steps if s.type == 'velocity_move']
    return {
        'session': f'{sequence.name}-{until:%Y%m%dT%H%M}',
        'sequence_sha256': sequence.sha256,
        'valid_until': until.isoformat(),
        'limits': {
            'max_speed_m_s': max([abs(s.params['vx']) for s in moves] or [0.0]),
            'max_yaw_rate_rad_s': max([abs(s.params['wz']) for s in moves] or [0.0]),
            'max_move_duration_s': max([s.params['duration_s'] for s in moves] or [0.0]),
            'allow_posture_commands': any(s.type == 'posture_transition'
                                          for s in sequence.all_steps),
        },
        'operator_confirmations': {name: False for name in REQUIRED_CONFIRMATIONS},
    }
