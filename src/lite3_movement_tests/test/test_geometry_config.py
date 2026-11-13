"""Posture geometry, telemetry assessment, sequence and authorization validation (no ROS)."""

import datetime as dt
from pathlib import Path

import pytest
import yaml

from lite3_movement_tests import config as cfg
from lite3_movement_tests.geometry import (
    JOINT_NAMES, LYING, STANDING, TRANSITION, UNKNOWN, assess_posture, leg_extension,
    quaternion_tilt)
from lite3_movement_tests.sim_model import LYING_POSE, STANDING_POSE
from lite3_movement_tests.telemetry import OdomSample, displacement

SEQUENCE = Path(__file__).resolve().parents[1] / 'config' / 'basic_walk.yaml'
LEVEL = (0.0, 0.0)


def test_leg_extension_matches_urdf_extremes():
    assert leg_extension(0.0) == pytest.approx(0.41012)
    assert leg_extension(-2.75) == pytest.approx(0.0806, abs=1e-3)   # live lying sample
    assert leg_extension(2.75) == leg_extension(-2.75)                 # sign independent


def test_live_lying_sample_classifies_lying():
    a = assess_posture([LYING_POSE] * 20, LEVEL)
    assert a.posture == LYING and a.confident


def test_standing_geometry_classifies_standing():
    a = assess_posture([STANDING_POSE] * 20, LEVEL)
    assert a.posture == STANDING and a.confident


def test_halfway_is_transition_and_not_confident():
    mid = [(a + b) / 2 for a, b in zip(LYING_POSE, STANDING_POSE)]
    a = assess_posture([mid] * 20, LEVEL)
    assert a.posture == TRANSITION and not a.confident


def test_moving_joints_are_not_settled():
    moving = [list(STANDING_POSE) for _ in range(20)]
    moving[5][2] -= 0.3
    assert not assess_posture(moving, LEVEL).confident


def test_disagreeing_legs_tilt_and_bad_imu_reject():
    legs = list(STANDING_POSE)
    legs[2] = -2.2      # one leg folded (0.19 m) while the others stand
    assert not assess_posture([legs] * 10, LEVEL).confident
    assert not assess_posture([STANDING_POSE] * 10, (0.4, 0.0)).confident
    assert not assess_posture([STANDING_POSE] * 10, None).confident


def test_wrong_joint_count_or_nan_is_unknown():
    assert assess_posture([STANDING_POSE[:11]], LEVEL).posture == UNKNOWN
    bad = list(STANDING_POSE)
    bad[0] = float('nan')
    assert assess_posture([bad], LEVEL).posture == UNKNOWN
    assert assess_posture([], LEVEL).posture == UNKNOWN


def test_joint_contract_order():
    assert JOINT_NAMES[2] == 'LF_Joint_2' and len(JOINT_NAMES) == 12


def test_quaternion_tilt_rejects_non_unit():
    assert quaternion_tilt(0, 0, 0, 1) == (0.0, 0.0)
    assert quaternion_tilt(0, 0, 0, 2) is None


def test_displacement_in_start_heading_frame():
    start = OdomSample(1.0, 1.0, 0.3, 1.5707963, 0, 0, 0)
    end = OdomSample(1.0, 1.5, 0.3, 1.5707963, 0, 0, 0)
    d = displacement(start, end)
    assert d['forward_m'] == pytest.approx(0.5) and d['lateral_m'] == pytest.approx(0.0, abs=1e-6)


def _write(tmp_path, data):
    path = tmp_path / 'seq.yaml'
    path.write_text(yaml.safe_dump(data))
    return path


def _base():
    return yaml.safe_load(SEQUENCE.read_text())


def test_default_sequence_loads_within_hard_limits():
    seq = cfg.load_sequence(SEQUENCE)
    moves = [s for s in seq.all_steps if s.type == 'velocity_move']
    assert moves and all(abs(s.params['vx']) <= 0.03 for s in moves)
    assert sum(s.timeout_s for s in seq.all_steps) <= cfg.HARD_MAX_SESSION_S


@pytest.mark.parametrize('mutate, message', [
    (lambda d: d['steps'][5].update(vx=0.5), 'vx'),
    (lambda d: d['steps'][5].update(duration_s=10), 'duration_s'),
    (lambda d: d['steps'][5].pop('expect'), 'expect'),
    (lambda d: d['steps'].pop(6), 'stop_verify'),
    (lambda d: d['steps'].insert(0, d['steps'].pop(2)), 'readiness'),
    (lambda d: d['steps'][3].update(target='jump'), 'target'),
    (lambda d: d['steps'][3].update(timeout_s=60), 'timeout'),
    (lambda d: d['steps'][0].update(bogus=1), 'unknown'),
    (lambda d: d['safety'].update(geofence_radius_m=10), 'geofence'),
    (lambda d: d['finalize'].append(dict(d['steps'][5], id='x')), 'finalize'),
])
def test_sequence_validation_rejects(tmp_path, mutate, message):
    data = _base()
    mutate(data)
    with pytest.raises(cfg.ConfigError, match=message):
        cfg.load_sequence(_write(tmp_path, data))


def _auth(seq, **changes):
    template = cfg.authorization_template(seq)
    template['operator_confirmations'] = {k: True for k in cfg.REQUIRED_CONFIRMATIONS}
    template.update(changes)
    return template


def _load_auth(tmp_path, data):
    path = tmp_path / 'auth.yaml'
    path.write_text(yaml.safe_dump(data))
    return cfg.load_authorization(path)


def test_template_is_unconfirmed_and_refused(tmp_path):
    seq = cfg.load_sequence(SEQUENCE)
    auth = _load_auth(tmp_path, cfg.authorization_template(seq))
    problems = cfg.check_authorization(auth, seq)
    assert len([p for p in problems if 'confirmation missing' in p]) == len(cfg.REQUIRED_CONFIRMATIONS)


def test_complete_authorization_accepted(tmp_path):
    seq = cfg.load_sequence(SEQUENCE)
    assert cfg.check_authorization(_load_auth(tmp_path, _auth(seq)), seq) == []


def test_authorization_bound_to_sequence_hash_and_expiry(tmp_path):
    seq = cfg.load_sequence(SEQUENCE)
    stale = _auth(seq, sequence_sha256='0' * 64)
    assert any('sha256' in p for p in cfg.check_authorization(_load_auth(tmp_path, stale), seq))
    past = (dt.datetime.now(dt.timezone.utc) - dt.timedelta(minutes=1)).isoformat()
    assert any('expired' in p for p in cfg.check_authorization(
        _load_auth(tmp_path, _auth(seq, valid_until=past)), seq))
    far = (dt.datetime.now(dt.timezone.utc) + dt.timedelta(hours=9)).isoformat()
    assert any('4 hours' in p for p in cfg.check_authorization(
        _load_auth(tmp_path, _auth(seq, valid_until=far)), seq))


def test_authorization_scope_limits(tmp_path):
    seq = cfg.load_sequence(SEQUENCE)
    data = _auth(seq)
    data['limits'].update(max_speed_m_s=0.01, allow_posture_commands=False)
    problems = cfg.check_authorization(_load_auth(tmp_path, data), seq)
    assert any('speed exceeds' in p for p in problems)
    assert any('posture commands not authorized' in p for p in problems)
    data['limits']['max_speed_m_s'] = 5.0
    with pytest.raises(cfg.ConfigError):
        _load_auth(tmp_path, data)


def test_authorization_requires_literal_true(tmp_path):
    seq = cfg.load_sequence(SEQUENCE)
    data = _auth(seq)
    data['operator_confirmations']['exclusive_robot_control'] = 'yes'
    assert any('exclusive_robot_control' in p for p in
               cfg.check_authorization(_load_auth(tmp_path, data), seq))
