"""CLI refusal paths (no ROS is imported before authorization succeeds)."""

from pathlib import Path
import sys

import yaml

from lite3_movement_tests import cli, config as cfg

SEQUENCE = str(Path(__file__).resolve().parents[1] / 'config' / 'basic_walk.yaml')


def test_plan_prints_hash(capsys):
    assert cli.main(['--sequence', SEQUENCE, 'plan']) == 0
    assert cfg.load_sequence(SEQUENCE).sha256 in capsys.readouterr().out


def test_template_then_physical_is_refused_without_confirmations(tmp_path, capsys):
    auth = tmp_path / 'auth.yaml'
    assert cli.main(['--sequence', SEQUENCE, 'authorize-template', '--output', str(auth)]) == 0
    assert cli.main(['--sequence', SEQUENCE, 'authorize-template', '--output', str(auth)]) == 3
    sys.modules.pop('rclpy', None)
    assert cli.main(['--sequence', SEQUENCE, 'physical', '--authorization', str(auth)]) == 3
    out = capsys.readouterr().out
    assert 'REFUSED' in out and 'nothing was published' in out
    assert 'rclpy' not in sys.modules


def test_physical_refused_in_isolated_environment(tmp_path, monkeypatch, capsys):
    seq = cfg.load_sequence(SEQUENCE)
    data = cfg.authorization_template(seq)
    data['operator_confirmations'] = {k: True for k in cfg.REQUIRED_CONFIRMATIONS}
    auth = tmp_path / 'auth.yaml'
    auth.write_text(yaml.safe_dump(data))
    monkeypatch.setenv('ROS_LOCALHOST_ONLY', '1')
    assert cli.main(['--sequence', SEQUENCE, 'physical', '--authorization', str(auth)]) == 3
    assert 'isolated' in capsys.readouterr().out


def test_invalid_sequence_refused(tmp_path):
    bad = tmp_path / 'bad.yaml'
    bad.write_text('name: x\nsteps: []\n')
    assert cli.main(['--sequence', str(bad), 'plan']) == 3
