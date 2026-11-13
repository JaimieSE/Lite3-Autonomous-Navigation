"""Opt-in end-to-end DDS dry runs against the simulator (isolated loopback domain).

Run with LITE3_MT_RUN_DDS=1 after building and sourcing the workspace.
"""

import os
import subprocess

import pytest

pytestmark = pytest.mark.skipif(os.environ.get('LITE3_MT_RUN_DDS') != '1',
                                reason='opt-in: needs sourced ROS 2 and transfer_interfaces')


@pytest.mark.parametrize('fault, code', [('none', 0), ('march_in_place', 1),
                                         ('remote_takeover', 2), ('ignore_posture', 2),
                                         ('telemetry_dropout', 2)])
def test_sim_dry_run_exit_codes(tmp_path, fault, code):
    result = subprocess.run(
        ['ros2', 'run', 'lite3_movement_tests', 'movement_test', '--report-dir', str(tmp_path),
         'dry-run', '--sim', '--fault', fault], capture_output=True, text=True, timeout=150)
    assert result.returncode == code, result.stdout[-3000:] + result.stderr[-2000:]
    assert list(tmp_path.glob('*-simulation/report.json'))
