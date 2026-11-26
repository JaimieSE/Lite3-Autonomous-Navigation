"""Leg-odometry jumps (run 26 Nov 04:24: /leg_odom2 jumped 14.8 m on entering AI state, the watchdog tripped)."""
import pytest

from lite3_movement_tests import roam as R
from lite3_movement_tests.odom_jumps import JumpFreeOdom
from lite3_movement_tests.telemetry import Telemetry


def test_ordinary_motion_counts_and_a_jump_is_dropped():
    o = JumpFreeOdom()
    for i in range(11):                                   # 0.5 m at 0.5 m/s, 10 Hz
        assert o.update(i * 0.1, i * 0.05, 0.0) is None
    assert o.distance() == pytest.approx(0.5)
    assert o.update(1.1, 14.8, 0.3) == pytest.approx(14.30, abs=0.01)   # reset to a new frame
    assert o.distance() == pytest.approx(0.5)
    for i in range(1, 5):                                 # walking on in the new frame still counts
        o.update(1.1 + i * 0.1, 14.8 + i * 0.05, 0.3)
    assert o.distance() == pytest.approx(0.7) and len(o.jumps) == 1


def test_a_long_gap_between_readings_allows_a_longer_real_step():
    o = JumpFreeOdom()
    o.update(0.0, 0.0, 0.0)
    assert o.update(2.0, 1.5, 0.0) is None                # 2 s at <= 1 m/s: real
    assert o.distance() == pytest.approx(1.5)


def test_a_preset_origin_is_kept():
    o = JumpFreeOdom(origin=(0.0, 0.0))
    o.update(0.0, 4.0, 0.0)
    assert o.distance() == pytest.approx(4.0)


def _odom(t, ts, x):
    t.on_joint(ts, ('a',), (0.0,))
    t.on_odom(ts, x, 0, 0.3, 0, 0, 0, 1, 0, 0, 0)
    t.on_imu(ts, 0, 0, 0, 1, (0, 0, 0), (0, 0, 9.8))
    t.raw_cloud.add(ts, 360000)
    t.on_system_cmd(ts, 0.05, 0.0, 0.0)


def test_watchdog_geofence_ignores_a_jump_but_still_trips_on_real_distance():
    logic = R.WatchdogLogic(0.5, 10.0)
    t = Telemetry()
    _odom(t, 100.0, 0.0)
    assert logic.evaluate(t, 100.0) == []
    _odom(t, 100.1, 14.8)                                 # the 04:24 jump
    assert not any('geofence' in r for r in logic.evaluate(t, 100.1))
    assert logic.notes and 'jump' in logic.notes[0]
    x, ts = 14.8, 100.1
    while ts < 130.0:                                     # then a real 11 m walk at 0.4 m/s
        ts, x = ts + 0.1, x + 0.04
        _odom(t, ts, x)
        reasons = logic.evaluate(t, ts)
        if any('geofence' in r for r in reasons):
            break
    assert any('geofence' in r for r in reasons) and x - 14.8 == pytest.approx(10.8, abs=0.1)
