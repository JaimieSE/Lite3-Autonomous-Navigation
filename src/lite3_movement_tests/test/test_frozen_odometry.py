"""Rest check vs the vendor leg-odometry freeze measured live on 2026-11-16 (robot lying):
x/y position bit-identical, planar speed magnitude bit-identical at 0.0681 m/s while the vx/vy
components rotate with the noisy orientation."""

import math

from lite3_movement_tests.geometry import JOINT_NAMES
from lite3_movement_tests.sim_model import LYING_POSE
from lite3_movement_tests.telemetry import Telemetry, assess_stationary

SPEED = 0.06805


def fill(position_step=0.0, joint_wiggle=0.0, speed_jitter=0.0, gyro=0.005, seconds=2.0):
    t = Telemetry()
    for i in range(int(seconds * 200)):
        ts = i / 200
        ang = 0.3 + 0.01 * math.sin(i)                    # noisy heading re-rotates the vector
        speed = SPEED + (speed_jitter if i % 2 else 0.0)
        t.on_odom(ts, 3.1 + position_step * i, 8.4, 0.08, 0, 0, math.sin(ang / 2), math.cos(ang / 2),
                  speed * math.cos(ang), speed * math.sin(ang), 0.003 * math.sin(i))
        pose = list(LYING_POSE)
        pose[0] += joint_wiggle * math.sin(i / 5)
        t.on_joint(ts, JOINT_NAMES, pose)
        t.on_imu(ts, 0, 0, 0, 1, (gyro, -gyro, gyro), (0.0, 0.0, 9.8))
    return t


def check(t):
    return assess_stationary(t, 2.0, 1.5)


def test_measured_freeze_is_rest():
    still, detail = check(fill())
    assert still, detail
    assert detail['odometry_frozen'] is True


def test_any_position_change_keeps_the_speed_criterion():
    still, detail = check(fill(position_step=1e-6))
    assert not still and 'odometry speed above rest threshold' in detail['reasons']


def test_moving_joints_keep_the_speed_criterion():
    still, detail = check(fill(joint_wiggle=0.1))
    assert not still and 'odometry speed above rest threshold' in detail['reasons']


def test_varying_speed_magnitude_is_not_a_freeze():
    still, detail = check(fill(speed_jitter=1e-4))
    assert not still and 'odometry speed above rest threshold' in detail['reasons']


def test_gyro_motion_keeps_the_speed_criterion():
    still, detail = check(fill(gyro=0.3))
    assert not still and 'odometry speed above rest threshold' in detail['reasons']
