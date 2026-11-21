"""Depth image -> base_link points (no ROS)."""
import numpy as np

from lite3_locomotion import depth_points as D

FX = FY = 372.5
CX, CY = 316.1, 239.0


def render_floor(floor_z=-0.33, h=480, w=640):
    """Depth image (mm) of a flat floor floor_z below base_link, seen from the robot's camera pose."""
    v, u = np.mgrid[0:h, 0:w]
    rays = np.stack([(u - CX) / FX, (v - CY) / FY, np.ones_like(u, float)], axis=-1) @ D.CAMERA_R.T
    t = (floor_z - D.CAMERA_T[2]) / rays[..., 2]
    depth = np.where(t > 0, t, 0.0)                      # optical z of the hit = t (unit z ray)
    return (depth * 1000).astype(np.uint16)


def test_camera_looks_forward_and_down():
    forward = D.CAMERA_R @ np.array([0.0, 0.0, 1.0])     # optical axis in base_link
    assert forward[0] > 0.9 and forward[2] < -0.2          # ~20 deg below horizontal


def test_flat_floor_comes_back_at_floor_height_in_the_near_field():
    pts = D.depth_to_base(render_floor(), FX, FY, CX, CY)
    assert 1000 < len(pts) < 5000                          # sparse (stride 8), near field only
    assert np.allclose(pts[:, 2], -0.33, atol=0.01)
    assert pts[:, 0].min() > 0.2 and np.hypot(pts[:, 0] - D.CAMERA_T[0], pts[:, 2] - D.CAMERA_T[2]).max() < 1.6


def test_invalid_and_far_pixels_are_dropped():
    depth = np.zeros((480, 640), np.uint16)
    depth[:, :320] = 5000                                  # 5 m: beyond the near field
    assert len(D.depth_to_base(depth, FX, FY, CX, CY)) == 0
