"""RealSense depth image -> a sparse near-field point set in base_link (operator 2026-11-21: "add the depth camera").

The LiDAR cannot see the floor right in front of the feet (94 % of the near field unmapped on the course
run), so low edges (the ~0.10-0.12 m ledge lip) and wall bases were met blind. The Lite3 Pro/LiDAR carries an
Intel RealSense D435i (Perception guide s4); on this robot it publishes /camera/depth/image_raw (640 x 480,
16-bit mm, ~10 Hz, 0.61 MB) and /camera/depth/points (3.67 MB per cloud, too heavy for the robot link).
So the relay takes the image and keeps every STRIDE-th pixel within MAX_RANGE_M: ~4800 points, ~60 KB.

Camera pose in base_link: read from the robot's TF on 2026-11-21 (base_link -> camera_depth_optical_frame:
t = (0.252, -0.001, 0.074) m, q_xyzw = (0.579, -0.579, 0.406, -0.406)), matching the URDF camera mount
(0.25228, -0.0005, 0.0741, pitched 20 deg down). Intrinsics come from /camera/depth/camera_info.
"""

import numpy as np

CAMERA_T = np.array([0.252, -0.001, 0.074])
CAMERA_Q_XYZW = (0.579, -0.579, 0.406, -0.406)
STRIDE = 8                 # pixels: 640 x 480 -> 80 x 60
MIN_RANGE_M = 0.15         # D435 minimum depth is ~0.2 m at this resolution; nearer values are noise
MAX_RANGE_M = 1.5          # near field only: the LiDAR covers further out
DEPTH_SCALE_M = 0.001      # 16UC1 depth in millimetres


def quat_to_matrix(x, y, z, w):
    n = np.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / n, y / n, z / n, w / n
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


CAMERA_R = quat_to_matrix(*CAMERA_Q_XYZW)


def depth_to_base(depth, fx, fy, cx, cy, stride=STRIDE, min_range=MIN_RANGE_M, max_range=MAX_RANGE_M,
                  rotation=CAMERA_R, translation=CAMERA_T):
    """depth: HxW uint16 (mm). Returns Nx3 float32 points in base_link. Pure: unit-tested."""
    sub = depth[::stride, ::stride].astype(np.float32) * DEPTH_SCALE_M
    v, u = np.mgrid[0:depth.shape[0]:stride, 0:depth.shape[1]:stride]
    keep = (sub > min_range) & (sub < max_range)
    z = sub[keep]
    x = (u[keep] - cx) * z / fx
    y = (v[keep] - cy) * z / fy
    optical = np.stack([x, y, z], axis=1)
    return (optical @ rotation.T + translation).astype(np.float32)
