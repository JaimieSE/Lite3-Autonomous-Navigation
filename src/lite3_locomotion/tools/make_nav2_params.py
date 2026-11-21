"""Generate params/nav2_lite3.yaml = university nav2_params.yaml + explicit Lite3 patches.

The university file is read, never edited. Every change is listed in PATCHES so it
is reviewable and tested. Re-run after the university file changes:

    python3 tools/make_nav2_params.py [--check]

Validated envelope (physical trial 2026-11-13): 0.05-0.10 m/s forward/back (ceiling raised to 0.5 m/s
on 2026-11-17 and to 1.0 m/s on 2026-11-18: speeds above 0.10 are not validated, each session caps them),
+-0.2 rad/s turns, verified stops; the robot coasts ~1.6 s when commands stop.
Reversing is disabled because the Mid-360 sees almost nothing behind the robot.
"""

import argparse
import copy
from pathlib import Path
import sys

import yaml

UNIVERSITY = Path('/home/user/ros2_ws/src/lite3_launcher/config/nav2_params.yaml')
OUTPUT = Path(__file__).resolve().parents[1] / 'params' / 'nav2_lite3.yaml'
SHARE_PLACEHOLDER = '__LITE3_LOCOMOTION_SHARE__'

MAX_VX = 1.00     # hard ceiling (2026-11-18, was 0.50); every runner caps the session speed below it, verified
MAX_WZ = 1.50     # manufacturer navigation-mode limit (Motion Host Communication Interface 1.2.12, 0x0141: [-1.5, 1.5] rad/s)
# Perception reach for up to 1.0 m/s: the collision boundary can reach 3.0 m from the nose (3.3 m from
# base_link), so costmaps must see beyond it.
LOCAL_COSTMAP_SIZE_M = 8
# Speed (2026-11-21): the robot walked ~0.2 m/s at a 0.5 m/s cap because MPPI's goal critic aims to arrive
# at the end of its horizon (speed ~ distance / horizon on short walk goals) and samples speeds narrowly.
# The vendor lite_nav2.yaml (Perception guide s8.3.3) is on the robot and was not readable here.
MPPI_VX_STD = 0.2          # was 0.1 (university); 0.2 is the Nav2 MPPI default
MPPI_TIME_STEPS = 40       # was 56 (2.8 s horizon at model_dt 0.05); 40 = 2.0 s
CONTROLLER_FREQUENCY_HZ = 20.0   # was 30; = 1 / model_dt as Nav2 warns at startup
SCAN_OBSTACLE_RANGE_M = 3.5
SCAN_RAYTRACE_RANGE_M = 4.0

HAZARD_SOURCE = {
    'topic': '/lite3_terrain/hazards',
    'data_type': 'PointCloud2',
    'marking': True,
    'clearing': False,
    'min_obstacle_height': -2.0,   # drop-off cells sit at floor level, below odom z=0
    'max_obstacle_height': 2.0,
    'obstacle_max_range': 1.0,     # was 1.5 (operator: smaller hazard cloud), 4.0 (2026-11-21 RViz: hazards to 4 m left lethal blobs all round
                                   # the course that nothing clears; LiDAR scan + SLAM cover longer range)
    'obstacle_min_range': 0.0,
    'expected_update_rate': 0.5,   # costmap goes stale (controller stops) if terrain_monitor dies
}


def _set(tree, dotted, value):
    keys = dotted.split('.')
    node = tree
    for key in keys[:-1]:
        node = node.setdefault(key, {})
    node[keys[-1]] = value


def _get(tree, dotted):
    node = tree
    for key in dotted.split('.'):
        node = node[key]
    return node


def patches():
    mppi = 'controller_server.ros__parameters.FollowPath.'
    out = [
        (mppi + 'vx_max', MAX_VX),
        (mppi + 'vx_min', 0.0),
        (mppi + 'vy_max', 0.0),
        (mppi + 'wz_max', MAX_WZ),
        (mppi + 'vx_std', MPPI_VX_STD),
        (mppi + 'time_steps', MPPI_TIME_STEPS),
        ('controller_server.ros__parameters.controller_frequency', CONTROLLER_FREQUENCY_HZ),
        (mppi + 'wz_std', 0.4),            # Nav2 default; 0.2 barely explored turns once wz_max was 1.5
        (mppi + 'ax_max', 0.5),
        (mppi + 'ax_min', -0.5),
        (mppi + 'az_max', 0.5),
        (mppi + 'prune_distance', 3.0),     # >= MPPI horizon at 1.0 m/s (40 x 0.05 s = 2.0 m)
        ('local_costmap.local_costmap.ros__parameters.width', LOCAL_COSTMAP_SIZE_M),
        ('local_costmap.local_costmap.ros__parameters.height', LOCAL_COSTMAP_SIZE_M),
        ('velocity_smoother.ros__parameters.smoothing_frequency', 20.0),
        ('velocity_smoother.ros__parameters.feedback', 'OPEN_LOOP'),
        ('velocity_smoother.ros__parameters.max_velocity', [MAX_VX, 0.0, MAX_WZ]),
        ('velocity_smoother.ros__parameters.min_velocity', [0.0, 0.0, -MAX_WZ]),
        ('velocity_smoother.ros__parameters.max_accel', [0.5, 0.0, 0.5]),
        ('velocity_smoother.ros__parameters.max_decel', [-0.5, 0.0, -0.5]),
        ('velocity_smoother.ros__parameters.velocity_timeout', 0.5),
        ('behavior_server.ros__parameters.costmap_topic', 'local_costmap/costmap_raw'),
        ('behavior_server.ros__parameters.footprint_topic', 'local_costmap/published_footprint'),
        ('behavior_server.ros__parameters.cycle_frequency', 10.0),
        ('behavior_server.ros__parameters.behavior_plugins', ['spin', 'wait']),
        ('behavior_server.ros__parameters.spin', {'plugin': 'nav2_behaviors/Spin'}),
        ('behavior_server.ros__parameters.wait', {'plugin': 'nav2_behaviors/Wait'}),
        ('behavior_server.ros__parameters.global_frame', 'odom'),
        ('behavior_server.ros__parameters.robot_base_frame', 'base_link'),
        ('behavior_server.ros__parameters.transform_tolerance', 0.1),
        ('behavior_server.ros__parameters.simulate_ahead_time', 2.0),
        ('behavior_server.ros__parameters.max_rotational_vel', MAX_WZ),
        ('behavior_server.ros__parameters.min_rotational_vel', 0.1),
        ('behavior_server.ros__parameters.rotational_acc_lim', 0.5),
        ('bt_navigator.ros__parameters.default_nav_to_pose_bt_xml',
         SHARE_PLACEHOLDER + '/params/navigate_to_pose_no_backup.xml'),
        ('bt_navigator.ros__parameters.default_nav_through_poses_bt_xml',
         SHARE_PLACEHOLDER + '/params/navigate_through_poses_no_backup.xml'),
    ]
    for costmap in ('local_costmap.local_costmap', 'global_costmap.global_costmap'):
        base = costmap + '.ros__parameters.obstacle_layer.'
        out.append((base + 'observation_sources', 'scan hazards'))
        out.append((base + 'hazards', dict(HAZARD_SOURCE)))
        # The voxel layer only duplicates the planar /scan already in obstacle_layer, and its z range
        # starts at origin_z 0.0 while the scan origin (base_link in odom) sits at about -0.01 m: it
        # "cannot raytrace" (thousands of warnings in every launch.log since 2026-11-17), so it marks
        # but never clears, and stale obstacles (people walking past) stay in both costmaps for good.
        out.append((costmap + '.ros__parameters.voxel_layer.enabled', False))
        # Course <= 1.5 m wide (operator 2026-11-20): a 1.0 m inflation covered the whole lane, so MPPI
        # paid obstacle cost everywhere; 0.55 leaves the middle ~0.4 m cost-free, steeper fall-off.
        # was 0.55 (operator 2026-11-21: course ~1.5 m wide, the inflation band hid the walkable path)
        out.append((costmap + '.ros__parameters.inflation_layer.inflation_radius', 0.30))
        out.append((costmap + '.ros__parameters.inflation_layer.cost_scaling_factor', 4.0))
        for layer in ('obstacle_layer', 'voxel_layer'):
            scan = costmap + '.ros__parameters.' + layer + '.scan.'
            out.append((scan + 'obstacle_max_range', SCAN_OBSTACLE_RANGE_M))
            out.append((scan + 'raytrace_max_range', SCAN_RAYTRACE_RANGE_M))
    return out


def generate(university_text):
    data = yaml.safe_load(university_text)
    result = copy.deepcopy(data)
    for dotted, value in patches():
        _set(result, dotted, value)
    header = ('# GENERATED by tools/make_nav2_params.py from the university\n'
              '# lite3_launcher/config/nav2_params.yaml. Do not edit; change PATCHES instead.\n')
    return header + yaml.safe_dump(result, sort_keys=False, default_flow_style=None)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check', action='store_true', help='fail if output is out of date')
    parser.add_argument('--university', default=str(UNIVERSITY))
    args = parser.parse_args()
    text = generate(Path(args.university).read_text())
    if args.check:
        if not OUTPUT.exists() or OUTPUT.read_text() != text:
            print('params/nav2_lite3.yaml is out of date with the university file')
            return 1
        print('up to date')
        return 0
    OUTPUT.parent.mkdir(exist_ok=True)
    OUTPUT.write_text(text)
    print(f'wrote {OUTPUT}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
