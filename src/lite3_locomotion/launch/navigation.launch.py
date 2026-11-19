"""University perception + SLAM + Nav2 with Lite3 limits, terrain hazards and proximity lock.

Reuses (unchanged) university launch files: lite3_bringup.launch.py (Fast-LIO,
odom relay, /scan) and lite3_navigation.launch.py (SLAM or localization + Nav2),
with params_file = generated params/nav2_lite3.yaml. Starts twist_mux with the
university topic config plus params/twist_mux_locks.yaml, and terrain_monitor.

Final command authority (one chain, every source):
  twist_mux (priorities, dead-man + terrain health/backup locks) -> /lite3_ops/mux_out
  -> nav2_collision_monitor (stop polygon from the measured stopping boundary) -> output
The collision_monitor is the ONLY publisher of the output topic.

SAFE DEFAULT: the output goes to /lite3_locomotion/dry_cmd_vel, which no
robot interface subscribes to. Driving the robot needs BOTH
  mux_output:=/cmd_vel allow_robot_motion:=true
and must only be used inside an authorized supervised session. In that mode
params/twist_mux_physical.yaml adds the operator dead-man lock (/lite3_ops/hold):
nothing below priority 50 reaches the robot unless a supervising test runner
holds it released, so an idle physical stack is locked.
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap

DRY_OUTPUT = '/lite3_locomotion/dry_cmd_vel'
MUX_OUT = '/lite3_ops/mux_out'
# Fast-LIO and terrain_monitor read robot sensors from lite3_locomotion/sensor_relay (domain 64 direct)
# instead of the robot-side bridge, which throttles them (IMU 12 Hz / 1.3 s late on 2026-11-18,
# joints 23 Hz on 2026-11-19): see sensor_relay.py.
IMU_SOURCE = '/livox/imu'
IMU_DIRECT = '/lite3/livox_imu'
LIDAR_SOURCE, LIDAR_DIRECT = '/livox/lidar', '/lite3/livox_lidar'      # sensor_relay, domain 64
POINTS_SOURCE, POINTS_DIRECT = '/livox/points', '/lite3/livox_points'


def _setup(context):
    share = get_package_share_directory('lite3_locomotion')
    university = get_package_share_directory('lite3_launcher')
    output = LaunchConfiguration('mux_output').perform(context)
    allow = LaunchConfiguration('allow_robot_motion').perform(context).lower() == 'true'
    if output != DRY_OUTPUT and not allow:
        raise RuntimeError(f'mux_output={output} requires allow_robot_motion:=true')
    if allow and output != '/cmd_vel':
        raise RuntimeError('allow_robot_motion:=true is only meaningful with mux_output:=/cmd_vel')
    text = open(os.path.join(share, 'params', 'nav2_lite3.yaml')).read()
    text = text.replace('__LITE3_LOCOMOTION_SHARE__', share)
    handle, params_file = tempfile.mkstemp(prefix='nav2_lite3_', suffix='.yaml')
    with os.fdopen(handle, 'w') as stream:
        stream.write(text)
    from lite3_locomotion import collision_profile as CP
    import yaml
    cal = CP.load_calibration(LaunchConfiguration('calibration').perform(context))
    tier = CP.tiers(float(LaunchConfiguration('clearance').perform(context)),
                    float(LaunchConfiguration('max_speed').perform(context)), cal)
    bound = tier['stop']
    cm_params = CP.collision_monitor_params(
        bound['boundary_m'], MUX_OUT, output,
        tier['slow']['boundary_m'] if tier['slow'] else None, tier['slowdown_ratio'])
    if tier['slow']:
        print(f'[lite3] collision_monitor SLOWDOWN polygon {tier["slow"]["boundary_m"]} m from the nose, '
              f'ratio {tier["slowdown_ratio"]} (-> <= {CP.SLOW_TIER_SPEED_M_S} m/s near obstacles)', flush=True)
    handle, cm_file = tempfile.mkstemp(prefix='collision_monitor_lite3_', suffix='.yaml')
    with os.fdopen(handle, 'w') as stream:
        yaml.safe_dump(cm_params, stream)
    print(f'[lite3] collision_monitor stop boundary {bound["boundary_m"]} m from the nose: {bound}; '
          f'calibration {cal}', flush=True)
    include = os.path.join(university, 'launch', 'include')
    mux_params = [os.path.join(university, 'config', 'twist_mux_topics.yaml'),
                  os.path.join(share, 'params', 'twist_mux_locks.yaml')]
    if allow:   # dead-man lock + supervised test/stop inputs; see params/twist_mux_physical.yaml
        mux_params.append(os.path.join(share, 'params', 'twist_mux_physical.yaml'))
    return [
        Node(package='lite3_locomotion', executable='sensor_relay', name='lite3_sensor_relay', output='screen'),
        GroupAction([SetRemap(src=IMU_SOURCE, dst=IMU_DIRECT),     # scoped: only the bringup (Fast-LIO)
                     SetRemap(src=LIDAR_SOURCE, dst=LIDAR_DIRECT),
                     IncludeLaunchDescription(PythonLaunchDescriptionSource(
                         os.path.join(include, 'lite3_bringup.launch.py')))]),
        Node(package='lite3_locomotion', executable='terrain_monitor', name='lite3_terrain_monitor',
             output='screen', remappings=[('/joint_states', '/lite3/joint_states'),
                                          ('/imu/data', '/lite3/imu_data'),
                                          (POINTS_SOURCE, POINTS_DIRECT)]),
        Node(package='twist_mux', executable='twist_mux', name='twist_mux', output='screen',
             parameters=mux_params, remappings=[('cmd_vel_out', MUX_OUT)]),
        Node(package='nav2_collision_monitor', executable='collision_monitor',
             name='collision_monitor', output='screen', parameters=[cm_file]),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_collision_monitor', output='screen',
             parameters=[{'autostart': True, 'node_names': ['collision_monitor'],
                          'bond_timeout': 4.0}]),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(include, 'lite3_navigation.launch.py')),
            launch_arguments={'slam': LaunchConfiguration('slam').perform(context),
                              'params_file': params_file}.items()),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('mux_output', default_value=DRY_OUTPUT),
        DeclareLaunchArgument('allow_robot_motion', default_value='false'),
        DeclareLaunchArgument('slam', default_value='True'),
        DeclareLaunchArgument('clearance', default_value='0.15',
                              description='minimum obstacle clearance (m) the stop must leave'),
        DeclareLaunchArgument('max_speed', default_value='0.10',
                              description='fastest speed the boundary must cover (m/s, <= 0.5; validated to 0.10)'),
        DeclareLaunchArgument('calibration', default_value='',
                              description='operator calibration.json (measured latency/coast)'),
        OpaqueFunction(function=_setup),
    ])
