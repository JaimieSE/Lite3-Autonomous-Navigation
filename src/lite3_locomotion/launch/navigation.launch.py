"""University perception + SLAM + Nav2 with Lite3 limits, terrain hazards and proximity lock.

Reuses (unchanged) university launch files: lite3_bringup.launch.py (Fast-LIO,
odom relay, /scan) and lite3_navigation.launch.py (SLAM or localization + Nav2),
with params_file = generated params/nav2_lite3.yaml. Starts twist_mux with the
university topic config plus params/twist_mux_locks.yaml, and terrain_monitor.

SAFE DEFAULT: twist_mux output goes to /lite3_locomotion/dry_cmd_vel, which no
robot interface subscribes to. Driving the robot needs BOTH
  mux_output:=/cmd_vel allow_robot_motion:=true
and must only be used inside an authorized supervised session.
"""

import os
import tempfile

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DRY_OUTPUT = '/lite3_locomotion/dry_cmd_vel'


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
    include = os.path.join(university, 'launch', 'include')
    return [
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
            os.path.join(include, 'lite3_bringup.launch.py'))),
        Node(package='lite3_locomotion', executable='terrain_monitor', name='lite3_terrain_monitor',
             output='screen'),
        Node(package='twist_mux', executable='twist_mux', name='twist_mux', output='screen',
             parameters=[os.path.join(university, 'config', 'twist_mux_topics.yaml'),
                         os.path.join(share, 'params', 'twist_mux_locks.yaml')],
             remappings=[('cmd_vel_out', output)]),
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
        OpaqueFunction(function=_setup),
    ])
