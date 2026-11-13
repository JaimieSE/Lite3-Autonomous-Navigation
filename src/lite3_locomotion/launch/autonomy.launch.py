"""One preview-only coordinator around the existing university stack.

No Nav2 servers, motion transport or mode services are launched. Navigation
output stays disabled in this development launch, including on a live domain.
"""
from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    executable = get_package_prefix('lite3_locomotion') + '/lib/lite3_locomotion/exploration_coordinator'
    return LaunchDescription([
        DeclareLaunchArgument('output_mode', default_value='dry-run', choices=['dry-run']),
        DeclareLaunchArgument('radius', default_value='3.0'),
        DeclareLaunchArgument('session_seconds', default_value='60.0'),
        DeclareLaunchArgument('nav2_namespace', default_value=''),
        ExecuteProcess(cmd=[executable, '--radius', LaunchConfiguration('radius'),
                            '--session-seconds', LaunchConfiguration('session_seconds'),
                            '--nav2-namespace', LaunchConfiguration('nav2_namespace')], output='screen'),
    ])
