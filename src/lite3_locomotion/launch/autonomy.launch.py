"""Nav2 + disarmed adapter + idle exploration. No robot driver or mode changes."""

from pathlib import Path
import uuid

from ament_index_python.packages import get_package_prefix, get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def generate_launch_description():
    share = Path(get_package_share_directory('lite3_locomotion'))
    prefix = Path(get_package_prefix('lite3_locomotion'))
    parameters = yaml.safe_load((share / 'config/nav2_frontiers.yaml').read_text())
    parameters['lite3_autonomy']['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = str(
        share / 'config/navigate_frontiers.xml')
    parameters['lite3_autonomy']['bt_navigator']['ros__parameters']['default_nav_through_poses_bt_xml'] = str(
        share / 'config/navigate_through_frontiers.xml')
    # Avoid launch-generated parameter files outside this package's installation.
    runtime = share / 'runtime'
    runtime.mkdir(exist_ok=True)
    params_file = runtime / ('nav2_' + uuid.uuid4().hex + '.yaml')
    params_file.write_text(yaml.safe_dump(parameters, sort_keys=False))
    nav2 = []
    for package, executable in (
        ('nav2_controller', 'controller_server'), ('nav2_planner', 'planner_server'),
        ('nav2_bt_navigator', 'bt_navigator'),
    ):
        nav2.append(Node(
            package=package, executable=executable, name=executable, namespace='lite3_autonomy',
            parameters=[str(params_file)], output='screen',
            remappings=[('tf', '/tf'), ('tf_static', '/tf_static'),
                        ('cmd_vel', '/lite3_autonomy/nav_cmd_vel')],
        ))
    manager_file = runtime / ('lifecycle_' + uuid.uuid4().hex + '.yaml')
    manager_file.write_text(yaml.safe_dump({'lite3_autonomy': {'lifecycle_manager': {'ros__parameters': {
        'autostart': True, 'use_sim_time': False,
        'node_names': ['controller_server', 'planner_server', 'bt_navigator'],
    }}}}))
    manager = Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
                   name='lifecycle_manager', namespace='lite3_autonomy',
                   parameters=[str(manager_file)], output='screen')
    adapter = ExecuteProcess(cmd=[str(prefix / 'lib/lite3_locomotion/guarded_movement_adapter'),
                                  '--output-mode', LaunchConfiguration('output_mode'),
                                  '--radius', LaunchConfiguration('radius'),
                                  '--session-seconds', LaunchConfiguration('session_seconds')], output='screen')
    explorer = ExecuteProcess(cmd=[str(prefix / 'lib/lite3_locomotion/exploration_coordinator'),
                                   '--radius', LaunchConfiguration('radius')], output='screen')
    shutdowns = [RegisterEventHandler(OnProcessExit(
        target_action=process,
        on_exit=[EmitEvent(event=Shutdown(reason='Autonomy component exited'))],
    )) for process in nav2 + [manager, adapter, explorer]]
    return LaunchDescription([
        DeclareLaunchArgument('output_mode', default_value='dry-run', choices=['dry-run', 'robot']),
        DeclareLaunchArgument('radius', default_value='3.0'),
        DeclareLaunchArgument('session_seconds', default_value='60.0'),
        *shutdowns, *nav2, manager, adapter, explorer,
    ])
