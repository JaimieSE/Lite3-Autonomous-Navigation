"""The generated Nav2 settings, behavior tree, mux lock and launch default stay safe."""

from pathlib import Path
import subprocess
import sys

import pytest
import yaml

ROOT = Path(__file__).resolve().parents[1]
PARAMS = yaml.safe_load((ROOT / 'params' / 'nav2_lite3.yaml').read_text())


def get(*keys):
    node = PARAMS
    for key in keys:
        node = node[key]
    return node


@pytest.mark.skipif(not Path('/home/user/ros2_ws/src/lite3_launcher/config/nav2_params.yaml').exists(),
                    reason='university workspace not present')
def test_generated_file_matches_university_source():
    result = subprocess.run([sys.executable, str(ROOT / 'tools' / 'make_nav2_params.py'), '--check'],
                            capture_output=True, text=True)
    assert result.returncode == 0, result.stdout


def test_velocity_caps_match_validated_envelope_and_no_reverse():
    mppi = get('controller_server', 'ros__parameters', 'FollowPath')
    assert mppi['vx_max'] <= 0.10 and mppi['vx_min'] == 0.0 and mppi['wz_max'] <= 0.20
    assert mppi['vy_max'] == 0.0
    smoother = get('velocity_smoother', 'ros__parameters')
    assert smoother['max_velocity'][0] <= 0.10 and smoother['min_velocity'][0] == 0.0
    assert smoother['max_velocity'][2] <= 0.20 and smoother['min_velocity'][2] >= -0.20
    behaviors = get('behavior_server', 'ros__parameters')
    assert behaviors['behavior_plugins'] == ['spin', 'wait']
    assert behaviors['max_rotational_vel'] <= 0.20


def test_behavior_tree_has_no_blind_reverse():
    for name, key in (('navigate_to_pose', 'default_nav_to_pose_bt_xml'),
                      ('navigate_through_poses', 'default_nav_through_poses_bt_xml')):
        xml = (ROOT / 'params' / f'{name}_no_backup.xml').read_text()
        assert '<BackUp' not in xml and '<Spin' in xml and '<FollowPath' in xml
        bt = get('bt_navigator', 'ros__parameters', key)
        assert bt.endswith(f'/params/{name}_no_backup.xml')


@pytest.mark.parametrize('costmap', ['local_costmap', 'global_costmap'])
def test_terrain_hazards_feed_both_costmaps_and_must_stay_fresh(costmap):
    layer = get(costmap, costmap, 'ros__parameters', 'obstacle_layer')
    assert layer['observation_sources'].split() == ['scan', 'hazards']
    hazards = layer['hazards']
    assert hazards['topic'] == '/lite3_terrain/hazards' and hazards['data_type'] == 'PointCloud2'
    assert hazards['marking'] and hazards['min_obstacle_height'] < -0.5
    assert 0 < hazards['expected_update_rate'] <= 1.0


def test_mux_lock_blocks_navigation_but_not_joystick():
    lock = yaml.safe_load((ROOT / 'params' / 'twist_mux_locks.yaml').read_text())
    lock = lock['twist_mux']['ros__parameters']['locks']['terrain_stop']
    topics = yaml.safe_load(Path('/home/user/ros2_ws/src/lite3_launcher/config/twist_mux_topics.yaml')
                            .read_text())['twist_mux']['ros__parameters']['topics'] \
        if Path('/home/user/ros2_ws').exists() else {'navigation': {'priority': 10},
                                                     'joystick': {'priority': 100}}
    assert lock['topic'] == '/lite3_terrain/stop' and 0 < lock['timeout'] <= 0.5
    assert topics['navigation']['priority'] < lock['priority'] < topics['joystick']['priority']


def test_launch_defaults_to_non_actuating_output():
    src = (ROOT / 'launch' / 'navigation.launch.py').read_text()
    assert "DRY_OUTPUT = '/lite3_locomotion/dry_cmd_vel'" in src
    assert "DeclareLaunchArgument('mux_output', default_value=DRY_OUTPUT)" in src
    assert "DeclareLaunchArgument('allow_robot_motion', default_value='false')" in src
    assert 'requires allow_robot_motion:=true' in src
