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
    from lite3_locomotion import collision_profile as CP
    # Static ceiling = the hard speed ceiling (2026-11-17: 0.5); every session caps it lower and verifies it.
    assert mppi['vx_max'] <= CP.MAX_SPEED_M_S and mppi['vx_min'] == 0.0 and mppi['wz_max'] <= 1.5
    assert mppi['vy_max'] == 0.0
    smoother = get('velocity_smoother', 'ros__parameters')
    assert smoother['max_velocity'][0] <= CP.MAX_SPEED_M_S and smoother['min_velocity'][0] == 0.0
    assert smoother['max_velocity'][2] <= 1.5 and smoother['min_velocity'][2] >= -1.5
    behaviors = get('behavior_server', 'ros__parameters')
    assert behaviors['behavior_plugins'] == ['spin', 'wait']
    assert behaviors['max_rotational_vel'] <= 1.5


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


@pytest.mark.parametrize('costmap', ['local_costmap', 'global_costmap'])
def test_scan_is_cleared_by_a_layer_that_can_raytrace(costmap):
    """The voxel layer (origin_z 0.0) cannot raytrace from the scan origin just below odom z=0, so it
    would mark without ever clearing; only obstacle_layer handles /scan, with clearing on."""
    params = get(costmap, costmap, 'ros__parameters')
    assert params.get('voxel_layer', {}).get('enabled') is False
    scan = params['obstacle_layer']['scan']
    assert scan['clearing'] and scan['marking']


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


def test_physical_mux_config_dead_man_and_stop_priorities():
    cfg = yaml.safe_load((ROOT / 'params' / 'twist_mux_physical.yaml').read_text())
    cfg = cfg['twist_mux']['ros__parameters']
    hold = cfg['locks']['operator_hold']
    terrain = yaml.safe_load((ROOT / 'params' / 'twist_mux_locks.yaml').read_text())[
        'twist_mux']['ros__parameters']['locks']['terrain_stop']
    assert hold['topic'] == '/lite3_ops/hold' and 0 < hold['timeout'] <= 0.5
    # Nav2 (10) and test moves (20) are blocked by BOTH locks; the zeros-only stop input is above.
    assert cfg['topics']['test_cmd']['priority'] < min(hold['priority'], terrain['priority'])
    assert cfg['topics']['safety_stop']['priority'] > max(hold['priority'], terrain['priority'], 100)
    assert cfg['topics']['safety_stop']['timeout'] <= 0.25


def test_physical_mux_config_loaded_only_when_motion_allowed():
    src = (ROOT / 'launch' / 'navigation.launch.py').read_text()
    assert "    if allow:   # dead-man lock" in src
    assert "mux_params.append(os.path.join(share, 'params', 'twist_mux_physical.yaml'))" in src
