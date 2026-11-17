"""A malformed ROS manifest can silently fall back to plain Python discovery."""

from pathlib import Path

from catkin_pkg.package import parse_package


def test_manifest_is_valid_and_declares_ament_python():
    root = Path(__file__).resolve().parents[1]
    package = parse_package(str(root / 'package.xml'))
    assert package.name == 'lite3_locomotion'
    assert package.get_build_type() == 'ament_python'


def test_active_install_has_no_duplicate_stack_or_motion_entry_points(monkeypatch):
    import runpy
    import setuptools
    captured = {}
    monkeypatch.setattr(setuptools, 'setup', lambda **kwargs: captured.update(kwargs))
    root = Path(__file__).resolve().parents[1]
    monkeypatch.chdir(root)
    runpy.run_path(str(root / 'setup.py'))
    assert captured['packages'] == ['lite3_locomotion']
    names = {entry.split('=')[0].strip() for entry in captured['entry_points']['console_scripts']}
    assert names == {'exploration_coordinator', 'autonomy_control', 'exploration_preview',
                     'locomotion_controller', 'terrain_monitor'}
    installed = [path for _, paths in captured['data_files'] for path in paths]
    assert sorted(p for p in installed if p.startswith('launch/')) == [
        'launch/autonomy.launch.py', 'launch/navigation.launch.py']
    assert sorted(p for p in installed if p.startswith('params/')) == [
        'params/nav2_lite3.yaml', 'params/navigate_through_poses_no_backup.xml',
        'params/navigate_to_pose_no_backup.xml', 'params/twist_mux_locks.yaml', 'params/twist_mux_physical.yaml']
    assert not any(p.startswith(('legacy/', 'config/')) for p in installed)


def test_installed_source_has_no_legacy_or_transport_dependencies():
    import ast
    root = Path(__file__).resolve().parents[1] / 'lite3_locomotion'
    retired = {'motion_sender', 'guarded_adapter', 'commissioning_pulse',
               'bounded_test', 'protocol_preview', 'movement_guard'}
    assert not retired.intersection(p.stem for p in root.glob('*.py'))
    for path in root.glob('*.py'):
        for node in ast.walk(ast.parse(path.read_text())):
            imports = ([node.module] if isinstance(node, ast.ImportFrom) and node.module else
                       [alias.name for alias in node.names] if isinstance(node, ast.Import) else [])
            assert not any(name.startswith(('lite3_legacy', 'socket', 'transfer_interfaces')) or
                           name in {'lite3_locomotion.' + module for module in retired}
                           for name in imports), path.name
