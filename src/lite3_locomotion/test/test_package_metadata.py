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
    names = {entry.split('=')[0].strip() for entry in captured['entry_points']['console_scripts']}
    assert names == {'exploration_coordinator', 'autonomy_control', 'exploration_preview', 'locomotion_controller'}
    installed = [path for _, paths in captured['data_files'] for path in paths]
    assert [p for p in installed if p.startswith('launch/')] == ['launch/autonomy.launch.py']
    assert not any(p.startswith(('legacy/', 'config/')) for p in installed)
