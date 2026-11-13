"""A malformed ROS manifest can silently fall back to plain Python discovery."""

from pathlib import Path

from catkin_pkg.package import parse_package


def test_manifest_is_valid_and_declares_ament_python():
    root = Path(__file__).resolve().parents[1]
    package = parse_package(str(root / 'package.xml'))
    assert package.name == 'lite3_locomotion'
    assert package.get_build_type() == 'ament_python'
