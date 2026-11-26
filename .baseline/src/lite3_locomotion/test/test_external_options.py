"""Offline argument and late-goal race checks; no DDS or robot connections."""
from types import SimpleNamespace

import pytest

from lite3_locomotion.exploration_coordinator import ACTION, parse_options


def test_university_defaults_are_preview_and_root_action():
    options = parse_options([])
    assert not options.enable_navigation
    assert options.nav2_namespace + ACTION == '/navigate_to_pose'
    assert parse_options(['--nav2-namespace', '/campus/']).nav2_namespace == '/campus'


@pytest.mark.parametrize('args', [
    ['--radius', 'nan'], ['--session-seconds', '0'], ['--session-seconds', 'inf'],
    ['--nav2-namespace', 'bad//name'], ['--nav2-namespace', '1bad'],
    ['--nav2-namespace', 'bad-name'], ['--ros-args', '-r', '__ns:=/robot'],
])
def test_invalid_options_fail_before_ros(args):
    with pytest.raises(SystemExit):
        parse_options(args)
