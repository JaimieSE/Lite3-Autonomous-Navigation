"""Offline packet preview based on official Lite3_ROS; no transport or robot I/O.

This is not the manufacturer SDK or a verified firmware-compatible driver.
Little-endian byte order is an explicit preview assumption. Upstream sends a
native C++ struct with 4-byte packing; the deployed ABI still needs verification.
"""

import struct
from typing import Tuple

from lite3_locomotion.command_policy import CommandPolicy, PolicyConfig, Velocity


UPSTREAM_COMMIT = 'e4ab5dc1c7f3ff46274162a1ef302253936578aa'
UPSTREAM_SOURCE = (
    'https://github.com/DeepRoboticsLab/Lite3_ROS/blob/' + UPSTREAM_COMMIT +
    '/src/transfer/src/Jetson2Motion.cpp'
)
_PACKET = struct.Struct('<iiid')  # Three int32 fields followed by an unpadded double.


def preview_packets(
    command: Velocity, config: PolicyConfig = PolicyConfig(),
) -> Tuple[bytes, bytes, bytes]:
    """Validate/limit velocity, then return forward/lateral/yaw packet previews.

    The caller receives immutable bytes only. This module never sends them.
    Upstream CmdVelCallback maps x to code 320, y to 325, and NEGATIVE yaw
    to 321, with payload length 8 and command type 1 for each packet.
    """
    policy = CommandPolicy(config, clock=lambda: 0.0)
    if not policy.accept(command):
        raise ValueError('Cannot preview an invalid or unsupported velocity command')
    limited = policy.output()
    return (
        _PACKET.pack(320, 8, 1, limited.linear_x),
        _PACKET.pack(325, 8, 1, limited.linear_y),
        _PACKET.pack(321, 8, 1, -limited.angular_z),
    )
