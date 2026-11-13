import struct

import pytest

from lite3_locomotion.command_policy import PolicyConfig, Velocity
from lite3_locomotion.protocol_preview import preview_packets


def test_packet_layout_headers_axis_order_and_yaw_sign():
    packets = preview_packets(Velocity(linear_x=0.04, linear_y=-0.02, angular_z=0.08))
    assert len(packets) == 3
    assert all(isinstance(packet, bytes) and len(packet) == 20 for packet in packets)
    expected = ((320, 8, 1, 0.04), (325, 8, 1, -0.02), (321, 8, 1, -0.08))
    for packet, fields in zip(packets, expected):
        assert struct.unpack('<iiid', packet) == fields


def test_fixed_golden_packet_has_no_double_alignment_padding():
    # Forward code 0x140, length 8, type 1, IEEE754 double 0.125 at offset 12.
    packets = preview_packets(Velocity(linear_x=0.125), PolicyConfig(max_linear_x=0.125))
    assert packets[0].hex() == '400100000800000001000000000000000000c03f'


@pytest.mark.parametrize('sign', [-1, 1])
def test_preview_enforces_configured_limits(sign):
    packets = preview_packets(
        Velocity(linear_x=sign * 10.0, linear_y=sign * 10.0, angular_z=sign * 10.0),
        PolicyConfig(max_linear_x=0.03, max_linear_y=0.01, max_angular_z=0.02),
    )
    assert [struct.unpack_from('<d', packet, 12)[0] for packet in packets] == [
        sign * 0.03, sign * 0.01, -sign * 0.02,
    ]


def test_zero_velocity_encodes_zero_for_all_axes():
    assert all(struct.unpack_from('<d', packet, 12)[0] == 0.0
               for packet in preview_packets(Velocity()))


@pytest.mark.parametrize('field', list(Velocity.__dataclass_fields__))
@pytest.mark.parametrize('value', [float('nan'), float('inf'), -float('inf')])
def test_nonfinite_commands_never_produce_packets(field, value):
    with pytest.raises(ValueError):
        preview_packets(Velocity(**{field: value}))


@pytest.mark.parametrize('field', ['linear_z', 'angular_x', 'angular_y'])
def test_unsupported_axis_never_produces_packets(field):
    with pytest.raises(ValueError):
        preview_packets(Velocity(**{field: 0.01}))


def test_wrong_command_type_never_produces_packets():
    with pytest.raises(ValueError):
        preview_packets(None)
