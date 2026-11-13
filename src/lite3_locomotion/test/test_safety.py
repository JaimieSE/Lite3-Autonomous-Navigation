"""Keep mission scan safety independent of retired velocity transport."""
import math

import pytest

from lite3_locomotion.safety import scan_is_clear


@pytest.mark.parametrize('ranges, expected', [
    ([2.0] * 360, True), ([0.4] * 360, True), ([0.399] + [2.] * 359, False),
    ([math.inf] * 360, False), ([math.nan] * 360, False),
    ([2.] * 3 + [math.nan] * 357, False), ([2.] * 300 + [math.inf] * 60, True),
    ([2.] * 300 + [math.nan] * 60, True), ([2.] * 250 + [math.nan] * 110, False),
    ([2.] * 7, False), ([], False),
])
def test_scan_quality_and_clearance(ranges, expected):
    assert scan_is_clear(ranges, 0.1, 10.) is expected


@pytest.mark.parametrize('low, high', [(1., 1.), (-1., 10.), (0.1, math.inf)])
def test_invalid_scan_bounds(low, high):
    assert not scan_is_clear([2.] * 360, low, high)
