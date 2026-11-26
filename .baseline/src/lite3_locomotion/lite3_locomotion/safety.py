"""Pure scan-quality and clearance checks shared by mission supervision."""

import math


def scan_is_clear(ranges, range_min, range_max, clearance=0.40):
    """Conservative all-around stop check; not terrain or drop-off perception."""
    if (not all(math.isfinite(v) for v in (range_min, range_max, clearance)) or
            not 0 <= range_min < range_max or clearance <= 0 or len(ranges) < 8):
        return False
    valid = [value for value in ranges if math.isfinite(value) and range_min <= value <= range_max]
    usable = sum(math.isfinite(value) and range_min <= value <= range_max or value == math.inf
                 for value in ranges)
    if len(valid) < 3 or usable < 0.8 * len(ranges):
        return False
    return min(valid) >= clearance
