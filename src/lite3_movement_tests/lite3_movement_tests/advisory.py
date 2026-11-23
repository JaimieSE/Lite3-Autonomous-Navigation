"""Stair / curtain advisory from the teammate's detector (~/stair_curtain/stair_curtain_advisory.py).

That node (domain 8) publishes std_msgs/String JSON on ADVISORY_TOPIC about every 0.15 s:
  {"obstacle": "stairs" | "curtain" | "unknown", "reason": "confirmed" | <why not>, "stamp_ns": int,
   "distance_m": float,                      # stairs: base_link forward to the first riser
                                             # curtain: RGB camera optical depth to the curtain
   "distance_reference": "base_link_forward" | "rgb_camera_optical_depth",
   "heading_correction_deg": float | None,   # stairs: turn this much (CCW +) to face the risers squarely
   "proposed_action": "stop_and_request_stair_gait" | "request_slow_crossing" | "hold"}
Only "confirmed" stairs/curtains count (it needs 3 / 5 agreeing frames). It only proposes: roam is the one
program that commands the robot, so run_stair_curtain.sh --armed must NOT run alongside roam.
Pure: unit-tested.
"""

import json
import math

ADVISORY_TOPIC = '/stair_curtain/advisory'
KINDS = ('stairs', 'curtain')


def parse_advisory(text):
    """{'obstacle', 'distance_m', 'heading_deg', 'reference'} for a confirmed stairs/curtain, else None."""
    try:
        data = json.loads(text)
    except (TypeError, ValueError):
        return None
    if not isinstance(data, dict) or data.get('reason') != 'confirmed' or data.get('obstacle') not in KINDS:
        return None
    try:
        distance = float(data.get('distance_m'))
        heading = float(data.get('heading_correction_deg') or 0.0)
    except (TypeError, ValueError):
        return None
    if not (math.isfinite(distance) and 0.0 < distance < 5.0 and math.isfinite(heading) and abs(heading) <= 90.0):
        return None
    return {'obstacle': data['obstacle'], 'distance_m': distance, 'heading_deg': heading,
            'reference': data.get('distance_reference')}
