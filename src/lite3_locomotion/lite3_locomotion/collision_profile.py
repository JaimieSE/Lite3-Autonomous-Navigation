"""Stopping boundary for the Nav2 collision_monitor stop polygon, from measured quantities.

Pure Python (no ROS): used by the operator start-up (to generate the collision_monitor
parameters), by the physical tests (to re-check the boundary against live measurements before
moving) and by unit tests.

    boundary (from the nose) = v_max * sensing_latency      reaction travel
                             + coast(v_max)                 travel after commands become zero
                             + sensor_margin                grid cell / point noise
                             + clearance                    what must remain to the obstacle

sensing_latency = worst observed age of the LiDAR data on arrival (header stamp vs local
clock, measured by preflight) + terrain processing + command path allowance.
coast = travel measured by M2 after robot-side commands stopped, scaled linearly up when
v_max exceeds the speed it was measured at (never scaled down).
"""

from dataclasses import asdict, dataclass
import json
import math
from pathlib import Path

NOSE_M = 0.305               # Nav2 footprint front edge ahead of base_link
REAR_M = 0.305
HALF_WIDTH_M = 0.185
CORRIDOR_HALF_WIDTH_M = 0.30  # same corridor as terrain_monitor (footprint + 0.115)
MIN_BOUNDARY_M = 0.30        # never closer than the validated terrain backup stop
MAX_SPEED_M_S = 0.50         # hard speed ceiling (operator decision 2026-11-17); validated only to 0.10
VALIDATED_SPEED_M_S = 0.10   # coast/latency behaviour measured up to here
COAST_EXPONENT_ABOVE_VALIDATED = 1.5
MAX_BOUNDARY_M = 1.50

# Conservative defaults used until the session has measured better values.
DEFAULTS = {'sensing_age_s': 0.50, 'processing_s': 0.05, 'path_latency_s': 0.10,
            'coast_m': 0.05, 'coast_speed_m_s': 0.05, 'sensor_margin_m': 0.05}


@dataclass(frozen=True)
class Calibration:
    sensing_age_s: float = DEFAULTS['sensing_age_s']
    processing_s: float = DEFAULTS['processing_s']
    path_latency_s: float = DEFAULTS['path_latency_s']
    coast_m: float = DEFAULTS['coast_m']
    coast_speed_m_s: float = DEFAULTS['coast_speed_m_s']
    sensor_margin_m: float = DEFAULTS['sensor_margin_m']
    sources: tuple = ()

    def __post_init__(self):
        for name in ('sensing_age_s', 'processing_s', 'path_latency_s', 'coast_m', 'sensor_margin_m'):
            value = getattr(self, name)
            if not (isinstance(value, (int, float)) and math.isfinite(value) and 0.0 <= value <= 2.0):
                raise ValueError(f'{name}={value} invalid')
        if not 0.01 <= self.coast_speed_m_s <= 0.5:
            raise ValueError('coast_speed_m_s invalid')
        if self.sensor_margin_m < 0.05:
            raise ValueError('sensor_margin_m must be >= 0.05 (terrain grid resolution)')


def boundary(clearance_m, v_max_m_s, cal=Calibration()):
    """Required stop distance from the nose and its parts. Raises ValueError if out of range."""
    if not (math.isfinite(clearance_m) and 0.05 <= clearance_m <= 1.0):
        raise ValueError('clearance must be 0.05-1.0 m')
    if not (math.isfinite(v_max_m_s) and 0.0 < v_max_m_s <= MAX_SPEED_M_S):
        raise ValueError(f'v_max must be (0, {MAX_SPEED_M_S}] m/s (hard speed limit)')
    latency = cal.sensing_age_s + cal.processing_s + cal.path_latency_s
    coast = cal.coast_m * max(1.0, min(v_max_m_s, VALIDATED_SPEED_M_S) / cal.coast_speed_m_s)
    if v_max_m_s > VALIDATED_SPEED_M_S:
        # Beyond the validated range braking distance grows faster than linearly (quadratic for a
        # constant deceleration, linear for a velocity-proportional one: the truth is in between).
        # ASSUMPTION, exponent 1.5, relative to the faster of 0.10 m/s and the speed the coast was
        # measured at (an M2 run at the target speed replaces the assumption with a measurement).
        coast *= (v_max_m_s / max(VALIDATED_SPEED_M_S, cal.coast_speed_m_s)) ** COAST_EXPONENT_ABOVE_VALIDATED
    parts = {'reaction_travel_m': v_max_m_s * latency, 'latency_s': latency, 'coast_m': coast,
             'sensor_margin_m': cal.sensor_margin_m, 'clearance_m': clearance_m}
    required = parts['reaction_travel_m'] + coast + cal.sensor_margin_m + clearance_m
    parts['required_m'] = required
    parts['boundary_m'] = max(MIN_BOUNDARY_M, math.ceil(required * 100) / 100)
    if parts['boundary_m'] > MAX_BOUNDARY_M:
        raise ValueError(f'boundary {parts["boundary_m"]:.2f} m exceeds {MAX_BOUNDARY_M} m')
    return {k: round(v, 4) for k, v in parts.items()}


def stop_polygon(boundary_m):
    """collision_monitor 'points' (base_link): corridor from behind the body to the boundary."""
    front = round(NOSE_M + boundary_m, 3)
    rear = -(REAR_M + 0.05)
    w = CORRIDOR_HALF_WIDTH_M
    return [front, w, front, -w, rear, -w, rear, w]


def polygon_boundary(points):
    """Inverse of stop_polygon: nose distance of the polygon's front edge."""
    return round(max(points[0::2]) - NOSE_M, 3)


def collision_monitor_params(boundary_m, cmd_in, cmd_out):
    """Full ros__parameters for nav2_collision_monitor (Humble 1.1.x)."""
    return {'collision_monitor': {'ros__parameters': {
        'use_sim_time': False,
        'base_frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'cmd_vel_in_topic': cmd_in,
        'cmd_vel_out_topic': cmd_out,
        'transform_tolerance': 0.5,
        # Humble IGNORES sources older than this (treats them as "no obstacle"). Observed data
        # age is <= 0.5 s; staleness beyond that is caught by the terrain_stop lock (0.5 s).
        'source_timeout': 1.0,
        'base_shift_correction': True,
                # Humble stops publishing ZEROS once the robot has been stopped for this long (verified on the real
        # binary 2026-11-17: 0 messages after 10 s idle, also on a fresh start). The collision_monitor only
        # publishes in reaction to its input (the mux output), so a long timeout adds no traffic while
        # idle; it keeps the commanded-stop path observable and alive for the whole session.
        'stop_pub_timeout': 86400.0,
        'polygons': ['PolygonStop'],
        'PolygonStop': {'type': 'polygon', 'points': stop_polygon(boundary_m), 'action_type': 'stop',
                        'max_points': 1, 'visualize': True, 'polygon_pub_topic': 'polygon_stop',
                        'enabled': True},
        'observation_sources': ['hazards'],
        'hazards': {'type': 'pointcloud', 'topic': '/lite3_terrain/hazards',
                    'min_height': -1.0, 'max_height': 1.0, 'enabled': True},
    }}}


def load_calibration(path):
    """Calibration from operator/calibration.json (missing keys -> conservative defaults)."""
    if not path or not Path(path).is_file():
        return Calibration(sources=('defaults',))
    path = Path(path)
    data = json.loads(path.read_text())
    known = {k: data[k] for k in DEFAULTS if k in data}
    return Calibration(**known, sources=tuple(data.get('sources', ())))


def update_calibration(path, sources=None, **values):
    """Merge measured values into calibration.json, keeping provenance."""
    path = Path(path)
    data = json.loads(path.read_text()) if path.exists() else {}
    data.update({k: v for k, v in values.items() if v is not None})
    data['sources'] = sorted(set(data.get('sources', [])) | set(sources or []))
    Calibration(**{k: data[k] for k in DEFAULTS if k in data})      # validate
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(data, indent=1))
    return data


def as_dict(cal):
    return asdict(cal)


def main(argv=None):
    """Print the boundary as JSON: python3 -m lite3_locomotion.collision_profile --clearance 0.4"""
    import argparse
    import sys
    parser = argparse.ArgumentParser(description=main.__doc__)
    parser.add_argument('--clearance', type=float, default=0.15)
    parser.add_argument('--max-speed', type=float, default=0.10)   # up to MAX_SPEED_M_S
    parser.add_argument('--calibration', default='')
    args = parser.parse_args(argv)
    cal = load_calibration(args.calibration)
    try:
        result = boundary(args.clearance, args.max_speed, cal)
    except ValueError as error:
        print(json.dumps({'error': str(error)}))
        return 2
    print(json.dumps({**result, 'calibration': {k: v for k, v in asdict(cal).items()}}))
    sys.stdout.flush()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
