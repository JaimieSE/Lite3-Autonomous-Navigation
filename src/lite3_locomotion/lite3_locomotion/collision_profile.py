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
CORRIDOR_HALF_WIDTH_M = 0.235  # same corridor as terrain_monitor (footprint 0.185 + 0.05; was 0.30)
MIN_BOUNDARY_M = 0.05        # operator decision 2026-11-20 (was 0.10, 0.20, 0.30): stop 5 cm before the
                             # nose, as the terrain stop (terrain.StopParams.stop_ahead_m - NOSE_M)
MAX_SPEED_M_S = 1.00         # hard speed ceiling (operator decision 2026-11-18, was 0.50); validated only to 0.10
VALIDATED_SPEED_M_S = 0.10   # coast/latency behaviour measured up to here
COAST_EXPONENT_ABOVE_VALIDATED = 1.5
MAX_BOUNDARY_M = 3.00        # was 1.50; 1.0 m/s needs ~2.83 m. Must stay inside the terrain grid's
                             # forward reach (TerrainParams.size_m / 2 - NOSE_M) or hazards go unseen.

# Conservative defaults used until the session has measured better values.
DEFAULTS = {'sensing_age_s': 0.50, 'processing_s': 0.05, 'path_latency_s': 0.10,
            'coast_m': 0.05, 'coast_speed_m_s': 0.05, 'sensor_margin_m': 0.025}   # half a 5 cm terrain cell


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
        if not 0.01 <= self.coast_speed_m_s <= MAX_SPEED_M_S:
            raise ValueError('coast_speed_m_s invalid')
        if self.sensor_margin_m < 0.025:
            raise ValueError('sensor_margin_m must be >= 0.025 (half the 5 cm terrain cell: hazards are cell centres)')


def boundary(clearance_m, v_max_m_s, cal=Calibration()):
    """Required stop distance from the nose and its parts. Raises ValueError if out of range."""
    if not (math.isfinite(clearance_m) and 0.0 <= clearance_m <= 1.0):
        raise ValueError('clearance must be 0.0-1.0 m')     # the 5 cm sensor margin is always added
    if not (math.isfinite(v_max_m_s) and 0.0 < v_max_m_s <= MAX_SPEED_M_S):
        raise ValueError(f'v_max must be (0, {MAX_SPEED_M_S}] m/s (hard speed limit)')
    latency = cal.sensing_age_s + cal.processing_s + cal.path_latency_s
    # Linear in speed up to 0.10 m/s, also BELOW the measured speed (2026-11-20, for the 10 cm stop line:
    # holding the 0.05 m/s coast as a floor left ~1 cm and forced a 0.017 m/s approach). ASSUMPTION below
    # the measured speed; an M2 run at the approach speed replaces it.
    if v_max_m_s < cal.coast_speed_m_s:
        coast = cal.coast_m * v_max_m_s / cal.coast_speed_m_s
    else:
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


SLOW_TIER_SPEED_M_S = VALIDATED_SPEED_M_S   # inside the slowdown polygon the robot runs at most this


APPROACH_SIM_STEP_S = 0.1     # Nav2 example value (nav2_collision_monitor collision_monitor_params.yaml)


def tiers(clearance_m, v_max_m_s, cal=Calibration()):
    """Stop polygon plus, above 0.10 m/s, a Nav2 "approach" footprint for a stack speed.

    Up to 0.10 m/s: one STOP polygon sized for v_max (unchanged). Above: the STOP polygon is sized for
    0.10 m/s and an APPROACH action (operator decision 2026-11-19, replacing the fixed slowdown polygon
    that scaled every command to 20% whenever anything was within ~1.6 m ahead) simulates the robot
    footprint along the CURRENT command and only scales it down when a collision is predicted within
    time_before_collision. Moving past or away from obstacles and turning in place are never slowed.
    time_before_collision = stop boundary / 0.10 m/s: approach commands v = d / T near an obstacle, so
    the robot is at <= 0.10 m/s when it reaches the stop polygon, which is sized for that speed."""
    if v_max_m_s <= SLOW_TIER_SPEED_M_S + 1e-9:
        return {'stop': boundary(clearance_m, v_max_m_s, cal), 'approach': None}
    # Stop line at MIN_BOUNDARY_M (10 cm, operator 2026-11-20): approach must deliver the robot there no
    # faster than the speed whose full stop (reaction + coast + margin + clearance) fits inside it.
    lo, hi = 0.0, SLOW_TIER_SPEED_M_S
    for _ in range(30):
        mid = (lo + hi) / 2
        if mid > 0 and boundary(clearance_m, mid, cal)['required_m'] <= MIN_BOUNDARY_M - 1e-4:
            lo = mid
        else:
            hi = mid
    if lo < 0.005:
        raise ValueError(f'a {MIN_BOUNDARY_M} m stop line cannot be met with {clearance_m} m clearance and '
                         f'the current sensing delay (lower --clearance or restart when the link is healthy)')
    stop = boundary(clearance_m, lo, cal)
    full = boundary(clearance_m, v_max_m_s, cal)
    approach_s = math.ceil(max(1.0, stop['boundary_m'] / lo) * 100) / 100   # up: never reach the line faster
    return {'stop': stop, 'approach': {'time_before_collision_s': approach_s,
                                       'full_speed_stop_m': full['required_m']}}


def stop_polygon(boundary_m):
    """collision_monitor 'points' (base_link): corridor from behind the body to the boundary."""
    front = round(NOSE_M + boundary_m, 3)
    rear = -(REAR_M + 0.05)
    w = CORRIDOR_HALF_WIDTH_M
    return [front, w, front, -w, rear, -w, rear, w]


def polygon_boundary(points):
    """Inverse of stop_polygon: nose distance of the polygon's front edge."""
    return round(max(points[0::2]) - NOSE_M, 3)


DOCS_ONLY_MAX_POINTS = 2147483647   # stop polygon kept (readable) but never triggers


def collision_monitor_params(boundary_m, cmd_in, cmd_out, approach_s=None, docs_only=False):
    """Full ros__parameters for nav2_collision_monitor (Humble 1.1.x); approach footprint optional.

    docs_only (operator 2026-11-20, "use the documentation's rules"): the vendor navigation (Perception
    Development Guide s8.2) avoids obstacles with Nav2 planning alone, no extra stop zone. The node stays
    in the command path as a pass-through: the stop polygon is published but needs more points than
    any cloud has, and there is no approach action."""
    params = {'collision_monitor': {'ros__parameters': {
        'use_sim_time': False,
        'base_frame_id': 'base_link',
        'odom_frame_id': 'odom',
        'cmd_vel_in_topic': cmd_in,
        'cmd_vel_out_topic': cmd_out,
        'transform_tolerance': 0.5,
        # Humble IGNORES sources older than this (treats them as "no obstacle"). Observed data
        # age is <= 0.5 s; staleness beyond that is caught by the terrain_stop lock (0.5 s).
        'source_timeout': 1.0,
        # OFF (2026-11-18). With it on, Humble looks up odom->base_link at the CURRENT time for every
        # cmd_vel, so each command blocks until the Fast-LIO TF catches up: throughput = 1 / TF latency
        # (~10 of 20 zeros at 0.1 s; 2 Hz at >= 0.5 s), and beyond transform_tolerance the hazard source
        # is dropped for that cycle (collision_monitor blind). Measured live: TF 1.4 s behind when the
        # domain-bridge link was congested -> M10 zero_delivery 2 of 20. The hazard cloud is already in
        # base_link, and boundary() covers robot travel during the sensing age, so the shift is redundant.
        'base_shift_correction': False,
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
    if docs_only:
        params['collision_monitor']['ros__parameters']['PolygonStop']['max_points'] = DOCS_ONLY_MAX_POINTS
        return params
    if approach_s:
        if not 0.5 <= approach_s <= 10.0:
            raise ValueError('time_before_collision must be 0.5-10 s')
        ros = params['collision_monitor']['ros__parameters']
        ros['polygons'] = ['PolygonStop', 'FootprintApproach']    # stop outranks approach in Humble
        # As the Nav2 example: the robot footprint from the local costmap, simulated along the command.
        ros['FootprintApproach'] = {'type': 'polygon', 'action_type': 'approach',
                                    'footprint_topic': '/local_costmap/published_footprint',
                                    'time_before_collision': float(approach_s),
                                    'simulation_time_step': APPROACH_SIM_STEP_S,
                                    'max_points': 1, 'visualize': False, 'enabled': True}
    return params


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
        tier = tiers(args.clearance, args.max_speed, cal)
        result = dict(tier['stop'])                  # boundary_m = the STOP polygon (state BOUNDARY)
        if tier['approach']:
            result.update(approach_time_s=tier['approach']['time_before_collision_s'],
                          full_speed_stop_m=tier['approach']['full_speed_stop_m'])
    except ValueError as error:
        print(json.dumps({'error': str(error)}))
        return 2
    print(json.dumps({**result, 'calibration': {k: v for k, v in asdict(cal).items()}}))
    sys.stdout.flush()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
