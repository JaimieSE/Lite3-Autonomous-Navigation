"""M10 autonomous roaming: explore without goals, keep mapping, avoid obstacles, until stopped.

Independent of M0-M9: no gates, no earlier report is read. M10 runs its OWN automated readiness
(telemetry contract, Nav2/SLAM/terrain/LiDAR, the physical mux wiring, the collision boundary
re-check against live sensing delay, zero-delivery to motion_sender with the idle dead-man check,
stand + verified standing) before anything moves.

Command path (all existing components; this module talks to none of them directly):

    frontier / patrol goal -> Nav2 (SLAM map, costmaps incl. terrain hazards, planner, MPPI)
      -> velocity_smoother (max_velocity = the selected speed mode, read back)
      -> /nav_cmd_vel (prio 10) -> twist_mux [dead-man fed only while every check passes,
         terrain_stop backup lock] -> collision_monitor STOP polygon -> /cmd_vel -> robot

Speed modes: the Lite3's gait modes cannot be verified on this deployment - the vendor ROS
bridge has no gait-switch command, the firmware's robot_gait_state is dropped by the deployed
motion_receiver and no ROS topic reports it (checked 2026-11-17). M10 therefore never sends gait
codes; it keeps the robot's current (standard) gait and selects a VELOCITY envelope: slow (the
default), normal, fast, each <= the session's authorized limit <= the stack's boundary speed
<= 0.5 m/s (validated to 0.10). The envelope is enforced three ways: velocity_smoother cap (verified), MPPI speed
limit, and robot-side command checks.

Safety layers while roaming: Nav2 avoidance and replanning; collision_monitor stop; terrain lock;
twist_mux dead-man (lapses in 0.5 s if this process stalls); this runner's per-tick checks
(SequenceRunner._check: telemetry, tilt, remote takeover, command path, robot-side speed, Nav2
outside roaming, map geofence) plus roaming checks (obstacle clearance, LiDAR/terrain/map
freshness, selected speed); and an INDEPENDENT watchdog process (roam_watchdog.py) that stops
this runner (SIGINT, then SIGKILL so the dead-man locks) and commands zeros if it sees unsafe
telemetry or loses this runner's heartbeat. Each supervises the other.
"""

import argparse
import datetime as dt
import json
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import threading
import time

import yaml

from .config import (ConfigError, HARD_MAX_NAV_GEOFENCE_M, HARD_MAX_SPEED_M_S,
                     HARD_MIN_ABORT_CLEARANCE_M, HARD_MIN_STOP_DISTANCE_M, NAV2_MAX_YAW_RATE_RAD_S,
                     authorization_template, check_authorization, load_authorization,
                     load_sequence)
from .runner import MODE_PHYSICAL, MODE_READONLY, MODE_SIM, SafetyAbort, SequenceRunner

try:
    from lite3_locomotion import collision_profile as CP
    from lite3_locomotion.frontiers import FrontierConfig, select_frontiers
except ImportError:      # source tree without the workspace install sourced
    sys.path.append(str(Path(__file__).resolve().parents[2] / 'lite3_locomotion'))
    from lite3_locomotion import collision_profile as CP
    from lite3_locomotion.frontiers import FrontierConfig, select_frontiers

# ---------------------------------------------------------------------------------- hard limits
HARD_MAX_ROAM_MINUTES = 120.0          # supervision / battery bound; roaming otherwise runs until stopped
HARD_MIN_GEOFENCE_M = 1.0
HARD_MAX_MAP_AGE_S = 30.0              # SLAM publishes every 2 s
HARD_MAX_SENSOR_AGE_S = 0.5            # = terrain_stop lock timeout
HARD_MIN_LIDAR_HZ = 8.0
HARD_MAX_WATCHDOG_S = 3.0
HARD_MAX_GOAL_S = 240.0
MODE_NAMES = ('slow', 'normal', 'fast')
SEARCH_CLEARANCE_M = 0.25             # frontier search (robot start + BFS): footprint half-width 0.185 + margin
COSTMAP_VETO = 70                      # Nav2 OccupancyGrid scale: 99 inscribed, 100 lethal
MIN_RAW_BYTES = 10000                  # serialized /livox/points; a healthy Mid-360 scan is ~0.2-0.5 MB
DEFAULT_MODES = {'slow': (0.05, 0.15), 'normal': (0.08, 0.20), 'fast': (0.10, 0.20)}
SPEED_GRACE_S = 2.0                    # after lowering the speed, the smoother ramps down
STATES = ('READY', 'ROAMING', 'PAUSED', 'STOPPING', 'STOPPED', 'SAFETY_STOP')
EXIT = {'STOPPED': 0, 'SAFETY_STOP': 1, 'INTERVENTION_REQUIRED': 2, 'REFUSED': 3}


def _r(value, digits=3):
    return None if value is None or not math.isfinite(value) else round(value, digits)


def _finite(value, name, low, high):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) \
            or not low <= value <= high:
        raise ConfigError(f'{name}={value} outside [{low}, {high}] (hard limit)')
    return float(value)


class RoamConfig:
    """All roaming limits; anything outside the validated envelope is refused, never clamped."""

    def __init__(self, speed_limit=0.10, modes=None, start_mode='slow', geofence_m=3.0,
                 max_minutes=HARD_MAX_ROAM_MINUTES, abort_clearance_m=0.08, frontier_radius_m=3.0,
                 frontier_clearance_m=0.40, stuck_s=20.0, max_map_age_s=15.0, max_sensor_age_s=0.5,
                 min_lidar_hz=HARD_MIN_LIDAR_HZ, watchdog_timeout_s=2.0, max_goal_failures=5,
                 stack_max_speed=0.10):
        self.speed_limit = _finite(speed_limit, 'speed_limit', 0.02, min(HARD_MAX_SPEED_M_S, stack_max_speed))
        modes = {**DEFAULT_MODES, **(modes or {})}         # unspecified modes keep their defaults
        self.modes = {}
        for name, value in modes.items():
            if name not in MODE_NAMES:
                raise ConfigError(f'unknown speed mode {name!r} (modes: {", ".join(MODE_NAMES)})')
            v, w = value if isinstance(value, (tuple, list)) else (value, DEFAULT_MODES[name][1])
            self.modes[name] = (_finite(v, f'{name} speed', 0.02, HARD_MAX_SPEED_M_S),
                                _finite(w, f'{name} yaw rate', 0.05, NAV2_MAX_YAW_RATE_RAD_S))
        if start_mode not in self.modes:
            raise ConfigError(f'start mode {start_mode!r} not defined')
        if self.modes[start_mode][0] > self.speed_limit + 1e-9:
            raise ConfigError(f'start mode {start_mode} ({self.modes[start_mode][0]} m/s) exceeds the '
                              f'session speed limit {self.speed_limit} m/s')
        self.start_mode = start_mode
        self.geofence_m = _finite(geofence_m, 'geofence', HARD_MIN_GEOFENCE_M, HARD_MAX_NAV_GEOFENCE_M)
        self.max_minutes = _finite(max_minutes, 'max_minutes', 1.0, HARD_MAX_ROAM_MINUTES)
        self.abort_clearance_m = _finite(abort_clearance_m, 'abort_clearance', HARD_MIN_ABORT_CLEARANCE_M, 0.5)
        self.frontier_radius_m = _finite(frontier_radius_m, 'frontier_radius', 1.0, 5.0)
        self.frontier_clearance_m = _finite(frontier_clearance_m, 'frontier_clearance', 0.30, 1.0)
        self.stuck_s = _finite(stuck_s, 'stuck_s', 5.0, 60.0)
        self.max_map_age_s = _finite(max_map_age_s, 'max_map_age', 3.0, HARD_MAX_MAP_AGE_S)
        self.max_sensor_age_s = _finite(max_sensor_age_s, 'max_sensor_age', 0.1, HARD_MAX_SENSOR_AGE_S)
        self.min_lidar_hz = _finite(min_lidar_hz, 'min_lidar_hz', HARD_MIN_LIDAR_HZ, 20.0)
        self.watchdog_timeout_s = _finite(watchdog_timeout_s, 'watchdog_timeout', 0.5, HARD_MAX_WATCHDOG_S)
        self.max_goal_failures = int(_finite(max_goal_failures, 'max_goal_failures', 1, 20))

    def mode_speed(self, mode):
        return self.modes[mode]

    def as_dict(self):
        return {k: v for k, v in vars(self).items()}


def parse_modes(text):
    """'slow=0.04,normal=0.07,fast=0.09' (optionally speed:yaw) -> {name: (v, w)}."""
    modes = dict(DEFAULT_MODES)
    for part in (text or '').split(','):
        if not part.strip():
            continue
        name, _, value = part.partition('=')
        name = name.strip()
        if name not in MODE_NAMES:
            raise ConfigError(f'unknown speed mode {name!r}')
        speed, _, yaw = value.partition(':')
        try:
            modes[name] = (float(speed), float(yaw) if yaw else DEFAULT_MODES[name][1])
        except ValueError as error:
            raise ConfigError(f'speed mode {part!r}: {error}') from error
    return modes


# ---------------------------------------------------------------------------------- control channel
class ControlChannel:
    """Append-only command file + status file, so any terminal can command a running session."""

    def __init__(self, directory):
        self.dir = Path(directory)
        self.dir.mkdir(parents=True, exist_ok=True)
        self.control = self.dir / 'control.jsonl'
        self.status_path = self.dir / 'status.json'
        self.watchdog_path = self.dir / 'watchdog.json'
        self.offset = self.control.stat().st_size if self.control.exists() else 0

    @staticmethod
    def send(directory, cmd, arg=None):
        path = Path(directory) / 'control.jsonl'
        path.parent.mkdir(parents=True, exist_ok=True)
        entry = {'id': time.time_ns(), 'cmd': cmd, 'arg': arg, 'time': dt.datetime.now().isoformat()}
        with open(path, 'a') as stream:
            stream.write(json.dumps(entry) + '\n')
        return entry

    def poll(self):
        if not self.control.exists():
            return []
        with open(self.control) as stream:
            stream.seek(self.offset)
            text = stream.read()
        complete = text[:text.rfind('\n') + 1] if '\n' in text else ''
        self.offset += len(complete.encode())
        out = []
        for line in complete.splitlines():
            try:
                out.append(json.loads(line))
            except ValueError:
                continue
        return out

    def write_status(self, status):
        tmp = self.status_path.with_suffix('.tmp')
        tmp.write_text(json.dumps(status, default=str))
        tmp.replace(self.status_path)

    def read_json(self, path):
        try:
            return json.loads(Path(path).read_text())
        except (OSError, ValueError):
            return None


# ---------------------------------------------------------------------------------- exploration
class Explorer:
    """Frontier goals (lite3_locomotion.frontiers) inside the geofence; patrol when none remain."""

    def __init__(self, cfg, anchor):
        self.cfg = cfg
        self.anchor = anchor
        self.blocked = {}              # key -> until
        self.recent = []               # recent goal positions
        self.last_status = None
        self.costmap = None
        self.vetoed = 0                # goals rejected by point-cloud hazards in the costmap

    @staticmethod
    def key(x, y):
        return round(x / 0.5), round(y / 0.5)

    def inside(self, x, y, margin=0.4):
        return math.hypot(x - self.anchor[0], y - self.anchor[1]) <= self.cfg.geofence_m - margin

    def block(self, x, y, now, seconds):
        self.blocked[self.key(x, y)] = now + seconds

    def map_clear(self, grid, x, y):
        """No occupied map cell within the goal clearance (unknown is allowed: a frontier goal is
        next to unknown space by definition; the search already keeps 0.25 m from it)."""
        cx, cy = grid.cell(x, y)
        pad = max(1, math.ceil(self.cfg.frontier_clearance_m / grid.resolution))
        return not any(grid.contains((i, j)) and grid.value((i, j)) > 20
                       for j in range(cy - pad, cy + pad + 1) for i in range(cx - pad, cx + pad + 1))

    def hazard_free(self, costmap, x, y):
        """Nav2 global costmap (2D SLAM map + LiDAR point-cloud hazards from terrain_monitor) has no
        high-cost cell within the goal clearance. Catches tables, low objects and drops that the 2D
        SLAM map cannot see."""
        if costmap is None:
            return True
        cx, cy = costmap.cell(x, y)
        pad = max(1, math.ceil(self.cfg.frontier_clearance_m / costmap.resolution))
        for j in range(cy - pad, cy + pad + 1):
            for i in range(cx - pad, cx + pad + 1):
                if costmap.contains((i, j)) and costmap.value((i, j)) >= COSTMAP_VETO:
                    return False
        return True

    def choose(self, grid, pose, now, costmap=None):
        self.costmap = costmap
        self.blocked = {k: t for k, t in self.blocked.items() if t > now}
        # Search with a footprint-sized clearance (the robot may start next to something; Nav2 plans
        # the real path), then require the full goal clearance around the GOAL on the map and costmap.
        result = select_frontiers(grid, pose[0], pose[1], FrontierConfig(
            radius=self.cfg.frontier_radius_m, clearance=min(self.cfg.frontier_clearance_m, SEARCH_CLEARANCE_M)))
        self.last_status = result.status
        for c in result.candidates:
            if self.inside(c.x, c.y) and math.hypot(c.x - pose[0], c.y - pose[1]) >= 0.5 and \
                    self.key(c.x, c.y) not in self.blocked:
                if not self.map_clear(grid, c.x, c.y):
                    continue
                if not self.hazard_free(costmap, c.x, c.y):
                    self.vetoed += 1
                    continue
                return {'kind': 'frontier', 'x': c.x, 'y': c.y, 'yaw': c.yaw, 'score': _r(c.score)}
        return self._patrol(grid, pose)

    def _patrol(self, grid, pose):
        """No frontier left inside the geofence: visit known free space far from recent goals."""
        pad = max(1, math.ceil(self.cfg.frontier_clearance_m / grid.resolution))
        stride = max(1, round(0.2 / grid.resolution))
        cx, cy = grid.cell(*self.anchor)
        reach = math.ceil(self.cfg.geofence_m / grid.resolution)
        best = None
        for y in range(max(pad, cy - reach), min(grid.height - pad, cy + reach + 1), stride):
            for x in range(max(pad, cx - reach), min(grid.width - pad, cx + reach + 1), stride):
                if not 0 <= grid.value((x, y)) <= 20:
                    continue
                wx, wy = grid.world((x, y))
                d_robot = math.hypot(wx - pose[0], wy - pose[1])
                if d_robot < 0.8 or not self.inside(wx, wy) or self.key(wx, wy) in self.blocked:
                    continue
                if any(not 0 <= grid.value((x + i, y + j)) <= 20
                       for i in range(-pad, pad + 1) for j in range(-pad, pad + 1)):
                    continue
                if not self.hazard_free(self.costmap, wx, wy):
                    continue
                novelty = min([math.hypot(wx - gx, wy - gy) for gx, gy in self.recent[-6:]] or [3.0])
                score = min(novelty, 3.0) + 0.3 * min(d_robot, 2.0)
                if best is None or score > best[0]:
                    best = (score, wx, wy)
        if best is None:
            return None
        _, wx, wy = best
        return {'kind': 'patrol', 'x': wx, 'y': wy, 'yaw': math.atan2(wy - pose[1], wx - pose[0]),
                'score': _r(best[0])}

    def sent(self, goal):
        self.recent.append((goal['x'], goal['y']))
        self.recent = self.recent[-20:]


def known_area_m2(grid):
    return sum(1 for v in grid.data if v >= 0) * grid.resolution ** 2


# ---------------------------------------------------------------------------------- watchdog logic
class WatchdogLogic:
    """Independent limits (deliberately looser than the runner's, different sources where
    possible: leg odometry for the area, robot-side commands for speed). Pure: unit-tested and
    used by the watchdog process and the simulator."""

    def __init__(self, speed_limit, geofence_m, max_tilt=0.40, telemetry_age=0.5, heartbeat_s=2.0):
        self.speed_limit = speed_limit
        self.geofence_m = geofence_m
        self.max_tilt = max_tilt
        self.telemetry_age = telemetry_age
        self.heartbeat_s = heartbeat_s
        self.origin = None
        self.fast_since = None

    def evaluate(self, telemetry, now, runner_status=None, wall_now=None):
        reasons = []
        for name in ('joints', 'odom', 'imu'):
            age = getattr(telemetry, name).age(now)
            if age > self.telemetry_age:
                reasons.append(f'{name} telemetry stale ({_r(age, 2)} s)')
        tilt = telemetry.current_tilt()
        if tilt is None or max(abs(tilt[0]), abs(tilt[1])) > self.max_tilt:
            reasons.append('IMU tilt beyond limit or orientation invalid')
        odom = telemetry.odom.latest()
        if odom is not None:
            if self.origin is None:
                self.origin = (odom[1].x, odom[1].y)
            d = math.hypot(odom[1].x - self.origin[0], odom[1].y - self.origin[1])
            if d > self.geofence_m + 0.75:
                reasons.append(f'leg-odometry distance {d:.2f} m beyond geofence {self.geofence_m} m + 0.75')
        raw = getattr(telemetry, 'raw_cloud', None)
        if raw is not None and raw.total:
            if raw.age(now) > 1.0:
                reasons.append(f'raw LiDAR /livox/points stale ({_r(raw.age(now), 2)} s)')
            elif max((n for _, n in raw.window(now, 1.0)), default=0) < MIN_RAW_BYTES:
                reasons.append('raw LiDAR nearly empty (sensor blind or blocked)')
        odom_w = telemetry.odom.window(now, 0.5)
        moving = max(2.0 * self.speed_limit, self.speed_limit + 0.05)
        if len(odom_w) >= 5 and min(math.hypot(o.vx, o.vy) for _, o in odom_w) > moving:
            reasons.append(f'leg-odometry speed above {moving:.2f} m/s for 0.5 s (runaway)')
        cmds = telemetry.system_cmd.window(now, 0.5)
        fast = [c for _, c in cmds if c.vx > self.speed_limit + 0.02 or abs(c.wz) > NAV2_MAX_YAW_RATE_RAD_S + 0.05
                or c.vx < -0.01]
        if fast:
            self.fast_since = self.fast_since or now
            if now - self.fast_since >= 0.3:
                c = fast[-1]
                reasons.append(f'robot-side command beyond limits (vx {c.vx:+.3f}, wz {c.wz:+.3f})')
        else:
            self.fast_since = None
        if runner_status is not None and runner_status.get('state') in ('ROAMING', 'PAUSED', 'READY'):
            age = (wall_now or time.time()) - runner_status.get('heartbeat', 0.0)
            if age > self.heartbeat_s:
                reasons.append(f'runner heartbeat lost ({_r(age, 1)} s)')
        return reasons


# ---------------------------------------------------------------------------------- readiness
VALIDATED_SPEED_M_S = 0.10             # backup-lock distance and coast were validated up to here
DEFAULT_STOP_AHEAD_M = 0.60            # terrain_monitor default (from base_link centre), restored afterwards


def backup_distance(cfg, boundary_m, mode):
    """Nose distance of the terrain backup lock. 0.30 m is the validated backup up to 0.10 m/s. Above
    that the robot would travel past it before stopping, so it moves out to just inside the
    collision_monitor boundary (one 5 cm grid cell), and the stack must have been started with a
    boundary computed for that speed."""
    if mode != MODE_PHYSICAL or cfg.speed_limit <= VALIDATED_SPEED_M_S + 1e-9 or not boundary_m:
        return HARD_MIN_STOP_DISTANCE_M
    return min(1.5, max(HARD_MIN_STOP_DISTANCE_M, round(boundary_m - 0.05, 2)))


def readiness_sequence(mode, boundary_check, backup_nose=HARD_MIN_STOP_DISTANCE_M):
    nav = dict({'id': 'nav_readiness', 'type': 'nav_readiness', 'timeout_s': 15,
                'stop_distance_m': backup_nose}, **boundary_check)
    steps = [{'id': 'readiness', 'type': 'readiness', 'timeout_s': 10, 'quiet_s': 2.0}, nav,
             {'id': 'telemetry_contract', 'type': 'telemetry_validation', 'timeout_s': 5},
             {'id': 'initial_rest', 'type': 'stationary_check', 'timeout_s': 6, 'hold_s': 2.0}]
    if mode != MODE_READONLY:
        steps += [{'id': 'zero_delivery', 'type': 'zero_delivery', 'duration_s': 1.0},
                  {'id': 'stand', 'type': 'posture_transition', 'target': 'standing', 'timeout_s': 12,
                   'hold_s': 1.5},
                  {'id': 'verify_standing', 'type': 'posture_check', 'expect': 'standing',
                   'timeout_s': 8, 'hold_s': 2.0}]
    else:
        steps.append({'id': 'zero_delivery', 'type': 'zero_delivery', 'duration_s': 1.0})
    return {'name': 'M10_readiness', 'description': 'M10 autonomous roaming: own readiness checks',
            'interface': 'nav',
            'safety': {'geofence_radius_m': 0.5, 'max_tilt_rad': 0.35, 'command_period_s': 0.05,
                       'zero_seconds': 2.0, 'stop_timeout_s': 6.0, 'stop_hold_s': 1.5},
            'steps': steps}


def boundary_check(state):
    if not state.get('BOUNDARY'):
        return {}
    cal = CP.load_calibration(Path(os.environ.get('LITE3_STATE_FILE', '')).parent / 'calibration.json')
    return {'cm_boundary_m': float(state['BOUNDARY']),
            'cm_clearance_m': float(state.get('CLEARANCE', 0.15)),
            'cm_max_speed_m_s': float(state.get('MAX_SPEED', 0.10)),
            'coast_m': cal.coast_m, 'coast_speed_m_s': cal.coast_speed_m_s}


# ---------------------------------------------------------------------------------- session
class RoamSession:
    def __init__(self, robot, cfg, mode, channel, log=print, authorization=None, watchdog=None,
                 script=None, sim_stop_after_s=None):
        self.robot = robot
        self.cfg = cfg
        self.mode = mode
        self.channel = channel
        self.log = log
        self.auth = authorization
        self.watchdog = watchdog                 # in-process WatchdogLogic (simulation) or None
        self.watchdog_external = False           # True when a watchdog process must be alive
        self.script = sorted(script or [])       # [(seconds after roaming starts, cmd, arg)]
        self.sim_stop_after_s = sim_stop_after_s
        self.state = 'READY'
        self.speed_mode = cfg.start_mode
        self.speed_changed = -math.inf
        self.goal = None
        self.goal_sent_at = None
        self.goal_timeout = None
        self.progress = None                     # (time, x, y) of last 0.10 m of progress
        self.failures = 0
        self.events = []
        self.stats = {'goals_sent': 0, 'goals_succeeded': 0, 'goals_failed': 0, 'frontier_goals': 0,
                      'patrol_goals': 0, 'distance_m': 0.0, 'max_from_anchor_m': 0.0,
                      'min_clearance_m': math.inf, 'pauses': 0, 'speed_changes': 0}
        self.reason = None
        self.runner = None
        self.anchor = None
        self.explorer = None
        self.map = None
        self.area0 = None
        self.last_pose = None
        self.t_roam = None

    # -------------------------------------------------------------- helpers
    def event(self, name, **detail):
        t = self.robot.now() - (self.t_roam or self.robot.now())
        self.events.append({"t_s": _r(t, 2), "event": name, **detail})
        self.log(f"[{name}] {detail}" if detail else f"[{name}]")

    def _grid(self):
        item = self.robot.map_grid()
        if item is not None:
            self.map = item
        return self.map

    def _pose(self):
        return self.robot.map_pose()

    def _costmap(self):
        item = self.robot.costmap_grid() if hasattr(self.robot, 'costmap_grid') else None
        return None if item is None or self.robot.now() - item[0] > 10.0 else item[1]

    def lidar_violation(self, now):
        """Raw point clouds, independent of the terrain monitor's processing: the LiDAR is
        delivering real scans (not just a healthy-looking derived status) and Fast-LIO is alive."""
        raw = self.robot.telemetry.raw_cloud
        if raw.age(now) > self.cfg.max_sensor_age_s:
            return f'raw LiDAR /livox/points stale ({_r(raw.age(now), 2)} s)'
        if now - self.t_roam > 2.0 and len(raw.window(now, 2.0)) / 2.0 < self.cfg.min_lidar_hz - 0.5:
            return f'raw LiDAR rate {len(raw.window(now, 2.0)) / 2.0:.1f} Hz < {self.cfg.min_lidar_hz} Hz'
        scans = raw.window(now, 1.0)
        if scans and max(n for _, n in scans) < MIN_RAW_BYTES:
            return f'raw LiDAR nearly empty ({max(n for _, n in scans)} bytes/scan): sensor blind or blocked'
        lio = self.robot.telemetry.lio_cloud
        if lio.total and lio.age(now) > 1.0:
            return f'Fast-LIO /cloud_registered_body stale ({_r(lio.age(now), 2)} s): SLAM odometry lost'
        return None

    def _set_speed(self, mode):
        v, w = self.cfg.mode_speed(mode)
        if v > self.cfg.speed_limit + 1e-9:
            return f'{mode} ({v} m/s) exceeds the session speed limit {self.cfg.speed_limit} m/s'
        if self.robot.can_actuate:
            cap = self.robot.set_nav_velocity_cap(v, w)
            if not cap.get('ok'):
                return f'velocity_smoother cap not verified ({cap.get("reason")})'
            self.robot.set_speed_limit(v)
        if mode != self.speed_mode:
            self.stats['speed_changes'] += 1
        old_v = self.cfg.mode_speed(self.speed_mode)[0]
        if v < old_v:
            self.speed_changed = self.robot.now()
        self.speed_mode = mode
        return None

    def _speed_command(self, arg):
        text = str(arg or '').strip().lower()
        if text in MODE_NAMES:
            if text not in self.cfg.modes:
                return f'mode {text} not configured'
            return self._set_speed(text)
        if text in ('up', 'down'):
            order = [m for m in MODE_NAMES if m in self.cfg.modes]
            i = order.index(self.speed_mode) + (1 if text == 'up' else -1)
            return self._set_speed(order[max(0, min(len(order) - 1, i))])
        return f'unknown speed {arg!r} (use slow, normal, fast, up or down)'

    # -------------------------------------------------------------- checks
    def roam_violation(self, now):
        """Roaming-specific unsafe conditions (the runner's _check covers the rest)."""
        t = self.robot.telemetry
        item = t.terrain.latest()
        if item is None or now - item[0] > self.cfg.max_sensor_age_s:
            return 'LiDAR/terrain status stale'
        if item[1].problems:
            return f'terrain inputs invalid: {list(item[1].problems)}'
        if t.clearance.age(now) > 0.5:
            return 'hazard cloud stale: obstacle clearance unknown'
        clearance = t.clearance.latest()[1]
        self.stats['min_clearance_m'] = min(self.stats['min_clearance_m'], clearance)
        if clearance < self.cfg.abort_clearance_m:
            return f'obstacle {clearance:.2f} m from the footprint (abort below {self.cfg.abort_clearance_m} m)'
        if now - self.t_roam > 2.0:
            hz = len(t.terrain.window(now, 2.0)) / 2.0
            if hz < self.cfg.min_lidar_hz - 0.5:
                return f'LiDAR/terrain rate {hz:.1f} Hz < {self.cfg.min_lidar_hz} Hz'
        if self.map is None or now - self.map[0] > self.cfg.max_map_age_s:
            return 'SLAM map not updating'
        why = self.lidar_violation(now)
        if why:
            return why
        v, w = self.cfg.mode_speed(self.speed_mode)
        if now - self.speed_changed > SPEED_GRACE_S:
            for _, c in t.system_cmd.window(now, 0.3):
                if c.vx > v + 0.01 or abs(c.wz) > w + 0.02:
                    return (f'robot-side command vx {c.vx:+.3f} wz {c.wz:+.3f} above the selected '
                            f'{self.speed_mode} mode ({v} m/s, {w} rad/s)')
        for _, c in t.system_cmd.window(now, 0.3):
            if c.vx < -0.005:
                return 'reverse command reached the robot (no rear sensing)'
        # Actual motion (leg odometry) must match what is commanded: a runaway gait or slipping
        # estimator is caught even while every command is within limits.
        limit = max(2.0 * self.cfg.mode_speed(self.speed_mode)[0], self.cfg.mode_speed(self.speed_mode)[0] + 0.05)
        if now - self.speed_changed <= SPEED_GRACE_S + 1.0:
            limit = max(limit, 2.0 * self.cfg.speed_limit)       # ramp-down after lowering the mode
        odom = t.odom.window(now, 0.5)
        if len(odom) >= 5 and min(math.hypot(o.vx, o.vy) for _, o in odom) > limit:
            return (f'unexpected motion: leg-odometry speed {math.hypot(odom[-1][1].vx, odom[-1][1].vy):.2f} '
                    f'm/s for 0.5 s, more than {limit:.2f} m/s')
        if self.watchdog_external:
            wd = self.channel.read_json(self.channel.watchdog_path) or {}
            age = time.time() - wd.get('heartbeat', 0.0)
            if age > self.cfg.watchdog_timeout_s:
                return f'independent watchdog heartbeat lost ({_r(age, 1)} s)'
            if wd.get('tripped'):
                return f'watchdog tripped: {wd.get("reasons")}'
        if self.watchdog is not None:
            reasons = self.watchdog.evaluate(t, now)
            if reasons:
                return f'watchdog: {reasons}'
        return None

    # -------------------------------------------------------------- goals
    def _send_goal(self, now, pose):
        grid = self._grid()
        if grid is None:
            return
        goal = self.explorer.choose(grid[1], pose, now, self._costmap())
        if goal is None:
            if not getattr(self, '_no_goal_logged', False):
                self.event('no_goal', status=self.explorer.last_status)
                self._no_goal_logged = True
            return
        self._no_goal_logged = False
        if not self.robot.nav_send_goal(goal['x'], goal['y'], goal['yaw']):
            self.explorer.block(goal['x'], goal['y'], now, 60)
            self._goal_failed(now, 'Nav2 rejected the goal')
            return
        self.robot.set_speed_limit(self.cfg.mode_speed(self.speed_mode)[0])   # after the MPPI reset
        self.explorer.sent(goal)
        self.goal = goal
        self.goal_sent_at = now
        d = math.hypot(goal['x'] - pose[0], goal['y'] - pose[1])
        self.goal_timeout = min(HARD_MAX_GOAL_S, d / self.cfg.mode_speed(self.speed_mode)[0] * 3 + 30)
        self.progress = (now, pose[0], pose[1])
        self.stats['goals_sent'] += 1
        self.stats[f'{goal["kind"]}_goals'] += 1
        self.event('goal', kind=goal['kind'], x=_r(goal['x'], 2), y=_r(goal['y'], 2), distance_m=_r(d, 2))

    def _goal_failed(self, now, why):
        self.stats['goals_failed'] += 1
        self.failures += 1
        self.event('goal_failed', reason=why, consecutive=self.failures)
        if self.goal:
            self.explorer.block(self.goal['x'], self.goal['y'], now, 90)
        self.goal = None
        if self.failures >= self.cfg.max_goal_failures:
            self._pause(f'{self.failures} consecutive goals failed: check the area, then resume')

    def _manage_goal(self, now, pose):
        if self.goal is None:
            self._send_goal(now, pose)
            return
        status = self.robot.nav_status()
        result = status.get('result')
        if result == 'SUCCEEDED':
            self.stats['goals_succeeded'] += 1
            self.failures = 0
            self.explorer.block(self.goal['x'], self.goal['y'], now, 30)
            self.event('goal_reached', kind=self.goal['kind'])
            self.goal = None
            self._send_goal(now, pose)
        elif result:
            self._goal_failed(now, f'Nav2 {result}')
        else:
            if math.hypot(pose[0] - self.progress[1], pose[1] - self.progress[2]) >= 0.10:
                self.progress = (now, pose[0], pose[1])
            if now - self.goal_sent_at > self.goal_timeout or now - self.progress[0] > self.cfg.stuck_s:
                why = 'goal timeout' if now - self.goal_sent_at > self.goal_timeout else \
                    f'no progress for {self.cfg.stuck_s:.0f} s'
                self.robot.nav_cancel()
                self._goal_failed(now, why)

    # -------------------------------------------------------------- state changes
    def _pause(self, why):
        if self.state != 'ROAMING':
            return
        self.robot.nav_cancel() if self.robot.can_actuate else None
        self.runner.nav_active_until = self.robot.now() + 1.5
        self.state = 'PAUSED'
        self.stats['pauses'] += 1
        self.reason = why
        self.event('paused', reason=why)
        if self.robot.can_actuate:
            self.runner._zero_burst(1.5)

    def _resume(self):
        if self.state != 'PAUSED':
            return 'not paused'
        now = self.robot.now()
        why = self.roam_violation(now)
        if why:
            return f'not resumed: {why}'
        ok, pre = self.runner._standing_precondition(require_still=False)
        if not ok and self.robot.can_actuate:
            return 'not resumed: robot not verified standing'
        problem = self._set_speed(self.speed_mode)
        if problem:
            return f'not resumed: {problem}'
        self.failures = 0
        self.goal = None
        self.state = 'ROAMING'
        self.reason = None
        self.runner.nav_active_until = math.inf
        self.event('resumed', speed_mode=self.speed_mode)
        return None

    def _handle(self, command):
        cmd, arg = command.get('cmd'), command.get('arg')
        result = None
        if cmd == 'pause':
            if self.state == 'ROAMING':
                self._pause('operator pause')
            else:
                result = f'not roaming ({self.state})'
        elif cmd == 'resume':
            result = self._resume()
        elif cmd == 'stop':
            self.reason = 'operator stop'
            self.state = 'STOPPING'
        elif cmd == 'speed':
            result = self._speed_command(arg)
            if result is None:
                self.event('speed', mode=self.speed_mode, m_s=self.cfg.mode_speed(self.speed_mode)[0])
        elif cmd == 'start':
            result = 'already started (use resume after pause)'
        else:
            result = f'unknown command {cmd!r}'
        self.last_command = {'id': command.get('id'), 'cmd': cmd, 'arg': arg, 'result': result or 'ok'}
        if result:
            self.log(f'[command {cmd} {arg or ""}] {result}')
        return result

    # -------------------------------------------------------------- status
    def status(self, now, pose):
        t = self.robot.telemetry
        cmd = t.system_cmd.latest()
        grid = self.map
        return {
            'state': self.state, 'reason': self.reason, 'mode': self.mode, 'pid': os.getpid(),
            'heartbeat': time.time(), 'elapsed_s': _r(now - (self.t_roam or now), 1),
            'speed_mode': self.speed_mode, 'speed_m_s': self.cfg.mode_speed(self.speed_mode)[0],
            'speed_limit_m_s': self.cfg.speed_limit, 'modes': self.cfg.modes,
            'goal': self.goal, 'explorer': self.explorer.last_status if self.explorer else None,
            'pose': [_r(v, 2) for v in pose] if pose else None,
            'from_anchor_m': _r(math.hypot(pose[0] - self.anchor[0], pose[1] - self.anchor[1]), 2)
            if pose and self.anchor else None,
            'geofence_m': self.cfg.geofence_m,
            'clearance_m': _r(t.clearance.latest()[1], 2) if t.clearance.latest() else None,
            'lidar_hz': len(t.terrain.window(now, 1.0)),
            'raw_lidar_hz': len(t.raw_cloud.window(now, 1.0)),
            'raw_kb': round(t.raw_cloud.latest()[1] / 1000) if t.raw_cloud.latest() else None,
            'goals_vetoed_by_pointcloud': self.explorer.vetoed if self.explorer else 0,
            'map_area_m2': _r(known_area_m2(grid[1]), 1) if grid else None,
            'map_age_s': _r(now - grid[0], 1) if grid else None,
            'robot_cmd': [_r(cmd[1].vx), _r(cmd[1].wz)] if cmd and now - cmd[0] < 0.5 else None,
            'stats': {k: _r(v) if isinstance(v, float) else v for k, v in self.stats.items()},
            'last_command': getattr(self, 'last_command', None),
        }

    def _line(self, s):
        goal = s['goal']
        self.log(f'    [{s["elapsed_s"]:6.0f}s] {s["state"]:<8} {s["speed_mode"]} {s["speed_m_s"]} m/s | '
                 f'goal {goal["kind"] + " (%.1f, %.1f)" % (goal["x"], goal["y"]) if goal else "- (%s)" % s["explorer"]} | '
                 f'travelled {s["stats"]["distance_m"]:.1f} m | clearance {s["clearance_m"]} m | '
                 f'geofence {s["from_anchor_m"]}/{s["geofence_m"]} m | map {s["map_area_m2"]} m2 '
                 f'(age {s["map_age_s"]} s) | LiDAR {s["lidar_hz"]} Hz (raw {s["raw_lidar_hz"]} Hz, {s["raw_kb"]} kB) | '
                 f'robot cmd {s["robot_cmd"]}')

    # -------------------------------------------------------------- main loop
    def run(self, readiness_seq):
        self.runner = SequenceRunner(readiness_seq, self.robot, self.mode, log=self.log,
                                     authorization=self.auth)
        self.runner.session_limit_s = self.cfg.max_minutes * 60 + 300
        self.log('=== M10 readiness (own checks; M0-M9 are not required) ===')
        beat = threading.Event()

        def heartbeat():          # readiness only; the roam loop writes its own heartbeat
            while not beat.wait(1.0):
                self.channel.write_status({'state': 'READINESS', 'heartbeat': time.time(),
                                           'pid': os.getpid(), 'mode': self.mode})
        thread = threading.Thread(target=heartbeat, daemon=True)
        if self.mode != MODE_SIM:
            self.channel.write_status({'state': 'READINESS', 'heartbeat': time.time(), 'pid': os.getpid()})
            thread.start()
        try:
            readiness = self.runner.run()
        finally:
            beat.set()
        ok = readiness['verdict'] in ('PASSED', 'READONLY_CHECKS_PASSED')
        summary = {'readiness': readiness}
        if not ok and self.mode != MODE_READONLY:
            self.state = 'STOPPED'
            self.reason = f'readiness {readiness["verdict"]}'
            summary['blockers'] = [(s['id'], s['summary']) for s in readiness['steps']
                                   if s['status'] in ('FAILED', 'ABORTED')]
            return self._finish(summary, 'REFUSED' if readiness['verdict'] == 'FAILED_SAFE'
                                else 'INTERVENTION_REQUIRED')
        if not ok:
            summary['blockers'] = [(s['id'], s['summary']) for s in readiness['steps']
                                   if s['status'] in ('FAILED', 'ABORTED')]
            self.log(f'DRY: physical roaming would be refused now: {summary["blockers"]}')
        return self._roam(summary)

    def _roam(self, summary):
        r = self.robot
        pose = self._pose()
        grid = self._grid()
        if pose is None or grid is None:
            self.reason = 'no SLAM pose or map'
            self.state = 'SAFETY_STOP' if r.can_actuate else 'STOPPED'
            return self._finish(summary, 'REFUSED')
        self.anchor = (pose[0], pose[1])
        self.area0 = known_area_m2(grid[1])
        self.explorer = Explorer(self.cfg, self.anchor)
        self.runner.geofence = (self.anchor[0], self.anchor[1], self.cfg.geofence_m)
        self.runner.origin = None
        self.t_roam = r.now()
        self.last_pose = self.counted_pose = pose
        problem = self._set_speed(self.cfg.start_mode)
        if problem:
            self.reason = problem
            self.state = 'SAFETY_STOP'
            return self._finish(summary, 'REFUSED')
        self.state = 'ROAMING'
        self.runner.nav_active_until = math.inf
        self.event('roaming', anchor=[_r(v, 2) for v in self.anchor], geofence_m=self.cfg.geofence_m,
                   speed_mode=self.speed_mode, dry=not r.can_actuate)
        last_status = last_line = -math.inf
        script = list(self.script)
        outcome = 'STOPPED'
        try:
            while self.state in ('ROAMING', 'PAUSED'):
                now = r.now()
                elapsed = now - self.t_roam
                while script and script[0][0] <= elapsed:
                    _, cmd, arg = script.pop(0)
                    ControlChannel.send(self.channel.dir, cmd, arg)
                if self.sim_stop_after_s is not None and elapsed >= self.sim_stop_after_s:
                    ControlChannel.send(self.channel.dir, 'stop')
                    self.sim_stop_after_s = None
                for command in self.channel.poll():
                    self._handle(command)
                if self.state not in ('ROAMING', 'PAUSED'):
                    break
                if r.cancelled():
                    wd = self.channel.read_json(self.channel.watchdog_path) or {} \
                        if self.watchdog_external else {}
                    if wd.get('tripped'):
                        raise SafetyAbort(f'independent watchdog tripped: {wd.get("reasons")}')
                    self.reason = 'operator stop (Ctrl+C / signal)'
                    self.state = 'STOPPING'
                    break
                if elapsed > self.cfg.max_minutes * 60:
                    self.reason = f'session limit {self.cfg.max_minutes:.0f} min reached'
                    self.state = 'STOPPING'
                    break
                pose = self._pose() or self.last_pose
                self._grid()
                if pose and self.counted_pose:
                    step = math.hypot(pose[0] - self.counted_pose[0], pose[1] - self.counted_pose[1])
                    if step >= 0.05:                     # ignore SLAM pose jitter (seen live)
                        self.stats['distance_m'] += step
                        self.counted_pose = pose
                    self.stats['max_from_anchor_m'] = max(self.stats['max_from_anchor_m'], math.hypot(
                        pose[0] - self.anchor[0], pose[1] - self.anchor[1]))
                self.last_pose = pose
                if r.can_actuate:
                    if self.state == 'ROAMING':
                        self.runner._check()                    # feeds the dead-man only if all pass
                        why = self.roam_violation(now)
                        if why:
                            raise SafetyAbort(why)
                        self._manage_goal(now, pose)
                        if now - getattr(self, '_limit_sent', -math.inf) >= 1.0:
                            r.set_speed_limit(self.cfg.mode_speed(self.speed_mode)[0])
                            self._limit_sent = now
                    else:
                        self.runner._check(feed=False)          # paused: dead-man stays locked
                else:
                    self._dry_tick(now, pose)
                if now - last_status >= 0.5:
                    last_status = now
                    status = self.status(now, pose)
                    self.channel.write_status(status)
                    if now - last_line >= 1.0:
                        last_line = now
                        self._line(status)
                r.wait(0.05)
        except SafetyAbort as error:
            self.reason = str(error)
            self.state = 'SAFETY_STOP'
            outcome = 'SAFETY_STOP'
            self.event('safety_stop', reason=self.reason)
        except Exception as error:  # noqa: BLE001 - stop safely, then report
            self.reason = f'internal error {type(error).__name__}: {error}'
            self.state = 'SAFETY_STOP'
            outcome = 'SAFETY_STOP'
            self.event('safety_stop', reason=self.reason)
        # ---- stop: tell the watchdog first (it must not kill a runner that is stopping)
        try:
            self.channel.write_status(self.status(r.now(), self.last_pose))
        except Exception:  # noqa: BLE001
            pass
        # ---- stop: cancel Nav2, zeros above every lock, verify rest
        if r.can_actuate:
            try:
                r.nav_cancel()
            except Exception:  # noqa: BLE001
                pass
            self.runner.nav_active_until = r.now() + 1.5
            verified, detail = self.runner.safe_stop()
            summary['rest_verified'] = verified
            summary['rest_detail'] = detail
            if not verified:
                outcome = 'INTERVENTION_REQUIRED'
        if self.state == 'STOPPING':
            self.state = 'STOPPED'
        return self._finish(summary, outcome)

    def _dry_tick(self, now, pose):
        """Dry: nothing is published. Evaluate what roaming would do and report health."""
        if self.goal is None or now - (self.goal_sent_at or -math.inf) > 5.0:
            grid = self.map
            if grid is not None and pose is not None:
                goal = self.explorer.choose(grid[1], pose, now, self._costmap())
                if goal and (self.goal is None or (goal['x'], goal['y']) != (self.goal['x'], self.goal['y'])):
                    self.event('would_send_goal', kind=goal['kind'], x=_r(goal['x'], 2), y=_r(goal['y'], 2))
                self.goal, self.goal_sent_at = goal, now
        why = self.roam_violation(now)
        if why and why != getattr(self, '_dry_last', None):
            self.event('dry_would_stop', reason=why)
        self._dry_last = why

    def _finish(self, summary, outcome):
        if self.state in ('ROAMING', 'PAUSED', 'STOPPING', 'READY') and outcome == 'STOPPED':
            self.state = 'STOPPED'
        grid = self.map
        summary.update({
            'test': 'M10', 'title': 'Autonomous roaming', 'mode': self.mode, 'outcome': outcome,
            'final_state': self.state, 'stop_reason': self.reason,
            'config': self.cfg.as_dict(), 'speed_mode_at_end': self.speed_mode,
            'anchor': self.anchor, 'stats': {k: _r(v) if isinstance(v, float) else v
                                             for k, v in self.stats.items()},
            'map_area_m2': {'start': _r(self.area0, 1),
                            'end': _r(known_area_m2(grid[1]), 1) if grid else None},
            'roaming_s': _r(self.robot.now() - self.t_roam, 1) if self.t_roam else 0.0,
            'events': self.events,
            'goals_vetoed_by_pointcloud': self.explorer.vetoed if self.explorer else 0,
        })
        try:
            status = self.status(self.robot.now(), self.last_pose)
            status['state'] = self.state
            self.channel.write_status(status)
        except Exception:  # noqa: BLE001 - status is informational
            pass
        return outcome, summary


# ---------------------------------------------------------------------------------- reports
def write_roam_report(summary, directory):
    from .report import write_reports
    directory = Path(directory)
    directory.mkdir(parents=True, exist_ok=True)
    readiness = summary.get('readiness')
    if readiness:
        write_reports(readiness, directory / 'readiness')
    data = {k: v for k, v in summary.items() if k != 'readiness'}
    data['readiness_verdict'] = readiness['verdict'] if readiness else None
    (directory / 'report.json').write_text(json.dumps(data, indent=1, default=str))
    s = data['stats']
    lines = [f'# M10 autonomous roaming: {data["outcome"]}', '',
             f'- Mode: {data["mode"]}; roaming {data["roaming_s"]} s; final state {data["final_state"]}',
             f'- Stop reason: {data["stop_reason"]}',
             f'- Readiness: {data["readiness_verdict"]} (report in readiness/)',
             f'- Rest verified after stop: {data.get("rest_verified")}',
             f'- Goals: {s["goals_sent"]} sent ({s["frontier_goals"]} frontier, {s["patrol_goals"]} patrol), '
             f'{s["goals_succeeded"]} reached, {s["goals_failed"]} failed',
             f'- Distance (SLAM pose): {s["distance_m"]} m; farthest from start {s["max_from_anchor_m"]} m '
             f'(geofence {data["config"]["geofence_m"]} m)',
             f'- Minimum obstacle clearance: {s["min_clearance_m"]} m',
             f'- Mapped area: {data["map_area_m2"]["start"]} -> {data["map_area_m2"]["end"]} m2',
             f'- Pauses: {s["pauses"]}; speed changes: {s["speed_changes"]}; speed mode at end '
             f'{data["speed_mode_at_end"]}', '', '## Events', '']
    lines += [f'- {json.dumps(e, default=str)}' for e in data['events']]
    if data.get('blockers'):
        lines += ['', '## Blockers', ''] + [f'- {a}: {b}' for a, b in data['blockers']]
    (directory / 'report.md').write_text('\n'.join(lines) + '\n')
    return directory / 'report.md'


# ---------------------------------------------------------------------------------- CLI
CONFIRM = (
    'The handheld remote / emergency stop is in YOUR hand, tested today, and reachable at all times.',
    'The robot is in Navigation (auto) mode on flat ground; nobody else is commanding it.',
    'No other group or program is using this robot now.',
    'The roaming area (geofence radius around the start) is enclosed or watched; people stay out.',
    'No stairs, drops or fragile objects inside the area (stairs are not supported).',
    'You will supervise the robot for the WHOLE session and can see it at all times.',
    'You accept the robot may coast ~1.6 s after commands stop (measured, not guaranteed).',
)


def stack_state():
    state = {}
    path = os.environ.get('LITE3_STATE_FILE')
    if path and Path(path).exists():
        for line in Path(path).read_text().splitlines():
            if '=' in line:
                k, v = line.split('=', 1)
                state[k] = v
    return state


def control_dir(state=None):
    root = Path(os.environ.get('LITE3_STATE_FILE') or Path.home() / 'lite3_ws/log/operator/state.env').parent
    return root / 'roam'


def parse(argv):
    p = argparse.ArgumentParser(prog='roam', description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = p.add_subparsers(dest='action', required=True)
    run = sub.add_parser('run', help='start a roaming session (dry unless --physical)')
    mode = run.add_mutually_exclusive_group()
    mode.add_argument('--physical', action='store_true', help='ACTUATE (armed stack + typed ROAM)')
    mode.add_argument('--sim', action='store_true', help='software stand-in, no robot')
    run.add_argument('--speed', default='slow', choices=MODE_NAMES, help='start speed mode (default slow)')
    run.add_argument('--speed-limit', type=float, default=None,
                     help='session ceiling m/s (default: the stack\'s MAX_SPEED, <= 0.10)')
    run.add_argument('--modes', default='', help='override modes, e.g. "slow=0.04,normal=0.07,fast=0.09" '
                     '(optionally speed:yaw)')
    run.add_argument('--geofence', type=float, default=3.0, help='radius around the start, m [1-10]')
    run.add_argument('--max-minutes', type=float, default=HARD_MAX_ROAM_MINUTES,
                     help=f'session bound, minutes [1-{HARD_MAX_ROAM_MINUTES:.0f}]; roaming otherwise '
                          'runs until you stop it')
    run.add_argument('--abort-clearance', type=float, default=0.08, help='footprint clearance abort, m')
    run.add_argument('--frontier-radius', type=float, default=3.0, help='frontier search radius, m [1-5]')
    run.add_argument('--frontier-clearance', type=float, default=0.40, help='goal clearance, m [0.3-1]')
    run.add_argument('--max-map-age', type=float, default=15.0, help='SLAM map freshness, s [3-30]')
    run.add_argument('--max-sensor-age', type=float, default=0.5, help='LiDAR/terrain freshness, s')
    run.add_argument('--min-lidar-hz', type=float, default=HARD_MIN_LIDAR_HZ)
    run.add_argument('--direct-telemetry', action='store_true')
    run.add_argument('--no-watchdog', action='store_true', help='dry/sim only')
    run.add_argument('--sim-seconds', type=float, default=240.0, help='sim: stop after this long')
    run.add_argument('--sim-script', default='', help='sim: "30:pause;40:resume;60:speed:fast"')
    run.add_argument('--report-dir', default=None)
    for name, text in (('pause', 'pause (cancel goal, zeros, dead-man locked)'),
                       ('resume', 'resume after pause (re-checks first)'),
                       ('stop', 'stop roaming: cancel goal, zeros, verify rest, report'),
                       ('status', 'show the running session')):
        sub.add_parser(name, help=text)
    sp = sub.add_parser('speed', help='switch speed mode: slow | normal | fast | up | down')
    sp.add_argument('value')
    sub.add_parser('modes', help='show speed modes and what is (not) verifiable on this robot')
    return p.parse_args(argv)


def build_config(a, state):
    stack_max = float(state.get('MAX_SPEED', 0.10)) if state.get('MAX_SPEED') else 0.10
    limit = a.speed_limit if a.speed_limit is not None else stack_max
    if limit > stack_max + 1e-9:
        raise ConfigError(f'--speed-limit {limit} exceeds the {stack_max} m/s the collision stop boundary '
                          'was computed for (restart navigation with --max-speed)')
    modes = parse_modes(a.modes)
    # Modes above the session limit stay listed but cannot be selected (refused at switch time).
    return RoamConfig(speed_limit=limit, modes=modes, start_mode=a.speed, geofence_m=a.geofence,
                      max_minutes=a.max_minutes, abort_clearance_m=a.abort_clearance,
                      frontier_radius_m=a.frontier_radius, frontier_clearance_m=a.frontier_clearance,
                      max_map_age_s=a.max_map_age, max_sensor_age_s=a.max_sensor_age,
                      min_lidar_hz=a.min_lidar_hz, stack_max_speed=stack_max)


def parse_script(text):
    out = []
    for part in (text or '').split(';'):
        if not part.strip():
            continue
        bits = part.split(':')
        out.append((float(bits[0]), bits[1], bits[2] if len(bits) > 2 else None))
    return out


def confirm(cfg):
    if not sys.stdin.isatty():
        print('REFUSED: physical roaming needs an interactive terminal for the operator confirmation.')
        return False
    print('\nConfirm ALL of the following:')
    for text in CONFIRM:
        print(f'  [ ] {text}')
    print(f'  Roaming limits: geofence {cfg.geofence_m} m around the start, start mode {cfg.start_mode} '
          f'({cfg.mode_speed(cfg.start_mode)[0]} m/s), session limit {cfg.speed_limit} m/s, '
          f'at most {cfg.max_minutes:.0f} min.')
    previous = signal.signal(signal.SIGINT, signal.default_int_handler)
    try:
        answer = input('Type ROAM and press Enter to START (anything else cancels): ').strip()
    except (EOFError, KeyboardInterrupt):
        print()
        return False
    finally:
        signal.signal(signal.SIGINT, previous)
    return answer == 'ROAM'


def make_authorization(seq, cfg, directory):
    template = authorization_template(seq, hours=min(4.0, cfg.max_minutes / 60 + 0.5))
    template['limits'].update({'max_speed_m_s': cfg.speed_limit,
                               'max_yaw_rate_rad_s': NAV2_MAX_YAW_RATE_RAD_S,
                               'max_nav_distance_m': min(6.0, cfg.geofence_m)})
    template['operator_confirmations'] = {k: True for k in template['operator_confirmations']}
    template['confirmed_at'] = dt.datetime.now().isoformat(timespec='seconds')
    path = Path(directory) / 'authorization.yaml'
    path.write_text('# Generated from the operator\'s typed ROAM confirmation (M10)\n' +
                    yaml.safe_dump(template, sort_keys=False))
    auth = load_authorization(path)
    problems = check_authorization(auth, seq)
    if problems:
        raise ConfigError(f'authorization: {problems}')
    return auth


def show_modes(cfg_modes, limit):
    print('Speed modes (velocity envelopes in the robot\'s standard gait):')
    for name in MODE_NAMES:
        v, w = cfg_modes[name]
        print(f'  {name:<7} {v:.2f} m/s, {w:.2f} rad/s{"" if v <= limit + 1e-9 else "  (above the session limit: refused)"}')
    print('Gait modes: NOT selectable or verifiable on this deployment. The vendor ROS bridge has no gait\n'
          'command, the firmware\'s robot_gait_state is dropped by the deployed motion_receiver and no topic\n'
          'reports it, so M10 never sends gait codes; it keeps the gait the robot stands up in.')


def main(argv=None):
    a = parse(sys.argv[1:] if argv is None else argv)
    state = stack_state()
    directory = control_dir(state)
    if a.action in ('pause', 'resume', 'stop', 'speed'):
        status = ControlChannel(directory).read_json(directory / 'status.json') or {}
        alive = status.get('state') in ('READY', 'ROAMING', 'PAUSED') and \
            time.time() - status.get('heartbeat', 0) < 5
        if not alive:
            print(f'No roaming session is running (last state {status.get("state")}).')
            return 3
        sent = ControlChannel.send(directory, a.action, getattr(a, 'value', None))
        deadline = time.time() + 5
        answered = False
        while time.time() < deadline:
            time.sleep(0.2)
            status = ControlChannel(directory).read_json(directory / 'status.json') or {}
            if (status.get('last_command') or {}).get('id') == sent['id']:
                answered = True
                break
            if status.get('state') in ('STOPPED', 'SAFETY_STOP'):
                break
        if not answered and status.get('state') not in ('STOPPED', 'SAFETY_STOP'):
            print(f'{a.action} sent, but the session has not confirmed it within 5 s (state {status.get("state")}).')
            return 1
        print(f'{a.action} sent. Now: {status.get("state")} | speed {status.get("speed_mode")} '
              f'{status.get("speed_m_s")} m/s | last command {status.get("last_command")}')
        return 0
    if a.action == 'status':
        status = ControlChannel(directory).read_json(directory / 'status.json')
        if not status:
            print('No roaming status yet.')
            return 3
        age = time.time() - status.get('heartbeat', 0)
        print(json.dumps(status, indent=1, default=str))
        print(f'(status {age:.1f} s old)')
        return 0
    if a.action == 'modes':
        show_modes(DEFAULT_MODES, float(state.get('MAX_SPEED', 0.10)))
        return 0
    return run_session(a, state, directory)


def run_session(a, state, directory):
    mode = MODE_PHYSICAL if a.physical else MODE_SIM if a.sim else MODE_READONLY
    if mode == MODE_SIM:
        directory = Path(directory) / 'sim'      # never mixes with a real session's control files
    try:
        cfg = build_config(a, state if mode != MODE_SIM else {})
        script = parse_script(a.sim_script)
    except (ConfigError, ValueError) as error:
        print(f'REFUSED: {error}')
        return 3
    if mode == MODE_PHYSICAL:
        if state.get('MODE') != 'physical':
            print('REFUSED: navigation is not running in physical mode (operator/roam.sh start --physical '
                  'starts it for you, or operator/start_navigation.sh --physical --clearance 0.20).')
            return 3
        if os.environ.get('ROS_LOCALHOST_ONLY') == '1':
            print('REFUSED: ROS_LOCALHOST_ONLY=1 cannot reach the robot.')
            return 3
    status = ControlChannel(directory).read_json(directory / 'status.json') or {}
    if status.get('state') in ('READY', 'ROAMING', 'PAUSED') and time.time() - status.get('heartbeat', 0) < 5 \
            and status.get('pid') != os.getpid():
        print(f'REFUSED: a roaming session is already running (pid {status.get("pid")}).')
        return 3
    root = Path(a.report_dir) if a.report_dir else (
        Path(state['RUN_DIR']) / 'roam' if state.get('RUN_DIR') else directory / 'reports')
    out = root / f'{dt.datetime.now():%Y%m%d-%H%M%S}-{mode}'
    out.mkdir(parents=True, exist_ok=True)
    backup = backup_distance(cfg, float(state.get('BOUNDARY') or 0), mode)
    if cfg.speed_limit > VALIDATED_SPEED_M_S + 1e-9:
        print(f'NOTE: session limit {cfg.speed_limit} m/s is above the validated 0.10 m/s: braking distance is '
              f'ASSUMED (not measured) and the backup lock moves to {backup} m '
              f'(collision_monitor boundary {state.get("BOUNDARY")} m).')
    seq_dict = readiness_sequence(mode, boundary_check(state) if mode != MODE_SIM else {}, backup)
    seq_path = out / 'readiness_sequence.yaml'
    seq_path.write_text(yaml.safe_dump(seq_dict, sort_keys=False))
    seq = load_sequence(seq_path)
    print(f'M10 autonomous roaming - mode {mode.upper()}' + {
        MODE_PHYSICAL: ' (the robot WILL walk on its own after you type ROAM)',
        MODE_SIM: ' (software stand-in, no robot)',
        MODE_READONLY: ' (dry: nothing is published; shows what roaming would do)'}[mode])
    print(f'Limits: geofence {cfg.geofence_m} m | speed limit {cfg.speed_limit} m/s | start mode '
          f'{cfg.start_mode} {cfg.mode_speed(cfg.start_mode)} | abort clearance {cfg.abort_clearance_m} m | '
          f'session <= {cfg.max_minutes:.0f} min | map age <= {cfg.max_map_age_s} s | LiDAR >= '
          f'{cfg.min_lidar_hz} Hz')
    print('Control from any terminal: operator/roam.sh pause | resume | stop | speed slow|normal|fast | status')
    auth = None
    if mode == MODE_PHYSICAL:
        if not confirm(cfg):
            print('Not confirmed. Nothing was sent.')
            return 3
        try:
            auth = make_authorization(seq, cfg, out)
        except ConfigError as error:
            print(f'REFUSED: {error}')
            return 3
    cancel = threading.Event()
    old = {s: signal.signal(s, lambda *_: cancel.set()) for s in (signal.SIGINT, signal.SIGTERM)}
    channel = ControlChannel(directory)
    watchdog_proc = None
    try:
        use_watchdog = mode != MODE_SIM and not (a.no_watchdog and mode == MODE_READONLY)
        if use_watchdog:          # start first: its DDS discovery overlaps with ours
            channel.write_status({'state': 'READY', 'heartbeat': time.time(), 'pid': os.getpid()})
            watchdog_proc = start_watchdog(cfg, directory, out, publish=(mode == MODE_PHYSICAL),
                                           direct=a.direct_telemetry or state.get('TELEMETRY') == 'direct')
        if mode == MODE_SIM:
            from .roam_sim import make_sim_robot
            robot = make_sim_robot()
            robot.cancelled = cancel.is_set
            watchdog = None if a.no_watchdog else WatchdogLogic(cfg.speed_limit, cfg.geofence_m)
        else:
            from .roam_ros import DryRoamRobot, RoamRobot
            from .ros_io import NAV_WIRING, direct_telemetry
            direct = a.direct_telemetry or state.get('TELEMETRY') == 'direct'
            robot = (RoamRobot if mode == MODE_PHYSICAL else DryRoamRobot)(direct_telemetry(NAV_WIRING) if direct else NAV_WIRING,
                              allow_commands=(mode == MODE_PHYSICAL), cancel_flag=cancel)
            deadline = time.monotonic() + 10
            while time.monotonic() < deadline and robot.telemetry.joints.rate(time.monotonic()) < 50:
                time.sleep(0.2)
            watchdog = None
        session = RoamSession(robot, cfg, mode, channel, log=lambda m: print(m, flush=True),
                              authorization=auth, watchdog=watchdog, script=script,
                              sim_stop_after_s=a.sim_seconds if mode == MODE_SIM else None)
        if use_watchdog:
            channel.write_status({'state': 'READY', 'heartbeat': time.time(), 'pid': os.getpid()})
            if not wait_watchdog(channel, 30.0):
                print('REFUSED: the independent watchdog did not become ready within 30 s (see watchdog.log).')
                return 3
            print('Independent watchdog ready.')
            session.watchdog_external = True
        outcome, summary = session.run(seq)
        report = write_roam_report(summary, out)
        print(f'\nM10 OUTCOME: {outcome} - {summary.get("stop_reason")}')
        st = summary['stats']
        print(f'  goals {st["goals_sent"]} sent / {st["goals_succeeded"]} reached / {st["goals_failed"]} failed | '
              f'distance {st["distance_m"]} m | farthest {st["max_from_anchor_m"]} m | mapped '
              f'{summary["map_area_m2"]["start"]} -> {summary["map_area_m2"]["end"]} m2 | min clearance '
              f'{st["min_clearance_m"]} m | rest verified {summary.get("rest_verified")}')
        print(f'Report: {report}')
        return EXIT.get(outcome, 1)
    finally:
        try:
            if 'robot' in locals():
                if mode == MODE_PHYSICAL and backup > HARD_MIN_STOP_DISTANCE_M + 1e-9:
                    robot.set_stop_distance(DEFAULT_STOP_AHEAD_M)        # restore the default lock
                robot.close() if hasattr(robot, 'close') else None
        except Exception:  # noqa: BLE001
            pass
        if watchdog_proc is not None:
            try:
                watchdog_proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                watchdog_proc.terminate()
        for s, h in old.items():
            signal.signal(s, h)


def start_watchdog(cfg, directory, out, publish, direct):
    cmd = [sys.executable, '-m', 'lite3_movement_tests.roam_watchdog', '--runner-pid', str(os.getpid()),
           '--dir', str(directory), '--speed-limit', str(cfg.speed_limit), '--geofence', str(cfg.geofence_m),
           '--heartbeat', str(cfg.watchdog_timeout_s)]
    if publish:
        cmd.append('--publish-stop')
    if direct:
        cmd.append('--direct-telemetry')
    log = open(Path(out) / 'watchdog.log', 'w')
    return subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)


def wait_watchdog(channel, timeout):
    deadline = time.time() + timeout
    while time.time() < deadline:
        wd = channel.read_json(channel.watchdog_path) or {}
        if time.time() - wd.get('heartbeat', 0) < 1.5 and wd.get('ready'):
            return True
        time.sleep(0.25)
    return False


if __name__ == '__main__':
    raise SystemExit(main())
