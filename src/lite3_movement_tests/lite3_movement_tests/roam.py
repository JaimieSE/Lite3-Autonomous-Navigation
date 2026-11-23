"""M10 autonomous roaming: explore without goals, keep mapping, avoid obstacles, until stopped.

Independent of M0-M9: no gates, no earlier report is read. M10 runs its OWN automated readiness
(telemetry contract, Nav2/SLAM/terrain/LiDAR, the physical mux wiring, the collision boundary
re-check against live sensing delay, zero-delivery to motion_sender with the idle dead-man check,
stand + verified standing) before anything moves.

Command path (all existing components; this module talks to none of them directly):

    scan-walk (default): walk through a known open corridor. If none is visible, search in short
      left/right Nav2 turns and check each new map for an opening.
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

from .ros_io import AI_OPTIONS, GAITS
from .advisory import parse_advisory
from .coverage import CameraCoverage
from .mission_events import parse_mission_event
from .stairs import Action, AiModeStairs, HStepGaitStairs, StairConfig, StairRoutine
from .config import (ConfigError, HARD_MAX_NAV_GEOFENCE_M, HARD_MAX_SPEED_M_S,
                     HARD_MIN_ABORT_CLEARANCE_M, HARD_MIN_ROAM_STOP_DISTANCE_M, NAV2_MAX_YAW_RATE_RAD_S, NOSE_OFFSET_M,
                     authorization_template, check_authorization, load_authorization,
                     load_sequence)
from .telemetry import sustained_planar_overspeed
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
HARD_MIN_LIDAR_HZ = 6.0               # was 8.0; course 12:37 safety stop at 7.4 Hz (terrain_monitor 7-9 Hz
                                       # walking the course; LiDAR raw 10 Hz)
HARD_MAX_WATCHDOG_S = 3.0
HARD_MAX_GOAL_S = 240.0
DEFAULT_ROAM_SPEED = 0.50
MODE_NAMES = ('slow', 'normal', 'fast')
SEARCH_CLEARANCE_M = 0.25             # frontier search (robot start + BFS): footprint half-width 0.185 + margin
COSTMAP_CLEAR_COOLDOWN_S = 20.0        # Nav2 ClearEntireCostmap at start and before giving up (see _clear_costmaps)
COSTMAP_REFILL_S = 2.0                 # let live LiDAR/terrain re-mark real obstacles before choosing again
FRONTIER_FALLBACK_CLEARANCE_M = 0.05   # goal cell + neighbours (was 0.25: 12:41 run, the one exit gap sat 0.25 m from
                                       # inscribed cost; Nav2 checks the footprint along its own path)
COVERAGE_PERIOD_S = 0.3                # camera coverage update interval (coverage.py)
LOOK_RADIUS_M = 2.5                    # look-around: unseen mapped floor within this of the robot ...
LOOK_SIDE_HALF_DEG = 60.0              # ... in the sector +-60 deg around 90 deg left / right ...
LOOK_MIN_SIDE_M2 = 0.6                 # ... at least this much on a side: turn 90 deg to look at it
LOOK_SPACING_M = 1.2                   # no new look-around within this of an earlier one
COVER_RADIUS_M = 0.7                   # coverage: a spot this close to the walked path counts as covered
COVER_SAMPLE_M = 0.2                   # sample spacing along a candidate walk
COVER_MIN_NOVELTY = 0.3                # a walk with less than this fraction uncovered is "already walked"
COVER_MIN_WEIGHT = 0.2                 # score weight of a fully covered direction (1.0 when all new)
PATROL_MIN_NOVELTY_M = 1.0             # fallback patrol: only white space this far from everywhere walked
PATROL_MAP_CLEARANCE_M = 0.25          # patrol goal: free SLAM cells round the body half-width
FRONTIER_FALLBACK_VETO = 99            # Nav2 inscribed
COSTMAP_VETO = 99                      # Nav2 inscribed (was 70; 2026-11-21 live-map sweep at the curtain pocket:
                                       # only this opened the white path, inflation 70-98 fills ~1.5 m lanes)
TERRAIN_HOLD_MAX_S = 15.0              # proximity stop held this long -> pause for the operator
TERRAIN_HOLD_REASON = 'terrain proximity hold'
# Gaits (Comm Interface s1.2.5 codes in ros_io.GAITS). The Perception guide's waypoint recorder (s7.3.2,
# s8.2.2) pairs "even slow/medium gait" with flat floors and "uneven high step gait" with rough ground.
FAILED_WALK_MEMORY_S = 60.0           # a failed walk goal is avoided this long ...
FAILED_WALK_RADIUS_M = 0.4            # ... within this distance (scan-walk had no failure memory)
BACKTRACK_MIN_M = 0.8                 # dead end: walk back to the newest breadcrumb at least this far away
MAX_BACKTRACKS = 3                    # consecutive backtracks before pausing for the operator
TRAIL_SPACING_M = 0.5                 # breadcrumbs (walk start points) at least this far apart
EXIT_CHECK_PERIOD_S = 2.0
# Automatic stairs (operator 2026-11-21: "at the steps it should switch to AI mode, on its own"). Course ledge
# lip ~0.10-0.12 m; competition stairs ~0.05 m. Triggered from terrain_monitor's nearest step in the corridor.
STAIR_TRIGGER_M = 0.6                  # nearest step this close to base_link (nose ~0.3 m from it) ...
STAIR_MIN_RISE_M = 0.04                # ... and between these heights (up or down)
STAIR_MAX_RISE_M = 0.18                # = terrain max_step_m / max_drop_m (course lips 0.10-0.15 m)
STAIR_CONTINUE_M = 0.9                 # another lip this close when the routine would exit: stay in AI mode
STAIR_MAX_AI_S = 120.0                 # ... but at most this long per crossing
STAIR_COOLDOWN_S = 15.0                # no new trigger this long after a crossing (same edge behind the feet)
STAIR_MODES = ('ai', 'hstep')
ADVISORY_MODES = ('act', 'log', 'off')  # teammate's stair/curtain advisory: act on it, only log it, or ignore it
ADVISORY_MAX_AGE_S = 0.5               # it publishes every ~0.15 s; older = detector stopped
ADVISORY_STAIRS_TRIGGER_M = 0.9        # stairs nearer: start the crossing (its approach covers up to 1.0 m)
ADVISORY_STAIRS_WALK_TO_M = 0.6        # stairs farther: walk to this far from the first riser first
ADVISORY_ALIGN_DEG = 5.0               # turn to face the risers when the detector says more than this
CURTAIN_TRIGGER_M = 1.2                # curtain this near base_link: cross it
CURTAIN_CAMERA_AHEAD_M = 0.25          # RGB camera ahead of base_link (URDF camera mount x 0.252)
CURTAIN_PAST_M = 0.61 + 0.30           # walk on past the curtain: body length + margin
CURTAIN_SPEED_M_S = 0.15               # slow straight crossing ("request_slow_crossing")
CURTAIN_MAX_S = 20.0                   # at least; or twice the time the crossing takes at the speed
CURTAIN_STALL_S = 3.0                  # no CURTAIN_STALL_M of progress this long: blocked (not a curtain)
CURTAIN_STALL_M = 0.05
CURTAIN_MAX_TILT_RAD = 0.35
CURTAIN_COOLDOWN_S = 20.0
PASSAGE_ADVISORY_WAIT_S = 3.0          # at a blocked passage: wait this long for a curtain confirmation
# Uneven course (physical 2026-11-21, operator-authorised loosening): balance steps on lips read 0.15 m/s of
# leg odometry while idle in slow mode (limit was 0.10) and stopped 5 sessions at start; a 0.15 m lip tilts
# the 0.35 m hip base ~0.44 rad (limit was 0.35). The robot's own fall-down protection stays the last resort.
ODOM_MARGIN_M_S = 0.25                 # leg-odometry limit = max(2 x mode speed, mode + this); was 0.05
ROAM_MAX_TILT_RAD = 0.55               # runner body-tilt limit while roaming; was 0.35
WATCHDOG_MAX_TILT_RAD = 0.60           # independent watchdog (looser than the runner); was 0.40
OWN_AXIS_GRACE_S = 1.0                 # remote-takeover check skips /handle_state this long after our axis cmd
SCENE_MIN_CONFIDENCE = 0.5            # scene detections below this are ignored
SCENE_REPEAT_S = 10.0                 # the same label/QR is logged again only after this long
FLAT_GAIT = 'flat_medium'
ROUGH_GAIT = 'rug_hstep'       # stairs and rubble
GAIT_SETTLE_BURSTS = 3                 # x 0.5 s of zeros after a gait switch, before walking
LARGE_SPIN_RADIUS_M = 1.0              # above this (stack faster than ~0.15 m/s) warn: goals need ~2 m+ open
MIN_RAW_BYTES = 10000                  # serialized /livox/points; a healthy Mid-360 scan is ~0.2-0.5 MB
# Turn rates up to the manufacturer's navigation-mode limit (Comm Interface 1.2.12: 1.5 rad/s).
DEFAULT_MODES = {'slow': (0.05, 0.15), 'normal': (0.08, NAV2_MAX_YAW_RATE_RAD_S),
                 'fast': (0.10, NAV2_MAX_YAW_RATE_RAD_S)}
SPEED_GRACE_S = 2.0                    # after lowering the speed, the smoother ramps down
STRATEGIES = ('scan_walk', 'frontier')
SCAN_STEP_RAD = math.radians(30)
# Relative 30-degree steps: look up to 60 degrees to the preferred side, then 60 to the other.
SCAN_STEPS = (1, 1, -1, -1, -1, -1)
# behavior_server Spin publishes nav_cmd_vel directly (NOT through the velocity_smoother cap), at
# up to its max_rotational_vel = 0.2 rad/s = the validated turn rate. While spinning the robot-side
# yaw check therefore allows this instead of the selected mode's yaw rate.
SPIN_WZ_RAD_S = NAV2_MAX_YAW_RATE_RAD_S   # behavior_server max_rotational_vel (1.5 rad/s)
SPIN_STALL_S = 12.0                    # no turn progress (lock / collision_monitor holding) -> give up
TERRAIN_BODY_MARGIN_M = 0.05           # lite3_locomotion.terrain.StopParams.body_margin_m (default)
BODY_HALF_WIDTH_M = 0.185              # Nav2 footprint half width
BODY_TURN_RADIUS_M = 0.357             # footprint corner radius: swept by an in-place turn
MAP_SETTLE_S = 2.5                     # slam_toolbox map_update_interval 2 s: let the spin's scans land
WALK_PROGRESS_M = 0.3                  # a walk that got this far is a success even if Nav2 aborted
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
                 max_minutes=HARD_MAX_ROAM_MINUTES, abort_clearance_m=0.03, frontier_radius_m=3.0,
                 frontier_clearance_m=0.40, stuck_s=20.0, max_map_age_s=15.0, max_sensor_age_s=0.5,
                 min_lidar_hz=HARD_MIN_LIDAR_HZ, watchdog_timeout_s=2.0, max_goal_failures=5,
                 stack_max_speed=0.10, strategy='frontier', gait=None, docs_stops=False, heartbeat=False,
                 rough_gait=ROUGH_GAIT, ai_options=(), exit_open_radius_m=1.0, exit_min_area_m2=3.0,
                 exit_min_from_start_m=2.0, stop_at_exit=True, stair_mode='ai', auto_stairs=False,
                 ai_axis_forward=12000, advisory='act'):
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
        # 0 = off (operator 2026-11-20, docs-only stops: Nav2 avoidance only, no clearance abort).
        self.abort_clearance_m = 0.0 if abort_clearance_m == 0 else \
            _finite(abort_clearance_m, 'abort_clearance', HARD_MIN_ABORT_CLEARANCE_M, 0.5)
        if gait is not None and gait not in GAITS:
            raise ConfigError(f'unknown gait {gait!r} (gaits: {", ".join(GAITS)})')
        self.gait = gait                # None: keep the gait the robot stands up in
        if rough_gait not in GAITS:
            raise ConfigError(f'unknown rough gait {rough_gait!r} (gaits: {", ".join(GAITS)})')
        self.rough_gait = rough_gait    # switched to for stairs/rubble, then back to gait
        bad = [o for o in ai_options if o not in AI_OPTIONS]
        if bad:
            raise ConfigError(f'unknown AI option(s) {bad} (options: {", ".join(AI_OPTIONS)})')
        self.ai_options = tuple(ai_options)   # documented switches sent at start; none by default
        # Exit detector (competition: exit = any open area outside the maze). 0 disables it.
        self.exit_open_radius_m = 0.0 if not exit_open_radius_m else \
            _finite(exit_open_radius_m, 'exit_open_radius', 0.5, 3.0)
        self.exit_min_area_m2 = _finite(exit_min_area_m2, 'exit_min_area', 0.5, 50.0)
        self.exit_min_from_start_m = _finite(exit_min_from_start_m, 'exit_min_from_start', 0.0, 10.0)
        self.stop_at_exit = bool(stop_at_exit)
        if stair_mode not in STAIR_MODES:
            raise ConfigError(f'stair mode must be one of {STAIR_MODES}')
        self.stair_mode = stair_mode          # 'ai': AI state + axis commands; 'hstep': RUG H-Step gait
        # Off by default (operator 2026-11-21: AI mode only on stairs, not on the uneven course; stair
        # detection comes later). On: terrain-detected steps start the stair routine without the operator.
        self.auto_stairs = bool(auto_stairs)
        self.ai_axis_forward = int(_finite(ai_axis_forward, 'ai_axis_forward', 6554, 32767))
        if advisory not in ADVISORY_MODES:
            raise ConfigError(f'advisory must be one of {ADVISORY_MODES}')
        self.advisory = advisory
        # Navigation started with --stop-layers docs: no stop zone acts, so walk/turn planning is not
        # sized around one (only the body + STANDOFF_M / BODY_TURN_RADIUS_M margins).
        self.docs_stops = bool(docs_stops)
        self.heartbeat = bool(heartbeat)   # Comm Interface s1.2.1 heartbeat at 2 Hz (opt-in, unverified)
        self.frontier_radius_m = _finite(frontier_radius_m, 'frontier_radius', 1.0, 5.0)
        self.frontier_clearance_m = _finite(frontier_clearance_m, 'frontier_clearance', 0.30, 1.0)
        self.stuck_s = _finite(stuck_s, 'stuck_s', 5.0, 60.0)
        self.max_map_age_s = _finite(max_map_age_s, 'max_map_age', 3.0, HARD_MAX_MAP_AGE_S)
        self.max_sensor_age_s = _finite(max_sensor_age_s, 'max_sensor_age', 0.1, HARD_MAX_SENSOR_AGE_S)
        self.min_lidar_hz = _finite(min_lidar_hz, 'min_lidar_hz', HARD_MIN_LIDAR_HZ, 20.0)
        self.watchdog_timeout_s = _finite(watchdog_timeout_s, 'watchdog_timeout', 0.5, HARD_MAX_WATCHDOG_S)
        self.max_goal_failures = int(_finite(max_goal_failures, 'max_goal_failures', 1, 20))
        if strategy not in STRATEGIES:
            raise ConfigError(f'unknown strategy {strategy!r} ({", ".join(STRATEGIES)})')
        self.strategy = strategy

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
        self.clearance_m = cfg.frontier_clearance_m
        self.veto = COSTMAP_VETO
        self.visited = None            # [(x, y)] walked through; set -> patrol only goes somewhere new
        self.seen = None               # CameraCoverage; set -> patrol only to floor the camera has not seen

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
        pad = max(1, math.ceil(self.clearance_m / grid.resolution))
        return not any(grid.contains((i, j)) and grid.value((i, j)) > 20
                       for j in range(cy - pad, cy + pad + 1) for i in range(cx - pad, cx + pad + 1))

    def hazard_free(self, costmap, x, y):
        """Nav2 global costmap (2D SLAM map + LiDAR point-cloud hazards from terrain_monitor) has no
        high-cost cell within the goal clearance. Catches tables, low objects and drops that the 2D
        SLAM map cannot see."""
        if costmap is None:
            return True
        cx, cy = costmap.cell(x, y)
        pad = max(1, math.ceil(self.clearance_m / costmap.resolution))
        for j in range(cy - pad, cy + pad + 1):
            for i in range(cx - pad, cx + pad + 1):
                if costmap.contains((i, j)) and costmap.value((i, j)) >= self.veto:
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
        pad = max(1, math.ceil(max(self.clearance_m, PATROL_MAP_CLEARANCE_M) / grid.resolution))
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
                if self.seen is not None:
                    if self.seen.is_seen(wx, wy):
                        continue                   # the camera has looked there already
                elif self.visited is not None and any(math.hypot(wx - vx, wy - vy) < PATROL_MIN_NOVELTY_M
                                                      for vx, vy in self.visited):
                    continue
                score = min(novelty, 3.0) + 0.3 * min(d_robot, 2.0)
                if best is None or score > best[0]:
                    best = (score, wx, wy)
        if best is None:
            return None
        _, wx, wy = best
        return {'kind': 'patrol', 'x': wx, 'y': wy, 'yaw': math.atan2(wy - pose[1], wx - pose[0]),
                'score': _r(best[0])}

    def avoided_goal(self, goal, now):
        return self.key(goal['x'], goal['y']) in {k for k, t in self.blocked.items() if t > now}

    def sent(self, goal):
        self.recent.append((goal['x'], goal['y']))
        self.recent = self.recent[-20:]


class ScanWalk:
    """Select a walk from the current map, preferring an open corridor straight ahead.

    A direction is open while: the centre-line cell was seen FREE on the SLAM map (unknown = not
    seen, ends the ray), no occupied SLAM cell lies within the corridor half-width, and the Nav2
    global costmap (incl. the terrain point-cloud hazards) stays below the veto cost on the centre
    line. The walk goal stops WALK_STANDOFF_M before an obstacle. Pure: unit-tested."""

    HEADINGS = 36                     # every 10 deg
    MIN_WALK_M = 0.5                  # 0.3 tried 2026-11-20: walked the nose into unmapped space
    # The terrain proximity stop locks ALL Nav2 output - turning in place included - while a hazard is
    # within stop_ahead_m ahead (any motion) or TERRAIN_BODY_MARGIN_M round the body. A walk ending inside
    # either zone leaves the robot unable to spin away (seen in simulation). So: stop short of the LIVE
    # stop_ahead_m (it grows with speed: ~1.45 m from centre at 0.5 m/s) and keep the radius clear.
    STANDOFF_M = 0.355                # minimum: base_link this far before an obstacle = nose + 5 cm
                                      # (was 0.65, 0.455, 0.405; operator 2026-11-20 "decrease the boundaries")
    FORWARD_CLEAR_M = 1.0             # this much clear ahead of the nose: keep walking straight on
    STOP_AHEAD_MARGIN_M = 0.10        # beyond the live stop_ahead_m (coast + map/terrain disagreement)
    UNSEEN_STANDOFF_M = 0.20          # ... before unseen space / the geofence / the ray limit
    CORRIDOR_HALF_M = BODY_HALF_WIDTH_M + 0.03     # body + 3 cm each side (0.215 m; was +5 cm)
                                      # (was 0.40 then the 0.50 circle; course <= 1.5 m wide, operator 2026-11-20)
    BLIND_RADIUS_M = 0.45             # under/next to the robot: /scan range_min 0.4 leaves it unknown
    SPIN_CORRIDOR_HALF_M = 0.235      # terrain ahead-zone / collision_monitor polygon half-width
    BACKTRACK_FACTOR = 0.3            # directions within 60 deg of where we came from
    REVISIT_FACTOR = 0.5              # goals within 0.5 m of a recent goal

    def __init__(self, cfg, anchor):
        self.cfg = cfg
        self.anchor = anchor
        self.came_from = None          # heading of the last walk (its reverse is discouraged)
        # Operator 2026-11-20 "go in a straight line": the heading roaming HOLDS. Every walk goal
        # (and its final yaw) uses this heading, not the robot's current yaw, so small yaw errors at
        # the end of each walk do not add up into a zig-zag. Changed only when the way is blocked.
        self.heading_ref = None
        self.failed = []               # [(x, y, until)] walk goals that failed (FAILED_WALK_MEMORY_S)
        self.recent = []
        self.last_status = None
        self.last_rays = []
        self.vetoed = 0
        self.stop_ahead_m = None       # live terrain_monitor value (base_link centre), set each choose
        self.cm_front_m = None         # collision_monitor stop polygon front (base_link centre), at start
        # (x, y) -> True where terrain_monitor has OBSERVED floor (3D LiDAR). The SLAM map only marks
        # free space along /scan beams that hit something in the 0.43-0.83 m band, so open directions
        # stayed unknown and every scan found "no open direction" (physical 2026-11-20). Set per choose.
        self.floor_free = None
        # Coverage (operator 2026-11-23 "coverage exploration": missions in any order, the whole arena must
        # be seen). [(x, y)] walked through, shared with the frontier/patrol explorer; None = no memory.
        # Big course 2026-11-21: with no such memory roam paced one lane both ways for 45 s.
        self.visited = None

    def novelty(self, pose, heading, length):
        """Fraction of a walk along heading (from 0.3 m out to length) farther than COVER_RADIUS_M from the
        walked path. 1.0 without coverage memory or for a walk too short to sample. Pure."""
        if not self.visited or length <= 0.3:
            return 1.0
        import numpy as np
        n = max(2, int((length - 0.3) / COVER_SAMPLE_M) + 1)
        d = np.linspace(0.3, length, n)
        pts = np.stack([pose[0] + d * math.cos(heading), pose[1] + d * math.sin(heading)], axis=1)
        walked = np.asarray(self.visited, dtype=float)
        d2 = ((pts[:, None, :] - walked[None, :, :]) ** 2).sum(axis=2).min(axis=1)
        return float((d2 > COVER_RADIUS_M ** 2).mean())

    def stop_zone(self):
        """How far ahead (from base_link) something stops ALL motion, turning included: the terrain
        proximity stop ahead zone or the collision_monitor stop polygon, whichever reaches further."""
        return max(self.stop_ahead_m or 0.0, self.cm_front_m or 0.0)

    def standoff(self):
        return max(self.STANDOFF_M, self.stop_zone() + self.STOP_AHEAD_MARGIN_M)

    def spin_radius(self):
        """A full turn sweeps the stop zone (length x corridor) around the robot: nothing known may lie
        inside this radius of a goal, or the spin there stalls the moment it faces it."""
        return max(BODY_TURN_RADIUS_M, math.hypot(self.stop_zone(), ScanWalk.SPIN_CORRIDOR_HALF_M)) + 0.02

    @staticmethod
    def known_obstacles(grid, costmap, centre, reach):
        """World points of occupied SLAM cells and costmap-vetoed cells within reach of centre."""
        pts = []
        for g, test in ((grid, lambda v: v > 20), (costmap, lambda v: v >= COSTMAP_VETO)):
            if g is None:
                continue
            cx, cy = g.cell(*centre)
            r = math.ceil(reach / g.resolution)
            for j in range(max(0, cy - r), min(g.height, cy + r + 1)):
                for i in range(max(0, cx - r), min(g.width, cx + r + 1)):
                    if test(g.value((i, j))):
                        pts.append(g.world((i, j)))
        return pts

    def spin_clear(self, x, y, obstacles):
        r = self.spin_radius()
        return all(math.hypot(ox - x, oy - y) >= r for ox, oy in obstacles)

    def inside(self, x, y, margin=0.4):
        return math.hypot(x - self.anchor[0], y - self.anchor[1]) <= self.cfg.geofence_m - margin

    def block(self, x, y, now, seconds):
        self.failed = [f for f in self.failed if f[2] > now] + [(x, y, now + seconds)]

    def avoided(self, x, y, now):
        return now is not None and any(until > now and math.hypot(x - fx, y - fy) < FAILED_WALK_RADIUS_M
                                       for fx, fy, until in self.failed)

    def sent(self, goal):
        self.recent = (self.recent + [(goal['x'], goal['y'])])[-10:]
        self.came_from = goal['yaw']
        self.heading_ref = goal['yaw']

    def _blocked(self, grid, costmap, x, y, c, s, lateral, d):
        """Known obstacle at (x, y): occupied SLAM cell across the corridor, or costmap veto on the line."""
        if any(grid.contains(o) and grid.value(o) > 20
               for o in (grid.cell(x - k * s, y + k * c) for k in lateral)):
            return True
        if costmap is not None and d >= self.BLIND_RADIUS_M:
            cc = costmap.cell(x, y)
            return costmap.contains(cc) and costmap.value(cc) >= COSTMAP_VETO
        return False

    def ray(self, grid, costmap, pose, heading):
        """(open length, what ended it, distance of the first KNOWN obstacle) along heading.

        Open = seen free and unobstructed. The obstacle search continues past the open end (unseen
        space, geofence, range limit) by the standoff, so a goal can never be placed just in front of
        a known obstacle that lies beyond the point where the open stretch ends."""
        res = grid.resolution
        c, s = math.cos(heading), math.sin(heading)
        lateral = [k * res for k in range(-math.floor(self.CORRIDOR_HALF_M / res),
                                          math.floor(self.CORRIDOR_HALF_M / res) + 1)]
        limit = min(self.cfg.frontier_radius_m, self.cfg.geofence_m * 2)
        d, free, end = 0.0, None, 'range'
        horizon = limit + self.standoff()
        while d <= horizon:
            x, y = pose[0] + d * c, pose[1] + d * s
            if self._blocked(grid, costmap, x, y, c, s, lateral, d):
                return (d if free is None else free), ('obstacle' if free is None else end), d
            if free is None:
                cell = grid.cell(x, y)
                if d > limit:
                    free, end = limit, 'range'
                elif not self.inside(x, y):
                    free, end = d, 'geofence'
                elif (not grid.contains(cell) or (d >= self.BLIND_RADIUS_M and not 0 <= grid.value(cell) <= 20)) \
                        and not (d >= self.BLIND_RADIUS_M and self.floor_free and self.floor_free(x, y)):
                    free, end = d, 'unseen'
                if free is not None:
                    horizon = min(horizon, free + self.standoff())
            d += res / 2    # half-cell steps: whole-cell steps can round past a one-cell-thick SLAM wall
        return (limit if free is None else free), end, math.inf

    def heading(self, pose):
        """The held heading (the robot's yaw before the first walk)."""
        return pose[2] if self.heading_ref is None else self.heading_ref

    def straight_ahead(self, grid, pose, costmap=None, now=None):
        """Walk on along the HELD heading as far as the floor allows (up to frontier_radius_m), treating
        unseen floor beyond the blind near field as walkable; None if a KNOWN obstacle is too close.
        Long legs on one heading: each stop/restart is where the yaw drifted (zig-zag, 2026-11-20)."""
        heading = self.heading(pose)
        floor_free, self.floor_free = self.floor_free, (lambda x, y: True)
        try:
            free, end, obstacle = self.ray(grid, costmap, pose, heading)
        finally:
            self.floor_free = floor_free
        walk = min(self.cfg.frontier_radius_m, free - (0.0 if end == 'obstacle' else self.UNSEEN_STANDOFF_M),
                   obstacle - self.standoff())
        if walk < self.MIN_WALK_M or self.avoided(pose[0] + walk * math.cos(heading),
                                                  pose[1] + walk * math.sin(heading), now):
            return None
        if self.novelty(pose, heading, walk) < COVER_MIN_NOVELTY:
            self.last_status = 'walked_lane'               # straight on only into space not yet covered
            return None
        return {'kind': 'walk', 'x': pose[0] + walk * math.cos(heading), 'y': pose[1] + walk * math.sin(heading),
                'yaw': heading, 'score': _r(walk), 'free_m': _r(free, 2), 'ends_at': 'straight_on'}

    def choose(self, grid, pose, now, costmap=None, forward_blocked=False, allow_visited=False):
        rays = []
        reach = min(self.cfg.frontier_radius_m, self.cfg.geofence_m * 2) + self.spin_radius()
        obstacles = self.known_obstacles(grid, costmap, pose[:2], reach)
        self.spin_blocked = 0
        base = self.heading(pose)
        # Nearest turns first: 0, +10, -10, +20, -20 ... deg from the HELD heading.
        order = [0] + [s * k for k in range(1, self.HEADINGS // 2 + 1) for s in (1, -1)][:self.HEADINGS - 1]
        for i in order:
            heading = math.atan2(math.sin(base + i * 2 * math.pi / self.HEADINGS),
                                 math.cos(base + i * 2 * math.pi / self.HEADINGS))
            turn = abs(math.atan2(math.sin(heading - base), math.cos(heading - base)))
            if forward_blocked and turn <= SCAN_STEP_RAD:
                continue
            free, end, obstacle = self.ray(grid, costmap, pose, heading)
            walk = min(free - (0.0 if end == 'obstacle' else self.UNSEEN_STANDOFF_M), obstacle - self.standoff())
            if obstacle - self.standoff() < free - (0.0 if end == 'obstacle' else self.UNSEEN_STANDOFF_M):
                end = 'obstacle'
            # Operator 2026-11-20: with FORWARD_CLEAR_M clear ahead of the nose, keep walking straight on
            # (moving is what maps the space); the room-to-turn check at the end only applies to turns.
            straight_on = i == 0 and free - NOSE_OFFSET_M >= self.FORWARD_CLEAR_M and walk >= self.MIN_WALK_M
            # Back off along the ray until the next spin fits (nothing known inside spin_radius).
            while not straight_on and walk >= self.MIN_WALK_M and not self.spin_clear(
                    pose[0] + walk * math.cos(heading), pose[1] + walk * math.sin(heading), obstacles):
                walk -= 0.10
            if walk < self.MIN_WALK_M:
                self.spin_blocked += 1
                continue
            gx, gy = pose[0] + walk * math.cos(heading), pose[1] + walk * math.sin(heading)
            if self.avoided(gx, gy, now):
                continue
            nov = self.novelty(pose, heading, walk)
            if not allow_visited and nov < COVER_MIN_NOVELTY:
                continue                                   # already walked: frontier/patrol first
            score = walk * (COVER_MIN_WEIGHT + (1.0 - COVER_MIN_WEIGHT) * nov)
            if self.came_from is not None and abs(math.atan2(
                    math.sin(heading - self.came_from - math.pi), math.cos(heading - self.came_from - math.pi))) < math.pi / 3:
                score *= self.BACKTRACK_FACTOR
            if any(math.hypot(gx - rx, gy - ry) < 0.5 for rx, ry in self.recent):
                score *= self.REVISIT_FACTOR
            rays.append((round(score, 2), -turn, heading, walk, free, end, gx, gy))
        # Smallest turn from the held heading whose walk is worth taking: >= 75 % of the best walk available
        # (counted up to FORWARD_CLEAR_M). Not simply the nearest usable turn: with short minimum walks that
        # picked 0.3 m diagonal steps over a 2 m corridor 30 deg further round (a zig-zag again).
        if rays:
            want = 0.75 * min(max(r[0] for r in rays), self.FORWARD_CLEAR_M)
            worth = [r for r in rays if r[0] >= want - 1e-9]
            rays = [max(worth, key=lambda r: (r[1], r[0]))] + [r for r in rays if r not in worth]
        self.last_rays = [{'heading_deg': _r(math.degrees(h), 0), 'walk_m': _r(w, 2), 'free_m': _r(f, 2),
                           'ends_at': e} for _, _, h, w, f, e, _, _ in rays[:3]]
        if not rays:
            self.last_status = 'no_open_direction' if allow_visited or not self.visited else 'no_new_direction'
            return None
        self.last_status = 'open_direction'
        _, _, heading, walk, free, end, gx, gy = rays[0]
        return {'kind': 'walk', 'x': gx, 'y': gy, 'yaw': heading, 'score': _r(walk),
                'free_m': _r(free, 2), 'ends_at': end}


PASSAGE_REACH_M = 1.5          # wall within this distance of the robot
PASSAGE_BEHIND_M = 1.0         # look this far past the wall
PASSAGE_MIN_UNKNOWN = 0.6      # fraction of unseen cells behind the wall
PASSAGE_RAYS = 72              # every 5 deg
PASSAGE_MIN_WIDTH_RAYS = 3     # consecutive qualifying rays (>= body width at ~1 m)


def blocked_passage(grid, pose, reach_m=PASSAGE_REACH_M, behind_m=PASSAGE_BEHIND_M,
                    min_unknown=PASSAGE_MIN_UNKNOWN, rays=PASSAGE_RAYS, min_width=PASSAGE_MIN_WIDTH_RAYS):
    """Dead end: the nearby SLAM wall with mostly unseen space behind it, i.e. where the course goes on
    behind something the LiDAR reads as a wall (operator 2026-11-21: a curtain). Returns
    {'yaw', 'wall_m', 'unknown', 'width_deg'} for the middle of the widest qualifying run of rays, or None."""
    step = grid.resolution / 2
    hits = []
    for k in range(rays):
        a = pose[2] + k * 2 * math.pi / rays
        c, s_ = math.cos(a), math.sin(a)
        wall, d = None, 0.2
        while d <= reach_m:
            cell = grid.cell(pose[0] + d * c, pose[1] + d * s_)
            if not grid.contains(cell):
                break
            v = grid.value(cell)
            if v < 0:
                break                               # unseen before any wall: a frontier, not a wall
            if v > 50:
                wall = d
                break
            d += step
        frac = 0.0
        if wall is not None:
            seen = unseen = 0
            d = wall + 0.1
            while d <= wall + behind_m:
                cell = grid.cell(pose[0] + d * c, pose[1] + d * s_)
                if not grid.contains(cell) or grid.value(cell) < 0:
                    unseen += 1
                else:
                    seen += 1
                d += step
            frac = unseen / max(1, seen + unseen)
        hits.append((wall, frac))
    ok = [w is not None and f >= min_unknown for w, f in hits]
    if all(ok) or not any(ok):
        return None                                 # nothing, or the same on every side (no direction)
    start = ok.index(False)                         # walk the ring from a gap so runs do not wrap
    best, run = None, []
    for i in range(1, rays + 1):
        k = (start + i) % rays
        if ok[k]:
            run.append(k)
            continue
        if len(run) >= min_width:
            score = (len(run), sum(hits[j][1] for j in run) / len(run))
            if best is None or score > best[0]:
                best = (score, run)
        run = []
    if best is None:
        return None
    mid = best[1][len(best[1]) // 2]
    yaw = pose[2] + mid * 2 * math.pi / rays
    return {'yaw': math.atan2(math.sin(yaw), math.cos(yaw)), 'wall_m': _r(min(hits[j][0] for j in best[1]), 2),
            'unknown': _r(best[0][1], 2), 'width_deg': len(best[1]) * 360 // rays}


def detect_exit(grid, pose, anchor, open_radius_m=1.0, min_area_m2=3.0, min_from_start_m=2.0,
                window_m=5.0):
    """Competition exit = "any open area outside the maze", seen on the LIVE SLAM map. Assumptions: maze
    corridors are <= 1.5 m wide (so no known-free cell inside the maze is >= open_radius_m from every wall),
    and the open area is mapped (unknown cells count as walls, so unseen space never looks open).
    Returns None or {'x', 'y', 'area_m2', 'reached'}: the open cell nearest the robot. Pure: unit-tested."""
    res = grid.resolution
    cx, cy = grid.cell(pose[0], pose[1])
    r = int(window_m / res)
    x0, x1 = max(0, cx - r), min(grid.width - 1, cx + r)
    y0, y1 = max(0, cy - r), min(grid.height - 1, cy + r)
    w, h = x1 - x0 + 1, y1 - y0 + 1
    if w <= 0 or h <= 0:
        return None
    big = 1 << 30
    dist = [big] * (w * h)
    queue = []
    for y in range(y0, y1 + 1):
        for x in range(x0, x1 + 1):
            if not 0 <= grid.value((x, y)) <= 20:            # occupied or unknown: a "wall"
                i = (y - y0) * w + (x - x0)
                dist[i] = 0
                queue.append(i)
    head = 0
    while head < len(queue):                                  # 8-neighbour BFS: Chebyshev <= Euclidean
        i = queue[head]
        head += 1
        ix, iy = i % w, i // w
        for dx in (-1, 0, 1):
            for dy in (-1, 0, 1):
                jx, jy = ix + dx, iy + dy
                if (dx or dy) and 0 <= jx < w and 0 <= jy < h:
                    j = jy * w + jx
                    if dist[j] > dist[i] + 1:
                        dist[j] = dist[i] + 1
                        queue.append(j)
    need = math.ceil(open_radius_m / res)
    open_cells = []
    for i, d in enumerate(dist):
        if need <= d < big:
            x, y = grid.world((x0 + i % w, y0 + i // w))
            if math.hypot(x - anchor[0], y - anchor[1]) >= min_from_start_m:
                open_cells.append((x, y))
    area = len(open_cells) * res * res
    if area < min_area_m2:
        return None
    x, y = min(open_cells, key=lambda c: math.hypot(c[0] - pose[0], c[1] - pose[1]))
    return {'x': x, 'y': y, 'area_m2': round(area, 2),
            'reached': math.hypot(x - pose[0], y - pose[1]) <= 2 * res}


def parse_scene(text, min_confidence=SCENE_MIN_CONFIDENCE):
    """Scene-detection JSON (teammate's scene_node contract) -> [(kind, value, confidence)]. Pure: tested."""
    import json
    try:
        data = json.loads(text)
    except (TypeError, ValueError):
        return []
    found = [('object', str(o.get('label')), float(o.get('confidence', 0.0)))
             for o in data.get('objects', []) if isinstance(o, dict) and float(o.get('confidence', 0.0)) >= min_confidence]
    qr = data.get('qr')
    for value in (qr if isinstance(qr, list) else [qr] if qr else []):
        found.append(('qr', str(value), 1.0))
    return found


def known_area_m2(grid):
    return sum(1 for v in grid.data if v >= 0) * grid.resolution ** 2


# ---------------------------------------------------------------------------------- watchdog logic
class WatchdogLogic:
    """Independent limits (deliberately looser than the runner's, different sources where
    possible: leg odometry for the area, robot-side commands for speed). Pure: unit-tested and
    used by the watchdog process and the simulator."""

    # Staleness limits (2026-11-18, operator: fewer false trips from robot<->onboard-PC link delays).
    # Preflight max gaps over 53 sessions: telemetry 0.78 s (p95 0.45), /livox/points 0.83 s (p95 0.65).
    # Was 0.5 s / 1.0 s, which ordinary link hiccups exceeded. Kept just above the observed worst
    # case: the watchdog is the backstop; collision_monitor and the terrain lock act on obstacles.
    TELEMETRY_AGE_S = 1.0
    RAW_LIDAR_AGE_S = 1.25

    def __init__(self, speed_limit, geofence_m, max_tilt=WATCHDOG_MAX_TILT_RAD, telemetry_age=TELEMETRY_AGE_S,
                 heartbeat_s=2.0, raw_lidar_age=RAW_LIDAR_AGE_S):
        self.speed_limit = speed_limit
        self.geofence_m = geofence_m
        self.max_tilt = max_tilt
        self.telemetry_age = telemetry_age
        self.raw_lidar_age = raw_lidar_age
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
            if raw.age(now) > self.raw_lidar_age:
                reasons.append(f'raw LiDAR /livox/points stale ({_r(raw.age(now), 2)} s)')
            elif max((n for _, n in raw.window(now, 1.0)), default=0) < MIN_RAW_BYTES:
                reasons.append('raw LiDAR nearly empty (sensor blind or blocked)')
        moving = max(2.0 * self.speed_limit, self.speed_limit + 0.05)
        if sustained_planar_overspeed(telemetry, now, moving):
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
DEFAULT_STOP_AHEAD_M = 0.505           # terrain_monitor default (from base_link centre), restored afterwards


def backup_distance(cfg, boundary_m, mode):
    """Nose distance of the terrain backup lock. 0.20 m is the backup up to 0.10 m/s (0.30 m validated). Above
    that the robot would travel past it before stopping, so it moves out to just inside the
    collision_monitor boundary (one 5 cm grid cell), and the stack must have been started with a
    boundary computed for that speed."""
    if mode != MODE_PHYSICAL or cfg.speed_limit <= VALIDATED_SPEED_M_S + 1e-9 or not boundary_m:
        return HARD_MIN_ROAM_STOP_DISTANCE_M
    return min(2.95, max(HARD_MIN_ROAM_STOP_DISTANCE_M, round(boundary_m - 0.05, 2)))   # 3.0 m max boundary - 1 cell


# Roaming-only freshness limits (operator decision 2026-11-19; the physical tests keep 0.25 s / 0.5 s).
# The vendor documents set no freshness limit for our consumers; the robot's own safety is its 250 ms
# axis-command watchdog (Motion Host Communication Interface 1.2.3), which is unchanged, as are the
# independent watchdog (1.0 s), the 8 Hz LiDAR minimum and the collision/terrain stops, which act on
# the LiDAR hazards directly rather than on odometry.
ROAM_TELEMETRY_AGE_S = 1.0      # joints/IMU/leg odometry silence that stops roaming (was 0.25; = the
                                # independent watchdog's limit). 2026-11-19 13:48: a link stall (all robot
                                # streams gapped 0.3-0.75 s at once) aborted the stand at 0.5 s.
ROAM_MAX_ODOM_LAG_S = 1.0       # LiDAR odometry lag accepted at readiness (was 0.5)
ROAM_RAW_LIDAR_STALE_S = 1.0    # single raw-cloud gap that ends roaming (was max_sensor_age_s, 0.5)
TERRAIN_FLOOR_MAX_AGE_S = 1.0   # terrain grid used as "seen free" floor only while this fresh
ROAM_LIDAR_RATE_WINDOW_S = 5.0  # >= 8 Hz averaged over this (was 2 s: 15 of 20 clouds, so ~0.6 s silence
                                # already read as "7 Hz" and stopped the 10:16 run)


def readiness_sequence(mode, boundary_check, backup_nose=HARD_MIN_ROAM_STOP_DISTANCE_M):
    nav = dict({'id': 'nav_readiness', 'type': 'nav_readiness', 'timeout_s': 15,
                'stop_distance_m': backup_nose, 'max_odom_lag_s': ROAM_MAX_ODOM_LAG_S}, **boundary_check)
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
            'telemetry': {'max_age_s': ROAM_TELEMETRY_AGE_S},
            'safety': {'geofence_radius_m': 0.5, 'max_tilt_rad': ROAM_MAX_TILT_RAD, 'command_period_s': 0.05,
                       'zero_seconds': 2.0, 'stop_timeout_s': 6.0, 'stop_hold_s': 1.5},
            'steps': steps}


def boundary_check(state):
    # docs-only stops: no stop zone acts, so there is no braking boundary to re-check against live delay.
    if not state.get('BOUNDARY') or state.get('STOP_LAYERS') == 'docs':
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
        self.cancel_request = None
        self.trail = []                          # breadcrumbs: walk start points (dead-end backtracking)
        self.stairs = None                       # (StairRoutine, StairInterface, start xy) while on stairs
        self.stairs_until = -math.inf            # no automatic stairs before this (cooldown)
        self.scene_seen = {}                     # (kind, value) -> last time logged
        self.sightings = []                      # [{kind, value, confidence, x, y}] for the explorer/report
        self.backtracks = 0
        self.passage = None                      # dead end: blocked passage (curtain?) to stop in front of
        self.passages_done = []                  # (x, y) of passages already stopped at
        self.adv_task = None                     # advisory step in progress (turning to face stairs)
        self.mission_seen = set()                # mission event ids already logged
        self.coverage = None                     # CameraCoverage (scan-walk)
        self._coverage_t = -math.inf
        self.look = []                           # pending look-around turns (rad)
        self.look_spots = []                     # where look-arounds happened
        self.look_check = False                  # a walk just ended: consider a look-around
        self.adv_last = None                     # last logged advisory (kind, distance bucket)
        self.crossing = None                     # curtain crossing in progress
        self.curtain_until = -math.inf           # no new curtain crossing before this (cooldown)
        self.exit = None                         # detect_exit() result once an open area is seen
        self.exit_checked = -math.inf
        self.hold_since = None                   # terrain proximity hold started (None: not holding)
        self.goal_timeout = None
        self.progress = None                     # (time, x, y) of last 0.10 m of progress
        self.failures = 0
        self.events = []
        self.stats = {'goals_sent': 0, 'goals_succeeded': 0, 'goals_failed': 0, 'frontier_goals': 0,
                      'patrol_goals': 0, 'walk_goals': 0, 'spins': 0, 'spins_completed': 0,
                      'walks_to_obstacle': 0, 'distance_m': 0.0, 'max_from_anchor_m': 0.0,
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
        # One late cloud is not a blind LiDAR: allow ROAM_RAW_LIDAR_STALE_S (same as the Fast-LIO cloud
        # below). A real dropout is still caught by the 5 s rate check and by terrain_monitor's own
        # 0.5 s stale lock, which holds motion without ending the session (2026-11-19 13:43: a single
        # 0.53 s gap ended a 0.5 m/s session before it had moved).
        if raw.age(now) > ROAM_RAW_LIDAR_STALE_S:
            return f'raw LiDAR /livox/points stale ({_r(raw.age(now), 2)} s)'
        window = ROAM_LIDAR_RATE_WINDOW_S
        if now - self.t_roam > window and len(raw.window(now, window)) / window < self.cfg.min_lidar_hz - 0.5:
            return (f'raw LiDAR rate {len(raw.window(now, window)) / window:.1f} Hz < {self.cfg.min_lidar_hz} Hz '
                    f'over {window:.0f} s')
        scans = raw.window(now, 1.0)
        if scans and max(n for _, n in scans) < MIN_RAW_BYTES:
            return f'raw LiDAR nearly empty ({max(n for _, n in scans)} bytes/scan): sensor blind or blocked'
        lio = self.robot.telemetry.lio_cloud
        if lio.total and lio.age(now) > 1.0:
            return f'Fast-LIO sensor output stale ({_r(lio.age(now), 2)} s): SLAM odometry lost'
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
        if item is None or now - item[0] > self._terrain_max_age():
            return 'LiDAR/terrain status stale'
        if item[1].problems:
            return f'terrain inputs invalid: {list(item[1].problems)}'
        if t.clearance.age(now) > 0.5:
            return 'hazard cloud stale: obstacle clearance unknown'
        clearance = t.clearance.latest()[1]
        self.stats['min_clearance_m'] = min(self.stats['min_clearance_m'], clearance)
        if self.cfg.abort_clearance_m and clearance < self.cfg.abort_clearance_m:
            return f'obstacle {clearance:.2f} m from the footprint (abort below {self.cfg.abort_clearance_m} m)'
        # Same 5 s average as the raw-LiDAR rate (physical 2026-11-20 10:51: a 2 s window read one late
        # terrain update as "7.0 Hz" and ended a walk). A stopped terrain monitor is still caught at once by
        # the 0.5 s status / hazard-cloud staleness checks above.
        window = ROAM_LIDAR_RATE_WINDOW_S
        if now - self.t_roam > window:
            hz = len(t.terrain.window(now, window)) / window
            if hz < self.cfg.min_lidar_hz - 0.5:
                return f'LiDAR/terrain rate {hz:.1f} Hz < {self.cfg.min_lidar_hz} Hz over {window:.0f} s'
        if self.map is None or now - self.map[0] > self.cfg.max_map_age_s:
            return 'SLAM map not updating'
        why = self.lidar_violation(now)
        if why:
            return why
        v, w = self.cfg.mode_speed(self.speed_mode)
        if now - self.speed_changed > SPEED_GRACE_S:
            for _, c in t.system_cmd.window(now, 0.3):
                why = command_above_mode(c.vx, c.wz, v, w, self.speed_mode)
                if why:
                    return why
        for _, c in t.system_cmd.window(now, 0.3):
            if c.vx < -0.005:
                return 'reverse command reached the robot (no rear sensing)'
        # Actual motion (leg odometry) must match what is commanded: a runaway gait or slipping
        # estimator is caught even while every command is within limits.
        limit = max(2.0 * self.cfg.mode_speed(self.speed_mode)[0], self.cfg.mode_speed(self.speed_mode)[0] + ODOM_MARGIN_M_S)
        if now - self.speed_changed <= SPEED_GRACE_S + 1.0:
            limit = max(limit, 2.0 * self.cfg.speed_limit)       # ramp-down after lowering the mode
        odom = t.odom.window(now, 0.5)
        if sustained_planar_overspeed(t, now, limit):
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
        goal = self._choose(grid[1], now, pose)
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

    # -------------------------------------------------------------- spin-and-walk
    def _spinning(self, now):
        """A spin is active, or ended < 1 s ago (its last commands are still in the window)."""
        if self.goal and self.goal.get('kind') == 'spin':
            return True
        return now - getattr(self, '_spin_ended', -math.inf) < 1.0

    def _start_spin(self, now, pose, angle):
        # behavior_server min_rotational_vel 0.1 rad/s; the approach action never slows turning in place
        allowance = abs(angle) / 0.1 + 20.0
        if not self.robot.nav_spin(angle, allowance):
            self._walk_failed(now, 'Nav2 rejected the spin')
            return
        self.goal = {'kind': 'spin', 'x': pose[0], 'y': pose[1], 'yaw': pose[2]}
        self.scan_map_time = self.map[0]
        self.goal_sent_at = now
        self.spin_progress = (now, 0.0)
        self.stats['spins'] += 1
        self.event('spin', degrees=_r(math.degrees(angle), 0), map_m2=_r(known_area_m2(self.map[1]), 1) if self.map else None)

    def _end_spin(self, now, how, spun):
        self.event('spin_' + how, turned_deg=_r(math.degrees(spun or 0.0), 0))
        if how == 'done':
            self.stats['spins_completed'] += 1
        self.goal = None
        self._spin_ended = now
        self.choose_after = now if how == 'opening' else now + MAP_SETTLE_S

    def _floor_lookup(self, now, pose):
        """(x, y) -> observed floor, from the latest terrain_monitor grid (level base_link frame, centred
        on the robot). Only used while fresh: choices are made standing still after a turn."""
        item = self.robot.terrain_floor() if hasattr(self.robot, 'terrain_floor') else None
        if item is None or now - item[0] > TERRAIN_FLOOR_MAX_AGE_S:
            return None
        g = item[1]
        c, s = math.cos(pose[2]), math.sin(pose[2])

        def free(x, y):
            dx, dy = x - pose[0], y - pose[1]
            bx, by = c * dx + s * dy, -s * dx + c * dy           # map -> base_link (yaw only, level grid)
            i, j = int(math.floor((bx - g['ox']) / g['res'])), int(math.floor((by - g['oy']) / g['res']))
            return 0 <= i < g['w'] and 0 <= j < g['h'] and g['data'][j * g['w'] + i] == 0
        return free

    def _choose(self, grid, now, pose, **kwargs):
        if hasattr(self.explorer, 'floor_free'):
            self.explorer.floor_free = self._floor_lookup(now, pose)
        return self.explorer.choose(grid, pose, now, self._costmap(), **kwargs)

    def _walk_candidate(self, now, pose, allow_visited=False):
        sample = self._terrain_now(now)
        if sample is not None and sample.stop_ahead_m and not self.cfg.docs_stops:
            self.explorer.stop_ahead_m = sample.stop_ahead_m
        near = (sample is not None and sample.obstacle_ahead_m is not None
                and sample.obstacle_ahead_m <= self.explorer.standoff())
        return self._choose(self.map[1], now, pose, forward_blocked=near, allow_visited=allow_visited)

    def _backtrack(self, now, pose):
        """Dead end: walk back along the breadcrumb trail (known reachable) and avoid this spot."""
        if self.backtracks >= MAX_BACKTRACKS:
            return False
        while self.trail and math.hypot(self.trail[-1][0] - pose[0], self.trail[-1][1] - pose[1]) < BACKTRACK_MIN_M:
            self.trail.pop()
        if not self.trail:
            return False
        x, y = self.trail.pop()
        self.explorer.block(pose[0], pose[1], now, FAILED_WALK_MEMORY_S)
        self.backtracks += 1
        self.stats['backtracks'] = self.stats.get('backtracks', 0) + 1
        self.event('backtrack', to=[_r(x, 2), _r(y, 2)], consecutive=self.backtracks)
        self.scan_index = 0
        self._start_walk(now, pose, {'kind': 'walk', 'x': x, 'y': y, 'yaw': math.atan2(y - pose[1], x - pose[0]),
                                     'score': 0.0, 'free_m': None, 'ends_at': 'backtrack'})
        return True

    def _take_mission(self, pose):
        """Mission events (live recorder): log each new one once with the robot's map pose. Log only:
        operator 2026-11-23 "just log it" - roam's movement does not change."""
        inbox = getattr(self.robot, 'mission', None)
        if not inbox:
            return
        items, inbox[:] = list(inbox), []
        for _, text in items:
            ev = parse_mission_event(text)
            if ev is None or ev['event_id'] in self.mission_seen:
                continue
            self.mission_seen.add(ev['event_id'])
            sighting = {'kind': 'mission_' + ev['type'], 'value': ev['value'],
                        'confidence': _r(ev['confidence'], 2) if ev['confidence'] is not None else None,
                        'at': [_r(v, 2) for v in ev['location']] if ev['location'] else None,
                        'x': _r(pose[0], 2) if pose else None, 'y': _r(pose[1], 2) if pose else None,
                        'event_id': ev['event_id']}
            self.sightings.append(sighting)
            self.stats['mission_events'] = self.stats.get('mission_events', 0) + 1
            self.event('mission', **sighting)

    def _take_scene(self, now, pose):
        """Hook: detections from the teammate's scene node, tagged with the robot's map pose."""
        self._take_mission(pose)
        inbox = getattr(self.robot, 'scene', None)
        if not inbox:
            return
        items, inbox[:] = list(inbox), []
        for _, text in items:
            for kind, value, confidence in parse_scene(text):
                if now - self.scene_seen.get((kind, value), -math.inf) < SCENE_REPEAT_S:
                    continue
                self.scene_seen[(kind, value)] = now
                sighting = {'kind': kind, 'value': value, 'confidence': _r(confidence, 2),
                            'x': _r(pose[0], 2) if pose else None, 'y': _r(pose[1], 2) if pose else None}
                self.sightings.append(sighting)
                self.event('scene', **sighting)

    def _check_exit(self, now, pose):
        if not self.cfg.exit_open_radius_m or now < self.exit_checked + EXIT_CHECK_PERIOD_S or self.map is None:
            return
        self.exit_checked = now
        found = detect_exit(self.map[1], pose, self.anchor, self.cfg.exit_open_radius_m, self.cfg.exit_min_area_m2,
                            self.cfg.exit_min_from_start_m)
        if found is None:
            return
        if self.exit is None:
            self.event('exit_detected', **found)
        was_reached = bool(self.exit and self.exit.get('reached'))
        self.exit = found
        if found['reached']:
            if not was_reached:                          # once per arrival (was logged every check)
                self.event('exit_reached', **found)
            if self.cfg.stop_at_exit:
                self.reason = f'exit reached: open area {found["area_m2"]} m2 outside the maze'
                self.state = 'STOPPING'

    def _clear_costmaps(self, now, why):
        """Nav2 ClearEntireCostmap (its default recovery). Course 2026-11-21: the terrain hazard source marks
        without clearing, so after turning on a lip 842 cells of SLAM-free floor within 1.6 m were lethal
        and every direction looked blocked ("no open walking corridor" in open space)."""
        if not hasattr(self.robot, 'clear_costmaps') or now - self.last_clear < COSTMAP_CLEAR_COOLDOWN_S:
            return False
        self.last_clear = now
        done = self.robot.clear_costmaps()
        self.stats['costmap_clears'] = self.stats.get('costmap_clears', 0) + 1
        self.event('costmap_cleared', why=why, services=len(done or []))
        return bool(done)

    def _frontier_goal(self, now, pose):
        """Nearest worthwhile frontier (lite3_locomotion.frontiers) as a Nav2 walk goal; None if none."""
        if self.frontiers is None or self.map is None:
            return None
        g = self.frontiers.choose(self.map[1], pose, now, self._costmap())   # frontier, else patrol
        if g is None or self.frontiers.avoided_goal(g, now):
            return None
        # Operator 2026-11-21: "make it walk towards that white spot": known free space not visited yet
        # (patrol) when no frontier is left; Nav2 plans the way there round corners and through gaps.
        return {'kind': 'walk', 'x': g['x'], 'y': g['y'], 'yaw': g['yaw'], 'score': g['score'],
                'free_m': None, 'ends_at': g['kind']}

    def _find_passage(self, now, pose):
        """Operator 2026-11-21: at a dead end, stop in front of the curtain (a wall the LiDAR sees, with the
        course going on unseen behind it) instead of backtracking. Walk to the standoff, face it, pause."""
        if self.map is None:
            return False
        p = blocked_passage(self.map[1], pose)
        if p is None:
            return False
        px = pose[0] + p['wall_m'] * math.cos(p['yaw'])
        py = pose[1] + p['wall_m'] * math.sin(p['yaw'])
        if any(math.hypot(px - x, py - y) < 0.5 for x, y in self.passages_done):
            return False
        self.passage = dict(p, x=px, y=py, turned=False)
        self.event('blocked_passage', heading_deg=_r(math.degrees(p['yaw']), 0), wall_m=p['wall_m'],
                   unknown_behind=p['unknown'], width_deg=p['width_deg'])
        walk = p['wall_m'] - self.explorer.standoff()
        if walk >= 0.15:
            self._start_walk(now, pose, {'kind': 'walk', 'x': pose[0] + walk * math.cos(p['yaw']),
                                         'y': pose[1] + walk * math.sin(p['yaw']), 'yaw': p['yaw'],
                                         'score': 0.0, 'free_m': p['wall_m'], 'ends_at': 'passage'})
        return True

    def _passage_tick(self, now, pose):
        """Approach done (or not needed): turn to face the passage once, then pause in front of it."""
        p = self.passage
        err = math.atan2(math.sin(p['yaw'] - pose[2]), math.cos(p['yaw'] - pose[2]))
        if abs(err) > math.radians(10) and not p['turned']:
            p['turned'] = True
            self._start_spin(now, pose, err)
            return
        if self.cfg.advisory == 'act' and 'wait_until' not in p:
            p['wait_until'] = now + PASSAGE_ADVISORY_WAIT_S   # a curtain confirmation here crosses it instead
            self.event('passage_waiting', for_s=PASSAGE_ADVISORY_WAIT_S)
            return
        if now < p.get('wait_until', 0.0):
            return
        ahead = math.hypot(p['x'] - pose[0], p['y'] - pose[1])
        self.passages_done.append((p['x'], p['y']))
        self.passage = None
        self.stats['passage_stops'] = self.stats.get('passage_stops', 0) + 1
        self._pause(f'stopped in front of a blocked passage (possible curtain) {ahead:.2f} m ahead; '
                    'resume to explore on')

    def _plan_look(self, pose):
        """After a walk: turn to look at the sides that still hold unseen mapped floor (camera coverage).
        Operator 2026-11-23: look-arounds only where there is something left to see."""
        self.look_check = False
        if self.coverage is None or self.map is None or \
                any(math.hypot(pose[0] - x, pose[1] - y) < LOOK_SPACING_M for x, y in self.look_spots):
            return False
        grid, half = self.map[1], math.radians(LOOK_SIDE_HALF_DEG)
        left = self.coverage.unseen_free_m2(grid, pose[:2], LOOK_RADIUS_M, pose[2] + math.pi / 2, half)
        right = self.coverage.unseen_free_m2(grid, pose[:2], LOOK_RADIUS_M, pose[2] - math.pi / 2, half)
        turns = []
        if left >= LOOK_MIN_SIDE_M2:
            turns.append(90.0)
        if right >= LOOK_MIN_SIDE_M2:
            turns.append(-180.0 if turns else -90.0)
        if not turns:
            return False
        self.look_spots.append((pose[0], pose[1]))
        self.look = [math.radians(a) for a in turns]
        self.stats['look_arounds'] = self.stats.get('look_arounds', 0) + 1
        self.event('look_around', left_unseen_m2=_r(left, 1), right_unseen_m2=_r(right, 1), turns_deg=turns)
        return True

    def _search_turn(self, now, pose):
        if self.scan_index >= len(SCAN_STEPS):
            if self.passage is None and self._find_passage(now, pose):
                return
            if self._clear_costmaps(now, 'no open corridor after checking both sides'):
                self.choose_after = now + COSTMAP_REFILL_S   # choose again on the cleared costmap; no new
                return                                        # spin round (turning on a lip slips back)
            if not self._backtrack(now, pose):
                self._pause('no open walking corridor after checking both sides')
            return
        if self.scan_index == 0:
            costmap = self._costmap()
            left, right = (self.explorer.ray(self.map[1], costmap, pose, pose[2] + sign * SCAN_STEP_RAD)[0]
                           for sign in (1, -1))
            self.scan_sign = 1 if left >= right else -1
        angle = self.scan_sign * SCAN_STEPS[self.scan_index] * SCAN_STEP_RAD
        self.scan_index += 1
        self._start_spin(now, pose, angle)

    def _start_walk(self, now, pose, goal):
        if not self.robot.nav_send_goal(goal['x'], goal['y'], goal['yaw']):
            self._walk_failed(now, 'Nav2 rejected the walk goal')
            return
        self.robot.set_speed_limit(self.cfg.mode_speed(self.speed_mode)[0])   # after the MPPI reset
        self.explorer.sent(goal)
        self.goal = goal
        self.goal_sent_at = now
        d = math.hypot(goal['x'] - pose[0], goal['y'] - pose[1])
        self.goal_timeout = min(HARD_MAX_GOAL_S, d / self.cfg.mode_speed(self.speed_mode)[0] * 3 + 30)
        self.progress = (now, pose[0], pose[1])
        self.scan_index = 0
        self.walk_start = (pose[0], pose[1])
        if goal.get('ends_at') != 'backtrack':
            self.backtracks = 0
            if not self.trail or math.hypot(pose[0] - self.trail[-1][0], pose[1] - self.trail[-1][1]) >= TRAIL_SPACING_M:
                self.trail = (self.trail + [(pose[0], pose[1])])[-50:]
        self.stats['goals_sent'] += 1
        self.stats['walk_goals'] += 1
        if goal.get('ends_at') in ('frontier', 'patrol'):
            self.stats[goal['ends_at'] + '_goals'] += 1
            self.frontiers.sent(goal)
        self.event('walk', heading_deg=_r(math.degrees(goal['yaw']), 0), distance_m=_r(d, 2),
                   open_m=goal['free_m'], ends_at=goal['ends_at'], x=_r(goal['x'], 2), y=_r(goal['y'], 2))

    def _end_walk(self, now, pose, how):
        walked = math.hypot(pose[0] - self.walk_start[0], pose[1] - self.walk_start[1])
        self.look_check = True                           # new place: maybe look around (_plan_look)
        goal, self.goal = self.goal, None
        self.choose_after = now + MAP_SETTLE_S
        if goal and goal.get('ends_at') in ('frontier', 'patrol') and self.frontiers is not None:
            self.frontiers.block(goal['x'], goal['y'], now, 60)
        if how == 'reached' or walked >= WALK_PROGRESS_M:
            self.failures = 0
            self.stats['goals_succeeded'] += 1
            if how != 'reached':
                self.stats['walks_to_obstacle'] += 1
            self.event('walk_end', how=how, walked_m=_r(walked, 2))
        else:
            self._walk_failed(now, f'walk ended after {walked:.2f} m ({how})', goal)

    def _walk_failed(self, now, why, goal=None):
        # goal: _end_walk clears self.goal first. Course 2026-11-21 12:25: that lost the failed spot, so
        # roam re-picked the same terrain-blocked heading after every search round (3 x, 0.02-0.06 m each).
        failed = goal or self.goal
        if failed and failed.get('kind') == 'walk' and hasattr(self.explorer, 'avoided'):
            self.explorer.block(failed['x'], failed['y'], now, FAILED_WALK_MEMORY_S)
        if failed and failed.get('ends_at') in ('frontier', 'patrol') and self.frontiers is not None:
            self.frontiers.block(failed['x'], failed['y'], now, 90)
        self.stats['goals_failed'] += 1
        self.failures += 1
        self.goal = None
        self.choose_after = now + MAP_SETTLE_S
        sample = self._terrain_now(now)
        terrain = {} if sample is None else {'obstacle_ahead_m': sample.obstacle_ahead_m,
                                             'step_ahead_m': getattr(sample, 'step_ahead_m', None),
                                             'step_rise_m': getattr(sample, 'step_rise_m', None)}
        self.event('walk_failed', reason=why, consecutive=self.failures, **terrain)
        if self.failures >= self.cfg.max_goal_failures:
            sample = self._terrain_now(now)
            held = f' (terrain proximity stop holding: {sample.reason}; move the robot or the object)' \
                if sample is not None and sample.stop else ''
            self._pause(f'{self.failures} consecutive spins/walks found no way on{held}: check the area, '
                        'then resume')

    def _scan_walk(self, now, pose):
        """Walk on the first usable map; search both sides only while a corridor is missing."""
        if self.coverage is not None and self.map is not None and now - self._coverage_t >= COVERAGE_PERIOD_S:
            self._coverage_t = now
            self.coverage.update(pose, self.map[1])
            self.stats['camera_seen_m2'] = _r(self.coverage.seen_m2(), 1)
        if self.frontiers is not None and self.frontiers.visited is not None and \
                math.hypot(pose[0] - self.frontiers.visited[-1][0], pose[1] - self.frontiers.visited[-1][1]) >= 0.3:
            self.frontiers.visited.append((pose[0], pose[1]))
        sample = self._terrain_now(now)
        if sample is None:
            self._pause('LiDAR/terrain status stale; check the sensors before resuming')
            return
        if sample.stop:
            # Operator 2026-11-20: a proximity stop is a HOLD, not a pause. The twist_mux lock already
            # stops all Nav2 output; cancel the walk, wait for the area to clear, then carry on by itself.
            if self.hold_since is None:
                self.hold_since = now
                self.stats['holds'] = self.stats.get('holds', 0) + 1
                self.event('hold', reason=sample.reason)
            elif now - self.hold_since > TERRAIN_HOLD_MAX_S:
                self.hold_since = None
                self._pause(f'terrain proximity stop for {TERRAIN_HOLD_MAX_S:.0f} s ({sample.reason}); '
                            'check the area before resuming')
                return
            if self.goal is None:
                return
            if self.cancel_request is None:
                self._cancel_scan_walk(now, TERRAIN_HOLD_REASON)
                return
        elif self.hold_since is not None:
            self.event('hold_cleared', held_s=_r(now - self.hold_since, 1))
            self.hold_since = None
        self._take_scene(now, pose)
        self._check_exit(now, pose)
        if self.state == 'ROAMING' and self._advisory(now, pose):
            return
        if self.state != 'ROAMING' or self._auto_stairs(now, sample):
            return
        if self.goal is None:
            if self.adv_task is not None:
                if now >= self.choose_after:
                    self._advisory_tick(now, pose)
                return
            if self.passage is not None:
                if now >= self.choose_after:
                    self._passage_tick(now, pose)
                return
            if now >= self.choose_after and (self.look or (self.look_check and self._plan_look(pose))):
                self._start_spin(now, pose, self.look.pop(0))
                if self.goal is not None:
                    self.goal['look'] = True
                return
            if now >= self.choose_after and self.map is not None:
                goal = None
                if self.cfg.stop_at_exit and self.exit and not self.exit['reached'] and math.hypot(self.exit['x'] - pose[0],
                                                                          self.exit['y'] - pose[1]) >= 0.3 \
                        and not (hasattr(self.explorer, 'avoided')
                                 and self.explorer.avoided(self.exit['x'], self.exit['y'], now)):
                    goal = {'kind': 'walk', 'x': self.exit['x'], 'y': self.exit['y'],
                            'yaw': math.atan2(self.exit['y'] - pose[1], self.exit['x'] - pose[0]),
                            'score': 0.0, 'free_m': None, 'ends_at': 'exit'}
                if sample.stop_ahead_m and not self.cfg.docs_stops:
                    self.explorer.stop_ahead_m = sample.stop_ahead_m
                near = sample.obstacle_ahead_m is not None and sample.obstacle_ahead_m <= self.explorer.standoff()
                if goal is None and hasattr(self.explorer, 'straight_ahead') and not near:   # held heading
                    goal = self.explorer.straight_ahead(self.map[1], pose, self._costmap(), now)
                if goal is None:
                    goal = self._walk_candidate(now, pose)        # blocked: the smallest turn that opens
                if goal is None:
                    goal = self._frontier_goal(now, pose)         # Nav2 path to a frontier, before spinning
                if goal is None:
                    goal = self._walk_candidate(now, pose, allow_visited=True)   # nothing new: walk on anyway
                if goal is None:
                    self._search_turn(now, pose)
                else:
                    self._start_walk(now, pose, goal)
            return
        status = self.robot.nav_status()
        result = status.get('result')
        if self.cancel_request:
            requested_at, reason = self.cancel_request
            if result:
                self.cancel_request = None
                if self.goal['kind'] == 'spin':
                    self._end_spin(now, reason, status.get('spun_rad'))
                elif reason in ('obstacle ahead (walking standoff)', TERRAIN_HOLD_REASON):
                    self._end_walk(now, pose, reason)
                else:
                    self._walk_failed(now, reason)
            elif now - requested_at > 3.0:
                self._pause('Nav2 cancellation has no terminal result')
            return
        if self.goal['kind'] == 'spin':
            spun = status.get('spun_rad') or 0.0
            if not result and self.passage is None and self.adv_task is None and not self.goal.get('look') \
                    and self.map is not None and self.map[0] != self.scan_map_time:
                self.scan_map_time = self.map[0]
                if self._walk_candidate(now, pose) is not None:
                    self._cancel_scan_walk(now, 'opening')
                    return
            if spun > self.spin_progress[1] + 0.1:
                self.spin_progress = (now, spun)
            if result == 'SUCCEEDED':
                self._end_spin(now, 'done', spun)
            elif result:
                self._end_spin(now, 'incomplete', spun)       # partial scans still help: walk anyway
            elif now - self.spin_progress[0] > SPIN_STALL_S:
                self.event('spin_stalled', turned_deg=_r(math.degrees(spun), 0))
                self._pause('scan made no angular progress')
            return
        # End an approach while there is still room to replan before the proximity lock.
        heading_error = math.atan2(math.sin(self.goal['yaw'] - pose[2]), math.cos(self.goal['yaw'] - pose[2]))
        approaching = (self.goal.get('ends_at') not in ('frontier', 'patrol', 'stairs')   # Nav2 plans those
                       and abs(heading_error) <= SCAN_STEP_RAD and sample.obstacle_ahead_m is not None
                       and sample.obstacle_ahead_m <= self.explorer.standoff())
        if math.hypot(pose[0] - self.progress[1], pose[1] - self.progress[2]) >= 0.10:
            self.progress = (now, pose[0], pose[1])
        if result == 'SUCCEEDED':
            self._end_walk(now, pose, 'reached')
        elif result:
            self._end_walk(now, pose, f'Nav2 {result}')
        elif approaching:
            self._cancel_scan_walk(now, 'obstacle ahead (walking standoff)')
        elif now - self.goal_sent_at > self.goal_timeout:
            self._cancel_scan_walk(now, 'timeout')
        elif now - self.progress[0] > self.cfg.stuck_s:
            self._cancel_scan_walk(now, f'no progress for {self.cfg.stuck_s:.0f} s')

    def _cancel_scan_walk(self, now, reason):
        if self.robot.nav_cancel():
            self.cancel_request = (now, reason)
        else:
            self._pause(f'Nav2 did not acknowledge cancellation: {reason}')

    def _terrain_max_age(self):
        # docs-only stops: terrain is a costmap input only (Nav2 tolerates a late update), so a single
        # link hiccup up to ROAM_RAW_LIDAR_STALE_S no longer ends the session (two stops 2026-11-20).
        return ROAM_RAW_LIDAR_STALE_S if self.cfg.docs_stops else self.cfg.max_sensor_age_s

    def _terrain_now(self, now):
        item = self.robot.telemetry.terrain.latest()
        return None if item is None or now - item[0] > max(0.5, self._terrain_max_age()) else item[1]

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
        if self.cfg.strategy == 'scan_walk':
            return self._scan_walk(now, pose)
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
    def _leave_stairs(self, why):
        """Any pause/stop/safety stop during a crossing: never leave the robot in AI state (2026-11-21 it was
        left in AI mode after a safety stop). switch_out = axis 0, exit AI, navigation mode, normal gait."""
        if not self.stairs:
            return
        routine, iface, _ = self.stairs
        self.stairs = None
        try:
            if getattr(iface, 'active', True):
                self.event('stairs_switch_out', evidence=iface.switch_out(), reason=why)
            iface.restore_stops()
        except Exception as error:  # noqa: BLE001 - keep stopping; tell the operator
            self.event('stairs_switch_out_failed', error=str(error))
            self.log(f'WARNING: could not leave the stair mode ({error}); use the remote / app to exit AI mode')

    def _pause(self, why):
        self._leave_stairs(f'pause: {why}')
        self.crossing, self.adv_task, self.look = None, None, []
        if self.state != 'ROAMING':
            return
        # Keep the heartbeat alive through the pause: cancel + 1.5 s of zeros used to block the loop for
        # 2+ s and the independent watchdog ended the session (physical 2026-11-20 10:11, "runner
        # heartbeat lost (2.0 s)" 1.6 s after an ordinary pause). The zeros on the stop input hold the
        # robot whatever Nav2 does, so the cancel acknowledgement is awaited only briefly.
        self._beat()
        if self.robot.can_actuate:
            self.robot.nav_cancel(timeout_s=0.5)
        self.runner.nav_active_until = self.robot.now() + 1.5
        self.state = 'PAUSED'
        self.stats['pauses'] += 1
        self.reason = why
        self.event('paused', reason=why)
        self._beat()
        if self.robot.can_actuate:
            for _ in range(3):
                self.runner._zero_burst(0.5)
                self._beat()

    def _beat(self):
        """Write the status heartbeat now (for steps that block the main loop)."""
        try:
            self.channel.write_status(self.status(self.robot.now(), self.last_pose))
        except Exception:  # noqa: BLE001 - a heartbeat must never break a stop
            pass

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
        self.scan_index = 0
        self.cancel_request = None
        self.choose_after = now
        self.state = 'ROAMING'
        self.reason = None
        self.runner.nav_active_until = math.inf
        self.event('resumed', speed_mode=self.speed_mode)
        return None

    def _switch_gait(self, name):
        """Send one documented gait switch (Comm Interface s1.2.5) while the robot stands still. The
        deployed bridge does not report the active gait, so this is delivery evidence only: confirm by eye."""
        r = self.robot
        if not r.can_actuate or not hasattr(r, 'send_gait'):
            self.event('gait', gait=name, sent=False, reason='dry run: not sent')
            return None
        self._beat()
        try:
            evidence = r.send_gait(name)
        except Exception as exc:      # a missing gait is not a safety problem: keep roaming, say so
            evidence = {'published': False, 'reason': str(exc)}
        self.event('gait', gait=name, code=hex(GAITS[name]), **{k: v for k, v in evidence.items() if k != 'cmd_code'})
        if not evidence.get('published'):
            self.log(f'WARNING: gait {name} NOT sent ({evidence.get("reason")}); the robot keeps its current gait')
            return f'gait {name} not sent: {evidence.get("reason")}'
        for _ in range(GAIT_SETTLE_BURSTS):       # hold zeros while the gait transition finishes
            self.runner._zero_burst(0.5)
            self._beat()
        self.gait = name
        return None

    # -------------------------------------------------------------- stair / curtain advisory
    def _cancel_goal_now(self):
        if self.goal is not None:
            self.robot.nav_cancel(timeout_s=0.5)
            self.goal, self.cancel_request = None, None

    def _advisory(self, now, pose):
        """Teammate's stair/curtain detector (advisory.py). Logs confirmed sightings; with advisory 'act'
        it turns a confirmed stairs into the AI-mode crossing and a near curtain into a slow straight
        crossing. Returns True when it took over this tick."""
        item = getattr(self.robot, 'advisory', None)
        if item is None or self.cfg.advisory == 'off' or now - item[0] > ADVISORY_MAX_AGE_S:
            return False
        adv = parse_advisory(item[1])
        key = None if adv is None else (adv['obstacle'], round(adv['distance_m'] * 4) / 4)
        if adv is not None and key != self.adv_last:
            self.event('advisory', obstacle=adv['obstacle'], distance_m=_r(adv['distance_m'], 2),
                       heading_deg=_r(adv['heading_deg'], 1))
        self.adv_last = key
        if adv is None or self.cfg.advisory != 'act' or self.stairs or self.crossing:
            return False
        if adv['obstacle'] == 'stairs':
            return self._advisory_stairs(now, pose, adv)
        return self._advisory_curtain(now, pose, adv)

    def _advisory_stairs(self, now, pose, adv):
        """Far: walk to ADVISORY_STAIRS_WALK_TO_M before the first riser. Near: face the risers squarely
        (detector heading correction), then the stair routine (approach -> AI mode -> climb -> out)."""
        if now < self.stairs_until:
            return False
        d, h = adv['distance_m'], adv['heading_deg']
        if d > ADVISORY_STAIRS_TRIGGER_M:
            if self.adv_task is not None or (self.goal and self.goal.get('ends_at') == 'stairs'):
                return False
            walk = d - ADVISORY_STAIRS_WALK_TO_M
            if walk < 0.15:
                return False
            yaw = pose[2] + math.radians(h)
            self._cancel_goal_now()
            self.event('stairs_ahead', distance_m=_r(d, 2), heading_deg=_r(h, 1))
            self._start_walk(now, pose, {'kind': 'walk', 'x': pose[0] + walk * math.cos(yaw),
                                         'y': pose[1] + walk * math.sin(yaw), 'yaw': yaw, 'score': 0.0,
                                         'free_m': None, 'ends_at': 'stairs'})
            return True
        if self.adv_task is not None:
            return False                                   # turning to face them already
        self._cancel_goal_now()
        if abs(h) > ADVISORY_ALIGN_DEG:
            self.adv_task = {'kind': 'stairs_align'}
            self.event('stairs_align', heading_deg=_r(h, 1), distance_m=_r(d, 2))
            self._start_spin(now, pose, math.radians(h))
            return True
        self.event('stairs_detected', ahead_m=_r(d, 2), source='advisory')
        self._begin_stairs('up', 'approach', 'advisory', over=True)
        return True

    def _advisory_tick(self, now, pose):
        """A step of an advisory task finished (Nav2 goal None): go on with the next."""
        task, self.adv_task = self.adv_task, None
        if task['kind'] == 'stairs_align':
            self.event('stairs_detected', source='advisory, aligned')
            self._begin_stairs('up', 'approach', 'advisory', over=True)

    def _advisory_curtain(self, now, pose, adv):
        """Curtain within CURTAIN_TRIGGER_M of base_link: walk straight through it, outside Nav2 (the SLAM map
        and costmaps hold the curtain as a wall)."""
        if now < self.curtain_until:
            return False
        ahead = adv['distance_m'] + (CURTAIN_CAMERA_AHEAD_M if adv['reference'] == 'rgb_camera_optical_depth'
                                     else 0.0)
        if ahead > CURTAIN_TRIGGER_M:
            return False
        odom = self.robot.telemetry.odom.latest()
        if odom is None:
            return False
        self._cancel_goal_now()
        if self.passage is not None:
            self.passages_done.append((self.passage['x'], self.passage['y']))
            self.passage = None
        self.adv_task = None
        self.crossing = {'origin': (odom[1].x, odom[1].y), 't': now, 'length': ahead + CURTAIN_PAST_M,
                         'progress': (now, 0.0)}
        self.event('curtain_crossing', curtain_m=_r(ahead, 2), length_m=_r(self.crossing['length'], 2))
        return True

    def _crossing_tick(self, now):
        c = self.crossing
        odom = self.robot.telemetry.odom.latest()
        if odom is None:
            return self._end_crossing(now, 'no odometry')
        travelled = math.hypot(odom[1].x - c['origin'][0], odom[1].y - c['origin'][1])
        tilt = self.robot.telemetry.current_tilt()
        if tilt is not None and max(abs(tilt[0]), abs(tilt[1])) > CURTAIN_MAX_TILT_RAD:
            return self._end_crossing(now, f'tilt {max(abs(tilt[0]), abs(tilt[1])):.2f} rad', travelled)
        if travelled >= c['length']:
            return self._end_crossing(now, None, travelled)
        speed = min(CURTAIN_SPEED_M_S, self.cfg.mode_speed(self.speed_mode)[0])
        allowed = max(CURTAIN_MAX_S, 2.0 * c['length'] / speed)        # slow mode needs longer
        if now - c['t'] > allowed:
            return self._end_crossing(now, f'not through after {allowed:.0f} s', travelled)
        if travelled > c['progress'][1] + CURTAIN_STALL_M:
            c['progress'] = (now, travelled)
        elif now - c['progress'][0] > CURTAIN_STALL_S:
            return self._end_crossing(now, 'no progress: blocked, maybe not a curtain', travelled)
        self.robot.send_velocity(speed, 0.0)

    def _end_crossing(self, now, problem, travelled=None):
        self.robot.send_velocity(0.0, 0.0)
        self.crossing = None
        self.curtain_until = now + CURTAIN_COOLDOWN_S
        self.goal = None
        self.choose_after = now + MAP_SETTLE_S
        if problem:
            self.event('curtain_crossing_failed', reason=problem, travelled_m=_r(travelled, 2))
            self._pause(f'curtain crossing stopped: {problem}')
            return
        self.stats['curtain_crossings'] = self.stats.get('curtain_crossings', 0) + 1
        self.event('curtain_crossed', travelled_m=_r(travelled, 2))
        self.last_clear = -math.inf                      # the curtain is in the costmaps as a wall
        self._clear_costmaps(now, 'curtain crossed')

    def _start_stairs(self, direction):
        """Operator-started stair crossing (stairs.py), from PAUSED with the robot squarely facing the stairs."""
        if self.state != 'PAUSED':
            return f'stairs only from PAUSED (face the stairs, pause, then stairs up|down); now {self.state}'
        if direction not in ('up', 'down'):
            return "stairs needs 'up' or 'down'"
        if not self.robot.can_actuate or not hasattr(self.robot, 'send_gait'):
            return 'stairs need the physical robot'
        problem = self._resume()
        if problem:
            return problem
        self._begin_stairs(direction, 'approach', 'operator')
        return None

    def _begin_stairs(self, direction, start, why, over=False):
        normal = self.cfg.gait or FLAT_GAIT
        iface = AiModeStairs(self.robot, self.cfg.ai_axis_forward, normal) if self.cfg.stair_mode == 'ai' \
            else HStepGaitStairs(self.robot, self.cfg.rough_gait, normal)
        iface.relax_stops()
        odom = self.robot.telemetry.odom.latest()
        origin = (odom[1].x, odom[1].y) if odom else (0.0, 0.0)
        routine = StairRoutine(direction, StairConfig(switch_settle_s=iface.settle_s), start=start, over=over)
        self.stairs = (routine, iface, origin)
        self.stairs_began = self.robot.now()
        self.event('stairs_start', direction=direction, mode=self.cfg.stair_mode, trigger=why, over=over)

    def _auto_stairs(self, now, sample):
        """Terrain sees a step (up or down) of stair height right ahead: cross it without the operator."""
        if not (self.cfg.auto_stairs and self.robot.can_actuate and hasattr(self.robot, 'send_gait')):
            return False
        if sample.step_ahead_m is None or sample.step_rise_m is None or now < self.stairs_until:
            return False
        if sample.step_ahead_m > STAIR_TRIGGER_M or not STAIR_MIN_RISE_M <= abs(sample.step_rise_m) <= STAIR_MAX_RISE_M:
            return False
        if self.goal is not None:
            self.robot.nav_cancel(timeout_s=0.5)
            self.goal, self.cancel_request = None, None
        self.event('stairs_detected', ahead_m=_r(sample.step_ahead_m, 2), rise_m=_r(sample.step_rise_m, 3))
        self._begin_stairs('up' if sample.step_rise_m > 0 else 'down', 'switch_in', 'terrain')
        return True

    def _stairs_tick(self, now):
        routine, iface, start = self.stairs
        tilt = self.robot.telemetry.current_tilt()
        odom = self.robot.telemetry.odom.latest()
        travelled = math.hypot(odom[1].x - start[0], odom[1].y - start[1]) if odom else 0.0
        action = routine.step(now, tilt[1] if tilt else None, tilt[0] if tilt else None, travelled)
        if action.switch == 'out' and not action.aborted and now - self.stairs_began < STAIR_MAX_AI_S:
            sample = self._terrain_now(now)          # uneven course: more lips ahead -> keep climbing in AI mode
            if sample is not None and sample.step_ahead_m is not None and sample.step_rise_m is not None and \
                    sample.step_ahead_m <= STAIR_CONTINUE_M and \
                    STAIR_MIN_RISE_M <= abs(sample.step_rise_m) <= STAIR_MAX_RISE_M:
                routine.state, routine.t_state, routine.d_state = 'climb', now, travelled
                routine.inclined, routine.level_since = False, None
                self.event('stairs_continue', ahead_m=_r(sample.step_ahead_m, 2), rise_m=_r(sample.step_rise_m, 3))
                action = Action(vx=routine.cfg.climb_speed_m_s)
        if routine.state != getattr(self, '_stair_state', None):
            self._stair_state = routine.state
            if routine.state in ('landing', 'descend'):
                self.event('stairs_' + routine.state, travelled_m=_r(travelled, 2))
        if action.switch == 'in':
            self.event('stairs_switch_in', evidence=iface.switch_in(routine.direction))
        elif action.switch == 'out':
            self.event('stairs_switch_out', evidence=iface.switch_out())
        if getattr(iface, 'active', False):
            self.runner.own_axis_until = now + OWN_AXIS_GRACE_S     # our axis commands echo on /handle_state
        iface.drive(min(action.vx, self.cfg.mode_speed(self.speed_mode)[0]))   # never above the mode
        if action.done or action.aborted:
            iface.restore_stops()
            self.stairs = None
            self.stairs_until = now + STAIR_COOLDOWN_S
            self.goal = None
            self.choose_after = now + MAP_SETTLE_S
            if action.aborted:
                self.event('stairs_aborted', reason=action.reason)
                self._pause(f'stairs aborted: {action.reason}')
            else:
                self.event('stairs_done', direction=routine.direction, travelled_m=_r(travelled, 2))

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
        elif cmd == 'gait':
            if self.state != 'PAUSED':
                result = f'gait changes only while paused (robot still); now {self.state}'
            elif arg not in GAITS:
                result = f'unknown gait {arg!r} (gaits: {", ".join(GAITS)})'
            else:
                result = self._switch_gait(arg)
        elif cmd == 'stairs':
            result = self._start_stairs(arg)
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
            'speed_limit_m_s': self.cfg.speed_limit, 'modes': self.cfg.modes, 'gait': self.cfg.gait,
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
        self.explorer = (ScanWalk if self.cfg.strategy == 'scan_walk' else Explorer)(self.cfg, self.anchor)
        # Operator 2026-11-21 "engage frontier navigation as well": scan-walk falls back to Nav2 frontier
        # goals (planned path around corners) before turning in place to search.
        self.frontiers = Explorer(self.cfg, self.anchor) if self.cfg.strategy == 'scan_walk' else None
        if self.frontiers is not None:
            # Judged as Nav2 does: no lethal/inscribed cost (>= 99) within the body half-width. With the
            # 0.40 m box at cost 70 every frontier in a ~1.5 m course sat in wall inflation (12:37 run).
            self.frontiers.clearance_m, self.frontiers.veto = FRONTIER_FALLBACK_CLEARANCE_M, FRONTIER_FALLBACK_VETO
            self.frontiers.visited = [self.anchor]
            self.explorer.visited = self.frontiers.visited      # one walked path for coverage and patrol
            self.coverage = CameraCoverage(self.anchor, self.cfg.geofence_m + 1.0)
            self.frontiers.seen = self.coverage
        self.scan_index = 0
        self.scan_sign = 1
        self.choose_after = -math.inf
        self.last_clear = -math.inf
        if r.can_actuate and self._clear_costmaps(r.now(), 'start'):
            self.choose_after = r.now() + COSTMAP_REFILL_S
        if self.cfg.strategy == 'scan_walk' and hasattr(r, 'collision_boundary'):
            boundary = r.collision_boundary()                  # running polygon, nose distance
            self.explorer.cm_front_m = None if boundary is None or self.cfg.docs_stops else boundary + 0.305
            approach = r.collision_approach_time() if hasattr(r, 'collision_approach_time') else None
            self.event('scan_walk', collision_polygon_front_m=_r(self.explorer.cm_front_m, 2),
                       approach_time_s=approach, spin_radius_m=_r(self.explorer.spin_radius(), 2))
            if self.explorer.spin_radius() > LARGE_SPIN_RADIUS_M:
                # Physical 2026-11-18: navigation ran with a 1.19 m polygon (high --max-speed stack), so
                # every goal needed ~3 m of open floor around it and nothing in the room qualified.
                self.log(f'WARNING: the running navigation stack stops all motion within '
                         f'{self.explorer.stop_zone():.2f} m ahead, so every walk goal needs an open circle '
                         f'about {2 * self.explorer.spin_radius():.1f} m across. In a cluttered area roaming '
                         f'will find no way on and pause. For roaming restart navigation at 0.10 m/s: '
                         f'operator/stop_navigation.sh, then operator/roam.sh start --physical.')
        self.runner.geofence = (self.anchor[0], self.anchor[1], self.cfg.geofence_m)
        self.runner.origin = None
        self.t_roam = r.now()
        self.last_pose = self.counted_pose = pose
        for option in self.cfg.ai_options:      # Comm Interface s1.2.11, off unless asked for
            if r.can_actuate and hasattr(r, 'send_ai_option'):
                self.event('ai_option', option=option, **r.send_ai_option(option))
        if self.cfg.heartbeat and r.can_actuate and hasattr(r, 'start_heartbeat'):
            self.event('heartbeat', started=r.start_heartbeat())
        if self.cfg.gait:
            self._switch_gait(self.cfg.gait)
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
                        if self.stairs:
                            self._stairs_tick(now)
                        elif self.crossing:
                            self._crossing_tick(now)
                        else:
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
        # ---- stop: leave any stair mode first (AI state), then tell the watchdog
        self._leave_stairs(f'session ending: {self.reason}')
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
        if self.goal_sent_at is None or now - self.goal_sent_at > 5.0:
            grid = self.map
            if grid is not None and pose is not None:
                if self.cfg.strategy == 'scan_walk':
                    sample = self._terrain_now(now)
                    goal = self._walk_candidate(now, pose) if sample and not sample.stop else None
                    if goal is None:
                        self.event('would_scan' if sample and not sample.stop else 'would_pause')
                else:
                    goal = self._choose(grid[1], now, pose)
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
            'sightings': self.sightings, 'exit': self.exit,
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
             f'- Strategy: {data["config"].get("strategy", "frontier")}',
             f'- Goals: {s["goals_sent"]} sent ({s["frontier_goals"]} frontier, {s["patrol_goals"]} patrol, '
             f'{s.get("walk_goals", 0)} walk), {s["goals_succeeded"]} reached, {s["goals_failed"]} failed',
             f'- Spins: {s.get("spins", 0)} started, {s.get("spins_completed", 0)} completed; walks ended at an '
             f'obstacle: {s.get("walks_to_obstacle", 0)}',
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
    run.add_argument('--speed', default='normal', choices=MODE_NAMES,
                     help='start mode (default normal; was slow. Operator 2026-11-23: every test at regular speed)')
    run.add_argument('--speed-limit', type=float, default=None,
                     help='session ceiling m/s (default: the stack\'s MAX_SPEED, up to 1.0)')
    run.add_argument('--modes', default='', help='override modes, e.g. "slow=0.04,normal=0.07,fast=0.09" '
                     '(optionally speed:yaw)')
    run.add_argument('--geofence', type=float, default=10.0,
                     help='radius around the start, m [1-10] (default 10: operator 2026-11-20, was 3)')
    run.add_argument('--max-minutes', type=float, default=HARD_MAX_ROAM_MINUTES,
                     help=f'session bound, minutes [1-{HARD_MAX_ROAM_MINUTES:.0f}]; roaming otherwise '
                          'runs until you stop it')
    run.add_argument('--abort-clearance', type=float, default=0.0,
                     help='footprint clearance abort, m (default 0 = off: Nav2 avoidance only, as the docs)')
    run.add_argument('--rough-gait', default=ROUGH_GAIT, choices=sorted(GAITS),
                     help=f'gait for stairs and rubble (default {ROUGH_GAIT}: the documented uneven high-step gait)')
    run.add_argument('--ai-option', action='append', default=[], choices=sorted(AI_OPTIONS),
                     help='documented perception switch sent at start (repeatable; none by default). The '
                          'Perception guide s7.1.3/s8.1.3 says to turn all AI options off before mapping. '
                          'Caution: the vendor app code stops the RealSense service on value 0x00 (all_off).')
    run.add_argument('--exit-open-radius', type=float, default=1.0,
                     help='exit = known-free cells this far from any wall/unknown (m; 0 = no exit detection)')
    run.add_argument('--exit-min-area', type=float, default=3.0, help='exit open area at least this large, m2')
    run.add_argument('--exit-min-from-start', type=float, default=2.0, help='ignore open space near the start, m')
    run.add_argument('--no-stop-at-exit', action='store_true', help='keep exploring after reaching the exit')
    run.add_argument('--stair-mode', default='ai', choices=STAIR_MODES,
                     help='steps: ai = AI state + axis commands (default, operator 2026-11-21); hstep = RUG H-Step gait')
    run.add_argument('--auto-stairs', action='store_true',
                     help='cross terrain-detected steps automatically in the stair mode (off: AI mode only on '
                          'stairs, whose detection is not done yet; operator 2026-11-21)')
    run.add_argument('--advisory', default='act', choices=ADVISORY_MODES,
                     help="stair/curtain advisory (~/stair_curtain): act on it, only log it, or ignore it")
    run.add_argument('--ai-axis-forward', type=int, default=12000,
                     help='AI-state forward axis value (dead zone 6553; range not in the docs: test it)')
    run.add_argument('--heartbeat', action='store_true',
                     help='send the documented 2 Hz heartbeat 0x21040001 (Comm Interface s1.2.1); unverified')
    run.add_argument('--gait', default=None, choices=sorted(GAITS),
                     help='gait sent once after the stand check (Comm Interface s1.2.5), e.g. rug_general '
                          'for rugs/uneven floor; default: keep the gait the robot stands up in')
    run.add_argument('--strategy', default='scan-walk', choices=('scan-walk', 'frontier'),
                     help='scan-walk (default): walk an open corridor, scanning in short turns only when needed; '
                          'repeat; frontier: frontier goals with patrol fallback')
    run.add_argument('--frontier-radius', type=float, default=3.0,
                     help='frontier search radius / longest scan-walk leg, m [1-5]')
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
    st = sub.add_parser('stairs', help='cross stairs from PAUSED, facing them: up | down (stairs.py)')
    st.add_argument('value', choices=('up', 'down'))
    gp = sub.add_parser('gait', help=f'switch gait while PAUSED: {" | ".join(sorted(GAITS))}')
    gp.add_argument('value', choices=sorted(GAITS))
    sub.add_parser('modes', help='show speed modes and what is (not) verifiable on this robot')
    plan = sub.add_parser('plan', help='print the effective speed limits, stop zones and turning clearance '
                                       'for a stack speed (nothing is started or commanded)')
    plan.add_argument('--max-speed', type=float, default=DEFAULT_ROAM_SPEED,
                      help='navigation stack speed the boundary is computed for (default 0.50)')
    plan.add_argument('--clearance', type=float, default=0.0, help='collision stop clearance beyond the 5 cm margin, m')
    plan.add_argument('--speed-limit', type=float, default=None, help='session ceiling (default --max-speed)')
    plan.add_argument('--modes', default='')
    plan.add_argument('--calibration', default=None, help='calibration.json (default: the operator one)')
    plan.add_argument('--json', action='store_true')
    return p.parse_args(argv)


def command_above_mode(vx, wz, mode_v, mode_w, mode_name):
    """Robot-side command beyond the selected mode, or None.

    Forward speed is held to the mode. Turning is held to the larger of the mode's yaw rate and the
    manufacturer 1.5 rad/s in-place turn (was 0.20): Nav2's own recovery Spin (behavior_server, max_rotational_vel
    0.20) bypasses the velocity smoother's mode cap, and on 2026-11-19 13:28 a recovery spin during a
    slow-mode walk ("Failed to make progress") stopped roaming at wz 0.200 > 0.15."""
    yaw_cap = max(mode_w, SPIN_WZ_RAD_S)
    if vx > mode_v + 0.01 or abs(wz) > yaw_cap + 0.02:
        return (f'robot-side command vx {vx:+.3f} wz {wz:+.3f} above the selected '
                f'{mode_name} mode ({mode_v} m/s, {yaw_cap} rad/s)')
    return None


def session_modes(text, limit):
    """Normal requests the default roam speed; fast can use the configured session maximum."""
    modes = parse_modes(text)
    custom = {part.split('=')[0].strip() for part in (text or '').split(',')}
    if 'normal' not in custom:
        modes['normal'] = (min(DEFAULT_ROAM_SPEED, limit), DEFAULT_MODES['normal'][1])
    if 'fast' not in custom:
        modes['fast'] = (limit, DEFAULT_MODES['fast'][1])
    return modes


def terrain_backup_nose(speed_limit, boundary_m):
    """backup_distance() for a physical session at speed_limit (nose distance of the terrain lock)."""
    if speed_limit <= VALIDATED_SPEED_M_S + 1e-9 or not boundary_m:
        return HARD_MIN_ROAM_STOP_DISTANCE_M
    return min(2.95, max(HARD_MIN_ROAM_STOP_DISTANCE_M, round(boundary_m - 0.05, 2)))


def effective_settings(max_speed, clearance=0.0, cal=None, speed_limit=None, modes_text=''):
    """Every speed-dependent setting a physical roam at this stack speed would run with. Pure."""
    cal = cal or CP.Calibration()
    limit = max_speed if speed_limit is None else speed_limit
    tier = CP.tiers(clearance, max_speed, cal)                  # ValueError beyond the 3.0 m cap
    b = tier['stop']
    full = CP.boundary(clearance, max_speed, cal)               # full stop from the stack speed (reference)
    cm_front = b['boundary_m'] + CP.NOSE_M
    backup = terrain_backup_nose(limit, b['boundary_m'])
    sw = ScanWalk(RoamConfig(speed_limit=min(limit, max_speed), stack_max_speed=max_speed,
                             strategy='scan_walk'), (0.0, 0.0))
    sw.stop_ahead_m, sw.cm_front_m = backup + CP.NOSE_M, cm_front
    modes = session_modes(modes_text, limit)
    accel = 0.5                                                 # velocity_smoother / MPPI max_accel
    extrapolated = max_speed > max(cal.coast_speed_m_s, VALIDATED_SPEED_M_S) + 1e-9
    return {
        'stack_max_speed_m_s': max_speed, 'session_speed_limit_m_s': limit, 'clearance_m': clearance,
        'modes': {name: {'speed_m_s': v, 'yaw_rad_s': w, 'selectable': v <= limit + 1e-9,
                         'smoother_cap': [v, 0.0, w], 'mppi_speed_limit_m_s': v,
                         'robot_cmd_abort_above_m_s': round(v + 0.01, 3),
                         'leg_odom_abort_above_m_s': round(max(2 * v, v + 0.05), 3)}
                  for name, (v, w) in modes.items()},
        'watchdog_cmd_abort_above_m_s': round(limit + 0.02, 3),
        'collision_boundary_from_nose_m': b['boundary_m'],
        'approach_time_s': tier['approach']['time_before_collision_s'] if tier['approach'] else None,
        'boundary_parts': {k: full[k] for k in ('reaction_travel_m', 'latency_s', 'coast_m', 'sensor_margin_m',
                                                'clearance_m', 'required_m')},
        'coast_basis': (f'ASSUMED: {cal.coast_m} m measured at {cal.coast_speed_m_s} m/s, scaled '
                        f'(speed ratio)^{CP.COAST_EXPONENT_ABOVE_VALIDATED} above 0.10 m/s'
                        if extrapolated else f'measured ({cal.coast_m} m at {cal.coast_speed_m_s} m/s, '
                                             'scaled linearly up to 0.10 m/s)'),
        'terrain_stop_from_nose_m': backup,
        'stop_zone_from_centre_m': round(sw.stop_zone(), 3),
        'walk_standoff_from_centre_m': round(sw.standoff(), 3),
        'turn_clearance_radius_m': round(sw.spin_radius(), 3),
        'open_floor_needed_across_m': round(2 * sw.spin_radius(), 2),
        'time_to_full_speed_s': round(limit / accel, 2),
        'distance_to_full_speed_m': round(limit ** 2 / (2 * accel), 3),
        'scan_step_s': round(SCAN_STEP_RAD / NAV2_MAX_YAW_RATE_RAD_S, 1),
        'spin_360_s': round(2 * math.pi / NAV2_MAX_YAW_RATE_RAD_S, 1),
        'validated': max_speed <= VALIDATED_SPEED_M_S + 1e-9,
    }


def print_plan(s, scale_speeds=(0.10, 0.15, 0.20, 0.30, 0.40, 0.50), clearance=0.05, cal=None):
    ok = 'VALIDATED' if s['validated'] else 'NOT VALIDATED (stopping distance above 0.10 m/s is assumed)'
    print(f'Roam speed plan: stack --max-speed {s["stack_max_speed_m_s"]} m/s, session limit '
          f'{s["session_speed_limit_m_s"]} m/s, clearance {s["clearance_m"]} m  [{ok}]')
    print('\nSpeed limit chain (each layer caps the one before it):')
    print('  1 Nav2 static ceilings  MPPI vx_max 1.0, velocity_smoother max_velocity 1.0 (never the limit)')
    print(f'  2 collision boundary    computed for {s["stack_max_speed_m_s"]} m/s at navigation start '
          '(roam refuses a session limit above it)')
    print(f'  3 session limit         {s["session_speed_limit_m_s"]} m/s: no mode above it can be selected')
    print('  4 selected mode         velocity_smoother max_velocity set + read back (hard clamp after MPPI);')
    print('                          MPPI /speed_limit re-sent after every goal start (Humble resets it)')
    print('  5 runtime checks        robot-side command > mode + 0.01 m/s, leg odometry > max(2v, v+0.05),')
    print(f'                          watchdog command > {s["watchdog_cmd_abort_above_m_s"]} m/s -> safety stop')
    print('\nSpeed modes (sessions start in normal):')
    for name, m in s['modes'].items():
        print(f'  {name:<7} {m["speed_m_s"]:.2f} m/s {m["yaw_rad_s"]:.2f} rad/s  '
              f'{"selectable" if m["selectable"] else "ABOVE LIMIT: refused"}  '
              f'(odometry abort > {m["leg_odom_abort_above_m_s"]} m/s)')
    p = s['boundary_parts']
    print('\nStopping (at the stack speed):')
    full_stop = (f'reaction {p["reaction_travel_m"]:.3f} ({p["latency_s"]:.2f} s) + coast {p["coast_m"]:.3f} + '
                 f'margin {p["sensor_margin_m"]:.2f} + clearance {p["clearance_m"]:.2f} = {p["required_m"]:.2f}')
    if s['approach_time_s']:
        print(f'  collision_monitor approach  full speed unless a collision along the current command is '
              f'predicted within {s["approach_time_s"]} s (head-on: slowing starts '
              f'{s["stack_max_speed_m_s"] * s["approach_time_s"]:.2f} m out); turning never slowed')
        print(f'  collision_monitor stop  {s["collision_boundary_from_nose_m"]:.2f} m from the nose (sized for '
              f'0.10 m/s); full stop from {s["stack_max_speed_m_s"]} m/s would need {full_stop}')
    else:
        print(f'  collision_monitor stop  {s["collision_boundary_from_nose_m"]:.2f} m from the nose = {full_stop}')
    print(f'  coast                   {s["coast_basis"]}')
    print(f'  terrain backup stop     {s["terrain_stop_from_nose_m"]:.2f} m from the nose')
    print(f'  scan-walk               walk ends {s["walk_standoff_from_centre_m"]:.2f} m (centre) before '
          f'obstacles; turning needs {s["turn_clearance_radius_m"]:.2f} m radius clear = '
          f'{s["open_floor_needed_across_m"]:.1f} m of open floor across')
    print(f'  acceleration            full speed after {s["time_to_full_speed_s"]} s / '
          f'{s["distance_to_full_speed_m"]} m (0.5 m/s^2); 30 deg scan step {s["scan_step_s"]} s ({NAV2_MAX_YAW_RATE_RAD_S} rad/s)')
    print('\nScaling (same clearance and calibration):')
    print('  speed  stop      approach  terrain  turn radius  open floor  coast basis')
    for v in scale_speeds:
        try:
            e = effective_settings(v, clearance, cal)
        except ValueError as error:
            print(f'  {v:.2f}   refused: {error}')
            continue
        ap = e['approach_time_s']
        print(f'  {v:.2f}   {e["collision_boundary_from_nose_m"]:.2f} m    '
              f'{(f"{ap:.1f} s" if ap else "-"):<8}  {e["terrain_stop_from_nose_m"]:.2f} m   '
              f'{e["turn_clearance_radius_m"]:.2f} m       {e["open_floor_needed_across_m"]:.1f} m      '
              f'{"measured" if e["validated"] else "assumed"}')


def build_config(a, state):
    stack_max = float(state.get('MAX_SPEED', 0.10)) if state.get('MAX_SPEED') else 0.10
    limit = a.speed_limit if a.speed_limit is not None else stack_max
    if limit > stack_max + 1e-9:
        raise ConfigError(f'--speed-limit {limit} exceeds the {stack_max} m/s the collision stop boundary '
                          'was computed for (restart navigation with --max-speed)')
    modes = session_modes(a.modes, limit)
    # Modes above the session limit stay listed but cannot be selected (refused at switch time).
    return RoamConfig(speed_limit=limit, modes=modes, start_mode=a.speed, geofence_m=a.geofence,
                      max_minutes=a.max_minutes, abort_clearance_m=a.abort_clearance,
                      frontier_radius_m=a.frontier_radius, frontier_clearance_m=a.frontier_clearance,
                      max_map_age_s=a.max_map_age, max_sensor_age_s=a.max_sensor_age,
                      min_lidar_hz=a.min_lidar_hz, stack_max_speed=stack_max,
                      strategy=a.strategy.replace('-', '_'), gait=a.gait,
                      docs_stops=state.get('STOP_LAYERS') == 'docs', heartbeat=a.heartbeat,
                      rough_gait=a.rough_gait, ai_options=a.ai_option,
                      exit_open_radius_m=a.exit_open_radius, exit_min_area_m2=a.exit_min_area,
                      exit_min_from_start_m=a.exit_min_from_start, stop_at_exit=not a.no_stop_at_exit,
                      stair_mode=a.stair_mode, auto_stairs=a.auto_stairs, ai_axis_forward=a.ai_axis_forward,
                      advisory=a.advisory)


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
    print('Gaits (Motion Host Communication Interface s1.2.5, sent on /simple_cmd like the stand toggle;\n'
          'only outside AI state, in move mode): ' + ', '.join(sorted(GAITS)) + '.\n'
          'roam start --gait NAME sends one after the stand check; operator/roam.sh gait NAME while paused.\n'
          'The active gait is NOT reported on this deployment (motion_receiver drops robot_gait_state):\n'
          'confirm the switch by eye or on the remote app.')


def main(argv=None):
    a = parse(sys.argv[1:] if argv is None else argv)
    state = stack_state()
    directory = control_dir(state)
    if a.action in ('pause', 'resume', 'stop', 'speed', 'gait', 'stairs'):
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
        limit = float(state.get('MAX_SPEED', DEFAULT_ROAM_SPEED))
        show_modes(session_modes('', limit), limit)
        return 0
    if a.action == 'plan':
        cal_path = a.calibration or (Path(os.environ.get('LITE3_STATE_FILE') or
                                          Path.home() / 'lite3_ws/log/operator/state.env').parent / 'calibration.json')
        cal = CP.load_calibration(cal_path)
        if a.speed_limit is not None and a.speed_limit > a.max_speed + 1e-9:
            print(f'REFUSED: --speed-limit {a.speed_limit} exceeds --max-speed {a.max_speed}')
            return 2
        try:
            s = effective_settings(a.max_speed, a.clearance, cal, a.speed_limit, a.modes)
        except (ValueError, ConfigError) as error:
            print(f'REFUSED: {error}')
            return 2
        if a.json:
            print(json.dumps(s, indent=1))
        else:
            print(f'Calibration: {cal_path} ({len(cal.sources)} sources)')
            print_plan(s, clearance=a.clearance, cal=cal)
            if state.get('MAX_SPEED'):
                print(f'\nRunning navigation: --max-speed {state.get("MAX_SPEED")} --clearance '
                      f'{state.get("CLEARANCE")} (boundary {state.get("BOUNDARY")} m, {state.get("MODE")} mode)'
                      + ('' if abs(float(state['MAX_SPEED']) - a.max_speed) < 1e-9 else
                         ' - DIFFERENT from this plan: operator/stop_navigation.sh first'))
        return 0
    return run_session(a, state, directory)


def run_session(a, state, directory):
    mode = MODE_PHYSICAL if a.physical else MODE_SIM if a.sim else MODE_READONLY
    if mode == MODE_SIM:
        directory = Path(directory) / 'sim'      # never mixes with a real session's control files
    try:
        # sim: no stack; the stand-in runs with the boundary roam.sh would start navigation with
        cfg = build_config(a, state if mode != MODE_SIM else
                           {'MAX_SPEED': str(a.speed_limit or DEFAULT_ROAM_SPEED)})
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
            tier = CP.tiers(0.0, min(max(cfg.speed_limit, 0.08), 0.5))   # stand-in polygon (any speed)
            robot = make_sim_robot(cm_boundary=tier['stop']['boundary_m'],
                                   cm_approach_s=tier['approach']['time_before_collision_s']
                                   if tier['approach'] else None)
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
                if mode == MODE_PHYSICAL and backup > HARD_MIN_ROAM_STOP_DISTANCE_M + 1e-9:
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
