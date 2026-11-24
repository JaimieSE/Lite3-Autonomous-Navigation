"""Stair routine for the competition stairs (3 steps up, 3 down, about 5 cm each).

Documentation status (2026-11-21):
- Comm Interface s1.2.5 documents the gaits; "RUG gait in H-Step gear" (0x21010407) is the vendor's
  high-lift stepping gait, switchable in move mode outside AI state. Perception guide s7.3.2/s8.2.2 pairs
  "uneven high step gait" with rough ground. Speed commands work in navigation mode (s1.2.12).
- No guide describes stair climbing, a stair-specific mode or a climbable step height.
- The robot's own stair mode (said to be set up in the robot computer's .bashrc) could not be read: that
  needs a login to the robot computer, which needs lab approval. RobotStairMode lists what is unknown.

So the routine is a pure state machine (unit-tested) driving a StairInterface. HStepGaitStairs is the
documented implementation: switch to the high-step gait, walk straight on /cmd_vel, switch back.
"""

import math
from dataclasses import dataclass


@dataclass(frozen=True)
class StairConfig:
    approach_speed_m_s: float = 0.15     # straight-on approach in the normal gait
    approach_max_m: float = 1.0          # give up approaching if no incline within this distance
    climb_speed_m_s: float = 0.20        # walking speed on the stairs (high-step gait)
    switch_settle_s: float = 1.5         # stand still after a gait switch (same as roam's gait settle)
    max_climb_s: float = 20.0            # one attempt; 3 x 5 cm steps take a few seconds at 0.2 m/s
    min_climb_m: float = 0.30            # travelled on the incline before the top can count
    incline_pitch_rad: float = 0.05      # |pitch| beyond this = on the stairs (5 cm over ~0.3 m tread ~ 0.17)
    level_pitch_rad: float = 0.03        # |pitch| below this ...
    level_hold_s: float = 1.0            # ... for this long = top (or bottom) reached
    max_pitch_rad: float = 0.45          # abort beyond (fall-down protection, Motion Dev Guide s1.4.2, is
    max_roll_rad: float = 0.30           # the robot's own last resort)
    max_retries: int = 1                 # extra attempts after a timed-out climb
    approach_max_s: float = 8.0          # approach also gives up after this long (blocked feet)
    flat_cross_m: float = 0.6            # a step too small to tilt the body: crossed after this far, level
    landing_max_m: float = 1.2           # over: walk the top landing this far looking for the way down

    def __post_init__(self):
        if not 0 < self.level_pitch_rad < self.incline_pitch_rad < self.max_pitch_rad:
            raise ValueError('need 0 < level_pitch < incline_pitch < max_pitch')
        if not 0 < self.approach_speed_m_s <= 1.0 or not 0 < self.climb_speed_m_s <= 1.0:
            raise ValueError('speeds must be within the documented 1.0 m/s (Comm Interface s1.2.12)')
        if self.max_retries < 0 or self.max_climb_s <= 0:
            raise ValueError('bad retry/time limits')


@dataclass(frozen=True)
class Action:
    vx: float = 0.0                      # forward command (m/s); never sideways or backwards
    switch: str = None                   # 'in' (stair gait) / 'out' (normal gait) to do now, else None
    done: bool = False
    aborted: bool = False
    reason: str = None


class StairRoutine:
    """approach -> switch_in -> climb -> switch_out -> done. Abort on pitch/roll limits; bounded retries.
    over=True (competition staircase: 3 up then 3 down, operator 2026-11-23): after the top, stay in the stair
    mode on the landing until the body tilts nose-down, descend to level ground, then switch out; no way
    down within landing_max_m = a plain top, switch out there.
    step() is called every tick with the robot's pitch/roll (rad) and the distance travelled (m)."""

    def __init__(self, direction, cfg=StairConfig(), start='approach', over=False):
        if direction not in ('up', 'down'):
            raise ValueError("direction must be 'up' or 'down'")
        if start not in ('approach', 'switch_in'):
            raise ValueError("start must be 'approach' or 'switch_in'")
        self.direction = direction
        self.cfg = cfg
        self.over = bool(over) and direction == 'up'
        self.state = start            # 'switch_in': already at the step (automatic trigger)
        self.switch_sent = False
        self.t_state = None
        self.d_state = 0.0
        self.inclined = False
        self.level_since = None
        self.retries = 0
        self.reason = None

    def _enter(self, state, t, travelled):
        self.state, self.t_state, self.d_state = state, t, travelled

    def _abort(self, why):
        self.reason = why
        self.state = 'aborted'
        return Action(switch='out', aborted=True, reason=why)

    def step(self, t, pitch, roll, travelled):
        c = self.cfg
        if self.t_state is None:
            self.t_state, self.d_state = t, travelled
        if self.state in ('done', 'aborted'):
            return Action(done=self.state == 'done', aborted=self.state == 'aborted', reason=self.reason)
        if pitch is None or roll is None or not (math.isfinite(pitch) and math.isfinite(roll)):
            return self._abort('no valid IMU attitude')
        if abs(pitch) > c.max_pitch_rad or abs(roll) > c.max_roll_rad:
            return self._abort(f'attitude limit: pitch {pitch:.2f} roll {roll:.2f} rad')
        if self.state == 'approach':
            if abs(pitch) >= c.incline_pitch_rad:                 # front feet on the first step
                self._enter('switch_in', t, travelled)
                self.switch_sent = True
                return Action(switch='in')
            if travelled - self.d_state > c.approach_max_m or t - self.t_state > c.approach_max_s:
                return self._abort(f'no incline within {c.approach_max_m} m / {c.approach_max_s:.0f} s')
            return Action(vx=c.approach_speed_m_s)
        if self.state == 'switch_in':
            if not self.switch_sent:
                self.switch_sent = True
                self.t_state = t
                return Action(switch='in')
            if t - self.t_state < c.switch_settle_s:
                return Action()
            self._enter('climb', t, travelled)
            self.inclined, self.level_since = False, None
        if self.state == 'climb':
            if abs(pitch) >= c.incline_pitch_rad:
                self.inclined, self.level_since = True, None
            elif self.inclined and abs(pitch) < c.level_pitch_rad and travelled - self.d_state >= c.min_climb_m:
                self.level_since = t if self.level_since is None else self.level_since
                if t - self.level_since >= c.level_hold_s:        # level again after the incline: top
                    if self.over and self.direction == 'up':
                        self._enter('landing', t, travelled)
                        return Action(vx=c.climb_speed_m_s)
                    self._enter('switch_out', t, travelled)
                    return Action(switch='out')
            elif not self.inclined and abs(pitch) < c.level_pitch_rad and travelled - self.d_state >= c.flat_cross_m:
                self._enter('switch_out', t, travelled)               # small step: crossed without tilting
                return Action(switch='out')
            else:
                self.level_since = None
            if t - self.t_state > c.max_climb_s:
                if self.retries >= c.max_retries:
                    return self._abort(f'no {"top" if self.direction == "up" else "bottom"} within '
                                       f'{c.max_climb_s:.0f} s ({self.retries} retries)')
                self.retries += 1
                self._enter('climb', t, travelled)
                return Action()
            return Action(vx=c.climb_speed_m_s)
        if self.state == 'landing':
            if abs(pitch) >= c.incline_pitch_rad:                 # front feet over the first step down
                self.direction = 'down'
                self._enter('descend', t, travelled)
                self.inclined, self.level_since = True, None
                return Action(vx=c.climb_speed_m_s)
            if travelled - self.d_state > c.landing_max_m:       # no way down: a plain top
                self._enter('switch_out', t, travelled)
                return Action(switch='out')
            return Action(vx=c.climb_speed_m_s)
        if self.state == 'descend':
            if abs(pitch) >= c.incline_pitch_rad:
                self.inclined, self.level_since = True, None
            elif abs(pitch) < c.level_pitch_rad and travelled - self.d_state >= c.min_climb_m:
                self.level_since = t if self.level_since is None else self.level_since
                if t - self.level_since >= c.level_hold_s:        # level at the bottom
                    self._enter('switch_out', t, travelled)
                    return Action(switch='out')
            else:
                self.level_since = None
            if t - self.t_state > c.max_climb_s:
                return self._abort(f'no bottom within {c.max_climb_s:.0f} s of the way down')
            return Action(vx=c.climb_speed_m_s)
        if self.state == 'switch_out':
            if t - self.t_state < c.switch_settle_s:
                return Action()
            self.state = 'done'
            return Action(done=True)
        return self._abort(f'unknown state {self.state}')


class StairInterface:
    """What the routine needs from the robot. Implementations must not leave the stair gait/mode active."""
    settle_s = 1.5                       # stand still this long after switching in/out

    def drive(self, vx):
        """Forward command for this tick (navigation-mode speed command by default)."""
        self.robot.send_velocity(vx, 0.0)

    def switch_in(self, direction):
        raise NotImplementedError

    def switch_out(self):
        raise NotImplementedError

    # (relax_stops / restore_stops removed 2026-11-24 with the terrain proximity stop)


class HStepGaitStairs(StairInterface):
    """Documented path: RUG H-Step gait in, normal gait out (Comm Interface s1.2.5); motion stays on the
    navigation-mode speed commands (s1.2.12), so the robot never leaves navigation mode."""

    def __init__(self, robot, stair_gait='rug_hstep', normal_gait='flat_medium'):
        self.robot, self.stair_gait, self.normal_gait = robot, stair_gait, normal_gait

    def switch_in(self, direction):
        return self.robot.send_gait(self.stair_gait)

    def switch_out(self):
        return self.robot.send_gait(self.normal_gait)



class AiModeStairs(StairInterface):
    """Operator 2026-11-21: switch to the robot's AI mode on steps. Documented (Comm Interface): enter AI state
    0x21010528, AI Basic gait 0x2101052A (s1.2.13), move with axis commands at >= 20 Hz (s1.2.3.2; forward
    0x21010130, dead zone +-6553), exit 0x2101052B, then navigation mode 0x21010C03 again (s1.2.7: not
    available inside AI state). NOT documented: the axis value range (lost from the PDF export), so
    forward_value is config and must be checked on the robot; and how long entering AI state takes."""
    settle_s = 3.0

    def __init__(self, robot, forward_value=12000, normal_gait='flat_medium'):
        from .ros_io import AXIS_FORWARD_DEAD_ZONE
        if not AXIS_FORWARD_DEAD_ZONE < forward_value <= 32767:
            raise ValueError(f'forward axis value must be past the dead zone ({AXIS_FORWARD_DEAD_ZONE}) and <= 32767')
        self.robot, self.forward_value, self.normal_gait = robot, int(forward_value), normal_gait
        self.active = False

    def switch_in(self, direction):
        from .ros_io import AI_ENTER, AI_GAIT_BASIC
        self.robot.send_velocity(0.0, 0.0)
        first = self.robot.send_simple_command(AI_ENTER)
        self.robot.send_simple_command(AI_GAIT_BASIC)
        self.active = True
        return first

    def drive(self, vx):
        from .ros_io import AXIS_FORWARD
        if not self.active:                       # approach / after exit: normal navigation-mode command
            self.robot.send_velocity(vx, 0.0)
            return
        self.robot.send_axis(AXIS_FORWARD, self.forward_value if vx > 0 else 0)   # every tick (>= 20 Hz)

    def switch_out(self):
        from .ros_io import AI_EXIT, AXIS_FORWARD, NAVIGATION_MODE
        if self.active:
            self.robot.send_axis(AXIS_FORWARD, 0)
        evidence = self.robot.send_simple_command(AI_EXIT)
        self.active = False
        self.robot.send_simple_command(NAVIGATION_MODE)
        self.robot.send_gait(self.normal_gait)
        return evidence


class RobotStairMode(StairInterface):
    """The robot computer's own stair mode (operator: set up in its .bashrc). NOT READABLE without a
    lab-approved login, so every method raises. Unknown, to be filled in from a read-only look at it:
      1. how it is switched on/off (script, service, UDP code, ROS topic);
      2. whether it puts the robot in AI state (0x21010528; then navigation mode is NOT available,
         Comm Interface s1.2.7, and motion needs axis commands at >= 20 Hz, s1.2.3.2);
      3. its command type (axis commands vs /cmd_vel speed commands) and value ranges (the axis range
         formula is missing from the PDF text export of s1.2.3);
      4. what reports that it is active (robot_policy_state / robot_gait_state are not published here)."""

    def switch_in(self, direction):
        raise NotImplementedError('robot stair mode interface unknown (see class docstring)')

    def switch_out(self):
        raise NotImplementedError('robot stair mode interface unknown (see class docstring)')
