"""Review-only bounded movement sequencing. Returns intents, never performs I/O.

No ROS publisher, SDK, socket, mode switch, or live transport is implemented.
Callers must not interpret an intent or a simulated confirmation as robot state.
"""

from dataclasses import dataclass
from enum import Enum
import math
import time
from typing import Callable, Optional, Tuple

from lite3_locomotion.command_policy import Velocity, ZERO


MANUAL_MODE_CODE = 0x21010C02  # ROS Lab Exercise 3, part II, page 8; not transmitted.


@dataclass(frozen=True)
class TestPlan:
    __test__ = False
    speed: float = 0.03
    duration: float = 0.5
    update_timeout: float = 0.2
    handback_timeout: float = 2.0

    def __post_init__(self):
        for name, upper_bound in (
            ('speed', 0.05), ('duration', 1.0),
            ('update_timeout', 0.25), ('handback_timeout', 5.0),
        ):
            value = getattr(self, name)
            try:
                valid = (not isinstance(value, bool) and isinstance(value, (int, float)) and
                         math.isfinite(value) and 0 < value <= upper_bound)
            except OverflowError:
                valid = False
            if not valid:
                raise ValueError(f'{name} must be finite and in (0, {upper_bound}]')


class TestState(str, Enum):
    __test__ = False
    DISARMED = 'disarmed'
    RUNNING = 'running'
    HANDBACK = 'awaiting_manual_confirmation'
    COMPLETE = 'manual_confirmed'
    FAULT = 'manual_confirmation_missing'


@dataclass(frozen=True)
class Intent:
    kind: str
    velocity: Velocity = ZERO
    cmd_code: Optional[int] = None


def stop_intents() -> Tuple[Intent, Intent]:
    # Ordering is part of the proposed contract, not a physical-stop guarantee.
    return (Intent('velocity', ZERO), Intent('manual_mode_request', cmd_code=MANUAL_MODE_CODE))


class BoundedMovementTest:
    """One-shot forward-only plan with latched abort and explicit handback.

    The proposed displacement is nominal speed * duration, NOT a measured bound.
    Every update must arrive within update_timeout. Tick gaps, duration expiry,
    kill, and shutdown all prohibit further nonzero intents. A new object is
    required for any subsequent test. The real robot needs an independent
    watchdog; an uncalled Python method cannot stop a crashed process.
    """

    def __init__(self, plan: TestPlan = TestPlan(), clock: Callable[[], float] = time.monotonic):
        self.plan = plan
        self._clock = clock
        self.state = TestState.DISARMED
        self.reason = 'not_started'
        self._started = None
        self._last_update = None
        self._handback_started = None

    def start(self, *, navigation_confirmed: bool, operator_ready: bool) -> Tuple[Intent, ...]:
        if self.state != TestState.DISARMED:
            raise RuntimeError('Test is one-shot; a killed or completed test cannot rearm')
        if navigation_confirmed is not True or operator_ready is not True:
            raise ValueError('Explicit navigation and operator confirmations are required')
        self._started = self._last_update = self._clock()
        self.state = TestState.RUNNING
        self.reason = 'running'
        return (Intent('velocity', Velocity(linear_x=self.plan.speed)),)

    def tick(self) -> Tuple[Intent, ...]:
        now = self._clock()
        if self.state == TestState.RUNNING:
            if now < self._last_update or now - self._last_update >= self.plan.update_timeout:
                return self.kill('update_watchdog')
            if now - self._started >= self.plan.duration:
                return self.kill('duration_complete')
            self._last_update = now
            return (Intent('velocity', Velocity(linear_x=self.plan.speed)),)
        if self.state == TestState.HANDBACK:
            if now - self._handback_started >= self.plan.handback_timeout:
                self.state = TestState.FAULT
                return stop_intents()
            return stop_intents()
        return ()

    def kill(self, reason: str = 'operator_kill') -> Tuple[Intent, ...]:
        if self.state in (TestState.COMPLETE, TestState.FAULT):
            return ()
        if self.state != TestState.HANDBACK:
            self.reason = reason
            self._handback_started = self._clock()
            self.state = TestState.HANDBACK
        return stop_intents()

    def confirm_manual(self, confirmed: bool) -> None:
        if (self.state == TestState.HANDBACK and
                self._clock() - self._handback_started >= self.plan.handback_timeout):
            self.state = TestState.FAULT
        if self.state != TestState.HANDBACK or confirmed is not True:
            raise ValueError('Manual confirmation is only valid during handback')
        self.state = TestState.COMPLETE

    def shutdown(self) -> Tuple[Intent, ...]:
        return self.kill('shutdown')
