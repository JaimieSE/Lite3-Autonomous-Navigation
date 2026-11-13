"""ROS-independent velocity validation and stop policy for offline tests."""

from dataclasses import dataclass
from enum import Enum
import math
import time
from typing import Callable, Optional


def _finite_number(value: object) -> bool:
    if not isinstance(value, (int, float)) or isinstance(value, bool):
        return False
    try:
        return math.isfinite(value)
    except OverflowError:
        return False


@dataclass(frozen=True)
class PolicyConfig:
    """Placeholder test limits, not measured or certified Lite3 capabilities."""

    max_linear_x: float = 0.10
    max_linear_y: float = 0.05
    max_angular_z: float = 0.20
    command_timeout: float = 0.50

    def __post_init__(self) -> None:
        for name in ('max_linear_x', 'max_linear_y', 'max_angular_z'):
            value = getattr(self, name)
            if not _finite_number(value) or value < 0:
                raise ValueError(f'{name} must be a finite nonnegative number')
        if not _finite_number(self.command_timeout) or self.command_timeout <= 0:
            raise ValueError('command_timeout must be a finite positive number')


@dataclass(frozen=True)
class Velocity:
    linear_x: float = 0.0
    linear_y: float = 0.0
    linear_z: float = 0.0
    angular_x: float = 0.0
    angular_y: float = 0.0
    angular_z: float = 0.0


ZERO = Velocity()


class StopState(str, Enum):
    WAITING = 'waiting'
    ACTIVE = 'active'
    INVALID = 'invalid'
    TIMED_OUT = 'timed_out'
    STOPPED = 'stopped'


class CommandPolicy:
    """Latest accepted command with monotonic receipt-time expiry.

    A software stop clears the command and stays latched. Releasing the stop
    requires a new accepted command; it never resumes the previous command.
    This class performs no I/O and knows nothing about the robot SDK.
    """

    def __init__(
        self, config: PolicyConfig, clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.config = config
        self._clock = clock
        self._command = ZERO
        self._received_at: Optional[float] = None
        self._stopped = False
        self.state = StopState.WAITING

    def accept(self, command: Velocity) -> bool:
        if self._stopped:
            return False
        if not isinstance(command, Velocity) or not all(
            _finite_number(value) for value in vars(command).values()
        ) or any((command.linear_z, command.angular_x, command.angular_y)):
            self._clear(StopState.INVALID)
            return False

        def clamp(value: float, limit: float) -> float:
            return float(max(-limit, min(limit, value)))

        self._command = Velocity(
            linear_x=clamp(command.linear_x, self.config.max_linear_x),
            linear_y=clamp(command.linear_y, self.config.max_linear_y),
            angular_z=clamp(command.angular_z, self.config.max_angular_z),
        )
        self._received_at = self._clock()
        self.state = StopState.ACTIVE
        return True

    def output(self) -> Velocity:
        if self._received_at is not None:
            if self._clock() - self._received_at >= self.config.command_timeout:
                self._clear(StopState.TIMED_OUT)
        return self._command

    def set_stop(self, active: bool) -> None:
        if not isinstance(active, bool):
            raise ValueError('stop flag must be a bool')
        if active:
            self._stopped = True
            self._clear(StopState.STOPPED)
        elif self._stopped:
            self._stopped = False
            self._clear(StopState.WAITING)

    def _clear(self, state: StopState) -> None:
        self._command = ZERO
        self._received_at = None
        self.state = state
