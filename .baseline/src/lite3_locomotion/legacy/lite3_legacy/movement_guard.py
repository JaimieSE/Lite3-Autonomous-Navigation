"""ROS-independent, explicitly armed command gate for the separate live adapter."""

import math
import time

from lite3_locomotion.command_policy import CommandPolicy, PolicyConfig, StopState, ZERO


class MovementGuard:
    def __init__(self, config=None, *, session_seconds=60.0, clock=time.monotonic):
        if not math.isfinite(session_seconds) or not 0 < session_seconds <= 300:
            raise ValueError('Session duration must be in (0, 300] seconds')
        self.clock = clock
        self.policy = CommandPolicy(config or PolicyConfig(0.10, 0.0, 0.20, 0.4), clock)
        self.session_seconds = session_seconds
        self.armed = False
        self.ever_armed = False
        self.reason = 'disarmed'
        self.started = None
        self.policy.set_stop(True)

    def arm(self, healthy):
        if not healthy:
            return False
        if self.armed:
            return True  # Repeated arm cannot extend the mission deadline.
        self.policy.set_stop(False)
        self.armed = self.ever_armed = True
        self.started = self.clock()
        self.reason = 'waiting_for_fresh_command'
        return True

    def stop(self, reason='operator_stop'):
        self.armed = False
        self.reason = reason
        self.policy.set_stop(True)
        return ZERO

    def accept(self, command, healthy):
        if not self.armed:
            return False
        if not healthy:
            self.stop('sensor_tf_or_link_unhealthy')
            return False
        if not self.policy.accept(command):
            self.stop('invalid_command')
            return False
        self.reason = 'active'
        return True

    def output(self, healthy):
        if not self.armed:
            return ZERO
        if not healthy:
            return self.stop('sensor_tf_or_link_unhealthy')
        elapsed = self.clock() - self.started
        if elapsed < 0 or elapsed >= self.session_seconds:
            return self.stop('session_deadline')
        command = self.policy.output()
        if self.policy.state == StopState.TIMED_OUT:
            return self.stop('command_timeout')
        if self.policy.state == StopState.WAITING and elapsed >= 2.0:
            return self.stop('first_command_timeout')
        return command



# Shared predicate remains in the active mission safety module.
from lite3_locomotion.safety import scan_is_clear
