"""Write simulated bounded-test traces. No ROS, SDK, transport, or robot I/O."""

from dataclasses import asdict
import json
from pathlib import Path
import sys


ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(ROOT / "legacy"))

from lite3_legacy.bounded_test import BoundedMovementTest, TestState


def simulate(kill_at=None):
    now = [0.0]
    test = BoundedMovementTest(clock=lambda: now[0])
    trace = []

    def record(intents):
        trace.append({'simulated_time': now[0], 'state': test.state.value,
                      'reason': test.reason, 'intents': [asdict(item) for item in intents]})

    record(test.start(navigation_confirmed=True, operator_ready=True))
    for step in range(1, 21):
        now[0] = step / 20.0
        if kill_at is not None and now[0] >= kill_at and test.state == TestState.RUNNING:
            record(test.kill())
        else:
            record(test.tick())
        if test.state == TestState.HANDBACK:
            # This is an explicit fake confirmation, never a received robot status.
            test.confirm_manual(True)
            record(())
            break
    return trace


def main():
    result = {
        'mode': 'OFFLINE SIMULATION ONLY; no robot commands transmitted',
        'manual_confirmations': 'simulated, not robot acknowledgments',
        'normal_completion': simulate(),
        'operator_kill_at_0_2s': simulate(kill_at=0.2),
    }
    path = ROOT / '.colcon' / 'review' / 'movement_plan.json'
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(result, indent=2) + '\n')
    print(f'Offline movement/handback intent traces: {path}')


if __name__ == '__main__':
    main()
