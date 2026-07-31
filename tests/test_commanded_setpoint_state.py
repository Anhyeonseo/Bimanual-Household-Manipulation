import math
import sys
import threading
import unittest
from pathlib import Path


PACKAGE_ROOT = Path("ros2_ws/src/single_arm_bridge")
sys.path.insert(0, str(PACKAGE_ROOT))

from single_arm_bridge.commanded_setpoint_state import (  # noqa: E402
    CommandedSetpointState,
)


class CommandedSetpointStateTests(unittest.TestCase):
    def test_bridge_wires_one_shared_state_into_both_actions(self) -> None:
        bridge_source = (
            PACKAGE_ROOT
            / "single_arm_bridge"
            / "bridge_node.py"
        ).read_text(encoding="utf-8")

        self.assertEqual(
            bridge_source.count(
                "self._commanded_setpoints = CommandedSetpointState()"
            ),
            1,
        )
        self.assertEqual(
            bridge_source.count(
                "setpoint_state=self._commanded_setpoints"
            ),
            2,
        )

    def test_new_state_has_no_commanded_target(self) -> None:
        state = CommandedSetpointState()

        self.assertIsNone(state.snapshot())

    def test_successful_target_commit_survives_feedback_residual(self) -> None:
        state = CommandedSetpointState()
        target = (0.1, 0.2, 0.3, 0.4, 0.5, 0.13)

        state.commit(target)

        self.assertEqual(state.snapshot(), target)

    def test_reset_requires_fresh_feedback_before_reuse(self) -> None:
        state = CommandedSetpointState()
        state.commit((0.1,) * 6)
        state.reset()

        self.assertIsNone(state.snapshot())
        state.commit((0.2,) * 6)
        self.assertEqual(state.snapshot(), (0.2,) * 6)

    def test_invalid_count_and_non_finite_values_are_rejected(self) -> None:
        state = CommandedSetpointState()

        with self.assertRaisesRegex(ValueError, "setpoint count"):
            state.commit((0.0,) * 5)
        with self.assertRaisesRegex(ValueError, "non-finite"):
            state.commit((0.0, 0.0, math.nan, 0.0, 0.0, 0.0))

    def test_concurrent_commits_never_expose_a_partial_tuple(self) -> None:
        state = CommandedSetpointState()
        barrier = threading.Barrier(3)

        def commit(value: float) -> None:
            barrier.wait()
            state.commit((value,) * 6)

        threads = [
            threading.Thread(target=commit, args=(0.1,)),
            threading.Thread(target=commit, args=(0.2,)),
        ]
        for thread in threads:
            thread.start()
        barrier.wait()
        for thread in threads:
            thread.join()

        self.assertIn(state.snapshot(), ((0.1,) * 6, (0.2,) * 6))


if __name__ == "__main__":
    unittest.main()
