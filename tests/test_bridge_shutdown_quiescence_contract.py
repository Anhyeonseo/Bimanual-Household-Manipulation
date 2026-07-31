from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
BRIDGE = (
    ROOT
    / "ros2_ws"
    / "src"
    / "single_arm_bridge"
    / "single_arm_bridge"
    / "bridge_node.py"
).read_text(encoding="utf-8")


def function_body(source: str, signature: str) -> str:
    start = source.index(signature)
    brace = source.index(":", start)
    next_method = source.find("\n    def ", brace + 1)
    return source[brace + 1 : next_method if next_method >= 0 else len(source)]


def test_periodic_callbacks_are_silent_after_shutdown_starts() -> None:
    heartbeat = function_body(BRIDGE, "def _send_heartbeat(self) -> None")
    feedback = function_body(BRIDGE, "def _publish_feedback(self) -> None")

    assert "self._shutdown_requested or self._faulted" in heartbeat
    assert "self._shutdown_requested or self._faulted" in feedback


def test_transport_error_does_not_log_into_an_invalid_context() -> None:
    handler = function_body(BRIDGE, "def _handle_transport_error(")
    context_guard = "not rclpy.ok(context=self.context)"

    assert context_guard in handler
    assert handler.index(context_guard) < handler.index("self.get_logger().warning")
    assert handler.index(context_guard) < handler.index("self.get_logger().error")


def test_prepare_shutdown_sets_flag_before_cancelling_timers() -> None:
    shutdown = function_body(BRIDGE, "def prepare_shutdown(self) -> None")
    flag = "self._shutdown_requested = True"
    cancel = "timer.cancel()"

    assert "if self._shutdown_requested:" in shutdown
    assert flag in shutdown
    assert cancel in shutdown
    assert shutdown.index(flag) < shutdown.index(cancel)
    assert shutdown.index(cancel) < shutdown.index("notify_connection_loss")
