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


def test_read_only_startup_physically_disables_all_servos() -> None:
    constructor = function_body(BRIDGE, "def __init__(self) -> None")
    read_only_branch = constructor.index(
        "# READ_ONLY is a physical contract"
    )
    mode_log = constructor.index('mode = "READ_ONLY"')

    assert "self._transport.disable()" in constructor[read_only_branch:mode_log]


def test_latched_startup_disables_before_exposing_blocked_backend() -> None:
    constructor = function_body(BRIDGE, "def __init__(self) -> None")
    latch_branch = constructor.index("if hello.stop_latched:")
    latch_warning = constructor.index(
        '"STM32 stop is latched; physical torque disabled; "',
        latch_branch,
    )

    assert constructor.index(
        "self._transport.disable()", latch_branch
    ) < latch_warning


def test_shutdown_disable_is_not_gated_by_motion_or_heartbeat() -> None:
    shutdown = function_body(BRIDGE, "def destroy_node(self) -> bool")
    disable_call = shutdown.index("self._transport.disable()")
    guard = shutdown[:disable_call]

    assert "self._allow_motion" not in guard
    assert "self._motion_armed" not in guard
    assert "not self._faulted" not in guard
    assert "self._transport.heartbeat()" not in shutdown
