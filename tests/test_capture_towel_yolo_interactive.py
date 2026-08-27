from __future__ import annotations

import pytest

from tools.run.capture_towel_yolo_interactive import remote_script, safe_component


def test_safe_component_rejects_shell_and_path_syntax():
    assert safe_component("heldout-20260827_01", "session") == "heldout-20260827_01"
    for value in ("../escape", "has space", "x;touch", ""):
        with pytest.raises(ValueError):
            safe_component(value, "session")


def test_heldout_remote_script_requires_reposition_and_records_episode_manifest():
    script = remote_script(
        "/home/pi/SO101-Bimanual-Manipulation",
        "20260827_top_validation_01",
        "02_light_wrinkle",
        "validation",
    )
    assert 'if [ "$split" != train ] && [ "$answer" != moved ]' in script
    assert 'record_kind": "towel_capture_episode"' in script
    assert '"episode_id": episode_id' in script
    assert '"physical_reposition_confirmed": split != "train"' in script
    assert "independent_physical_reposition_per_frame_v1" in script


def test_remote_script_pins_session_split_and_resolution():
    script = remote_script(
        "/repo",
        "session-01",
        "00_empty_table",
        "test",
    )
    assert 'split=test' in script
    assert 'image.shape[:2] != (960, 1280)' in script
    assert 'session metadata mismatch' in script


def test_episode_burst_keeps_one_episode_id_for_three_settled_frames():
    script = remote_script(
        "/repo",
        "lifecycle-validation-01",
        "01_flat",
        "validation",
        frames_per_episode=3,
    )
    assert "independent_physical_reposition_episode_burst_v1" in script
    assert "frames_per_episode=3" in script
    assert 'else f"{episode_id}-frame-{frame_index:02d}"' in script
    assert '"episode_id": episode_id' in script
