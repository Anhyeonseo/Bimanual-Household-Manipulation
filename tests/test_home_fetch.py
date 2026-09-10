import copy
import json
from pathlib import Path

import pytest

from home_robot_tasks.cli import main
from home_robot_tasks.fetch import FetchRequest, InvalidTask, plan_fetch

ROOT = Path(__file__).resolve().parents[1]


@pytest.fixture
def world():
    return json.loads((ROOT / "config/home.example.json").read_text())


def request(**changes):
    return FetchRequest.from_dict({"task": "fetch_object", "object_id": "remote_control",
                                   "source_room": "living_room", "destination_place": "bedroom_drop_zone",
                                   **changes})


def test_fetch_requires_search_and_verification_on_both_sides_of_transport(world):
    plan = plan_fetch(request(), world)
    steps = plan["steps"]
    skills = [s["skill"] for s in steps]
    assert skills.index("search") < skills.index("pick") < skills.index("navigate_with_load")
    assert skills.index("place") < skills.index("verify_delivery")
    assert "grasp_verified" in steps[skills.index("pick")]["success_requires"]
    assert "load_retained" in steps[skills.index("navigate_with_load")]["success_requires"]
    assert "object_at_destination" in steps[-1]["success_requires"]
    assert all(s["timeout_s"] > 0 and s["max_attempts"] == 1 for s in steps)
    assert not plan["motion_authorized"] and not plan["executable"]
    assert not plan["physical_task_completed"]


@pytest.mark.parametrize("changes", [
    {"object_id": "unknown"}, {"source_room": "unknown"},
    {"destination_place": "bedroom"}, {"destination_place": "unknown"},
])
def test_ambiguous_or_unregistered_target_is_rejected(world, changes):
    with pytest.raises(InvalidTask):
        plan_fetch(request(**changes), world)


@pytest.mark.parametrize("value", [None, True, 3, [], "", " "])
def test_request_identifiers_are_not_guessed(value):
    with pytest.raises(InvalidTask):
        request(object_id=value)


def test_model_cannot_add_motion_permission_to_request():
    with pytest.raises(InvalidTask):
        request(motion_authorized=True)


def test_other_housework_is_not_misrepresented_as_fetch():
    with pytest.raises(InvalidTask):
        request(task="collect_laundry")


@pytest.mark.parametrize("locations", [[], ["coffee_table", "coffee_table"], ["bedroom_drop_zone"], [None]])
def test_search_has_finite_distinct_locations_in_the_source_room(world, locations):
    world["rooms"]["living_room"]["search_locations"] = locations
    with pytest.raises(InvalidTask):
        plan_fetch(request(), world)


def test_transport_choice_cannot_be_silently_ignored(world):
    world["transport_mode"] = "tray"
    with pytest.raises(InvalidTask):
        plan_fetch(request(), world)


def test_planning_preserves_input_and_does_not_claim_execution(world):
    initial = copy.deepcopy(world)
    assert plan_fetch(request(), world) == plan_fetch(request(), world)
    assert world == initial


def test_cli_emits_only_a_non_executable_plan(capsys):
    assert main(["--world", str(ROOT / "config/home.example.json"), "--request",
                 str(ROOT / "config/fetch_remote.example.json")]) == 0
    assert json.loads(capsys.readouterr().out)["mode"] == "plan_only"


def test_invalid_json_is_a_terminal_cli_error(tmp_path):
    broken = tmp_path / "broken.json"
    broken.write_text("{")
    with pytest.raises(SystemExit) as error:
        main(["--world", str(broken), "--request", str(broken)])
    assert error.value.code == 2


@pytest.mark.parametrize("room", [None, [], {}, True, 42, "unknown"])
def test_malformed_destination_room_is_rejected(world, room):
    world["places"]["bedroom_drop_zone"]["room"] = room
    with pytest.raises(InvalidTask):
        plan_fetch(request(), world)
