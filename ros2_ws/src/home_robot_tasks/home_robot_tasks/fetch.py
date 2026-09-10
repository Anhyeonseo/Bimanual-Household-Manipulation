"""Compile a household fetch request into a bounded, non-executable task plan.

Rooms and locations are semantic names, not navigation poses. A physical
executor must resolve them against a commissioned map and fresh observations.
"""
from __future__ import annotations

from dataclasses import asdict, dataclass
from typing import Any, Mapping


class InvalidTask(ValueError):
    """The request or world description cannot define an unambiguous plan."""


def _identifier(value: Any, field: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise InvalidTask(f"{field} must be a non-empty string")
    return value.strip()


def _mapping(value: Any, field: str) -> Mapping[str, Any]:
    if not isinstance(value, dict) or not value:
        raise InvalidTask(f"{field} must be a non-empty object")
    return value


@dataclass(frozen=True)
class FetchRequest:
    object_id: str
    source_room: str
    destination_place: str

    @classmethod
    def from_dict(cls, document: Mapping[str, Any]) -> FetchRequest:
        if not isinstance(document, dict):
            raise InvalidTask("request must be an object")
        if document.get("task") != "fetch_object":
            raise InvalidTask("only fetch_object is implemented")
        expected = {"task", "object_id", "source_room", "destination_place"}
        if set(document) != expected:
            raise InvalidTask("request fields must be task, object_id, source_room, destination_place")
        return cls(*(_identifier(document[key], key) for key in
                     ("object_id", "source_room", "destination_place")))


@dataclass(frozen=True)
class TaskStep:
    skill: str
    parameters: dict[str, Any]
    success_requires: tuple[str, ...]
    timeout_s: int
    max_attempts: int = 1


def plan_fetch(request: FetchRequest, world: Mapping[str, Any]) -> dict[str, Any]:
    if not isinstance(world, dict) or type(world.get("schema_version")) is not int or world["schema_version"] != 1:
        raise InvalidTask("world schema_version must be 1")
    if world.get("transport_mode") != "carry_in_gripper":
        raise InvalidTask("this planning example supports carry_in_gripper only")
    rooms = _mapping(world.get("rooms"), "rooms")
    places = _mapping(world.get("places"), "places")
    objects = _mapping(world.get("objects"), "objects")
    object_id = _identifier(request.object_id, "object_id")
    room_id = _identifier(request.source_room, "source_room")
    destination = _identifier(request.destination_place, "destination_place")
    if object_id not in objects or not isinstance(objects[object_id], dict):
        raise InvalidTask(f"unknown object: {object_id}")
    if room_id not in rooms or not isinstance(rooms[room_id], dict):
        raise InvalidTask(f"unknown source room: {room_id}")
    place = places.get(destination)
    if not isinstance(place, dict):
        raise InvalidTask("destination must name a registered place with a known room")
    destination_room = _identifier(place.get("room"), "destination room")
    if destination_room not in rooms or not isinstance(rooms[destination_room], dict):
        raise InvalidTask("destination must belong to a known room")
    locations = rooms[room_id].get("search_locations")
    if not isinstance(locations, list) or not locations:
        raise InvalidTask("source room needs a finite list of search locations")
    locations = [_identifier(location, "search_location") for location in locations]
    if len(set(locations)) != len(locations):
        raise InvalidTask("search locations must be unique")
    if any(not isinstance(places.get(location), dict) or places[location].get("room") != room_id
           for location in locations):
        raise InvalidTask("each search location must belong to the source room")
    target = {"object_id": object_id}
    steps = (
        TaskStep("navigate", {"room": room_id}, ("localized_in_source_room", "base_stopped"), 180),
        TaskStep("search", {**target, "locations": locations},
                 ("target_observed", "fresh_observation", "target_pose_resolved"), 120),
        TaskStep("pick", target, ("base_stopped", "grasp_verified"), 60),
        TaskStep("prepare_transport", target, ("arm_in_transport_pose", "load_retained"), 30),
        TaskStep("navigate_with_load", {"destination_place": destination},
                 ("localized_at_destination", "load_retained", "base_stopped"), 180),
        TaskStep("place", {**target, "destination_place": destination},
                 ("base_stopped", "release_verified", "arm_clear"), 60),
        TaskStep("verify_delivery", {**target, "destination_place": destination},
                 ("fresh_observation", "object_at_destination"), 30),
    )
    return {
        "schema_version": 1,
        "task": "fetch_object",
        "request": asdict(FetchRequest(object_id, room_id, destination)),
        "mode": "plan_only",
        "motion_authorized": False,
        "executable": False,
        "physical_task_completed": False,
        "transport_mode": "carry_in_gripper",
        "steps": [asdict(step) for step in steps],
        "failure_policy": {
            "automatic_retry": False,
            "abort_on": ["timeout", "object_not_found", "stale_observation", "localization_lost",
                         "grasp_failed", "load_lost", "path_blocked", "release_failed", "cancelled"],
            "required_response": "stop_base_and_hold_arms_then_report",
        },
        "unimplemented_dependencies": ["navigation", "perception", "pick_and_place", "task_executor"],
    }
