from __future__ import annotations

from copy import deepcopy
from pathlib import Path

import pytest

from tools.lib.towel_schemas import (
    TowelSchemaError,
    load_json,
    load_schema,
    validate_instance,
)


ROOT = Path(__file__).resolve().parents[1]


def test_repository_examples_match_their_schemas():
    annotation_schema = load_schema(ROOT / "config/towel_annotation.schema.json")
    runtime_schema = load_schema(
        ROOT / "config/towel_state_observation.schema.json"
    )
    validate_instance(
        annotation_schema,
        load_json(ROOT / "config/towel_annotation.example.json"),
        label="annotation",
    )
    validate_instance(
        runtime_schema,
        load_json(ROOT / "config/towel_observation.example.json"),
        label="observation",
    )


def test_runtime_schema_rejects_motion_and_unknown_fields():
    schema = load_schema(ROOT / "config/towel_state_observation.schema.json")
    value = deepcopy(load_json(ROOT / "config/towel_observation.example.json"))
    value["motion_authorized"] = True
    with pytest.raises(TowelSchemaError, match="motion_authorized"):
        validate_instance(schema, value, label="unsafe observation")
