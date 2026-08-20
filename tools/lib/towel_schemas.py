"""JSON Schema checks for towel dataset and runtime observation artifacts."""

from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Mapping

from jsonschema import Draft202012Validator
from jsonschema.exceptions import SchemaError, ValidationError


class TowelSchemaError(ValueError):
    """A schema or one of its instances is invalid."""


def load_json(path: Path) -> Any:
    try:
        return json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise TowelSchemaError(f"could not load JSON {path}: {exc}") from exc


def load_schema(path: Path) -> dict[str, Any]:
    value = load_json(path)
    if not isinstance(value, dict):
        raise TowelSchemaError(f"schema root must be an object: {path}")
    try:
        Draft202012Validator.check_schema(value)
    except SchemaError as exc:
        raise TowelSchemaError(f"invalid JSON Schema {path}: {exc.message}") from exc
    return value


def validate_instance(
    schema: Mapping[str, Any], instance: Any, *, label: str
) -> None:
    try:
        Draft202012Validator(schema).validate(instance)
    except ValidationError as exc:
        location = "/".join(str(part) for part in exc.absolute_path) or "<root>"
        raise TowelSchemaError(
            f"{label} violates schema at {location}: {exc.message}"
        ) from exc
