#!/usr/bin/env python3
"""Build a hash-pinned Top-pen OBB training manifest from simple metadata."""

from __future__ import annotations

import argparse
import hashlib
import json
import sys
from pathlib import Path


YAW_SEMANTICS = "undirected_long_axis_modulo_pi"


def load_json(path: Path) -> dict:
    with path.open(encoding="utf-8") as stream:
        document = json.load(stream)
    if not isinstance(document, dict):
        raise ValueError(f"JSON root must be an object: {path}")
    return document


def file_sha256(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def resolve_under(root: Path, value: object, field: str) -> Path:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{field} must be a non-empty relative path")
    relative = Path(value)
    if relative.is_absolute():
        raise ValueError(f"{field} must be relative to the dataset root")
    resolved = (root / relative).resolve()
    try:
        resolved.relative_to(root)
    except ValueError as error:
        raise ValueError(f"{field} must stay inside the dataset root") from error
    return resolved


def build_manifest(dataset_root: Path, metadata_path: Path) -> dict:
    dataset_root = dataset_root.resolve()
    metadata = load_json(metadata_path.resolve())
    if metadata.get("protocol_version") != 1:
        raise ValueError("metadata protocol_version must be 1")
    dataset_id = metadata.get("dataset_id")
    if not isinstance(dataset_id, str) or not dataset_id.strip():
        raise ValueError("metadata dataset_id must be a non-empty string")
    data_yaml = metadata.get("ultralytics_data_yaml")
    data_yaml_path = resolve_under(
        dataset_root,
        data_yaml,
        "ultralytics_data_yaml",
    )
    if not data_yaml_path.is_file():
        raise ValueError(f"Ultralytics data YAML is missing: {data_yaml_path}")
    cases = metadata.get("cases")
    if not isinstance(cases, list) or not cases:
        raise ValueError("metadata cases must be a non-empty list")

    splits = {"train": [], "validation": []}
    identifiers = set()
    for case in cases:
        if not isinstance(case, dict):
            raise ValueError("every metadata case must be an object")
        identifier = case.get("id")
        if not isinstance(identifier, str) or not identifier:
            raise ValueError("every metadata case needs a non-empty id")
        if identifier in identifiers:
            raise ValueError(f"duplicate metadata case id: {identifier}")
        identifiers.add(identifier)
        split = case.get("split")
        if split not in splits:
            raise ValueError(f"case {identifier} split must be train or validation")
        expected_present = case.get("expected_present")
        if not isinstance(expected_present, bool):
            raise ValueError(
                f"case {identifier} expected_present must be boolean"
            )
        image_path = resolve_under(dataset_root, case.get("image"), "image")
        label_path = resolve_under(dataset_root, case.get("label"), "label")
        if not image_path.is_file() or not label_path.is_file():
            raise ValueError(f"case {identifier} image or label is missing")
        condition = case.get("condition")
        if not isinstance(condition, dict):
            raise ValueError(f"case {identifier} condition is required")
        normalized_condition = {}
        for key in ("background", "lighting", "glare"):
            value = condition.get(key)
            if not isinstance(value, str) or not value.strip():
                raise ValueError(f"case {identifier} condition.{key} is required")
            normalized_condition[key] = value.strip()
        splits[split].append(
            {
                "id": identifier,
                "image": str(image_path.relative_to(dataset_root)),
                "image_sha256": file_sha256(image_path),
                "label": str(label_path.relative_to(dataset_root)),
                "label_sha256": file_sha256(label_path),
                "expected_present": expected_present,
                "condition": normalized_condition,
            }
        )

    return {
        "protocol_version": 1,
        "dataset_id": dataset_id.strip(),
        "dataset_role": "training",
        "class_names": ["pen"],
        "yaw_semantics": YAW_SEMANTICS,
        "ultralytics_data_yaml": str(data_yaml_path.relative_to(dataset_root)),
        "splits": splits,
    }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Build a SHA-pinned Top-pen OBB training manifest."
    )
    parser.add_argument("--dataset-root", required=True, type=Path)
    parser.add_argument("--metadata", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        dataset_root = args.dataset_root.resolve()
        output = args.output.resolve()
        try:
            output.relative_to(dataset_root)
        except ValueError as error:
            raise ValueError("output must stay inside the dataset root") from error
        manifest = build_manifest(dataset_root, args.metadata)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(
            json.dumps(manifest, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        print("TOP_PEN_OBB_TRAINING_MANIFEST_BUILD_PASS")
        print(f"MANIFEST={output}")
        print(f"MANIFEST_SHA256={file_sha256(output)}")
        return 0
    except Exception as error:
        print(
            f"TOP_PEN_OBB_TRAINING_MANIFEST_BUILD_ERROR reason={error}",
            file=sys.stderr,
        )
        return 3


if __name__ == "__main__":
    raise SystemExit(main())
