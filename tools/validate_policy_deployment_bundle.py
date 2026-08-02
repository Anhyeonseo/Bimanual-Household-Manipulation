#!/usr/bin/env python3
"""Validate a hash-pinned SO-101 policy ONNX deployment bundle."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
from pathlib import Path
import re
import sys
from typing import Any, Callable


BUNDLE_KIND = "so101_policy_onnx"
CONTRACT_KIND = "so101_policy_deployment"
OBSERVATION_KIND = "so101_policy_observation"
REQUIRED_SAFETY = {
    "deadline_miss": "reject",
    "stale_observation": "reject",
    "source_skew": "reject",
    "nonfinite_output": "reject",
    "out_of_bounds_output": "reject",
    "manifest_mismatch": "refuse_start",
}
HASH_PATTERN = re.compile(r"^[0-9a-f]{64}$")
ONNX_DTYPE_NAMES = {
    1: "float32",
    2: "uint8",
    6: "int32",
    7: "int64",
    9: "bool",
    10: "float16",
    11: "float64",
}


def load_json(path: Path) -> dict[str, Any]:
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


def require_object(document: dict[str, Any], key: str) -> dict[str, Any]:
    value = document.get(key)
    if not isinstance(value, dict):
        raise ValueError(f"{key} must be an object")
    return value


def require_nonempty_string(document: dict[str, Any], key: str) -> str:
    value = document.get(key)
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{key} must be a non-empty string")
    return value.strip()


def require_positive_number(document: dict[str, Any], key: str) -> float:
    value = document.get(key)
    if (
        not isinstance(value, (int, float))
        or isinstance(value, bool)
        or not math.isfinite(float(value))
        or float(value) <= 0.0
    ):
        raise ValueError(f"{key} must be a positive finite number")
    return float(value)


def require_positive_integer(document: dict[str, Any], key: str) -> int:
    value = document.get(key)
    if not isinstance(value, int) or isinstance(value, bool) or value <= 0:
        raise ValueError(f"{key} must be a positive integer")
    return value


def require_sha256(document: dict[str, Any], key: str) -> str:
    value = document.get(key)
    if not isinstance(value, str) or HASH_PATTERN.fullmatch(value) is None:
        raise ValueError(f"{key} must contain 64 lowercase hex characters")
    return value


def resolve_inside(manifest_path: Path, value: object, field: str) -> Path:
    if not isinstance(value, str) or not value:
        raise ValueError(f"{field} must be a non-empty relative path")
    relative = Path(value)
    if relative.is_absolute():
        raise ValueError(f"{field} must be relative to the bundle")
    root = manifest_path.parent.resolve()
    resolved = (root / relative).resolve()
    try:
        resolved.relative_to(root)
    except ValueError as error:
        raise ValueError(f"{field} must stay inside the bundle") from error
    return resolved


def validate_contract(contract: dict[str, Any]) -> None:
    if contract.get("schema_version") != 1:
        raise ValueError("deployment contract schema_version must be 1")
    if contract.get("contract_kind") != CONTRACT_KIND:
        raise ValueError(f"deployment contract_kind must be {CONTRACT_KIND}")
    if contract.get("bundle_kind") != BUNDLE_KIND:
        raise ValueError(f"deployment bundle_kind must be {BUNDLE_KIND}")
    if contract.get("motion_authorized") is not False:
        raise ValueError("deployment contract motion_authorized must be false")

    paths = require_object(contract, "paths")
    if paths.get("must_be_relative") is not True:
        raise ValueError("deployment contract must require relative paths")
    if paths.get("must_stay_inside_bundle") is not True:
        raise ValueError("deployment contract must confine bundle paths")

    onnx_contract = require_object(contract, "onnx")
    if onnx_contract.get("format") != "onnx":
        raise ValueError("deployment contract model format must be onnx")
    for key in (
        "require_exact_io_names_dtypes_shapes",
        "require_static_batch_one",
        "require_static_nonbatch_dimensions",
    ):
        if onnx_contract.get(key) is not True:
            raise ValueError(f"deployment contract must set onnx.{key}=true")
    dtypes = onnx_contract.get("allowed_dtypes")
    if not isinstance(dtypes, list) or not dtypes:
        raise ValueError("deployment contract allowed_dtypes must be non-empty")

    observation = require_object(contract, "observation")
    if observation.get("stale_action") != "reject":
        raise ValueError("deployment contract must reject stale observations")
    if observation.get("skew_action") != "reject":
        raise ValueError("deployment contract must reject source skew")
    modes = observation.get("allowed_modes")
    if not isinstance(modes, list) or not modes:
        raise ValueError("deployment contract allowed_modes must be non-empty")

    runtime = require_object(contract, "runtime")
    if runtime.get("required_mode") != "SHADOW":
        raise ValueError("deployment contract runtime mode must be SHADOW")
    if runtime.get("required_backend") != "onnxruntime_cpu":
        raise ValueError("deployment contract backend must be onnxruntime_cpu")
    if runtime.get("command_publications_allowed") is not False:
        raise ValueError("deployment contract must forbid command publications")
    minimum_fraction = require_positive_number(runtime, "minimum_rate_fraction")
    if minimum_fraction > 1.0:
        raise ValueError("minimum_rate_fraction must not exceed 1")
    require_positive_number(runtime, "maximum_inference_p95_ms")

    if require_object(contract, "safety") != REQUIRED_SAFETY:
        raise ValueError("deployment contract safety policy was weakened")
    action = require_object(contract, "action")
    representations = action.get("allowed_representations")
    if not isinstance(representations, list) or not representations:
        raise ValueError("deployment contract action representations are required")
    if action.get("require_order_scale_lower_upper_same_length") is not True:
        raise ValueError("deployment contract action vector lengths must match")

    provenance = require_object(contract, "provenance")
    required_hashes = provenance.get("required_sha256")
    if required_hashes != [
        "checkpoint_sha256",
        "training_config_sha256",
        "export_config_sha256",
    ]:
        raise ValueError("deployment contract provenance hashes are incomplete")


def validate_tensor_specs(
    specs: object,
    allowed_dtypes: set[str],
    field: str,
) -> list[dict[str, Any]]:
    if not isinstance(specs, list) or not specs:
        raise ValueError(f"{field} must be a non-empty list")
    normalized: list[dict[str, Any]] = []
    names: set[str] = set()
    for index, raw in enumerate(specs):
        if not isinstance(raw, dict):
            raise ValueError(f"{field}[{index}] must be an object")
        name = require_nonempty_string(raw, "name")
        if name in names:
            raise ValueError(f"{field} contains duplicate tensor name: {name}")
        names.add(name)
        dtype = require_nonempty_string(raw, "dtype")
        if dtype not in allowed_dtypes:
            raise ValueError(f"{field}.{name} dtype is not allowed: {dtype}")
        shape = raw.get("shape")
        if (
            not isinstance(shape, list)
            or not shape
            or any(
                not isinstance(value, int)
                or isinstance(value, bool)
                or value <= 0
                for value in shape
            )
        ):
            raise ValueError(f"{field}.{name} shape must contain positive integers")
        if shape[0] != 1:
            raise ValueError(f"{field}.{name} batch dimension must be 1")
        normalized.append({"name": name, "dtype": dtype, "shape": shape})
    return normalized


def _flat_dimension(shape: list[int]) -> int:
    result = 1
    for value in shape[1:]:
        result *= value
    return result


def _validate_numeric_vector(
    document: dict[str, Any],
    key: str,
    length: int,
    *,
    strictly_positive: bool = False,
) -> list[float]:
    values = document.get(key)
    if not isinstance(values, list) or len(values) != length:
        raise ValueError(f"{key} must contain {length} values")
    result: list[float] = []
    for value in values:
        if (
            not isinstance(value, (int, float))
            or isinstance(value, bool)
            or not math.isfinite(float(value))
        ):
            raise ValueError(f"{key} must contain finite numbers")
        number = float(value)
        if strictly_positive and number <= 0.0:
            raise ValueError(f"{key} values must be positive")
        result.append(number)
    return result


def validate_observation_contract(
    observation: dict[str, Any],
    deployment_contract: dict[str, Any],
) -> list[dict[str, Any]]:
    if observation.get("schema_version") != 1:
        raise ValueError("observation contract schema_version must be 1")
    if observation.get("contract_kind") != OBSERVATION_KIND:
        raise ValueError(f"observation contract_kind must be {OBSERVATION_KIND}")
    if observation.get("motion_authorized") is not False:
        raise ValueError("observation contract motion_authorized must be false")
    mode = require_nonempty_string(observation, "mode")
    allowed_modes = deployment_contract["observation"]["allowed_modes"]
    if mode not in allowed_modes:
        raise ValueError(f"observation mode is not allowed: {mode}")

    timestamp = require_object(observation, "timestamp")
    require_positive_number(timestamp, "max_age_ms")
    require_positive_number(timestamp, "max_source_skew_ms")
    if timestamp.get("stale_action") != "reject":
        raise ValueError("observation timestamp must reject stale inputs")
    if timestamp.get("skew_action") != "reject":
        raise ValueError("observation timestamp must reject source skew")

    allowed_dtypes = set(deployment_contract["onnx"]["allowed_dtypes"])
    specs = validate_tensor_specs(
        observation.get("inputs"),
        allowed_dtypes,
        "observation.inputs",
    )
    raw_inputs = observation["inputs"]
    source_kinds: set[str] = set()
    allowed_cameras = deployment_contract["observation"][
        "allowed_camera_order"
    ]
    observed_camera_order: list[str] = []
    for raw, spec in zip(raw_inputs, specs):
        source = require_object(raw, "source")
        kind = require_nonempty_string(source, "kind")
        source_kinds.add(kind)
        preprocessing = require_object(raw, "preprocessing")
        if kind == "structured_vector":
            features = source.get("ordered_features")
            if (
                not isinstance(features, list)
                or not features
                or any(not isinstance(value, str) or not value for value in features)
                or len(set(features)) != len(features)
            ):
                raise ValueError(
                    f"observation input {spec['name']} needs unique ordered_features"
                )
            if len(features) != _flat_dimension(spec["shape"]):
                raise ValueError(
                    f"observation input {spec['name']} feature count does not match shape"
                )
            kind_value = preprocessing.get("kind")
            if kind_value not in ("identity", "affine"):
                raise ValueError(
                    f"observation input {spec['name']} preprocessing must be identity or affine"
                )
            if kind_value == "affine":
                length = len(features)
                _validate_numeric_vector(preprocessing, "offset", length)
                _validate_numeric_vector(
                    preprocessing,
                    "scale",
                    length,
                    strictly_positive=True,
                )
        elif kind == "rgb_image":
            camera = require_nonempty_string(source, "camera")
            if camera not in allowed_cameras:
                raise ValueError(
                    f"observation input {spec['name']} camera is not allowed: {camera}"
                )
            observed_camera_order.append(camera)
            if source.get("encoding") != "rgb8":
                raise ValueError(
                    f"observation input {spec['name']} encoding must be rgb8"
                )
            layout = preprocessing.get("layout")
            resize_hw = preprocessing.get("resize_hw")
            if (
                layout not in ("NCHW", "NHWC")
                or not isinstance(resize_hw, list)
                or len(resize_hw) != 2
                or any(
                    not isinstance(value, int)
                    or isinstance(value, bool)
                    or value <= 0
                    for value in resize_hw
                )
            ):
                raise ValueError(
                    f"observation input {spec['name']} needs layout and resize_hw"
                )
            height, width = resize_hw
            expected_shape = (
                [1, 3, height, width]
                if layout == "NCHW"
                else [1, height, width, 3]
            )
            if spec["shape"] != expected_shape:
                raise ValueError(
                    f"observation input {spec['name']} image shape does not match resize/layout"
                )
            color_order = preprocessing.get("color_order")
            if color_order not in ("RGB", "BGR"):
                raise ValueError(
                    f"observation input {spec['name']} color_order must be RGB or BGR"
                )
            require_positive_number(preprocessing, "scale")
            _validate_numeric_vector(preprocessing, "mean", 3)
            _validate_numeric_vector(
                preprocessing,
                "std",
                3,
                strictly_positive=True,
            )
        else:
            raise ValueError(
                f"observation input {spec['name']} has unsupported source kind: {kind}"
            )

    if mode == "structured_state" and source_kinds != {"structured_vector"}:
        raise ValueError("structured_state may only contain structured_vector inputs")
    if mode == "rgb_tensor" and source_kinds != {"rgb_image"}:
        raise ValueError("rgb_tensor may only contain rgb_image inputs")
    if mode == "hybrid" and source_kinds != {"structured_vector", "rgb_image"}:
        raise ValueError("hybrid requires structured_vector and rgb_image inputs")
    if observed_camera_order:
        declared_order = observation.get("camera_order")
        if declared_order != observed_camera_order:
            raise ValueError("observation camera_order must match RGB input order")
    elif observation.get("camera_order") not in (None, []):
        raise ValueError("observation camera_order must be empty without RGB inputs")
    return specs


def inspect_onnx_model(model_path: Path) -> dict[str, Any]:
    try:
        import onnx
    except ImportError as error:
        raise RuntimeError(
            "onnx is required to inspect model I/O; use the validated export environment"
        ) from error

    model = onnx.load(str(model_path), load_external_data=True)
    onnx.checker.check_model(model)
    initializer_names = {initializer.name for initializer in model.graph.initializer}

    def tensor_spec(value_info: Any) -> dict[str, Any]:
        tensor_type = value_info.type.tensor_type
        dtype = ONNX_DTYPE_NAMES.get(int(tensor_type.elem_type))
        if dtype is None:
            raise ValueError(
                f"unsupported ONNX dtype for tensor {value_info.name}: "
                f"{tensor_type.elem_type}"
            )
        shape = []
        for dimension in tensor_type.shape.dim:
            if not dimension.HasField("dim_value") or dimension.dim_value <= 0:
                raise ValueError(
                    f"ONNX tensor {value_info.name} has a dynamic dimension"
                )
            shape.append(int(dimension.dim_value))
        return {"name": value_info.name, "dtype": dtype, "shape": shape}

    inputs = [
        tensor_spec(value)
        for value in model.graph.input
        if value.name not in initializer_names
    ]
    outputs = [tensor_spec(value) for value in model.graph.output]
    default_opsets = [
        int(item.version) for item in model.opset_import if item.domain in ("", "ai.onnx")
    ]
    if len(default_opsets) != 1:
        raise ValueError("ONNX model must declare exactly one default opset")
    return {
        "opset": default_opsets[0],
        "inputs": inputs,
        "outputs": outputs,
    }


def _compare_tensor_specs(
    declared: list[dict[str, Any]],
    actual: object,
    field: str,
) -> None:
    if actual != declared:
        raise ValueError(f"ONNX {field} do not match the deployment contract")


def validate_policy_deployment_bundle(
    manifest_path: Path,
    contract_path: Path,
    *,
    inspect_model: Callable[[Path], dict[str, Any]] = inspect_onnx_model,
) -> dict[str, Any]:
    manifest_path = manifest_path.resolve()
    contract_path = contract_path.resolve()
    manifest = load_json(manifest_path)
    contract = load_json(contract_path)
    validate_contract(contract)

    if manifest.get("schema_version") != 1:
        raise ValueError("bundle schema_version must be 1")
    if manifest.get("bundle_kind") != BUNDLE_KIND:
        raise ValueError(f"bundle_kind must be {BUNDLE_KIND}")
    bundle_id = require_nonempty_string(manifest, "bundle_id")
    if manifest.get("motion_authorized") is not False:
        raise ValueError("bundle motion_authorized must be false")
    if (
        require_sha256(manifest, "deployment_contract_sha256")
        != file_sha256(contract_path)
    ):
        raise ValueError("deployment_contract_sha256 mismatch")

    model = require_object(manifest, "model")
    if model.get("format") != contract["onnx"]["format"]:
        raise ValueError("model format must be onnx")
    model_path = resolve_inside(manifest_path, model.get("path"), "model.path")
    if not model_path.is_file():
        raise ValueError("model.path does not exist")
    model_hash = file_sha256(model_path)
    if require_sha256(model, "sha256") != model_hash:
        raise ValueError("model.sha256 mismatch")
    declared_opset = require_positive_integer(model, "opset")

    observation_ref = require_object(manifest, "observation_contract")
    observation_path = resolve_inside(
        manifest_path,
        observation_ref.get("path"),
        "observation_contract.path",
    )
    if not observation_path.is_file():
        raise ValueError("observation_contract.path does not exist")
    observation_hash = file_sha256(observation_path)
    if require_sha256(observation_ref, "sha256") != observation_hash:
        raise ValueError("observation_contract.sha256 mismatch")
    observation = load_json(observation_path)
    declared_inputs = validate_observation_contract(observation, contract)

    allowed_dtypes = set(contract["onnx"]["allowed_dtypes"])
    declared_outputs = validate_tensor_specs(
        model.get("outputs"),
        allowed_dtypes,
        "model.outputs",
    )
    actual_model = inspect_model(model_path)
    if actual_model.get("opset") != declared_opset:
        raise ValueError("ONNX opset does not match model.opset")
    _compare_tensor_specs(declared_inputs, actual_model.get("inputs"), "inputs")
    _compare_tensor_specs(declared_outputs, actual_model.get("outputs"), "outputs")

    runtime = require_object(manifest, "runtime")
    if runtime.get("mode") != contract["runtime"]["required_mode"]:
        raise ValueError("runtime.mode must be SHADOW")
    if runtime.get("backend") != contract["runtime"]["required_backend"]:
        raise ValueError("runtime.backend must be onnxruntime_cpu")
    if runtime.get("command_publications_allowed") is not False:
        raise ValueError("runtime must forbid command publications")
    control_dt_s = require_positive_number(runtime, "control_dt_s")
    target_hz = require_positive_number(runtime, "target_inference_hz")
    if not math.isclose(control_dt_s * target_hz, 1.0, rel_tol=1e-6):
        raise ValueError("runtime control_dt_s and target_inference_hz disagree")
    deadline_ms = require_positive_number(runtime, "deadline_ms")
    if deadline_ms > control_dt_s * 1000.0:
        raise ValueError("runtime deadline_ms must not exceed control_dt")
    require_positive_integer(runtime, "intra_op_threads")
    require_positive_integer(runtime, "inter_op_threads")

    if require_object(manifest, "safety") != REQUIRED_SAFETY:
        raise ValueError("bundle safety policy was weakened")

    action = require_object(manifest, "action")
    representation = require_nonempty_string(action, "representation")
    if representation not in contract["action"]["allowed_representations"]:
        raise ValueError(f"action representation is not allowed: {representation}")
    output_name = require_nonempty_string(action, "output_name")
    matching_outputs = [
        value for value in declared_outputs if value["name"] == output_name
    ]
    if len(matching_outputs) != 1:
        raise ValueError("action.output_name must identify one ONNX output")
    order = action.get("order")
    if (
        not isinstance(order, list)
        or not order
        or any(not isinstance(value, str) or not value for value in order)
        or len(set(order)) != len(order)
    ):
        raise ValueError("action.order must contain unique non-empty names")
    action_dimension = _flat_dimension(matching_outputs[0]["shape"])
    if len(order) != action_dimension:
        raise ValueError("action.order length does not match output shape")
    scale = _validate_numeric_vector(
        action,
        "scale",
        action_dimension,
        strictly_positive=True,
    )
    lower = _validate_numeric_vector(action, "lower", action_dimension)
    upper = _validate_numeric_vector(action, "upper", action_dimension)
    if any(low >= high for low, high in zip(lower, upper)):
        raise ValueError("every action.lower must be less than action.upper")
    if any(value <= 0.0 for value in scale):
        raise ValueError("every action.scale must be positive")

    provenance = require_object(manifest, "provenance")
    training_framework = require_nonempty_string(
        provenance,
        "training_framework",
    )
    for key in contract["provenance"]["required_sha256"]:
        require_sha256(provenance, key)

    return {
        "schema_version": 1,
        "status": "POLICY_DEPLOYMENT_BUNDLE_PASS",
        "passed": True,
        "motion_authorized": False,
        "robot_command_topics_created": 0,
        "command_publications_allowed": False,
        "bundle_id": bundle_id,
        "bundle_manifest": str(manifest_path),
        "bundle_manifest_sha256": file_sha256(manifest_path),
        "deployment_contract": str(contract_path),
        "deployment_contract_sha256": file_sha256(contract_path),
        "model": str(model_path),
        "model_sha256": model_hash,
        "model_opset": declared_opset,
        "observation_contract": str(observation_path),
        "observation_contract_sha256": observation_hash,
        "observation_mode": observation["mode"],
        "inputs": declared_inputs,
        "outputs": declared_outputs,
        "runtime": {
            "mode": "SHADOW",
            "backend": "onnxruntime_cpu",
            "control_dt_s": control_dt_s,
            "target_inference_hz": target_hz,
            "deadline_ms": deadline_ms,
            "intra_op_threads": runtime["intra_op_threads"],
            "inter_op_threads": runtime["inter_op_threads"],
        },
        "action": {
            "output_name": output_name,
            "representation": representation,
            "order": order,
            "scale": scale,
            "lower": lower,
            "upper": upper,
        },
        "training_framework": training_framework,
    }


def parse_args(argv: list[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Validate a fail-closed SO-101 policy ONNX bundle."
    )
    parser.add_argument("--manifest", required=True, type=Path)
    parser.add_argument(
        "--contract",
        type=Path,
        default=Path(__file__).parents[1]
        / "config"
        / "policy_deployment_contract.json",
    )
    parser.add_argument("--output", required=True, type=Path)
    return parser.parse_args(argv)


def main(argv: list[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        result = validate_policy_deployment_bundle(
            args.manifest,
            args.contract,
        )
        output = args.output.resolve()
        output.parent.mkdir(parents=True, exist_ok=True)
        temporary = output.with_suffix(output.suffix + ".tmp")
        temporary.write_text(
            json.dumps(result, indent=2, sort_keys=True) + "\n",
            encoding="utf-8",
        )
        temporary.replace(output)
        print(result["status"])
        print(f"POLICY_DEPLOYMENT_BUNDLE_ARTIFACT={output}")
        return 0
    except Exception as error:
        print(
            f"POLICY_DEPLOYMENT_BUNDLE_ERROR reason={error}",
            file=sys.stderr,
        )
        return 2


if __name__ == "__main__":
    raise SystemExit(main())
