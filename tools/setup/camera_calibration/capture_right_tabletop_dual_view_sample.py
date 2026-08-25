#!/usr/bin/env python3
"""Capture one stationary Top/right-wrist view of the tabletop GridBoard.

Simultaneous capture remains available, but a staged Top-then-wrist mode avoids
the structural Top-camera occlusion caused by the arm.  In staged mode the
board must remain rigidly fixed between stages.  The wrist stage is accepted
only while the resident adapter proves an armed right-arm hold with the
requested owner and epoch.  This tool never sends a motion request.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import time
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
    qos_profile_sensor_data,
)
from sensor_msgs.msg import Image, JointState
from std_srvs.srv import Trigger

from capture_top_eye_to_hand_sample import (
    RESIDENT_ANCHOR_TOPIC,
    RESIDENT_STATUS_SERVICE,
    resident_torque_hold_matches,
)
from capture_wrist_eye_in_hand_sample import (
    EXPECTED_MARKER_IDS,
    decode_image,
    detect_expected_gridboard,
    ordered_arm_positions,
    stamp_seconds,
)


STATUS = "RIGHT_TABLETOP_DUAL_VIEW_RESIDENT_HOLD_CAPTURE_PASS"
TOP_STAGE_STATUS = "RIGHT_TABLETOP_TOP_STAGE_CAPTURE_PASS"
STAGED_RECORD_KIND = "right_tabletop_staged_capture"
STATIONARY_BOARD_CONFIRMATION = "RIGHT_TABLETOP_BOARD_FIXED_BETWEEN_STAGES"
TOP_TOPIC = "/camera/top/image_raw"
WRIST_TOPIC = "/camera/wrist_b/image_raw"


def target_document() -> dict:
    return {
        "dictionary": "DICT_4X4_50",
        "markers_x": 4,
        "markers_y": 5,
        "marker_length_m": 0.020,
        "marker_separation_m": 0.005,
        "first_marker_id": 10,
    }


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


class RightTabletopDualViewCapture(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("right_tabletop_dual_view_capture")
        self._args = args
        self._latest_top: Image | None = None
        self._latest_wrist: Image | None = None
        self._latest_top_received_monotonic = -math.inf
        self._latest_wrist_received_monotonic = -math.inf
        self._latest_anchor: JointState | None = None
        self._resident_status: dict | None = None
        self._resident_status_monotonic = -math.inf
        self._resident_status_future = None
        self._last_capture_monotonic = -math.inf
        self._last_pair_stamps: tuple[float, float] | None = None
        self._top_images: list[np.ndarray] = []
        self._wrist_images: list[np.ndarray] = []
        self._top_stamps: list[float] = []
        self._wrist_stamps: list[float] = []
        self._joint_positions: list[np.ndarray] = []
        self._pair_skews: list[float] = []
        self._finished = False
        self._frames_seen = {"top": 0, "wrist": 0}
        self._rejections = {
            "waiting_for_hold": 0,
            "waiting_for_anchor": 0,
            "stamp_skew": 0,
            "stale_camera_frame": 0,
            "reused_camera_frame": 0,
            "top_markers": 0,
            "wrist_markers": 0,
        }

        anchor_qos = QoSProfile(depth=1)
        anchor_qos.reliability = ReliabilityPolicy.RELIABLE
        anchor_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            JointState,
            RESIDENT_ANCHOR_TOPIC,
            self._on_anchor,
            anchor_qos,
        )
        self.create_subscription(
            Image,
            args.top_topic,
            self._on_top,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            args.wrist_topic,
            self._on_wrist,
            qos_profile_sensor_data,
        )
        self._status_client = self.create_client(Trigger, RESIDENT_STATUS_SERVICE)
        if not self._status_client.wait_for_service(
            timeout_sec=min(args.timeout, 2.0)
        ):
            raise RuntimeError(
                f"resident status service unavailable: {RESIDENT_STATUS_SERVICE}"
            )
        self.create_timer(0.1, self._request_status)

    @property
    def finished(self) -> bool:
        return self._finished

    def _on_anchor(self, message: JointState) -> None:
        try:
            ordered_arm_positions(message, "right")
        except ValueError as error:
            self.get_logger().warning(str(error))
            return
        self._latest_anchor = message
        self._try_capture()

    def _on_top(self, message: Image) -> None:
        self._frames_seen["top"] += 1
        self._latest_top = message
        self._latest_top_received_monotonic = time.monotonic()
        self._try_capture()

    def _on_wrist(self, message: Image) -> None:
        self._frames_seen["wrist"] += 1
        self._latest_wrist = message
        self._latest_wrist_received_monotonic = time.monotonic()
        self._try_capture()

    def _request_status(self) -> None:
        future = self._resident_status_future
        if future is not None:
            if not future.done():
                return
            try:
                response = future.result()
                document = json.loads(response.message)
                self._resident_status = (
                    document
                    if response.success
                    and resident_torque_hold_matches(
                        document,
                        required_owner=self._args.resident_required_owner,
                        required_epoch=self._args.resident_required_epoch,
                    )
                    else None
                )
                self._resident_status_monotonic = time.monotonic()
            except Exception:  # noqa: BLE001 - timeout diagnostic carries state
                self._resident_status = None
            self._resident_status_future = None
        self._resident_status_future = self._status_client.call_async(
            Trigger.Request()
        )

    def _try_capture(self) -> None:
        if self._finished or self._latest_top is None or self._latest_wrist is None:
            return
        if (
            self._resident_status is None
            or time.monotonic() - self._resident_status_monotonic > 0.5
        ):
            self._rejections["waiting_for_hold"] += 1
            return
        if self._latest_anchor is None:
            self._rejections["waiting_for_anchor"] += 1
            return
        now = time.monotonic()
        if now - self._last_capture_monotonic < self._args.interval:
            return
        if (
            now - self._latest_top_received_monotonic > 0.5
            or now - self._latest_wrist_received_monotonic > 0.5
        ):
            self._rejections["stale_camera_frame"] += 1
            return

        top_stamp = stamp_seconds(self._latest_top)
        wrist_stamp = stamp_seconds(self._latest_wrist)
        if top_stamp <= 0.0 or wrist_stamp <= 0.0:
            return
        pair_stamps = (top_stamp, wrist_stamp)
        if pair_stamps == self._last_pair_stamps:
            return
        if self._last_pair_stamps is not None and (
            top_stamp == self._last_pair_stamps[0]
            or wrist_stamp == self._last_pair_stamps[1]
        ):
            self._rejections["reused_camera_frame"] += 1
            return
        skew = abs(top_stamp - wrist_stamp)
        if skew > self._args.max_pair_skew:
            self._rejections["stamp_skew"] += 1
            return

        try:
            top_image = decode_image(self._latest_top)
            wrist_image = decode_image(self._latest_wrist)
            positions = ordered_arm_positions(self._latest_anchor, "right")
        except ValueError as error:
            self.get_logger().warning(str(error))
            return
        top_ids = detect_expected_gridboard(top_image)
        if top_ids != EXPECTED_MARKER_IDS:
            self._rejections["top_markers"] += 1
            return
        wrist_ids = detect_expected_gridboard(wrist_image)
        if wrist_ids != EXPECTED_MARKER_IDS:
            self._rejections["wrist_markers"] += 1
            return

        self._top_images.append(top_image)
        self._wrist_images.append(wrist_image)
        self._top_stamps.append(top_stamp)
        self._wrist_stamps.append(wrist_stamp)
        self._joint_positions.append(positions)
        self._pair_skews.append(skew)
        self._last_pair_stamps = pair_stamps
        self._last_capture_monotonic = now
        self.get_logger().info(
            "RIGHT_TABLETOP_DUAL_VIEW_FRAME_ACCEPTED "
            f"count={len(self._top_images)}/{self._args.frames} "
            f"skew_ms={skew * 1000.0:.1f}"
        )
        if len(self._top_images) >= self._args.frames:
            self._write_capture()
            self._finished = True

    def _write_capture(self) -> None:
        positions = np.asarray(self._joint_positions, dtype=np.float64)
        span = np.ptp(positions, axis=0)
        maximum_span = float(np.max(span))
        if maximum_span > self._args.max_joint_span:
            raise RuntimeError(
                "right arm moved during dual-view capture: "
                f"max_joint_span_rad={maximum_span:.6f}"
            )
        status = self._resident_status
        if status is None:
            raise RuntimeError("resident torque hold disappeared before write")

        output = self._args.output_directory.resolve()
        output.mkdir(parents=True, exist_ok=False)
        top_files: list[str] = []
        wrist_files: list[str] = []
        for index, (top_image, wrist_image) in enumerate(
            zip(self._top_images, self._wrist_images, strict=True)
        ):
            top_path = output / f"top_frame_{index:03d}.png"
            wrist_path = output / f"wrist_frame_{index:03d}.png"
            if not cv2.imwrite(str(top_path), top_image):
                raise RuntimeError(f"failed to write {top_path}")
            if not cv2.imwrite(str(wrist_path), wrist_image):
                raise RuntimeError(f"failed to write {wrist_path}")
            top_files.append(top_path.name)
            wrist_files.append(wrist_path.name)

        document = {
            "schema_version": 1,
            "record_kind": "right_tabletop_dual_view_capture",
            "status": STATUS,
            "motion_authorized": False,
            "source_motion_authorized": True,
            "robot_target_available": False,
            "capture": {
                "id": self._args.capture_id,
                "arm": "right",
                "measured_arm_rad": [
                    float(value) for value in np.median(positions, axis=0)
                ],
                "joint_span_rad": [float(value) for value in span],
                "top_image_files": top_files,
                "wrist_image_files": wrist_files,
                "top_source_stamps": self._top_stamps,
                "wrist_source_stamps": self._wrist_stamps,
                "maximum_pair_skew_s": max(self._pair_skews),
                "pair_skew_limit_s": self._args.max_pair_skew,
                "detected_marker_ids": list(EXPECTED_MARKER_IDS),
                "joint_state_source": "resident_terminal_measured_anchor",
            },
            "target": target_document(),
            "resident_torque_hold": {
                "status_service": RESIDENT_STATUS_SERVICE,
                "owner": status["owner"],
                "arbiter_epoch": int(status["arbiter_epoch"]),
                "torque_hold_active": True,
                "terminal_anchor_stamp": stamp_seconds(self._latest_anchor),
                "required_owner": self._args.resident_required_owner,
                "required_epoch": self._args.resident_required_epoch,
            },
        }
        capture_path = output / "capture.yaml"
        capture_path.write_text(
            yaml.safe_dump(document, sort_keys=False), encoding="utf-8"
        )
        print(
            f"{STATUS} id={self._args.capture_id} "
            f"frames={len(self._top_images)} "
            f"max_joint_span_rad={maximum_span:.6f} "
            f"max_pair_skew_ms={max(self._pair_skews) * 1000.0:.1f} "
            f"output={capture_path}"
        )

    def timeout_diagnostic(self) -> str:
        return (
            "RIGHT_TABLETOP_DUAL_VIEW_CAPTURE_TIMEOUT "
            f"top_frames={self._frames_seen['top']} "
            f"wrist_frames={self._frames_seen['wrist']} "
            f"accepted={len(self._top_images)} "
            + " ".join(f"{key}={value}" for key, value in self._rejections.items())
        )


class RightTabletopTopStageCapture(Node):
    """Capture the unobstructed Top view before moving the right arm."""

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("right_tabletop_top_stage_capture")
        self._args = args
        self._last_capture_monotonic = -math.inf
        self._last_stamp: float | None = None
        self._images: list[np.ndarray] = []
        self._stamps: list[float] = []
        self._frames_seen = 0
        self._marker_rejections = 0
        self._finished = False
        self.create_subscription(
            Image,
            args.top_topic,
            self._on_top,
            qos_profile_sensor_data,
        )

    @property
    def finished(self) -> bool:
        return self._finished

    def _on_top(self, message: Image) -> None:
        self._frames_seen += 1
        now = time.monotonic()
        if now - self._last_capture_monotonic < self._args.interval:
            return
        stamp = stamp_seconds(message)
        if stamp <= 0.0 or stamp == self._last_stamp:
            return
        try:
            image = decode_image(message)
        except ValueError as error:
            self.get_logger().warning(str(error))
            return
        if detect_expected_gridboard(image) != EXPECTED_MARKER_IDS:
            self._marker_rejections += 1
            return
        self._images.append(image)
        self._stamps.append(stamp)
        self._last_stamp = stamp
        self._last_capture_monotonic = now
        self.get_logger().info(
            "RIGHT_TABLETOP_TOP_STAGE_FRAME_ACCEPTED "
            f"count={len(self._images)}/{self._args.frames}"
        )
        if len(self._images) >= self._args.frames:
            self._write_capture()
            self._finished = True

    def _write_capture(self) -> None:
        output = self._args.output_directory.resolve()
        output.mkdir(parents=True, exist_ok=False)
        files: list[str] = []
        for index, image in enumerate(self._images):
            path = output / f"top_frame_{index:03d}.png"
            if not cv2.imwrite(str(path), image):
                raise RuntimeError(f"failed to write {path}")
            files.append(path.name)
        document = {
            "schema_version": 1,
            "record_kind": "right_tabletop_top_stage_capture",
            "status": TOP_STAGE_STATUS,
            "motion_authorized": False,
            "robot_target_available": False,
            "capture": {
                "id": self._args.capture_id,
                "top_image_files": files,
                "top_source_stamps": self._stamps,
                "detected_marker_ids": list(EXPECTED_MARKER_IDS),
            },
            "target": target_document(),
            "stationary_board_contract": {
                "confirmation": self._args.stationary_board_confirmation,
                "instruction": (
                    "do not move the board until wrist-finalize completes"
                ),
            },
        }
        path = output / "top_stage.yaml"
        path.write_text(yaml.safe_dump(document, sort_keys=False), encoding="utf-8")
        print(
            f"{TOP_STAGE_STATUS} id={self._args.capture_id} "
            f"frames={len(self._images)} output={path}"
        )

    def timeout_diagnostic(self) -> str:
        return (
            "RIGHT_TABLETOP_TOP_STAGE_CAPTURE_TIMEOUT "
            f"frames_received={self._frames_seen} "
            f"accepted={len(self._images)} "
            f"marker_rejections={self._marker_rejections}"
        )


class RightTabletopWristFinalizeCapture(Node):
    """Finalize a fixed-board Top stage with held wrist images and anchor."""

    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__("right_tabletop_wrist_finalize_capture")
        self._args = args
        self._top_stage_path = args.output_directory.resolve() / "top_stage.yaml"
        self._top_stage = yaml.safe_load(
            self._top_stage_path.read_text(encoding="utf-8")
        )
        self._validate_top_stage()
        self._latest_anchor: JointState | None = None
        self._resident_status: dict | None = None
        self._resident_status_monotonic = -math.inf
        self._resident_status_future = None
        self._last_capture_monotonic = -math.inf
        self._last_stamp: float | None = None
        self._images: list[np.ndarray] = []
        self._stamps: list[float] = []
        self._joint_positions: list[np.ndarray] = []
        self._frames_seen = 0
        self._rejections = {
            "waiting_for_hold": 0,
            "waiting_for_anchor": 0,
            "wrist_markers": 0,
        }
        self._finished = False

        anchor_qos = QoSProfile(depth=1)
        anchor_qos.reliability = ReliabilityPolicy.RELIABLE
        anchor_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(
            JointState,
            RESIDENT_ANCHOR_TOPIC,
            self._on_anchor,
            anchor_qos,
        )
        self.create_subscription(
            Image,
            args.wrist_topic,
            self._on_wrist,
            qos_profile_sensor_data,
        )
        self._status_client = self.create_client(Trigger, RESIDENT_STATUS_SERVICE)
        if not self._status_client.wait_for_service(
            timeout_sec=min(args.timeout, 2.0)
        ):
            raise RuntimeError(
                f"resident status service unavailable: {RESIDENT_STATUS_SERVICE}"
            )
        self.create_timer(0.1, self._request_status)

    def _validate_top_stage(self) -> None:
        capture = self._top_stage.get("capture", {})
        contract = self._top_stage.get("stationary_board_contract", {})
        if (
            self._top_stage.get("schema_version") != 1
            or self._top_stage.get("status") != TOP_STAGE_STATUS
            or capture.get("id") != self._args.capture_id
            or len(capture.get("top_image_files", [])) < 5
            or contract.get("confirmation") != STATIONARY_BOARD_CONFIRMATION
            or self._args.stationary_board_confirmation
            != STATIONARY_BOARD_CONFIRMATION
        ):
            raise RuntimeError("invalid or mismatched staged Top capture")
        if self._top_stage.get("target") != target_document():
            raise RuntimeError("staged Top target contract changed")
        for name in capture["top_image_files"]:
            if not (self._top_stage_path.parent / name).is_file():
                raise RuntimeError(f"missing staged Top image: {name}")

    @property
    def finished(self) -> bool:
        return self._finished

    def _on_anchor(self, message: JointState) -> None:
        try:
            ordered_arm_positions(message, "right")
        except ValueError as error:
            self.get_logger().warning(str(error))
            return
        self._latest_anchor = message

    def _request_status(self) -> None:
        future = self._resident_status_future
        if future is not None:
            if not future.done():
                return
            try:
                response = future.result()
                document = json.loads(response.message)
                self._resident_status = (
                    document
                    if response.success
                    and resident_torque_hold_matches(
                        document,
                        required_owner=self._args.resident_required_owner,
                        required_epoch=self._args.resident_required_epoch,
                    )
                    else None
                )
                self._resident_status_monotonic = time.monotonic()
            except Exception:  # noqa: BLE001 - diagnostic is fail-closed
                self._resident_status = None
            self._resident_status_future = None
        self._resident_status_future = self._status_client.call_async(
            Trigger.Request()
        )

    def _on_wrist(self, message: Image) -> None:
        self._frames_seen += 1
        now = time.monotonic()
        if (
            self._resident_status is None
            or now - self._resident_status_monotonic > 0.5
        ):
            self._rejections["waiting_for_hold"] += 1
            return
        if self._latest_anchor is None:
            self._rejections["waiting_for_anchor"] += 1
            return
        if now - self._last_capture_monotonic < self._args.interval:
            return
        stamp = stamp_seconds(message)
        if stamp <= 0.0 or stamp == self._last_stamp:
            return
        top_stamps = self._top_stage["capture"]["top_source_stamps"]
        if stamp <= max(float(value) for value in top_stamps):
            return
        try:
            image = decode_image(message)
            positions = ordered_arm_positions(self._latest_anchor, "right")
        except ValueError as error:
            self.get_logger().warning(str(error))
            return
        if detect_expected_gridboard(image) != EXPECTED_MARKER_IDS:
            self._rejections["wrist_markers"] += 1
            return
        self._images.append(image)
        self._stamps.append(stamp)
        self._joint_positions.append(positions)
        self._last_stamp = stamp
        self._last_capture_monotonic = now
        self.get_logger().info(
            "RIGHT_TABLETOP_WRIST_FINALIZE_FRAME_ACCEPTED "
            f"count={len(self._images)}/{self._args.frames}"
        )
        if len(self._images) >= self._args.frames:
            self._write_capture()
            self._finished = True

    def _write_capture(self) -> None:
        positions = np.asarray(self._joint_positions, dtype=np.float64)
        span = np.ptp(positions, axis=0)
        maximum_span = float(np.max(span))
        if maximum_span > self._args.max_joint_span:
            raise RuntimeError(
                "right arm moved during wrist-finalize capture: "
                f"max_joint_span_rad={maximum_span:.6f}"
            )
        status = self._resident_status
        if status is None:
            raise RuntimeError("resident torque hold disappeared before write")
        output = self._args.output_directory.resolve()
        wrist_files: list[str] = []
        for index, image in enumerate(self._images):
            path = output / f"wrist_frame_{index:03d}.png"
            if not cv2.imwrite(str(path), image):
                raise RuntimeError(f"failed to write {path}")
            wrist_files.append(path.name)
        top_capture = self._top_stage["capture"]
        document = {
            "schema_version": 1,
            "record_kind": STAGED_RECORD_KIND,
            "status": STATUS,
            "motion_authorized": False,
            "source_motion_authorized": True,
            "robot_target_available": False,
            "capture": {
                "id": self._args.capture_id,
                "arm": "right",
                "capture_mode": "staged_top_then_wrist",
                "measured_arm_rad": [
                    float(value) for value in np.median(positions, axis=0)
                ],
                "joint_span_rad": [float(value) for value in span],
                "top_image_files": list(top_capture["top_image_files"]),
                "wrist_image_files": wrist_files,
                "top_source_stamps": list(top_capture["top_source_stamps"]),
                "wrist_source_stamps": self._stamps,
                "detected_marker_ids": list(EXPECTED_MARKER_IDS),
                "joint_state_source": "resident_terminal_measured_anchor",
            },
            "target": target_document(),
            "resident_torque_hold": {
                "status_service": RESIDENT_STATUS_SERVICE,
                "owner": status["owner"],
                "arbiter_epoch": int(status["arbiter_epoch"]),
                "torque_hold_active": True,
                "terminal_anchor_stamp": stamp_seconds(self._latest_anchor),
                "required_owner": self._args.resident_required_owner,
                "required_epoch": self._args.resident_required_epoch,
            },
            "staged_capture": {
                "top_stage_file": self._top_stage_path.name,
                "top_stage_sha256": sha256_file(self._top_stage_path),
                "stationary_board_confirmation": (
                    self._args.stationary_board_confirmation
                ),
                "top_source_stamp_last": max(top_capture["top_source_stamps"]),
                "wrist_source_stamp_first": min(self._stamps),
                "top_completed_before_wrist": True,
            },
        }
        capture_path = output / "capture.yaml"
        capture_path.write_text(
            yaml.safe_dump(document, sort_keys=False), encoding="utf-8"
        )
        print(
            "RIGHT_TABLETOP_STAGED_RESIDENT_HOLD_CAPTURE_PASS "
            f"id={self._args.capture_id} frames={len(self._images)} "
            f"max_joint_span_rad={maximum_span:.6f} output={capture_path}"
        )

    def timeout_diagnostic(self) -> str:
        return (
            "RIGHT_TABLETOP_WRIST_FINALIZE_CAPTURE_TIMEOUT "
            f"frames_received={self._frames_seen} "
            f"accepted={len(self._images)} "
            + " ".join(f"{key}={value}" for key, value in self._rejections.items())
        )


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--capture-mode",
        choices=("simultaneous", "top-stage", "wrist-finalize"),
        default="simultaneous",
    )
    parser.add_argument("--capture-id", required=True)
    parser.add_argument("--output-directory", required=True, type=Path)
    parser.add_argument("--top-topic", default=TOP_TOPIC)
    parser.add_argument("--wrist-topic", default=WRIST_TOPIC)
    parser.add_argument("--resident-required-owner")
    parser.add_argument("--resident-required-epoch", type=int)
    parser.add_argument("--stationary-board-confirmation")
    parser.add_argument("--frames", type=int, default=5)
    parser.add_argument("--interval", type=float, default=0.2)
    parser.add_argument("--max-pair-skew", type=float, default=0.25)
    parser.add_argument("--max-joint-span", type=float, default=0.003)
    parser.add_argument("--timeout", type=float, default=45.0)
    args = parser.parse_args()
    if args.frames < 5:
        parser.error("--frames must be at least 5")
    if args.capture_mode in ("top-stage", "wrist-finalize") and (
        args.stationary_board_confirmation != STATIONARY_BOARD_CONFIRMATION
    ):
        parser.error(
            "staged capture requires --stationary-board-confirmation "
            f"{STATIONARY_BOARD_CONFIRMATION}"
        )
    if args.capture_mode != "top-stage" and (
        not args.resident_required_owner
        or args.resident_required_epoch is None
        or args.resident_required_epoch <= 0
    ):
        parser.error(
            "held capture requires --resident-required-owner and a positive "
            "--resident-required-epoch"
        )
    for name in ("interval", "max_pair_skew", "max_joint_span", "timeout"):
        if getattr(args, name) <= 0.0:
            parser.error(f"--{name.replace('_', '-')} must be positive")
    if args.capture_mode in ("simultaneous", "top-stage") and (
        args.output_directory.exists()
    ):
        parser.error(
            f"refusing to overwrite existing output: {args.output_directory}"
        )
    if args.capture_mode == "wrist-finalize" and (
        not args.output_directory.is_dir()
        or not (args.output_directory / "top_stage.yaml").is_file()
        or (args.output_directory / "capture.yaml").exists()
    ):
        parser.error(
            "wrist-finalize requires an unfinished output directory from "
            "top-stage"
        )
    return args


def main() -> int:
    args = parse_args()
    rclpy.init()
    if args.capture_mode == "top-stage":
        node = RightTabletopTopStageCapture(args)
    elif args.capture_mode == "wrist-finalize":
        node = RightTabletopWristFinalizeCapture(args)
    else:
        node = RightTabletopDualViewCapture(args)
    deadline = time.monotonic() + args.timeout
    try:
        while rclpy.ok() and not node.finished:
            if time.monotonic() >= deadline:
                print(node.timeout_diagnostic())
                raise RuntimeError(f"{args.capture_mode} capture timed out")
            rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
