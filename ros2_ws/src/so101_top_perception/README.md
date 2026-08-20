# SO-101 Top Perception

`so101_top_perception` converts `/camera/top/image_raw` frames into a
board-relative observation of exactly one dark planar object.

The node is intentionally fail-closed:

- a pose is published only when the detected object's center lies inside the
  calibrated board span;
- dark contours whose complete footprint is outside the calibrated board span
  are ignored before object counting; contours intersecting the boundary
  remain blocking observations;
- an object's center must remain in the calibrated planar region, while the
  complete contour must remain at least `image_edge_margin_px` from the camera
  image edge;
- a long object may extend beyond the small calibration rectangle when its
  center is calibrated and the full object is visible; robot workspace checks
  apply to the grasp point rather than the complete object footprint;
- source timestamps must be present, sufficiently fresh, and not excessively
  in the future;
- camera resolution and the camera-info SHA-256 recorded by the homography
  must match;
- `motion_authorized` and `robot_target_available` are always `false`;
- the output is an observation in `top_board`, never a robot/base target.
- the output carries the detected center and source image size in raw pixels; arm routing uses these pixels directly rather than inferring image left/right from board axes.

`exclusion_rectangles_px` contains flattened `x,y,width,height` groups in raw
image pixels. The current lower-left rectangle masks only the fixed left-arm
footprint during target lock. It is not an occlusion tracker: after the target
is locked, approach motion must not replace the locked target with a new dark
contour observation.

## Topics

- input: `/camera/top/image_raw` (`sensor_msgs/msg/Image`, Sensor Data QoS)
- valid output: `/perception/top/object_pose_board`
  (`so101_interfaces/msg/TopObjectPose`, volatile depth 1)
- status: `/perception/top/diagnostics`
  (`diagnostic_msgs/msg/DiagnosticArray`)

## Run

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 launch so101_top_perception top_perception.launch.py
```

The default launch file loads `top_camera_info.yaml` and
`top_worktable_homography.yaml` from `manipulation_camera_manager`.

## YOLO-OBB runtime smoke

`so101_top_perception.obb_detector` provides a fail-closed OpenCV DNN runtime
for a hash-pinned, single-class Ultralytics OBB ONNX bundle. It preserves the
same calibrated-board, full-image-visibility and exactly-one-object contract
as the legacy detector. Pen yaw is an undirected long axis modulo pi; cap and
tip are intentionally not classified.

Candidate v3 passed the frozen 18-image holdout. The dedicated smoke launch
runs its hash-pinned ONNX bundle at a bounded rate while preserving
`motion_authorized=false` and `robot_target_available=false`:

```bash
python3 -m venv --system-site-packages \
  /home/pi/Manipulation/.venv-top-perception-opencv410
/home/pi/Manipulation/.venv-top-perception-opencv410/bin/python -m pip install \
  --require-hashes --no-deps \
  -r /home/pi/Manipulation/requirements/top-perception-runtime.txt

ros2 launch so101_top_perception top_obb_runtime_smoke.launch.py \
  python_executable:=/home/pi/Manipulation/.venv-top-perception-opencv410/bin/python \
  bundle_manifest:=/absolute/path/to/top_pen_yolo_obb_bundle.json \
  inference_hz:=4.0
```

The ordinary `top_perception.launch.py` still selects the legacy detector.
The OBB backend cannot become the operational default until the three-camera
Pi resource gate passes. Its diagnostics expose the pinned model and holdout
hashes, inference counts and bounded latency samples; no camera image is kept
by the runtime monitor. Ubuntu/ROS OpenCV 4.6 cannot parse the YOLO11 OBB
`Split` node, so the smoke launch requires the isolated, hash-pinned OpenCV
4.10 Python executable and never replaces the system OpenCV installation.

## Base-frame shadow target

`top_shadow.launch.py` adds a non-actionable `left_base_link` shadow output on
`/perception/top/object_shadow_left_base`. The node uses the current
two-position Planar GridBoard table registration, checks source freshness,
confidence, full camera visibility, calibrated center bounds, and conservative
left-arm grasp-point workspace bounds.

The table registration is validated, but the output remains deliberately
non-actionable: `motion_authorized` and `robot_target_available` are always
`false`. The historical `118.216 mm` disagreement came from mixing an obsolete
raised chessboard pose with a later eye-to-hand generation and is not used by
the current transform. The current conservative workspace is derived from the
approved low-grasp and pre-grasp joint-limit overlap. No MoveIt or hardware
command publisher exists in this package.
