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

## YOLO-OBB offline candidate

`so101_top_perception.obb_detector` provides a fail-closed OpenCV DNN runtime
for a hash-pinned, single-class Ultralytics OBB ONNX bundle. It preserves the
same calibrated-board, full-image-visibility and exactly-one-object contract
as the legacy detector. Pen yaw is an undirected long axis modulo pi; cap and
tip are intentionally not classified.

This backend is currently offline-only. Training uses the optional desktop
dependency in `requirements-training.txt`; Pi inference requires only the
existing OpenCV runtime. It must pass the frozen 18-image holdout and Pi
resource gate before it can replace the launch-time legacy detector.

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
approved low-grasp and pre-grasp joint-limit overlap; the camera-visible pen
used on 2026-07-30 is outside that hardware workspace. No MoveIt or hardware
command publisher exists in this package.

