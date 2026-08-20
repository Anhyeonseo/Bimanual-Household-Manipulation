# so101_description

Parameterized SO-101 description for the left-arm-first MoveIt and Isaac Sim integration.
The validated entrypoint is `urdf/so101_left.urdf.xacro`; its default prefix is `left_` and its fixed root is `workcell_base_link`.

The geometry and inertial data are derived from `TheRobotStudio/SO-ARM100`, file `Simulation/SO101/so101_new_calib.urdf`, pinned at commit `fda892cba81032c46c40976a48c9ceadbf40a9ca` under Apache-2.0.
This project changes link/joint names, arm joint signs, wrist-roll limits, and the gripper zero convention. The original `meshes/` STL files are unmodified copies from that pinned revision.

The physical left arm uses TheRobotStudio's Apache-2.0
`Optional/Wrist_Cam_Mount_32x32_UVC_Module` Wrist Roll replacement. Its official
`Wrist_Cam_Mount_32x32_UVC_Module_SO101.stl` is stored locally as
`meshes/wrist_cam_mount_32x32_uvc_module_so101.stl` with SHA-256
`b4345ccf23f1f2ed3f4885c205cac5afbed6ddd1b183617c4801751e3bafb7b4`.
The file is authored in millimetres, so the URDF applies a `0.001` mesh scale.
`so101_left.urdf.xacro` enables this replacement by default; pass
`use_wrist_camera_mount:=false` to restore the original Wrist Roll geometry.
This changes visual/collision geometry only and does not change the calibrated
wrist joint origin, q0 contract, or TCP frame.

When the replacement is enabled, the URDF also exports
`left_wrist_camera_mount_center_link`. Its origin is the midpoint of the
official STL's four M2 camera-module holes: a measured 27 x 27 mm pattern at
the middle of the 4 mm mounting plate. This is a repeatable CAD reference on
`left_gripper_link`; it is deliberately separate from
`left_gripper_frame_link` and from the camera optical frame. A marker attached
to a physical plate surface may require a signed local-Z surface/thickness
offset from this mid-plane reference.

For the 2026-07-28 Top–base registration session, the rigid yellow marker is
centered on the outer face of the 4 mm printed plate rather than on the UVC
camera body. `left_wrist_registration_marker_link` therefore applies a fixed
`-0.002 m` local-Z offset from the CAD mid-plane. The paper thickness is
neglected; this calibration-only frame must be revised if the marker is moved.

The Isaac Sim 6.0.1 asset uses the same replacement geometry through
`payloads/wrist_camera_mount_geometry.usd`. Regenerate that binary USD from
the verified STL with:

```bash
/home/an-hyeonseo/isaacsim-6.0.1-venv/bin/python \
  tools/setup/isaac/generate_isaac_wrist_camera_mount_geometry.py
```

Both the Isaac visual instance and its convex-hull collision instance reference
the generated geometry while retaining the existing `gripper_link` transform,
joint anchors, q0, and TCP. The current gripper mass/inertia still represents
the upstream model; camera-module mass-property calibration requires a later
physical measurement and is not inferred from the STL.

## Optional overhead webcam workcell

The optional static overhead-camera workcell uses the three Apache-2.0 parts
from TheRobotStudio/SO-ARM100
[`Optional/Overhead_Cam_Mount_Webcam`](https://github.com/TheRobotStudio/SO-ARM100/tree/main/Optional/Overhead_Cam_Mount_Webcam):

| Local mesh | SHA-256 |
| --- | --- |
| `overhead_webcam_arm_base.stl` | `169adfd40bcca689334efd1188c9b42cc03c914dc0afeaa98cb5431013610833` |
| `overhead_webcam_cam_mount_bottom.stl` | `b3545b6cae437210e17b7dcfee2e12e00dc7a59ece9264f4b13ab9fd8ceb8088` |
| `overhead_webcam_cam_mount_top.stl` | `177fbfae49cabba47b0b51811421b656d580dd1ab4f47f2248350f5184c75488` |
| `overhead_webcam_cam_mount_top_hinge_removed.stl` | `55319a9b26f9cdb7217c94000f7aec716d63b0a433150cd7672baa7b85a006cf` |

The STL files are authored in millimetres and use a `0.001` URDF scale. The
physical left-arm rig differs from the official adjustable assembly: only the
small center tip protruding beyond source-mesh `y=234.4404 mm` is broken and
absent; both surrounding printed end structures remain. `cam_mount_top` is
inserted end-for-end over the bottom tower's `y=223.1..230.95 mm` post. The
modified mesh is reproducible via `tools/setup/isaac/generate_overhead_top_hinge_removed.py`.
The insertion depth is 7.85 mm and every mount joint is fixed. Following the
RViz fit checks, `arm_base` keeps its confirmed robot-footprint alignment. The
complete `cam_mount_bottom` + `cam_mount_top` assembly is rotated 180 degrees
and inserted sideways—not from the front—into the 37.4 mm left-hand region seen
from the physical right side of `arm_base` (raw arm interval
`x=-93.9209137..-56.5209137 mm`). Only the assembly translation along that
confirmed insertion axis is then advanced by 10.0 mm, matching the full STL
groove depth; no other transform changes.

The workcell is off by default so that existing MoveIt planning geometry and
validated arm behaviour remain unchanged. Expand it explicitly for Isaac or
workcell visualization:

```bash
xacro urdf/so101_left.urdf.xacro \
  use_overhead_webcam_mount:=true \
  > /tmp/so101_left_with_overhead.urdf
```

For a simulation-only RViz assembly check (no hardware bridge or motor access):

```bash
ros2 launch so101_description display.launch.py \
  use_overhead_webcam_mount:=true
```

`top_camera_link` and `top_camera_optical_frame` are fixed calibration frames.
No camera-body mesh was supplied, and the default optical transform is only a
fixed-down provisional transform. Before using it for motion, replace
`top_camera_xyz` and `top_camera_optical_rpy` with a measured camera-to-base
extrinsic and preserve the perception stack's fail-closed authorization.

Only the left-arm configuration has been validated. The macro and prefix prepare later right-arm/bimanual composition but do not claim that a mirrored right-arm mount has been calibrated.

## Simulation-only bimanual preview

`urdf/so101_dual_preview.urdf.xacro` instantiates the same STL-backed arm
macro twice. The right arm is translated to negative Y because the workcell
convention is +X forward, +Y robot-left, +Z up. It is not mirrored: the two
physical arms use the same model, motor directions, and assembly convention.
The preview places an identical physical arm-base STL under each arm. The
previously validated overhead-camera bottom/top tower sits between the two
plates in top view: left base plate, camera tower, right base plate. Its single
URDF parent remains the left plate to avoid a closed kinematic loop. The
calibrated left wrist-camera mount/frame remains enabled. The right wrist
uses the same camera-mount replacement STL and the same wrist joint origin as
the left because its physical part and assembly are identical. A right camera
optical frame remains absent until the actual camera installation and its
eye-in-hand transform are independently confirmed.

The default right-base translation is `0 -0.232064146 0` m. It is derived
from the STL boundaries: the left plate already has the validated 10 mm camera
mount insertion, and this position gives the right plate the same 10 mm
insertion on the opposite side. It replaces the earlier rough 14-inch center
spacing. The value remains a CAD-fit candidate until visual and external
measurement confirm the physical assembly. It is intentionally isolated from
MoveIt and hardware control, and the preview URDF contains no `ros2_control`
block.
Generate a resolved URDF for Isaac Sim with:

```bash
python3 tools/setup/isaac/generate_isaac_bimanual_preview_urdf.py
```

In Isaac Sim 6.0.1, use **File > Import** to select the printed
`BIMANUAL_PREVIEW_URDF`. The generated preview carries the SHA-bound J1-L
arm-only candidate limits; grippers remain excluded and
`motion_authorized=false`. In the right-side **Model > ROS Package List** table,
use the printed `ISAAC_ROS_PACKAGE_NAME` and `ISAAC_ROS_PACKAGE_PATH` as one
row. Import with the timeline stopped. The generator also writes a manifest with
`simulation_only=true`, `motion_authorized=false`, and
`right_mount.status=PROVISIONAL_UNCALIBRATED`.

When a physical base transform has been measured, generate another candidate
without editing the source model:

```bash
python3 tools/setup/isaac/generate_isaac_bimanual_preview_urdf.py \
  --right-mount-xyz-m X Y Z \
  --right-mount-rpy-rad R P Y
```

STL improves link and mounting-surface registration, but it does not measure
the encoder zero or the transform between the two physical bases. Those remain
separate external-calibration gates. Detailed STL collision is also a preview
reference; a later MoveIt bimanual model should use validated, computationally
appropriate collision geometry rather than treating mesh detail as measured
clearance.

## Left-arm q0 contract

The 2026-07-26 visual registration established the physical raw-2048 Home as
the canonical zero for the five arm joints. The upstream model pose used to
reach that Home was:

| Project joint | Upstream-model pose absorbed into the joint origin |
| --- | ---: |
| `left_base_joint` | 0 deg |
| `left_shoulder_joint` | +90 deg |
| `left_elbow_joint` | -55 deg |
| `left_wrist_flex_joint` | -64.898281239 deg |
| `left_wrist_roll_joint` | -90 deg |

These are model-import offsets, not hardware command targets. After the
origins are shifted, physical raw 2048, ROS/MoveIt `q=0`, the SRDF `home`
state, and Isaac joint state 0 describe the same arm pose. The Isaac bridge
therefore keeps zero arm offsets; only its sign conversion remains.

The wrist-flex value was refined on 2026-07-30 using the rigid-target
eye-to-hand dataset. It applies a `-7.398281239 deg` model-origin correction
to the earlier photo-based `-57.5 deg` estimate. It does not change the
firmware, bridge calibration, raw-2048 zero, or any hardware command.

The wrist-roll zero establishes the physical gripper opening orientation.
Gripper aperture calibration is intentionally separate: this visual
registration does not claim that gripper raw 2048 is a measured open or closed
width. An initial `+90 deg` wrist-roll candidate was rejected when a mock
gripper-open motion exposed the reversed wrist orientation; the accepted model
offset is `-90 deg`.

The corrected all-zero wrist pose and the gripper open/closed direction were
then confirmed in both MoveIt mock execution and the Isaac Sim 6.0.1 Robot
Poser preview on 2026-07-26.

For hardware observation without a second `/joint_states` publisher, run:

```bash
ros2 launch so101_description read_only_tf.launch.py
```

This launch starts only `robot_state_publisher`. It intentionally excludes
`joint_state_publisher_gui`, RViz, MoveIt, controllers, and every motion Action
server. The real hardware bridge must remain the sole `/joint_states` publisher.
