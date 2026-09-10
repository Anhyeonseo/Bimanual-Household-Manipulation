# SO101 arm description

The arm macro retains the existing joint origins, axes, limits and inertial values. Meshes are derived from TheRobotStudio/SO-ARM100, `Simulation/SO101/so101_new_calib.urdf`, commit `fda892cba81032c46c40976a48c9ceadbf40a9ca` under Apache-2.0. The optional wrist camera mount uses the same project's `Optional/Wrist_Cam_Mount_32x32_UVC_Module` geometry.

`urdf/so101_arm.urdf.xacro` instantiates one arm with an `arm_slot` prefix and a configurable mount transform. Its root `arm_mount_frame` is a standalone reference, not a mobile base model. Both `left` and `right` instances must be attached to measured mount transforms when the ALOHA Mini 1 assembly is measured. The mobile base and lift are not modeled yet.

The fixed overhead camera, old table registration, fitted dual-arm preview and simulation controller configurations have been removed. Camera optical transforms and the new robot's collision model require registration after assembly. The arm macro includes the existing mount-center and marker reference geometry; these are not camera calibration results.

```bash
ros2 launch so101_description read_only_tf.launch.py arm_slot:=left
```

This publishes TF from external joint feedback. It starts no hardware driver or motion controller. Model limits and the registered hardware limits in `config/bimanual_operational_limits.json` remain separate; the complete mobile robot has not been collision-validated.
