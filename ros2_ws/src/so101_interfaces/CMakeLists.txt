cmake_minimum_required(VERSION 3.8)
project(so101_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ShadowObjectTarget.msg"
  "msg/TopObjectPose.msg"
  "msg/OrientedObjectObservation.msg"
  "msg/BimanualJointFeedback.msg"
  "srv/RightArmJogOnce.srv"
  "srv/RightArmTorqueEnableOnce.srv"
  "srv/RightArmConfiguration.srv"
  "srv/RightArmConfigureOnce.srv"
  "srv/BimanualStreamCommand.srv"
  DEPENDENCIES std_msgs trajectory_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
