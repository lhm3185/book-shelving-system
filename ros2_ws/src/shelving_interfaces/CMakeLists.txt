cmake_minimum_required(VERSION 3.8)
project(shelving_interfaces)

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(action_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TrayJob.msg"
  "msg/RobotStatus.msg"
  "msg/TargetSlot.msg"
  "msg/GraspObservation.msg"
  "action/NavigateToTarget.action"
  "action/DetectTargetSlot.action"
  "action/PlaceBook.action"
  "action/LookAt.action"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs action_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()
