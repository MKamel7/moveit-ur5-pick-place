cmake_minimum_required(VERSION 3.8)
project(ur5_pick_place_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(action_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "action/DetectObject.action"
  "action/PlanGrasp.action"
  "action/ExecutePick.action"
  DEPENDENCIES action_msgs geometry_msgs moveit_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
