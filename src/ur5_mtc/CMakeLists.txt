cmake_minimum_required(VERSION 3.8)
project(ur5_mtc)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_task_constructor_core REQUIRED)
find_package(moveit_core REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_library(pick_graph INTERFACE)
target_include_directories(pick_graph INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(mtc_pick src/mtc_pick_node.cpp)
target_include_directories(mtc_pick PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(mtc_pick
  rclcpp moveit_task_constructor_core moveit_core geometry_msgs moveit_msgs shape_msgs
  tf2_geometry_msgs)

install(TARGETS mtc_pick DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_pick_graph test/test_pick_graph.cpp)
  target_include_directories(test_pick_graph PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
endif()

ament_package()
