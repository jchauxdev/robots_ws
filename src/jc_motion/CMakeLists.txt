cmake_minimum_required(VERSION 3.8)
project(jc_motion)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_core REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(shape_msgs REQUIRED)
find_package(jc_interfaces REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(control_msgs REQUIRED)

set(DEPS rclcpp moveit_ros_planning_interface moveit_core tf2 tf2_ros
         tf2_geometry_msgs geometry_msgs moveit_msgs shape_msgs jc_interfaces)

add_executable(motion_server src/motion_server.cpp)
ament_target_dependencies(motion_server ${DEPS})
add_executable(pose_goal_demo src/pose_goal_demo.cpp)
ament_target_dependencies(pose_goal_demo ${DEPS})

add_executable(trajectory_client src/trajectory_client.cpp)
ament_target_dependencies(trajectory_client rclcpp rclcpp_action control_msgs)

install(TARGETS motion_server pose_goal_demo trajectory_client DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
ament_package()
