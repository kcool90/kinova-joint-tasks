cmake_minimum_required(VERSION 3.8)
project(moveit_joint_sender)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(control_msgs REQUIRED)

add_executable(send_joint_state_goal src/send_joint_state_goal.cpp)

ament_target_dependencies(send_joint_state_goal
  rclcpp
  moveit_ros_planning_interface
  control_msgs
)

target_link_libraries(send_joint_state_goal
  yaml-cpp
)

install(TARGETS send_joint_state_goal
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
