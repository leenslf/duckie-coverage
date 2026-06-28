cmake_minimum_required(VERSION 3.8)
project(kinematics)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(robot_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(kinematics_node src/kinematics.cpp)
ament_target_dependencies(kinematics_node rclcpp geometry_msgs robot_msgs)

add_executable(inverse_kinematics_node src/inverse_kinematics.cpp)
ament_target_dependencies(inverse_kinematics_node rclcpp nav_msgs robot_msgs tf2 tf2_ros geometry_msgs)

install(TARGETS kinematics_node inverse_kinematics_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
