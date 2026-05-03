cmake_minimum_required(VERSION 3.8)
project(roboteq_driver)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(robot_msgs REQUIRED)

include_directories(include)

add_executable(roboteq_driver
  src/driver_manager.cpp
  include/Linux/RoboteqDevice.o
)

ament_target_dependencies(roboteq_driver rclcpp std_msgs robot_msgs)

install(TARGETS roboteq_driver
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
