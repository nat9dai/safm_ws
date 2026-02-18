cmake_minimum_required(VERSION 3.8)
project(conversion_pkg)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(px4_msgs REQUIRED)

add_executable(sensor_combined_to_imu_node src/sensor_combined_to_imu_node.cpp)
ament_target_dependencies(sensor_combined_to_imu_node
  rclcpp
  sensor_msgs
  px4_msgs
)

install(TARGETS
  sensor_combined_to_imu_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
