cmake_minimum_required(VERSION 3.8)
project(nav_sensors)

# Find required dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_sensor_msgs REQUIRED)

# Include directories (for your NavSensors.h)
include_directories(
  include
)

# Create the executable
add_executable(nav_sensors
  src/NavSensors.cpp
)

# Link dependencies
ament_target_dependencies(nav_sensors
  rclcpp
  nav_sensor_msgs
)

# Install the executable
install(TARGETS
  nav_sensors
  DESTINATION lib/${PROJECT_NAME}
)

# Install headers (optional but best practice)
install(
  DIRECTORY include/
  DESTINATION include/
)

ament_package()
