cmake_minimum_required(VERSION 3.8)
project(nav_sensor_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# List all your .msg files here
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/CalibrationSetup.msg"
  "msg/Location.msg"
  "msg/Orientation.msg"
)

ament_package()
