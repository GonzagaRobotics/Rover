cmake_minimum_required(VERSION 3.8)
project(nav_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# List all your .msg files here
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Aruco.msg"
  "msg/CalibrationSetup.msg"
  "msg/Location.msg"
  "msg/Orientation.msg"
  DEPENDENCIES geometry_msgs sensor_msgs 
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)


ament_package()
