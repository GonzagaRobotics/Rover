cmake_minimum_required(VERSION 3.8)
project(auto_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Location.msg"
  "msg/Plan.msg"
  "msg/State.msg"
  "msg/Target.msg"
  "msg/Detection.msg"
  "action/Pathfind.action"
  "action/GoTo.action"
  DEPENDENCIES std_msgs sensor_msgs geometry_msgs
)

ament_package()