cmake_minimum_required(VERSION 3.8)
project(camera_source)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_BUILD_TYPE RelWithDebInfo)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(OpenCV REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include ${OpenCV_INCLUDE_DIRS})

add_library(source_comp SHARED src/source.cpp)
target_link_libraries(source_comp rclcpp::rclcpp rclcpp_components::component ${sensor_msgs_TARGETS} ${OpenCV_LIBS})

rclcpp_components_register_node(source_comp PLUGIN "CameraSource" EXECUTABLE camera_source)
install(TARGETS source_comp DESTINATION lib)
ament_package()
