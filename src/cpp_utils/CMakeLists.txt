cmake_minimum_required(VERSION 3.8)
project(cpp_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Header-only package: this target is a compile database anchor for tooling
add_library(cpp_utils_header_check OBJECT src/header_check.cpp)
target_include_directories(cpp_utils_header_check PRIVATE include)
ament_target_dependencies(cpp_utils_header_check rclcpp)

# Interface target carries the include path and rclcpp to consumers
add_library(cpp_utils INTERFACE)
target_include_directories(cpp_utils INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(cpp_utils INTERFACE rclcpp::rclcpp)

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)
install(TARGETS cpp_utils EXPORT export_${PROJECT_NAME})

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp)
ament_package()
