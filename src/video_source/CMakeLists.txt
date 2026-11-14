cmake_minimum_required(VERSION 3.8)
project(video_source)
include(FetchContent)

set(CMAKE_BUILD_TYPE RelWithDebInfo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# uvgRTP
FetchContent_Declare(
  uvgRTP
  GIT_REPOSITORY https://github.com/ultravideo/uvgRTP
  GIT_TAG v4.0.2
)
SET(UVGRTP_DISABLE_INSTALL ON)
SET(UVGRTP_BUILD_SHARED ON)
SET(UVGRTP_DISABLE_CRYPTO ON)
FetchContent_MakeAvailable(uvgrtp)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)

# x264 for video encoding
find_package(PkgConfig REQUIRED)
pkg_check_modules(x264 REQUIRED IMPORTED_TARGET x264)
find_path(LIBYUV_INCLUDE_DIR libyuv.h REQUIRED)
find_library(LIBYUV_LIBRARY yuv REQUIRED)

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

include_directories(include)

add_library(source_comp SHARED src/source.cpp)
target_link_libraries(source_comp rclcpp::rclcpp rclcpp_components::component ${sensor_msgs_TARGETS} PkgConfig::x264 ${LIBYUV_LIBRARY} uvgrtp)

rclcpp_components_register_node(source_comp PLUGIN "SourceNode" EXECUTABLE video_source)
install(TARGETS source_comp DESTINATION lib)
ament_package()
