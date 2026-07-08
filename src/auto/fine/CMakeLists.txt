cmake_minimum_required(VERSION 3.8)
project(fine)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(auto_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(Eigen3 REQUIRED )
find_package(PCL REQUIRED)

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

include_directories(include/fine)
include_directories(${PCL_INCLUDE_DIRS})

add_executable(fine src/main.cpp src/fine_node.cpp src/pgr.cpp)
target_link_libraries(fine rclcpp::rclcpp std_msgs::std_msgs sensor_msgs::sensor_msgs auto_msgs::auto_msgs ${PCL_LIBRARIES} Eigen3::Eigen tf2_ros::tf2_ros)
# These dependencies don't seem to have the proper targets needed for target_link_libraries, so for now, just use the old style
ament_target_dependencies(fine pcl_ros pcl_conversions)

install(TARGETS fine DESTINATION lib/${PROJECT_NAME})
ament_package()
