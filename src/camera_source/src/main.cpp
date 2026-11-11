#include "camera_source/source.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<CameraSource>();
  auto executor = rclcpp::executors::StaticSingleThreadedExecutor();
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}