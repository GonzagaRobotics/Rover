#include "video_source/source.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<SourceNode> node;
  node = std::make_shared<SourceNode>();

  rclcpp::executors::StaticSingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}