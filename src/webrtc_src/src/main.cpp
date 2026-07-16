#include "webrtc_node.hpp"

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<WebRTCNode> node;

  // Init can cause exceptions, so we need to catch them and shutdown cleanly
  try {
    node = std::make_shared<WebRTCNode>();
    node->init_ffmpeg();
  } catch (const std::exception & e) {
    std::cerr << "Exception: " << e.what() << std::endl;

    rclcpp::shutdown();
    return 1;
  }

  rclcpp::executors::StaticSingleThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}