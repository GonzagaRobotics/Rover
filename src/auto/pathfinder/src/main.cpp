#define STB_IMAGE_IMPLEMENTATION

#include "pathfinder.hpp"

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<Pathfinder>());

  rclcpp::shutdown();

  return 0;
}