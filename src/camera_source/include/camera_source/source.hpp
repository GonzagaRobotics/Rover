#pragma once

#include <atomic>
#include <mutex>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <thread>

class CameraSource : public rclcpp::Node
{
private:
  // Camera resource path
  std::string camera_name_;
  int width_;
  int height_;
  int fps_;
  int send_fps_;

  std::atomic<int> capture_state_{0};
  cv::VideoCapture cap_;

  std::mutex frame_mutex_;
  // Latest frame, owned until published so ownership can transfer to the middleware
  sensor_msgs::msg::Image::UniquePtr frame_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::thread capture_thread_;

  rcl_interfaces::msg::ParameterDescriptor read_only(std::string description)
  {
    rcl_interfaces::msg::ParameterDescriptor descriptor;
    descriptor.read_only = true;
    descriptor.description = description;
    return descriptor;
  }

  void capture_worker();

  void send_cb();

public:
  CameraSource() : CameraSource(rclcpp::NodeOptions()) {}
  CameraSource(const rclcpp::NodeOptions & options);

  ~CameraSource();
};

RCLCPP_COMPONENTS_REGISTER_NODE(CameraSource)
