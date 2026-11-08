#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <uvgrtp/lib.hh>

// Encoding
extern "C" {
#include <x264.h>
}
#include <libyuv.h>

using ImageMsg = sensor_msgs::msg::Image;

class SourceNode : public rclcpp::Node
{
private:
  int bitrate_ = 500000;
  int width_ = 1280;
  int height_ = 720;
  int fps_ = 30;

  uvgrtp::context ctx_;
  uvgrtp::session * session_;
  uvgrtp::media_stream * stream_;

  // Encoding
  x264_t * encoder_ = nullptr;
  x264_picture_t pic_in_;
  x264_picture_t pic_out_;
  bool pic_allocated_ = false;

  rclcpp::Subscription<ImageMsg>::SharedPtr image_sub_;

  void init_encoder();

public:
  SourceNode();
  ~SourceNode();

  void image_cb(const ImageMsg::SharedPtr msg);
};