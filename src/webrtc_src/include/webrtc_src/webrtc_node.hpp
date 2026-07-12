#pragma once

#include <atomic>
#include <functional>
#include <iostream>
#include <json.hpp>
#include <memory>
#include <mutex>
#include <rclcpp/rclcpp.hpp>
#include <rtc/rtc.hpp>
#include <thread>

// FFMPEG
extern "C" {
#include <libavcodec/avcodec.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}

#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"

using json = nlohmann::json;

using ImageMsg = sensor_msgs::msg::Image;
using StringMsg = std_msgs::msg::String;

#define BIND(x) std::bind(&WebRTCNode::x, this, std::placeholders::_1)

class WebRTCNode : public rclcpp::Node
{
private:
  std::atomic<bool> running_ = true;
  std::atomic<bool> pli_ = false;
  // PeerConnection is owned by rtc_thread_ and cannot be touched by others
  rtc::PeerConnection pc_;
  std::shared_ptr<rtc::Track> track_;
  std::string signal_data_;

  AVCodec * codec_;
  AVCodecContext * codec_ctx_;
  AVFrame * frame_;
  AVFrame * frame_yuv_;
  AVPacket * packet_;
  SwsContext * sws_ctx_;

  rclcpp::Publisher<StringMsg>::SharedPtr signal_pub_;
  rclcpp::Subscription<StringMsg>::SharedPtr signal_sub_;
  rclcpp::Subscription<ImageMsg>::SharedPtr image_sub_;

  std::mutex rtc_mutex_;
  std::thread rtc_thread_;

  void create_codec();

  void signal_cb(const StringMsg::SharedPtr msg);
  void image_cb(const ImageMsg::SharedPtr msg);

  void rtc_worker();

public:
  WebRTCNode();
  ~WebRTCNode();

  void init_ffmpeg();
};