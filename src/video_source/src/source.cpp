#include "video_source/source.hpp"

void SourceNode::init_encoder()
{
  x264_param_t param;
  if (x264_param_default_preset(&param, "ultrafast", "zerolatency") != 0) {
    throw std::runtime_error("Could not set x264 preset");
  }

  param.i_width = width_;
  param.i_height = height_;
  param.i_csp = X264_CSP_I420;
  param.i_fps_num = fps_;
  param.i_fps_den = 1;
  // PTS is in microseconds
  param.i_timebase_num = 1;
  param.i_timebase_den = 1000000;
  param.i_keyint_max = fps_ * 2;

  param.rc.i_rc_method = X264_RC_ABR;
  param.rc.i_bitrate = bitrate_ / 1000;

  param.b_repeat_headers = 1;
  param.b_annexb = 1;

  // Chrome does not seem to support B-frames
  if (x264_param_apply_profile(&param, "baseline") != 0) {
    throw std::runtime_error("Could not set x264 profile");
  }

  encoder_ = x264_encoder_open(&param);
  if (!encoder_) {
    throw std::runtime_error("Could not open x264 encoder");
  }

  if (x264_picture_alloc(&pic_in_, X264_CSP_I420, width_, height_) != 0) {
    // Clean up the encoder if picture allocation fails
    x264_encoder_close(encoder_);
    throw std::runtime_error("Could not allocate x264 picture");
  }
  pic_allocated_ = true;
}

SourceNode::SourceNode(const rclcpp::NodeOptions & options) : Node("video_source_node", options)
{
  rcl_interfaces::msg::ParameterDescriptor remote_ip_desc;
  remote_ip_desc.read_only = true;
  remote_ip_desc.description = "The IP address of the remote peer";
  auto remote_ip = this->declare_parameter("remote_ip", "127.0.0.1", remote_ip_desc);

  // RTP

  session_ = ctx_.create_session(remote_ip.c_str());
  stream_ = session_->create_stream(8890, RTP_FORMAT_H264, RCE_SEND_ONLY);

  if (!stream_) {
    // Clean up the session if stream creation fails
    ctx_.destroy_session(session_);
    throw std::runtime_error("Could not create stream");
  }

  try {
    init_encoder();
  } catch (const std::exception & e) {
    // Clean up the stream and session if encoder initialization fails
    session_->destroy_stream(stream_);
    ctx_.destroy_session(session_);

    throw;
  }

  image_sub_ = this->create_subscription<ImageMsg>(
    "/camera/image_raw", rclcpp::SensorDataQoS(), std::bind(&SourceNode::image_cb, this, std::placeholders::_1));

  RCLCPP_INFO(this->get_logger(), "Ready");
}

SourceNode::~SourceNode()
{
  if (stream_) {
    session_->destroy_stream(stream_);
  }
  if (session_) {
    ctx_.destroy_session(session_);
  }

  if (encoder_) {
    x264_encoder_close(encoder_);
  }
  if (pic_allocated_) {
    x264_picture_clean(&pic_in_);
  }
}

void SourceNode::image_cb(const ImageMsg::SharedPtr msg)
{
  if (msg->encoding != "bgr8" || msg->width != (uint32_t)width_ || msg->height != (uint32_t)height_) {
    RCLCPP_WARN(
      this->get_logger(), "Received image with unsupported encoding or size: %s, %dx%d", msg->encoding.c_str(),
      msg->width, msg->height);

    return;
  }

  auto start_t = std::chrono::high_resolution_clock::now();

  // libyuv "RGB24" is B,G,R byte order, i.e. ROS bgr8
  if (
    libyuv::RGB24ToI420(
      msg->data.data(), msg->step, pic_in_.img.plane[0], pic_in_.img.i_stride[0], pic_in_.img.plane[1],
      pic_in_.img.i_stride[1], pic_in_.img.plane[2], pic_in_.img.i_stride[2], width_, height_) != 0) {
    RCLCPP_ERROR(this->get_logger(), "Could not convert image");
    return;
  }

  pic_in_.i_pts = msg->header.stamp.sec * 1000000LL + msg->header.stamp.nanosec / 1000;

  x264_nal_t * nals;
  int n_nals;
  int size = x264_encoder_encode(encoder_, &nals, &n_nals, &pic_in_, &pic_out_);
  if (size < 0) {
    RCLCPP_ERROR(this->get_logger(), "Error encoding frame");
    return;
  }

  if (size > 0) {
    if (stream_->push_frame(nals[0].p_payload, size, RTP_NO_FLAGS) != RTP_OK) {
      RCLCPP_ERROR(this->get_logger(), "Could not send frame");
      return;
    }
  }

  auto end_t = std::chrono::high_resolution_clock::now();
  auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_t - start_t).count();
  if (duration > (1.0 / fps_) * 1000.0) {
    RCLCPP_WARN(this->get_logger(), "Encoding took too long: %f seconds", duration / 1000.f);
  }
}