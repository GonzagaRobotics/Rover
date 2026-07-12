#include "webrtc_node.hpp"

void WebRTCNode::create_codec()
{
  if (codec_ctx_) {
    avcodec_free_context(&codec_ctx_);
  }

  codec_ctx_ = avcodec_alloc_context3(codec_);
  if (!codec_ctx_) {
    throw std::runtime_error("Could not allocate video codec context");
  }

  codec_ctx_->bit_rate = 400000;
  codec_ctx_->width = 1280;
  codec_ctx_->height = 720;
  codec_ctx_->time_base = {1, 30};
  codec_ctx_->framerate = {30, 1};
  codec_ctx_->gop_size = 30;
  codec_ctx_->max_b_frames = 0;
  codec_ctx_->pix_fmt = AV_PIX_FMT_YUV420P;

  av_opt_set(codec_ctx_->priv_data, "preset", "p5", 0);

  if (avcodec_open2(codec_ctx_, codec_, nullptr) < 0) {
    throw std::runtime_error("Could not open codec");
  }
}

void WebRTCNode::signal_cb(const StringMsg::SharedPtr msg)
{
  if (msg->data.empty()) {
    std::cout << "Received empty signal" << std::endl;
    return;
  }

  std::lock_guard<std::mutex> lock(rtc_mutex_);
  std::cout << "Received signal: " << msg->data << std::endl;

  if (!signal_data_.empty()) {
    std::cout << "Warning: Overwriting existing signal data" << std::endl;
  }

  signal_data_ = msg->data;
}

void WebRTCNode::image_cb(const ImageMsg::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(rtc_mutex_);

  // FFMPEG may not be initialized yet, so we need to check
  if (!sws_ctx_) {
    return;
  }

  if (
    msg->encoding != "bgr8" || msg->width != (uint32_t)codec_ctx_->width ||
    msg->height != (uint32_t)codec_ctx_->height) {
    RCLCPP_WARN(
      this->get_logger(), "Received image with unsupported encoding or size: %s, %dx%d",
      msg->encoding.c_str(), msg->width, msg->height);

    return;
  }

  if (av_frame_make_writable(frame_) < 0 || av_frame_make_writable(frame_yuv_) < 0) {
    RCLCPP_ERROR(this->get_logger(), "Could not make frames writable");
    return;
  }

  frame_->data[0] = const_cast<uint8_t *>(msg->data.data());

  if (
    sws_scale(
      sws_ctx_, frame_->data, frame_->linesize, 0, codec_ctx_->height, frame_yuv_->data,
      frame_yuv_->linesize) < 0) {
    RCLCPP_ERROR(this->get_logger(), "Could not convert image");
    return;
  }

  frame_yuv_->pts = msg->header.stamp.sec * 1000000 + msg->header.stamp.nanosec / 1000;

  if (pli_) {
    create_codec();
    pli_ = false;
  }

  // Encode the frame
  int ret = avcodec_send_frame(codec_ctx_, frame_yuv_);
  if (ret < 0) {
    RCLCPP_ERROR(this->get_logger(), "Error sending frame to encoder");
    return;
  }

  while (ret >= 0) {
    ret = avcodec_receive_packet(codec_ctx_, packet_);
    if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
      break;
    } else if (ret < 0) {
      RCLCPP_ERROR(this->get_logger(), "Error encoding frame");
      return;
    }

    if (pc_.state() == rtc::PeerConnection::State::Connected) {
      rtc::FrameInfo info(frame_yuv_->pts);
      track_->sendFrame(reinterpret_cast<const std::byte *>(packet_->data), packet_->size, info);
    }

    av_packet_unref(packet_);
  }
}

void WebRTCNode::rtc_worker()
{
  pc_.onStateChange(
    [](rtc::PeerConnection::State state) { std::cout << "State: " << state << std::endl; });

  pc_.onGatheringStateChange([this](rtc::PeerConnection::GatheringState state) {
    std::cout << "Gathering State: " << state << std::endl;

    if (state == rtc::PeerConnection::GatheringState::Complete) {
      auto desc = pc_.localDescription();
      json msg_str = {{"type", desc->typeString()}, {"sdp", std::string(desc.value())}};
      std::cout << "Local Description: " << msg_str.dump() << std::endl;

      // TODO: Possible race condition?
      StringMsg msg;
      msg.data = msg_str.dump();
      signal_pub_->publish(msg);

      std::cout << "ICE Gathering Complete" << std::endl;
    }
  });

  const rtc::SSRC ssrc = 1;
  rtc::Description::Video media("video", rtc::Description::Direction::SendOnly);
  media.setBitrate(400000);
  media.addH264Codec(102);
  media.addSSRC(ssrc, "video_send");
  auto track = pc_.addTrack(media);

  auto rtp_config = std::make_shared<rtc::RtpPacketizationConfig>(
    1, "video_send", 102, rtc::H264RtpPacketizer::ClockRate);
  auto packetizer_ = std::make_shared<rtc::H264RtpPacketizer>(
    rtc::H264RtpPacketizer::Separator::StartSequence, rtp_config);
  auto sr_report = std::make_shared<rtc::RtcpSrReporter>(rtp_config);
  packetizer_->addToChain(sr_report);
  auto nack_response = std::make_shared<rtc::RtcpNackResponder>();
  packetizer_->addToChain(nack_response);
  auto pli_handler = std::make_shared<rtc::PliHandler>([this]() { pli_ = true; });
  packetizer_->addToChain(pli_handler);

  track->setMediaHandler(packetizer_);
  track_ = track;

  pc_.setLocalDescription();

  while (running_) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    json msg_json;
    {
      std::lock_guard<std::mutex> lock(rtc_mutex_);

      if (signal_data_.empty()) {
        continue;
      }

      msg_json = json::parse(signal_data_);
      signal_data_.clear();
    }

    rtc::Description answer(
      msg_json["sdp"].get<std::string>(), msg_json["type"].get<std::string>());
    pc_.setRemoteDescription(answer);
  }
}

WebRTCNode::WebRTCNode()
: rclcpp::Node("webrtc_src_node"), rtc_thread_(&WebRTCNode::rtc_worker, this)
{
  signal_pub_ = this->create_publisher<StringMsg>("webrtc/signal_src", 10);
  signal_sub_ = this->create_subscription<StringMsg>("webrtc/signal_sink", 10, BIND(signal_cb));
  image_sub_ = this->create_subscription<ImageMsg>("/vision/main/image_raw", 10, BIND(image_cb));
}

WebRTCNode::~WebRTCNode()
{
  running_ = false;
  rtc_thread_.join();

  if (frame_) {
    av_frame_free(&frame_);
  }
  if (frame_yuv_) {
    av_frame_free(&frame_yuv_);
  }
  if (codec_ctx_) {
    avcodec_free_context(&codec_ctx_);
  }
  if (packet_) {
    av_packet_free(&packet_);
  }
  if (sws_ctx_) {
    sws_freeContext(sws_ctx_);
  }
}

void WebRTCNode::init_ffmpeg()
{
  codec_ = avcodec_find_encoder_by_name("h264_nvenc");
  if (!codec_) {
    throw std::runtime_error("Codec not found");
  }

  create_codec();

  frame_ = av_frame_alloc();
  if (!frame_) {
    throw std::runtime_error("Could not allocate raw video frame");
  }
  frame_->format = AV_PIX_FMT_BGR24;
  frame_->width = codec_ctx_->width;
  frame_->height = codec_ctx_->height;
  if (av_frame_get_buffer(frame_, 0) < 0) {
    throw std::runtime_error("Could not allocate the raw video frame data");
  }

  frame_yuv_ = av_frame_alloc();
  if (!frame_yuv_) {
    throw std::runtime_error("Could not allocate YUV video frame");
  }
  frame_yuv_->format = codec_ctx_->pix_fmt;
  frame_yuv_->width = codec_ctx_->width;
  frame_yuv_->height = codec_ctx_->height;
  if (av_frame_get_buffer(frame_yuv_, 0) < 0) {
    throw std::runtime_error("Could not allocate the video frame data");
  }

  packet_ = av_packet_alloc();

  sws_ctx_ = sws_getContext(
    codec_ctx_->width, codec_ctx_->height, AV_PIX_FMT_BGR24, codec_ctx_->width, codec_ctx_->height,
    AV_PIX_FMT_YUV420P, SWS_BILINEAR, nullptr, nullptr, nullptr);
  if (!sws_ctx_) {
    throw std::runtime_error("Could not initialize the conversion context");
  }
}
