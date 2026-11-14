#include "camera_source/source.hpp"

#include <chrono>

void CameraSource::capture_worker()
{
  // Wait for the node to fully initialize
  while (capture_state_ == 0) {
    std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }

  while (capture_state_ == 1) {
    if (!cap_.grab()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to grab frame from camera");
      capture_state_ = 2;
      break;
    }

    std::lock_guard<std::mutex> lock(frame_mutex_);

    // Retrieve directly into the message buffer to avoid an extra copy
    auto msg = std::make_unique<sensor_msgs::msg::Image>();
    msg->header.stamp = this->now();
    msg->header.frame_id = "camera_frame";
    msg->height = height_;
    msg->width = width_;
    msg->encoding = "bgr8";
    msg->is_bigendian = false;
    msg->step = width_ * 3;
    msg->data.resize(size_t(msg->step) * msg->height);

    cv::Mat view(height_, width_, CV_8UC3, msg->data.data(), msg->step);
    cv::Mat target = view;
    if (!cap_.retrieve(target) || target.empty()) {
      continue;
    }

    // OpenCV reallocates if the frame does not match the expected layout
    if (target.data != msg->data.data()) {
      msg->height = target.rows;
      msg->width = target.cols;
      msg->step = target.cols * 3;
      msg->data.assign(target.data, target.data + target.total() * target.elemSize());
    }

    frame_ = std::move(msg);

    // If the timer is not set, we are responsible for sending the frame
    if (!timer_) {
      send_cb();
    }
  }
}

void CameraSource::send_cb()
{
  // We could be getting called directly from the capture thread
  if (timer_) {
    std::lock_guard<std::mutex> lock(frame_mutex_);
  }

  if (!frame_) {
    return;
  }

  // Publishing a unique_ptr lets intra-process subscribers share the buffer without copying
  publisher_->publish(std::move(frame_));
}

CameraSource::CameraSource(const rclcpp::NodeOptions & options)
: Node("camera_source_node", rclcpp::NodeOptions(options).use_intra_process_comms(true)),
  capture_thread_(&CameraSource::capture_worker, this)
{
  camera_name_ = declare_parameter("camera_name", "/dev/video0", read_only("Camera resource path"));
  width_ = declare_parameter("width", 1280, read_only("Camera image width"));
  height_ = declare_parameter("height", 720, read_only("Camera image height"));
  fps_ = declare_parameter("fps", 30, read_only("Camera image capture FPS"));
  send_fps_ = declare_parameter("send_fps", 30, read_only("Send every Nth frame to reduce bandwidth"));

  if (!cap_.open(camera_name_, cv::CAP_V4L2)) {
    capture_state_ = 2;
    capture_thread_.join();

    throw std::runtime_error("Failed to open camera");
  }

  cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
  cap_.set(cv::CAP_PROP_FRAME_WIDTH, width_);
  cap_.set(cv::CAP_PROP_FRAME_HEIGHT, height_);
  cap_.set(cv::CAP_PROP_FPS, fps_);

  int actual_width = int(cap_.get(cv::CAP_PROP_FRAME_WIDTH));
  int actual_height = int(cap_.get(cv::CAP_PROP_FRAME_HEIGHT));
  if (actual_width != width_ || actual_height != height_) {
    RCLCPP_WARN(
      this->get_logger(), "Requested resolution %dx%d, but got %dx%d", width_, height_, actual_width, actual_height);
  }
  int actual_fps = int(cap_.get(cv::CAP_PROP_FPS));
  if (actual_fps != fps_) {
    RCLCPP_WARN(this->get_logger(), "Requested FPS %d, but got %d", fps_, actual_fps);
  }

  publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", rclcpp::SensorDataQoS());
  // Only use the timer if our send and capture rates differ
  if (send_fps_ != fps_) {
    timer_ =
      this->create_wall_timer(std::chrono::milliseconds(1000 / send_fps_), std::bind(&CameraSource::send_cb, this));
  }

  capture_state_ = 1;
}

CameraSource::~CameraSource()
{
  capture_state_ = 2;

  if (capture_thread_.joinable()) {
    capture_thread_.join();
  }

  if (cap_.isOpened()) {
    cap_.release();
  }
}