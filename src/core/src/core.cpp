#include "core.hpp"

void Core::connect_cb(const HeartbeatCnSrv::Request::SharedPtr req, HeartbeatCnSrv::Response::SharedPtr res)
{
  if (active_) {
    res->accepted = false;
    res->msg = "Already connected";
    return;
  }

  // Validate the request parameters
  if (
    req->heartbeat_interval <= 0 || req->heartbeat_timeout <= 0 || req->heartbeat_timeout_limit <= 0 ||
    req->heartbeat_check_interval <= 0) {
    res->accepted = false;
    res->msg = "Invalid heartbeat parameters in request";
    return;
  }

  if (req->heartbeat_timeout <= req->heartbeat_interval) {
    res->accepted = false;
    res->msg = "Heartbeat timeout must be greater than heartbeat interval";
    return;
  }

  // Just in case. I don't think this should ever happen
  try {
    missed_heartbeats_ = 0;
    expected_heartbeat_id_ = 0;
    last_heartbeat_time_ = now();

    heartbeat_interval_ = std::chrono::milliseconds(req->heartbeat_interval);
    heartbeat_timeout_ = std::chrono::milliseconds(req->heartbeat_timeout);
    heartbeat_timeout_limit_ = req->heartbeat_timeout_limit;

    heartbeat_check_timer_ = create_wall_timer(
      std::chrono::milliseconds(req->heartbeat_check_interval), std::bind(&Core::heartbeat_check_cb, this));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(get_logger(), "Failed to set up heartbeat parameters: %s", e.what());
    res->accepted = false;
    res->msg = "Failed to set up heartbeat parameters: " + std::string(e.what());
    return;
  }

  res->accepted = true;
  active_ = true;

  KillswitchMsg out;
  killswitch_pub_->publish(out);
}

void Core::disconnect_cb(const HeartbeatDcMsg::SharedPtr)
{
  if (!active_) {
    RCLCPP_WARN(get_logger(), "Received disconnect request while not connected");
    return;
  }

  RCLCPP_INFO(get_logger(), "Received disconnect request");
  active_ = false;

  heartbeat_check_timer_ = nullptr;
}

void Core::heartbeat_cb(const HeartbeatMsg::SharedPtr msg)
{
  if (!active_) {
    RCLCPP_WARN(get_logger(), "Received heartbeat while not connected");
    return;
  }

  if (msg->id != expected_heartbeat_id_) {
    RCLCPP_WARN(get_logger(), "Received heartbeat with ID %d instead of %d", msg->id, expected_heartbeat_id_);

    if (msg->id < expected_heartbeat_id_) {
      return;
    }

    expected_heartbeat_id_ = msg->id + 1;
  } else {
    expected_heartbeat_id_++;
  }

  last_heartbeat_time_ = now();
  missed_heartbeats_ = 0;

  auto out = HeartbeatMsg();
  out.id = msg->id;
  heartbeat_pub_->publish(out);
}

void Core::heartbeat_check_cb()
{
  if (!active_) {
    return;
  }

  // Determine the time elapsed since the last heartbeat should have been received,
  // accounting for the number of missed heartbeats
  auto expectedHeartbeatTime = last_heartbeat_time_ + heartbeat_interval_ * (missed_heartbeats_ + 1);

  if (now() - expectedHeartbeatTime > heartbeat_timeout_) {
    missed_heartbeats_++;

    RCLCPP_WARN(get_logger(), "Missed heartbeat #%d", missed_heartbeats_);

    if (missed_heartbeats_ >= heartbeat_timeout_limit_) {
      RCLCPP_ERROR(get_logger(), "Missed too many heartbeats, killswitch and disconnecting");
      active_ = false;
      heartbeat_check_timer_ = nullptr;

      auto out = KillswitchMsg();
      out.on = true;
      killswitch_pub_->publish(out);
    }
  }
}

Core::Core() : rclcpp::Node("core_node"), heartbeat_interval_(0, 0), heartbeat_timeout_(0, 0)
{
  using namespace std::placeholders;

  connect_srv_ = create_service<HeartbeatCnSrv>("heartbeat/connect", std::bind(&Core::connect_cb, this, _1, _2));
  disconnect_sub_ =
    create_subscription<HeartbeatDcMsg>("heartbeat/disconnect", 10, std::bind(&Core::disconnect_cb, this, _1));

  heartbeat_sub_ = create_subscription<HeartbeatMsg>("heartbeat/rover", 10, std::bind(&Core::heartbeat_cb, this, _1));
  heartbeat_pub_ = create_publisher<HeartbeatMsg>("heartbeat/control", 10);

  killswitch_pub_ = create_publisher<KillswitchMsg>("killswitch", 10);

  RCLCPP_INFO(get_logger(), "Core ready");
}