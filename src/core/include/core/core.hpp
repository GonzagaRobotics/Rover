#pragma once

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <vector>

#include "core_interfaces/Types.hpp"
#include "rclcpp/rclcpp.hpp"

class Core : public rclcpp::Node
{
private:
  rclcpp::Duration heartbeat_interval_;
  rclcpp::Duration heartbeat_timeout_;
  uint32_t heartbeat_timeout_limit_;

  bool active_ = false;
  rclcpp::Time last_heartbeat_time_;
  uint32_t expected_heartbeat_id_;
  uint32_t missed_heartbeats_ = 0;

  rclcpp::Service<HeartbeatCnSrv>::SharedPtr connect_srv_;
  rclcpp::Subscription<HeartbeatDcMsg>::SharedPtr disconnect_sub_;

  rclcpp::Subscription<HeartbeatMsg>::SharedPtr heartbeat_sub_;
  rclcpp::Publisher<HeartbeatMsg>::SharedPtr heartbeat_pub_;

  rclcpp::Publisher<KillswitchMsg>::SharedPtr killswitch_pub_;

  rclcpp::TimerBase::SharedPtr heartbeat_check_timer_;

  void connect_cb(const HeartbeatCnSrv::Request::SharedPtr req, HeartbeatCnSrv::Response::SharedPtr res);

  void disconnect_cb(const HeartbeatDcMsg::SharedPtr);

  void heartbeat_cb(const HeartbeatMsg::SharedPtr msg);

  void heartbeat_check_cb();

public:
  Core();
};