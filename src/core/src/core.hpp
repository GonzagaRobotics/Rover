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
  rclcpp::Duration heartbeatInterval;
  rclcpp::Duration heartbeatTimeout;
  uint32_t heartbeatTimeoutLimit;

  bool active = false;
  rclcpp::Time lastHeartbeatTime;
  uint32_t expectedHeartbeatId;
  uint32_t missedHeartbeats = 0;

  rclcpp::Service<HeartbeatConnect>::SharedPtr connectService;

  void onConnect(
    const HeartbeatConnect::Request::SharedPtr request,
    HeartbeatConnect::Response::SharedPtr response);

  rclcpp::Subscription<HeartbeatDisconnect>::SharedPtr disconnectSub;

  void onDisconnect(const HeartbeatDisconnect::SharedPtr);

  rclcpp::Subscription<Heartbeat>::SharedPtr heartbeatSub;
  rclcpp::Publisher<Heartbeat>::SharedPtr heartbeatPub;

  void heartbeatSubscriberCallback(const Heartbeat::SharedPtr beat);

  rclcpp::TimerBase::SharedPtr heartbeatCheckTimer;

  void checkHeartbeat();

public:
  Core();
};