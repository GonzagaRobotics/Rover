#pragma once

#include <memory>

#include "core_interfaces/msg/heartbeat.hpp"
#include "core_interfaces/msg/heartbeat_disconnect.hpp"
#include "core_interfaces/srv/heartbeat_connect.hpp"
#include "rclcpp/rclcpp.hpp"

struct Heartbeat
{
  using SharedPtr = std::shared_ptr<Heartbeat>;

  uint32_t id;
};

template <>
struct rclcpp::TypeAdapter<Heartbeat, core_interfaces::msg::Heartbeat>
{
  using is_specialized = std::true_type;
  using custom_type = Heartbeat;
  using ros_message_type = core_interfaces::msg::Heartbeat;

  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination)
  {
    destination.id = source.id;
  }

  static void convert_to_custom(const ros_message_type & source, custom_type & destination)
  {
    destination.id = source.id;
  }
};

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(Heartbeat, core_interfaces::msg::Heartbeat);

using HeartbeatDisconnect = core_interfaces::msg::HeartbeatDisconnect;

using HeartbeatConnect = core_interfaces::srv::HeartbeatConnect;
