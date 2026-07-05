#pragma once

#include "auto_msgs/msg/location.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/empty.hpp"

class FineNode : public rclcpp::Node
{
private:
  auto_msgs::msg::Location::SharedPtr location_;
  auto_msgs::msg::Location::SharedPtr goal_location_;

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;

  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_sub_;
  rclcpp::Subscription<auto_msgs::msg::Location>::SharedPtr goal_sub_;

  rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr location_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  void fix_cb(const sensor_msgs::msg::NavSatFix::SharedPtr msg);
  void imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg);

  void goal_cb(const auto_msgs::msg::Location::SharedPtr msg);
  void stop_cb(const std_msgs::msg::Empty::SharedPtr);

  void timer_cb();

public:
  FineNode();
};