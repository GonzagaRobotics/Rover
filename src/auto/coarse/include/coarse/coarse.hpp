#pragma once

#include <chrono>

#include "auto_msgs/action/go_to.hpp"
#include "auto_msgs/action/pathfind.hpp"
#include "auto_msgs/msg/location.hpp"
#include "auto_msgs/msg/plan.hpp"
#include "auto_msgs/msg/target.hpp"
#include "json.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/empty.hpp"
#include "utils.hpp"

#define BIND(x) std::bind(&CoarseNode::x, this, std::placeholders::_1)

using json = nlohmann::json;

using EmptyMsg = std_msgs::msg::Empty;
using FixMsg = sensor_msgs::msg::NavSatFix;
using LocMsg = auto_msgs::msg::Location;

using Pathfind = auto_msgs::action::Pathfind;
using GoalHandlePathfind = rclcpp_action::ClientGoalHandle<Pathfind>;
using GoTo = auto_msgs::action::GoTo;
using GoalHandleGoTo = rclcpp_action::ServerGoalHandle<GoTo>;

class CoarseNode : public rclcpp::Node
{
private:
  LocMsg::SharedPtr location_;
  auto_msgs::msg::Target::SharedPtr target_;
  auto_msgs::msg::Plan::SharedPtr plan_;
  size_t wp_index_ = 0;

  std::shared_ptr<GoalHandleGoTo> goto_goal_handle_;
  std::shared_ptr<GoalHandlePathfind> pathfind_goal_handle_;

  /// @brief Needed to cancel the running action from the Control System
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr goto_stop_sub_;

  rclcpp::Subscription<FixMsg>::SharedPtr fix_sub_;
  rclcpp::Publisher<LocMsg>::SharedPtr fine_goal_pub_;
  rclcpp::Publisher<EmptyMsg>::SharedPtr fine_stop_pub_;

  rclcpp_action::Client<Pathfind>::SharedPtr pathfind_client_;
  rclcpp_action::Server<GoTo>::SharedPtr goto_server_;

  rclcpp::TimerBase::SharedPtr goto_check_timer_;

  void fix_cb(const FixMsg::SharedPtr msg);
  void stop_cb(const EmptyMsg::SharedPtr);

  rclcpp_action::GoalResponse goto_goal_cb(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const GoTo::Goal> goal);
  rclcpp_action::CancelResponse goto_cancel_cb(const std::shared_ptr<GoalHandleGoTo> goal_handle);
  void goto_accepted_cb(const std::shared_ptr<GoalHandleGoTo> goal_handle);

  void pathfind_goal_res_cb(const GoalHandlePathfind::SharedPtr & goal_handle);
  void pathfind_res_cb(const GoalHandlePathfind::WrappedResult & result);

  void goto_check();

  void goto_step();

public:
  CoarseNode();
};