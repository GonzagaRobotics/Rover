#include <chrono>

#include "auto_msgs/action/go_to.hpp"
#include "auto_msgs/action/pathfind.hpp"
#include "auto_msgs/msg/location.hpp"
#include "auto_msgs/msg/plan.hpp"
#include "auto_msgs/msg/target.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

using Pathfind = auto_msgs::action::Pathfind;
using GoalHandlePathfind = rclcpp_action::ClientGoalHandle<Pathfind>;
using GoTo = auto_msgs::action::GoTo;
using GoalHandleGoTo = rclcpp_action::ServerGoalHandle<GoTo>;

class CoarseNode : public rclcpp::Node
{
private:
  auto_msgs::msg::Location::SharedPtr last_location_;
  auto_msgs::msg::Target::SharedPtr target_;
  auto_msgs::msg::Plan::SharedPtr plan_;

  std::shared_ptr<GoalHandleGoTo> goto_goal_handle_;
  std::shared_ptr<GoalHandlePathfind> pathfind_goal_handle_;

  rclcpp::Subscription<sensor_msgs::msg::NavSatFix>::SharedPtr fix_sub_;

  rclcpp_action::Client<Pathfind>::SharedPtr pathfind_client_;
  rclcpp_action::Server<GoTo>::SharedPtr goto_server_;

  rclcpp::TimerBase::SharedPtr goto_check_timer_;

  void fix_cb(const sensor_msgs::msg::NavSatFix::SharedPtr msg);

  rclcpp_action::GoalResponse goto_goal_cb(
    const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const GoTo::Goal> goal);
  rclcpp_action::CancelResponse goto_cancel_cb(const std::shared_ptr<GoalHandleGoTo> goal_handle);
  void goto_accepted_cb(const std::shared_ptr<GoalHandleGoTo> goal_handle);

  void pathfind_goal_res_cb(const GoalHandlePathfind::SharedPtr & goal_handle);
  void pathfind_res_cb(const GoalHandlePathfind::WrappedResult & result);

  void goto_check();

public:
  CoarseNode();
};