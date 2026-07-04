#include "coarse.hpp"

void CoarseNode::fix_cb(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  if (msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) {
    RCLCPP_WARN(get_logger(), "No GPS fix");
    return;
  }

  if (!last_location_) {
    last_location_ = std::make_shared<auto_msgs::msg::Location>();
  }

  last_location_->latitude = msg->latitude;
  last_location_->longitude = msg->longitude;
  last_location_->altitude = msg->altitude;
}

rclcpp_action::GoalResponse CoarseNode::goto_goal_cb(
  const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const GoTo::Goal> goal)
{
  if (!last_location_) {
    RCLCPP_WARN(get_logger(), "No GPS fix, rejecting goto goal");
    return rclcpp_action::GoalResponse::REJECT;
  }

  target_ = std::make_shared<auto_msgs::msg::Target>(goal->target);
  plan_.reset();
  auto uuid_str = rclcpp_action::to_string(uuid);
  RCLCPP_INFO(get_logger(), "Accepting goto goal %s", uuid_str.c_str());

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse CoarseNode::goto_cancel_cb(
  const std::shared_ptr<GoalHandleGoTo> goal_handle)
{
  auto uuid_str = rclcpp_action::to_string(goal_handle->get_goal_id());
  RCLCPP_INFO(get_logger(), "Cancelling goto goal %s", uuid_str.c_str());

  return rclcpp_action::CancelResponse::ACCEPT;
}

void CoarseNode::goto_accepted_cb(const std::shared_ptr<GoalHandleGoTo> goal_handle)
{
  // Abort any existing goal
  if (goto_goal_handle_ && goto_goal_handle_->is_executing()) {
    goto_goal_handle_->abort(std::make_shared<GoTo::Result>());
  }

  goto_goal_handle_ = goal_handle;
  pathfind_goal_handle_.reset();

  if (!pathfind_client_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_ERROR(get_logger(), "Pathfind action server not available");
    goto_goal_handle_->abort(std::make_shared<GoTo::Result>());
    goto_goal_handle_.reset();
    target_.reset();

    return;
  }

  using namespace std::placeholders;

  auto pathfind_goal_opts = rclcpp_action::Client<Pathfind>::SendGoalOptions();
  pathfind_goal_opts.goal_response_callback =
    std::bind(&CoarseNode::pathfind_goal_res_cb, this, _1);
  pathfind_goal_opts.result_callback = std::bind(&CoarseNode::pathfind_res_cb, this, _1);

  auto pathfind_goal = Pathfind::Goal();
  pathfind_goal.current = *last_location_;
  pathfind_goal.target = *target_;

  pathfind_client_->async_send_goal(pathfind_goal, pathfind_goal_opts);
}

void CoarseNode::pathfind_goal_res_cb(const GoalHandlePathfind::SharedPtr & goal_handle)
{
  if (!goal_handle) {
    RCLCPP_ERROR(get_logger(), "Pathfind goal rejected");
    goto_goal_handle_->abort(std::make_shared<GoTo::Result>());
    goto_goal_handle_.reset();
    target_.reset();

    return;
  }

  RCLCPP_INFO(get_logger(), "Pathfind goal accepted");
  pathfind_goal_handle_ = goal_handle;
}

void CoarseNode::pathfind_res_cb(const GoalHandlePathfind::WrappedResult & result)
{
  if (result.code == rclcpp_action::ResultCode::ABORTED) {
    if (pathfind_goal_handle_ && result.goal_id == pathfind_goal_handle_->get_goal_id()) {
      RCLCPP_ERROR(get_logger(), "Pathfind goal aborted");
      goto_goal_handle_->abort(std::make_shared<GoTo::Result>());
      goto_goal_handle_.reset();
      target_.reset();
      pathfind_goal_handle_.reset();
    }

    return;
  } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
    if (goto_goal_handle_) {
      // This should not happen, but would cause a failure if it did
      RCLCPP_ERROR(
        get_logger(), "Pathfind goal canceled while handle still exists. This should not happen.");
      goto_goal_handle_->abort(std::make_shared<GoTo::Result>());
      goto_goal_handle_.reset();
    }

    return;
  }

  RCLCPP_INFO(
    get_logger(), "Pathfind result received with %ld waypoints",
    result.result->plan.waypoints.size());

  pathfind_goal_handle_.reset();
  plan_ = std::make_shared<auto_msgs::msg::Plan>(result.result->plan);
}

void CoarseNode::goto_check()
{
  if (!goto_goal_handle_) {
    return;
  }

  if (goto_goal_handle_->is_canceling()) {
    goto_goal_handle_->canceled(std::make_shared<GoTo::Result>());
    goto_goal_handle_.reset();

    if (pathfind_goal_handle_) {
      pathfind_client_->async_cancel_goal(pathfind_goal_handle_);
    }

    return;
  }

  if (!plan_) {
    return;
  }

  goto_goal_handle_->succeed(std::make_shared<GoTo::Result>());
  goto_goal_handle_.reset();
}

CoarseNode::CoarseNode() : Node("coarse_node", "auto")
{
  using namespace std::placeholders;

  fix_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
    "/fix", 10, std::bind(&CoarseNode::fix_cb, this, _1));
  pathfind_client_ = rclcpp_action::create_client<Pathfind>(this, "pathfind");

  goto_server_ = rclcpp_action::create_server<GoTo>(
    this, "goto", std::bind(&CoarseNode::goto_goal_cb, this, _1, _2),
    std::bind(&CoarseNode::goto_cancel_cb, this, _1),
    std::bind(&CoarseNode::goto_accepted_cb, this, _1));

  goto_check_timer_ =
    this->create_wall_timer(std::chrono::seconds(1), std::bind(&CoarseNode::goto_check, this));

  // Test init of location
  last_location_ = std::make_shared<auto_msgs::msg::Location>();
  last_location_->latitude = 38.40645261293369;
  last_location_->longitude = -110.79137336968033;
}