#include "coarse.hpp"

void CoarseNode::fix_cb(const FixMsg::SharedPtr msg)
{
  if (msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) {
    RCLCPP_WARN(get_logger(), "No GPS fix");
    return;
  }

  if (!location_) {
    location_ = std::make_shared<LocMsg>();
  }

  location_->latitude = msg->latitude;
  location_->longitude = msg->longitude;
}

rclcpp_action::GoalResponse CoarseNode::goto_goal_cb(
  const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const GoTo::Goal> goal)
{
  if (!location_) {
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
    auto res_msg = std::make_shared<GoTo::Result>();
    res_msg->state.state = StateMsg::FAILURE;
    goto_goal_handle_->abort(res_msg);
  }

  goto_goal_handle_ = goal_handle;
  pathfind_goal_handle_.reset();

  if (!pathfind_client_->wait_for_action_server(std::chrono::seconds(1))) {
    RCLCPP_ERROR(get_logger(), "Pathfind action server not available");

    auto res_msg = std::make_shared<GoTo::Result>();
    res_msg->state.state = StateMsg::FAILURE;
    goto_goal_handle_->abort(res_msg);

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
  pathfind_goal.current = *location_;
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

  auto feedback_msg = std::make_shared<GoTo::Feedback>();
  feedback_msg->state.state = StateMsg::PLANNING;
  goto_goal_handle_->publish_feedback(feedback_msg);

  pathfind_goal_handle_ = goal_handle;
}

void CoarseNode::pathfind_res_cb(const GoalHandlePathfind::WrappedResult & result)
{
  if (result.code == rclcpp_action::ResultCode::ABORTED) {
    if (pathfind_goal_handle_ && result.goal_id == pathfind_goal_handle_->get_goal_id()) {
      RCLCPP_ERROR(get_logger(), "Pathfind goal aborted");

      auto res_msg = std::make_shared<GoTo::Result>();
      res_msg->state.state = StateMsg::FAILURE;
      goto_goal_handle_->abort(res_msg);

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

      auto res_msg = std::make_shared<GoTo::Result>();
      res_msg->state.state = StateMsg::FAILURE;
      goto_goal_handle_->abort(res_msg);

      goto_goal_handle_.reset();
    }

    return;
  }

  RCLCPP_INFO(
    get_logger(), "Pathfind result received with %ld waypoints",
    result.result->plan.waypoints.size());

  json feedback;
  feedback["wp"] = json::array();
  for (const auto & wp : result.result->plan.waypoints) {
    json wp_json = {wp.latitude, wp.longitude};
    feedback["wp"].push_back(wp_json);
  }

  auto feedback_msg = std::make_shared<GoTo::Feedback>();
  feedback_msg->state.state = StateMsg::PLANNING;
  feedback_msg->status = feedback.dump();
  goto_goal_handle_->publish_feedback(feedback_msg);

  pathfind_goal_handle_.reset();
  plan_ = std::make_shared<auto_msgs::msg::Plan>(result.result->plan);
  wp_index_ = 0;
}

void CoarseNode::goto_check()
{
  if (!goto_goal_handle_) {
    return;
  }

  if (goto_goal_handle_->is_canceling()) {
    auto res_msg = std::make_shared<GoTo::Result>();
    res_msg->state.state = StateMsg::FAILURE;
    goto_goal_handle_->canceled(res_msg);
    goto_goal_handle_.reset();

    if (pathfind_goal_handle_) {
      pathfind_client_->async_cancel_goal(pathfind_goal_handle_);
    }

    fine_stop_pub_->publish(EmptyMsg());
    return;
  }

  if (!plan_) {
    return;
  }

  goto_step();
}

void CoarseNode::goto_step()
{
  if (plan_->waypoints.empty()) {
    RCLCPP_ERROR(get_logger(), "Plan is empty. Cannot proceed.");
    auto res_msg = std::make_shared<GoTo::Result>();
    res_msg->state.state = StateMsg::FAILURE;
    goto_goal_handle_->abort(res_msg);

    goto_goal_handle_.reset();
    fine_stop_pub_->publish(EmptyMsg());
    return;
  }

  auto next_wp = plan_->waypoints[wp_index_];
  bool last_wp = wp_index_ == plan_->waypoints.size() - 1;  // The last waypoint is the target
  double dist = loc_dist(*location_, next_wp);

  // TODO: Stop or move the rover

  if (!last_wp) {
    if (dist < 10) {
      wp_index_++;
    }

    fine_goal_pub_->publish(plan_->waypoints[wp_index_]);
    json feedback = {{"wp", wp_index_}};
    auto feedback_msg = std::make_shared<GoTo::Feedback>();
    feedback_msg->state.state = StateMsg::TRAVELING;
    feedback_msg->status = feedback.dump();
    goto_goal_handle_->publish_feedback(feedback_msg);

    RCLCPP_INFO(get_logger(), "Distance to waypoint %ld: %.2f meters", wp_index_, dist);
    return;
  }

  RCLCPP_INFO(get_logger(), "Distance to target: %.2f meters", dist);

  // GNSS targets have a tighter tolerance because they are not visible to the camera
  if (target_->type == auto_msgs::msg::Target::TYPE_GNSS) {
    if (dist > 1.5) {
      return;
    }
  } else {
    if (dist > 5) {
      return;
    }
  }

  auto res_msg = std::make_shared<GoTo::Result>();
  res_msg->state.state = StateMsg::SUCCESS;
  goto_goal_handle_->succeed(res_msg);
  goto_goal_handle_.reset();
  fine_stop_pub_->publish(EmptyMsg());
}

CoarseNode::CoarseNode() : Node("coarse_node", "auto")
{
  using namespace std::placeholders;

  fix_sub_ = this->create_subscription<FixMsg>("/fix", 10, BIND(fix_cb));
  fine_goal_pub_ = this->create_publisher<LocMsg>("fine_goal", 10);
  fine_stop_pub_ = this->create_publisher<EmptyMsg>("fine_stop", 10);

  pathfind_client_ = rclcpp_action::create_client<Pathfind>(this, "pathfind");

  goto_server_ = rclcpp_action::create_server<GoTo>(
    this, "goto", std::bind(&CoarseNode::goto_goal_cb, this, _1, _2),
    std::bind(&CoarseNode::goto_cancel_cb, this, _1),
    std::bind(&CoarseNode::goto_accepted_cb, this, _1));

  goto_check_timer_ =
    this->create_wall_timer(std::chrono::seconds(1), std::bind(&CoarseNode::goto_check, this));

  // Test init of location
  location_ = std::make_shared<LocMsg>();
  location_->latitude = 38.40645261293369;
  location_->longitude = -110.79137336968033;
}