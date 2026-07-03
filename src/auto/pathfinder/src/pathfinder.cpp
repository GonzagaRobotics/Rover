#include "pathfinder.hpp"

rclcpp_action::GoalResponse Pathfinder::handle_goal(
  const rclcpp_action::GoalUUID &, std::shared_ptr<const PathfindAction::Goal>)
{
  if (current_goal_handle_ && current_goal_handle_->is_executing()) {
    RCLCPP_WARN(this->get_logger(), "Canceling current pathfinding to start the new one");

    current_goal_handle_->abort(std::make_shared<PathfindAction::Result>());

    if (pathfinding) {
      pathfinding = false;
      pathfinderFuture.wait();
    }
  }

  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse Pathfinder::handle_cancel(
  const std::shared_ptr<PathfindGoalHandle> goal_handle)
{
  auto goal_id = rclcpp_action::to_string(goal_handle->get_goal_id());
  RCLCPP_INFO(this->get_logger(), "Canceling pathfinding %s", goal_id.c_str());

  if (current_goal_handle_ && current_goal_handle_->get_goal_id() != goal_handle->get_goal_id()) {
    RCLCPP_WARN(
      this->get_logger(), "Received cancel request for a goal that is not the current goal");
    return rclcpp_action::CancelResponse::REJECT;
  }

  canceled = true;

  return rclcpp_action::CancelResponse::ACCEPT;
}

void Pathfinder::handle_accepted(const std::shared_ptr<PathfindGoalHandle> goal_handle)
{
  auto goal_id = rclcpp_action::to_string(goal_handle->get_goal_id());
  RCLCPP_INFO(this->get_logger(), "Accepted pathfinding %s", goal_id.c_str());
  current_goal_handle_ = goal_handle;

  pathfinding = true;
  pathfinderFuture = std::async(std::launch::async, [this, goal_handle]() {
    Search search(this->site);

    auto goal_start = goal_handle->get_goal()->current;
    Location start;
    start.latitude = goal_start.latitude;
    start.longitude = goal_start.longitude;
    start.altitude = goal_start.altitude;

    auto goal_end = goal_handle->get_goal()->target;
    Location end;
    end.latitude = goal_end.location.latitude;
    end.longitude = goal_end.location.longitude;
    end.altitude = goal_end.location.altitude;

    return search.findPath(start, end, pathfinding);
  });
}

void Pathfinder::onPathfinderCheck()
{
  if (!pathfinding) {
    return;
  }

  if (canceled) {
    pathfinding = false;
    canceled = false;
    pathfinderFuture.wait();

    current_goal_handle_->canceled(std::make_shared<PathfindAction::Result>());
    current_goal_handle_.reset();
    return;
  }

  if (pathfinderFuture.wait_for(std::chrono::milliseconds(20)) != std::future_status::ready) {
    return;
  }

  auto result = pathfinderFuture.get();
  pathfinding = false;
  PathfindAction::Result result_msg{};

  if (result.second.empty() == false) {
    RCLCPP_ERROR(this->get_logger(), "Pathfinding failed: %s", result.second.c_str());
    result_msg.error = result.second;
    current_goal_handle_->abort(std::make_shared<PathfindAction::Result>(result_msg));

    return;
  }

  RCLCPP_INFO(this->get_logger(), "Pathfinding complete");

  for (const auto & location : result.first) {
    auto_msgs::msg::Location loc{};
    loc.latitude = location.latitude;
    loc.longitude = location.longitude;
    loc.altitude = location.altitude;
    result_msg.plan.waypoints.push_back(loc);
  }

  current_goal_handle_->succeed(std::make_shared<PathfindAction::Result>(result_msg));
}

Pathfinder::Pathfinder() : Node("pathfinder_node", "auto")
{
  using namespace std::placeholders;

  auto site_name = this->declare_parameter("site_name", rclcpp::PARAMETER_STRING);

  SiteLoader loader(static_dir);

  this->site = loader.load(site_name.get<std::string>());

  pathfinderCheckTimer = this->create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&Pathfinder::onPathfinderCheck, this));

  this->action_server_ = rclcpp_action::create_server<PathfindAction>(
    this, "pathfind", std::bind(&Pathfinder::handle_goal, this, _1, _2),
    std::bind(&Pathfinder::handle_cancel, this, _1),
    std::bind(&Pathfinder::handle_accepted, this, _1));

  RCLCPP_INFO(this->get_logger(), "Pathfinder ready");
}

Pathfinder::~Pathfinder()
{
  // If we are pathfinding, we need to stop
  if (pathfinding) {
    RCLCPP_WARN(this->get_logger(), "Canceling current pathfinding due to shutdown");

    pathfinding = false;
    pathfinderFuture.wait();
  }
}
