#pragma once

#include <atomic>
#include <functional>
#include <future>
#include <optional>
#include <stdexcept>
#include <thread>
#include <utility>
#include <vector>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "auto_msgs/action/pathfind.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "search.hpp"
#include "site.hpp"
#include "site_loader.hpp"

using PathfindAction = auto_msgs::action::Pathfind;
using PathfindGoalHandle = rclcpp_action::ServerGoalHandle<PathfindAction>;

/**
 * Finds paths through a site.
 */
class Pathfinder : public rclcpp::Node
{
private:
  const std::string static_dir =
    ament_index_cpp::get_package_share_directory("pathfinder") + "/sites/";

  /** The site we are currently on. */
  std::shared_ptr<Site> site;

  /** A flag indicating if the current pathfinding is canceled. */
  bool canceled = false;

  /** Are we currently pathfinding? */
  std::atomic<bool> pathfinding;

  /** The current goal handle. */
  std::shared_ptr<PathfindGoalHandle> current_goal_handle_;

  /** The future for the pathfinder's search. */
  std::future<std::pair<std::vector<Location>, std::string>> pathfinderFuture;

  /** The timer that checks if pathfinding is complete. */
  rclcpp::TimerBase::SharedPtr pathfinderCheckTimer;

  /** The action server for handling pathfinding requests. */
  rclcpp_action::Server<PathfindAction>::SharedPtr action_server_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID &, std::shared_ptr<const PathfindAction::Goal>);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<PathfindGoalHandle> goal_handle);

  void handle_accepted(const std::shared_ptr<PathfindGoalHandle> goal_handle);

  void onPathfinderCheck();

public:
  Pathfinder();

  ~Pathfinder();
};