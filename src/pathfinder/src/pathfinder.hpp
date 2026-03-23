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
#include "rclcpp/rclcpp.hpp"
#include "search.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "site.hpp"
#include "site_loader.hpp"

// #ifdef DEBUG
#include "debug/debug_kml.hpp"
// #endif

using FixMsg = sensor_msgs::msg::NavSatFix;

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

  Location current_location;

  /** Are we currently pathfinding? */
  std::atomic<bool> pathfinding;

  /** The future for the pathfinder's search. */
  std::future<std::pair<std::vector<Location>, std::string>> pathfinderFuture;

  /** The timer that checks if pathfinding is complete. */
  rclcpp::TimerBase::SharedPtr pathfinderCheckTimer;

  rclcpp::Subscription<FixMsg>::SharedPtr fix_sub;
  rclcpp::Subscription<Target>::SharedPtr target_sub;

  rclcpp::Publisher<Plan>::SharedPtr plan_pub;

  void onPathfinderCheck();

  void fix_cb(const FixMsg::SharedPtr msg);
  void target_cb(const Target::SharedPtr msg);

public:
  Pathfinder();

  ~Pathfinder();
};