#include "pathfinder.hpp"

void Pathfinder::onPathfinderCheck()
{
  if (!pathfinding) {
    return;
  }

  if (pathfinderFuture.wait_for(std::chrono::milliseconds(10)) != std::future_status::ready) {
    return;
  }

  auto result = pathfinderFuture.get();

  if (result.second.empty() == false) {
    RCLCPP_ERROR(this->get_logger(), "Pathfinding failed: %s", result.second.c_str());
    pathfinding = false;
    return;
  }

  RCLCPP_INFO(this->get_logger(), "Pathfinding complete");

  Plan plan;
  plan.waypoints = result.first;

  plan_pub->publish(plan);

  // #ifdef DEBUG
  debugKML(static_dir, current_location, Plan{result.first});
  // #endif

  pathfinding = false;
}

void Pathfinder::fix_cb(const FixMsg::SharedPtr msg)
{
  current_location.latitude = msg->latitude;
  current_location.longitude = msg->longitude;
}

void Pathfinder::target_cb(const Target::SharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "Received new target: %s", msg->to_string().c_str());

  if (pathfinding) {
    RCLCPP_WARN(this->get_logger(), "Canceling current pathfinding to start the new one");

    pathfinding = false;
    pathfinderFuture.wait();
  }

  pathfinding = true;

  pathfinderFuture = std::async(std::launch::async, [this, msg]() {
    Search search(this->site);

    return search.findPath(current_location, msg->location, pathfinding);
  });
}

Pathfinder::Pathfinder() : Node("pathfinder")
{
  using namespace std::placeholders;

  auto site_name = this->declare_parameter("site_name", rclcpp::PARAMETER_STRING);

  SiteLoader loader(static_dir);

  this->site = loader.load(site_name.get<std::string>());

  pathfinderCheckTimer = this->create_wall_timer(
    std::chrono::milliseconds(500), std::bind(&Pathfinder::onPathfinderCheck, this));

  plan_pub = this->create_publisher<Plan>("pathfinder/plan", 10);
  fix_sub = this->create_subscription<FixMsg>("fix", 10, std::bind(&Pathfinder::fix_cb, this, _1));
  target_sub = this->create_subscription<Target>(
    "pathfinder/target", 10, std::bind(&Pathfinder::target_cb, this, _1));

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
