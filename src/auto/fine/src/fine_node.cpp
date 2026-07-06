#include "fine_node.hpp"

void FineNode::fix_cb(const sensor_msgs::msg::NavSatFix::SharedPtr msg)
{
  if (msg->status.status == sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX) {
    return;
  }

  if (!location_) {
    location_ = std::make_shared<auto_msgs::msg::Location>();
  }

  location_->latitude = msg->latitude;
  location_->longitude = msg->longitude;
  location_->altitude = msg->altitude;
}

void FineNode::imu_cb(const sensor_msgs::msg::Imu::SharedPtr msg) {}

void FineNode::goal_cb(const auto_msgs::msg::Location::SharedPtr msg)
{
  if (!goal_location_) {
    goal_location_ = std::make_shared<auto_msgs::msg::Location>();
  }

  goal_location_->latitude = msg->latitude;
  goal_location_->longitude = msg->longitude;
  goal_location_->altitude = msg->altitude;
}

void FineNode::stop_cb(const std_msgs::msg::Empty::SharedPtr) { goal_location_.reset(); }

void FineNode::timer_cb()
{
  // Move towards the goal location, faking movement as a test
  if (!goal_location_ || !location_) {
    return;
  }

  double lat_diff = goal_location_->latitude - location_->latitude;
  double lon_diff = goal_location_->longitude - location_->longitude;

  double bearing = std::atan2(lon_diff, lat_diff);
  double lat_move = 0.0001 * std::cos(bearing);
  double lon_move = 0.0001 * std::sin(bearing);

  location_->latitude += std::abs(lat_move) < std::abs(lat_diff) ? lat_move : lat_diff;
  location_->longitude += std::abs(lon_move) < std::abs(lon_diff) ? lon_move : lon_diff;

  std::cout << "Moved to: (" << location_->latitude << ", " << location_->longitude << ")\n";

  sensor_msgs::msg::NavSatFix fix_msg;
  fix_msg.latitude = location_->latitude;
  fix_msg.longitude = location_->longitude;
  fix_msg.status.status = sensor_msgs::msg::NavSatStatus::STATUS_FIX;

  location_pub_->publish(fix_msg);
}

FineNode::FineNode() : rclcpp::Node("fine_node", "auto")
{
  // Test init of location
  location_ = std::make_shared<auto_msgs::msg::Location>();
  location_->latitude = 38.40645261293369;
  location_->longitude = -110.79137336968033;

  using namespace std::placeholders;

  //   fix_sub_ = this->create_subscription<sensor_msgs::msg::NavSatFix>(
  //     "/fix", 10, std::bind(&FineNode::fix_cb, this, _1));
  imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
    "/imu", 10, std::bind(&FineNode::imu_cb, this, _1));

  stop_sub_ = this->create_subscription<std_msgs::msg::Empty>(
    "fine_stop", 10, std::bind(&FineNode::stop_cb, this, _1));
  goal_sub_ = this->create_subscription<auto_msgs::msg::Location>(
    "fine_goal", 10, std::bind(&FineNode::goal_cb, this, _1));

  location_pub_ = this->create_publisher<sensor_msgs::msg::NavSatFix>("/fix", 10);

  timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&FineNode::timer_cb, this));
}