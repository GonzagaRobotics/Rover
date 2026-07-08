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
}

void FineNode::imu_cb(const sensor_msgs::msg::Imu::SharedPtr) {}

void FineNode::pcl_cb(const PclMsg::SharedPtr msg)
{
  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(*msg, *cloud);

  // Crop out the robot
  pcl::CropBox<pcl::PointXYZ> crop_box;
  crop_box.setInputCloud(cloud);
  crop_box.setMin(Eigen::Vector4f(-0.5, -0.5, -1.0, 1.0));
  crop_box.setMax(Eigen::Vector4f(0.5, 0.5, 1.0, 1.0));
  crop_box.setNegative(true);
  crop_box.filter(*cloud);

  // Crop out points that are too far away
  crop_box.setMin(Eigen::Vector4f(-10.0, -10.0, -10.0, 1.0));
  crop_box.setMax(Eigen::Vector4f(10.0, 10.0, 10.0, 1.0));
  crop_box.setNegative(false);
  crop_box.filter(*cloud);

  auto obstacle_cloud = pgr_.remove_ground(cloud);

  PclMsg output_msg;
  pcl::toROSMsg(obstacle_cloud, output_msg);
  output_msg.header = msg->header;
  pcl_pub_->publish(output_msg);
}

void FineNode::goal_cb(const auto_msgs::msg::Location::SharedPtr msg)
{
  if (!goal_location_) {
    goal_location_ = std::make_shared<auto_msgs::msg::Location>();
  }

  goal_location_->latitude = msg->latitude;
  goal_location_->longitude = msg->longitude;
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

  pcl_pub_ = this->create_publisher<PclMsg>("fine_pcl", 10);

  //   fix_sub_ = this->create_subscription<FixMsg>("/fix", 10, BIND(fix_cb));
  imu_sub_ = this->create_subscription<ImuMsg>("/imu", 10, BIND(imu_cb));
  pcl_sub_ = this->create_subscription<PclMsg>("/autonomy_module_lidar/points", 10, BIND(pcl_cb));

  stop_sub_ = this->create_subscription<EmptyMsg>("fine_stop", 10, BIND(stop_cb));
  goal_sub_ = this->create_subscription<LocMsg>("fine_goal", 10, BIND(goal_cb));

  location_pub_ = this->create_publisher<FixMsg>("/fix", 10);

  timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&FineNode::timer_cb, this));
}