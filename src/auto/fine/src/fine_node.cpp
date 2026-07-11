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

  auto obstacle_cloud = pgr_.remove_ground(cloud).makeShared();

  PclMsg output_msg;
  pcl::toROSMsg(*obstacle_cloud, output_msg);
  output_msg.header = msg->header;
  pcl_pub_->publish(output_msg);

  obstacles_.update(obstacle_cloud, Eigen::Vector2f::Zero());
  auto rep = obstacles_.get_grid();

  // Publish the vector as a marker for visualization
  auto rep_vec = obstacles_.compute_vector();
  visualization_msgs::msg::Marker marker_msg;
  marker_msg.header = msg->header;
  marker_msg.ns = "obstacle_vector";
  marker_msg.id = 0;
  marker_msg.type = visualization_msgs::msg::Marker::ARROW;
  marker_msg.action = visualization_msgs::msg::Marker::ADD;
  marker_msg.pose.orientation.w = 1.0;
  marker_msg.scale.x = 0.1;
  marker_msg.scale.y = 0.2;
  marker_msg.color.a = 1.0;
  marker_msg.color.r = 1.0;
  marker_msg.points.push_back(geometry_msgs::msg::Point());
  geometry_msgs::msg::Point end_point;
  end_point.x = rep_vec.x();
  end_point.y = rep_vec.y();
  marker_msg.points.push_back(end_point);
  marker_pub_->publish(marker_msg);

  nav_msgs::msg::OccupancyGrid grid_msg;
  grid_msg.header = msg->header;
  grid_msg.info.resolution = 0.1;
  grid_msg.info.width = rep.cols();
  grid_msg.info.height = rep.rows();
  grid_msg.info.origin.position.x = -10.0;
  grid_msg.info.origin.position.y = -10.0;
  grid_msg.data.resize(rep.size());
  for (int i = 0; i < rep.rows(); ++i) {
    for (int j = 0; j < rep.cols(); ++j) {
      grid_msg.data[j * rep.cols() + i] = static_cast<int8_t>(rep(i, j));
    }
  }
  grid_pub_->publish(grid_msg);
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

  grid_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("fine_grid", 10);
  pcl_pub_ = this->create_publisher<PclMsg>("fine_pcl", 10);
  marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>("fine_marker", 10);

  //   fix_sub_ = this->create_subscription<FixMsg>("/fix", 10, BIND(fix_cb));
  imu_sub_ = this->create_subscription<ImuMsg>("/imu", 10, BIND(imu_cb));
  pcl_sub_ = this->create_subscription<PclMsg>("/autonomy_module_lidar/points", 10, BIND(pcl_cb));

  stop_sub_ = this->create_subscription<EmptyMsg>("fine_stop", 10, BIND(stop_cb));
  goal_sub_ = this->create_subscription<LocMsg>("fine_goal", 10, BIND(goal_cb));

  location_pub_ = this->create_publisher<FixMsg>("/fix", 10);

  timer_ = this->create_wall_timer(std::chrono::seconds(1), std::bind(&FineNode::timer_cb, this));
}