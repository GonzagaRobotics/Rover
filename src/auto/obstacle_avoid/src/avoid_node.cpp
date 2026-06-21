#include "avoid_node.hpp"

AvoidNode::AvoidNode() : rclcpp::Node("avoid", "auto")
{
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
  tf_s_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  // TODO: Set these
  robot_crop_box_.setMin(Eigen::Vector4f(0.f, 0.f, 0.f, 0.f));
  robot_crop_box_.setMax(Eigen::Vector4f(0.f, 0.f, 0.f, 0.f));

  make_transforms();

  using std::placeholders::_1;

  pcl_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
    "/unilidar/cloud", rclcpp::SensorDataQoS(), std::bind(&AvoidNode::pcl_cb, this, _1));
}

void AvoidNode::make_transforms()
{
  geometry_msgs::msg::TransformStamped t;
  t.header.stamp = this->get_clock()->now();
  t.header.frame_id = "base_link";
  t.child_frame_id = "unilidar_lidar";

  // TODO: Set these
  t.transform.translation.x = 0.0;
  t.transform.translation.y = 0.0;
  t.transform.translation.z = 0.0;

  tf2::Quaternion q;
  q.setRPY(0.0, 0.0, 0.0);
  t.transform.rotation.w = q.w();
  t.transform.rotation.x = q.x();
  t.transform.rotation.y = q.y();
  t.transform.rotation.z = q.z();

  tf_s_broadcaster_->sendTransform(t);
}

void AvoidNode::pcl_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
  geometry_msgs::msg::TransformStamped transform;

  try {
    transform = tf_buffer_->lookupTransform(
      "base_link", msg->header.frame_id, msg->header.stamp, rclcpp::Duration::from_seconds(0.1));
  } catch (tf2::TransformException & ex) {
    RCLCPP_ERROR(this->get_logger(), "Transform error: %s", ex.what());
    return;
  }

  auto cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromROSMsg(*msg, *cloud);

  // Transform the cloud to the robot frame, and crop out points that are on the robot
  pcl_ros::transformPointCloud(*cloud, *cloud, transform);
  robot_crop_box_.setInputCloud(cloud);
  robot_crop_box_.filter(*cloud);

  auto obstacles = pgr_.remove_ground(cloud);
}
