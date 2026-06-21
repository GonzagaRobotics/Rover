#pragma once

#include <functional>
#include <memory>

#include "geometry_msgs/msg/transform_stamped.hpp"
#include "pcl/filters/crop_box.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"
#include "pgr.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"
#include "tf2_ros/transform_listener.hpp"

class AvoidNode : public rclcpp::Node
{
  PGR pgr_;
  pcl::CropBox<pcl::PointXYZ> robot_crop_box_;

  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_s_broadcaster_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pcl_sub_;

  void make_transforms();

  void pcl_cb(const sensor_msgs::msg::PointCloud2::SharedPtr msg);

public:
  AvoidNode();
};