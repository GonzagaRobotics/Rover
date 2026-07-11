#pragma once

#include <Eigen/Dense>
#include <functional>
#include <rclcpp/rclcpp.hpp>

#include "auto_msgs/msg/location.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "obstacles.hpp"
#include "pcl/filters/crop_box.h"
#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl_conversions/pcl_conversions.h"
#include "pcl_ros/transforms.hpp"
#include "pgr.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/empty.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/static_transform_broadcaster.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "visualization_msgs/msg/marker.hpp"

#define BIND(f) std::bind(&FineNode::f, this, std::placeholders::_1)

using ImuMsg = sensor_msgs::msg::Imu;
using FixMsg = sensor_msgs::msg::NavSatFix;
using PclMsg = sensor_msgs::msg::PointCloud2;
using EmptyMsg = std_msgs::msg::Empty;
using LocMsg = auto_msgs::msg::Location;

class FineNode : public rclcpp::Node
{
private:
  auto_msgs::msg::Location::SharedPtr location_;
  auto_msgs::msg::Location::SharedPtr goal_location_;

  PGR pgr_;
  Obstacles obstacles_;
  // testing
  rclcpp::Publisher<PclMsg>::SharedPtr pcl_pub_;
  // testing
  rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr grid_pub_;
  // testing
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_pub_;

  rclcpp::Subscription<PclMsg>::SharedPtr pcl_sub_;
  rclcpp::Subscription<FixMsg>::SharedPtr fix_sub_;
  rclcpp::Subscription<ImuMsg>::SharedPtr imu_sub_;

  rclcpp::Subscription<EmptyMsg>::SharedPtr stop_sub_;
  rclcpp::Subscription<LocMsg>::SharedPtr goal_sub_;

  // for testing, publish a fake location to simulate movement
  rclcpp::Publisher<FixMsg>::SharedPtr location_pub_;

  rclcpp::TimerBase::SharedPtr timer_;

  void fix_cb(const FixMsg::SharedPtr msg);
  void imu_cb(const ImuMsg::SharedPtr msg);
  void pcl_cb(const PclMsg::SharedPtr msg);

  void goal_cb(const LocMsg::SharedPtr msg);
  void stop_cb(const EmptyMsg::SharedPtr);

  void timer_cb();

public:
  FineNode();
};