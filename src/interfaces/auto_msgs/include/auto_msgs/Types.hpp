#pragma once

#include <string>
#include <vector>

#include "auto_msgs/msg/location.hpp"
#include "auto_msgs/msg/plan.hpp"
#include "auto_msgs/msg/state.hpp"
#include "auto_msgs/msg/target.hpp"
#include "auto_msgs/srv/get_state.hpp"
#include "auto_msgs/srv/instruct.hpp"
#include "auto_msgs/srv/set_target.hpp"
#include "rclcpp/rclcpp.hpp"

#define TYPE_ADAPTER_OPEN(custom, ros)     \
  template <>                              \
  struct rclcpp::TypeAdapter<custom, ros>  \
  {                                        \
    using is_specialized = std::true_type; \
    using custom_type = custom;            \
    using ros_message_type = ros;

#define TYPE_ADAPTER_CLOSE() \
  }                          \
  ;

#define TO_ROS_OPEN()                                                                            \
  static void convert_to_ros_message(const custom_type & source, ros_message_type & destination) \
  {
#define TO_ROS_CLOSE() }

#define TO_CUSTOM_OPEN()                                                                    \
  static void convert_to_custom(const ros_message_type & source, custom_type & destination) \
  {
#define TO_CUSTOM_CLOSE() }

/**
 * The types of targets that AutoNav can navigate to.
 */
enum class TargetType : uint8_t
{
  /** High precision GNSS coordinates. */
  GNSS,
  /** Post marked with ARUCO tags. */
  ARUCO,
  /** Water bottle. */
  BOTTLE,
  /** Rubber mallet. */
  MALLET,
  /** Rock hammer. */
  HAMMER
};

/**
 * A struct representing a location on Earth.
 */
struct Location
{
  using SharedPtr = std::shared_ptr<Location>;

  /** The latitude in degrees. */
  double latitude;
  /** The longitude in degrees. */
  double longitude;
  /** The altitude in meters. */
  double altitude;

  std::string to_string() const
  {
    return "(" + std::to_string(latitude) + ", " + std::to_string(longitude) + ", " +
           std::to_string(altitude) + ")";
  }
};

/**
 * A target that AutoNav can navigate to.
 */
struct Target
{
  using SharedPtr = std::shared_ptr<Target>;

  /** Where the target is located. */
  Location location;
  /** The type of target. */
  TargetType type;

  std::string to_string() const
  {
    std::string typeString;

    switch (type) {
      case TargetType::GNSS:
        typeString = "GNSS Coords";
        break;
      case TargetType::ARUCO:
        typeString = "ARUCO Post";
        break;
      case TargetType::MALLET:
        typeString = "Rubber Mallet";
        break;
      case TargetType::BOTTLE:
        typeString = "Water Bottle";
        break;
      case TargetType::HAMMER:
        typeString = "Rock Hammer";
        break;
    }

    return typeString + " at " + location.to_string();
  }
};

/**
 * A plan to navigate to a target.
 */
struct Plan
{
  using SharedPtr = std::shared_ptr<Plan>;

  /** The waypoints to reach the target. The final element is the target. */
  std::vector<Location> waypoints;
};

// Location

TYPE_ADAPTER_OPEN(Location, auto_msgs::msg::Location)
TO_ROS_OPEN()
destination.latitude = source.latitude;
destination.longitude = source.longitude;
destination.altitude = source.altitude;
TO_ROS_CLOSE()

TO_CUSTOM_OPEN()
destination.latitude = source.latitude;
destination.longitude = source.longitude;
destination.altitude = source.altitude;
TO_CUSTOM_CLOSE()
TYPE_ADAPTER_CLOSE()

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(Location, auto_msgs::msg::Location);

// Target

TYPE_ADAPTER_OPEN(Target, auto_msgs::msg::Target)
TO_ROS_OPEN()
rclcpp::TypeAdapter<Location, auto_msgs::msg::Location>::convert_to_ros_message(
  source.location, destination.location);

destination.type = static_cast<uint8_t>(source.type);
TO_ROS_CLOSE()

TO_CUSTOM_OPEN()
rclcpp::TypeAdapter<Location, auto_msgs::msg::Location>::convert_to_custom(
  source.location, destination.location);

destination.type = static_cast<TargetType>(source.type);
TO_CUSTOM_CLOSE()
TYPE_ADAPTER_CLOSE()

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(Target, auto_msgs::msg::Target);

// Plan

TYPE_ADAPTER_OPEN(Plan, auto_msgs::msg::Plan)
TO_ROS_OPEN()
for (const auto & waypoint : source.waypoints) {
  auto_msgs::msg::Location location;
  rclcpp::TypeAdapter<Location, auto_msgs::msg::Location>::convert_to_ros_message(
    waypoint, location);

  destination.waypoints.push_back(location);
}
TO_ROS_CLOSE()

TO_CUSTOM_OPEN()
for (const auto & loc : source.waypoints) {
  Location waypoint;
  rclcpp::TypeAdapter<Location, auto_msgs::msg::Location>::convert_to_custom(loc, waypoint);

  destination.waypoints.push_back(waypoint);
}
TO_CUSTOM_CLOSE()
TYPE_ADAPTER_CLOSE()

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(Plan, auto_msgs::msg::Plan);