#pragma once

#include <string>
#include <vector>

#include "nav_interfaces/msg/location.hpp"
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

TYPE_ADAPTER_OPEN(Location, nav_interfaces::msg::Location)
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

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(Location, nav_interfaces::msg::Location);