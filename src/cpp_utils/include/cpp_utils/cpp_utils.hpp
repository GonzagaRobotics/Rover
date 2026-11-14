#pragma once

#include <rclcpp/rclcpp.hpp>

template <typename T>
T declare_init_parameter(
  rclcpp::Node & node, const std::string & name, const T & default_value, const std::string & description = "")
{
  rcl_interfaces::msg::ParameterDescriptor param_desc;
  param_desc.read_only = true;
  param_desc.description = description;
  return node.declare_parameter<T>(name, default_value, param_desc);
}