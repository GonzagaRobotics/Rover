#pragma once

#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "drive_interfaces/msg/drive_command.hpp"

struct DriveCommand
{
    using SharedPtr = std::shared_ptr<DriveCommand>;

    float forwardBackward;
    float leftRight;
};

template <>
struct rclcpp::TypeAdapter<DriveCommand, drive_interfaces::msg::DriveCommand>
{
    using is_specialized = std::true_type;
    using custom_type = DriveCommand;
    using ros_message_type = drive_interfaces::msg::DriveCommand;

    static void convert_to_ros_message(const custom_type &source, ros_message_type &destination)
    {
        destination.forward_backward = source.forwardBackward;
        destination.left_right = source.leftRight;
    }

    static void convert_to_custom(const ros_message_type &source, custom_type &destination)
    {
        destination.forwardBackward = source.forward_backward;
        destination.leftRight = source.left_right;
    }
};

RCLCPP_USING_CUSTOM_TYPE_AS_ROS_MESSAGE_TYPE(DriveCommand, drive_interfaces::msg::DriveCommand);