#ifndef NAV_SENSORS_H
#define NAV_SENSORS_H

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <cstdio>
#include "ArduinoJson-v7.4.2.h"
#include "rclcpp/rclcpp.hpp"
#include "nav_sensor_msgs/msg/location.hpp"
#include "nav_sensor_msgs/msg/orientation.hpp"
#include "nav_sensor_msgs/msg/calibration_setup.hpp"

using LocationMSG = nav_sensor_msgs::msg::Location;
using OrientationMSG = nav_sensor_msgs::msg::Orientation;
using CalibrationMSG = nav_sensor_msgs::msg::CalibrationSetup;
    struct Calibration_Setup_Struct{
        bool system_ok;                 // System is ok if bit 0 is 1
        bool gps_fix;                   // GPS has fix if bit 1 is 1
        bool mag_calibrated;            // Magnetometer is calibrated if bits 2 and 3 are both 1
        bool accel_calibrated;          // Accelerometer is calibrated if bits 4 and 5 are both 1
        bool gyro_calibrated; 
    };
    struct Orientation_Struct
    {
        _Float64 yaw;         // Degrees
        _Float64 pitch;         // Degrees
        _Float64 roll;  
    };
    
    struct Location_Struct
    {
        _Float64 latitude;         // Degrees
        _Float64 longitude;
        _Float64 altitude;
    };

class Nav_Sensors : public rclcpp::Node {
    private:
        rclcpp::TimerBase::SharedPtr timer;

        Location_Struct location;
        Orientation_Struct orientation;
        Calibration_Setup_Struct calibration_setup;

        rclcpp::Publisher<LocationMSG>::SharedPtr location_publisher;
        rclcpp::Publisher<OrientationMSG>::SharedPtr orientation_publisher;
        rclcpp::Publisher<CalibrationMSG>::SharedPtr calibration_publisher;

        
        
        // void publishMessage(std::string messageName, std::string dataType, std::string description);

    public:
        void publish_data();
        void update_data(const JsonDocument &doc);
        Nav_Sensors();
};

#endif