#include "NavSensors.h"
void Nav_Sensors::publish_data(){
    LocationMSG loc_msg;
    loc_msg.latitude = location.latitude;
    loc_msg.longitude = location.longitude;
    loc_msg.altitude = location.altitude;
    location_publisher->publish(loc_msg);

    OrientationMSG orn_msg;
    orn_msg.yaw = orientation.yaw;
    orn_msg.pitch = orientation.pitch;
    orn_msg.roll = orientation.roll;
    orientation_publisher->publish(orn_msg);

    CalibrationMSG cal_msg;
    cal_msg.system_ok = calibration_setup.system_ok;
    cal_msg.gps_fix = calibration_setup.gps_fix;
    cal_msg.mag_calibrated = calibration_setup.mag_calibrated;
    cal_msg.accel_calibrated = calibration_setup.accel_calibrated;
    cal_msg.gyro_calibrated = calibration_setup.gyro_calibrated;
    calibration_publisher->publish(cal_msg);

}
void Nav_Sensors::update_data(const JsonDocument &doc){
    location.latitude = doc["latitude"]; // Numbers are not permanent, just general idea
    location.longitude = doc["longitude"];
    location.altitude = doc["altitude"];

    orientation.yaw = doc["yaw"];
    orientation.pitch = doc["pitch"];
    orientation.roll = doc["roll"];


    calibration_setup.system_ok = doc["system_ok"];
    calibration_setup.gps_fix = doc["gps_fix"];
    calibration_setup.mag_calibrated = doc["mag_calibrated"];
    calibration_setup.accel_calibrated = doc["accel_calibrated"];
    calibration_setup.gyro_calibrated = doc["gyro_calibrated"];
}
Nav_Sensors::Nav_Sensors() : Node("nav_sensors_node") {
    location_publisher = this->create_publisher<LocationMSG>("Location", 10);
    orientation_publisher = this->create_publisher<OrientationMSG>("Orientation", 10);
    calibration_publisher = this->create_publisher<CalibrationMSG>("Calibration", 10);

    auto timer_callback = [this]() {
        publish_data();
    };

    timer = this->create_wall_timer(std::chrono::seconds(1), timer_callback);
}


int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);

    auto nav_sensor = std::make_shared<Nav_Sensors>();
    JsonDocument doc;
    std::string input = R"({
        "latitude":48.75608,
        "longitude":2.302038,
        "altitude":35.5,
        "yaw":10.5,
        "pitch":2.5,
        "roll":-1.2,
        "system_ok":true,
        "gps_fix":true,
        "mag_calibrated":false,
        "accel_calibrated":true,
        "gyro_calibrated":false
    })";


    deserializeJson(doc,input);
    
    nav_sensor->update_data(doc);

    


    rclcpp::spin(nav_sensor);
    rclcpp::shutdown();
    return 0;
}