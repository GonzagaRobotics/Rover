import json
import rclpy
import serial
from rclpy.node import Node, Subscription
from rclpy.parameter import Parameter
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus


class Drive(Node):
    drive_sub: Subscription
    ser: serial.Serial

    def __init__(self):
        super().__init__('nav_sensors_antenna')

        ser_name = self.declare_parameter('serial_name', 'ttyCH341USB0').value
        assert ser_name != "", "Serial port name cannot be empty"

        self.ser = self.find_serial(ser_name)

        self.fix_pub = self.create_publisher(NavSatFix, 'fix', 10)
        self.imu_pub = self.create_publisher(Imu, 'imu', 10)

        self.create_timer(1.0 / 10, self.timer_cb)

    def find_serial(self, name: str) -> serial.Serial:
        return serial.Serial(f'/dev/{name}', baudrate=115200)

    def timer_cb(self):
        if not self.ser.is_open:
            self.get_logger().error("Serial port is not open.")
            return

        line = self.ser.readline()

        if not line:
            return

        try:
            data: dict = json.loads(line.decode(), parse_int=float)
        except json.JSONDecodeError as exc:
            self.get_logger().warn(f"Failed to decode JSON: {exc.msg} | Raw: {exc.doc}")

        # IMU
        msg = Imu()
        msg.header.frame_id = "imu_link"
        msg.header.stamp = self.get_clock().now().to_msg()

        msg.orientation.w = data.get('ow', 1.0)
        msg.orientation.x = data.get('ox', 0.0)
        msg.orientation.y = data.get('oy', 0.0)
        msg.orientation.z = data.get('oz', 0.0)

        msg.angular_velocity.x = data.get('ax', 0.0)
        msg.angular_velocity.y = data.get('ay', 0.0)
        msg.angular_velocity.z = data.get('az', 0.0)

        msg.linear_acceleration.x = data.get('gx', 0.0)
        msg.linear_acceleration.y = data.get('gy', 0.0)
        msg.linear_acceleration.z = data.get('gz', 0.0)

        # To indicate incomplete calibration, set covariance to high values
        if data.get("cal_sys", 0) < 3:
            msg.orientation_covariance = [1e6, 0.0, 0.0, 0.0, 1e6, 0.0, 0.0, 0.0, 1e6]
            msg.angular_velocity_covariance = [1e6, 0.0, 0.0, 0.0, 1e6, 0.0, 0.0, 0.0, 1e6]
            msg.linear_acceleration_covariance = [1e6, 0.0, 0.0, 0.0, 1e6, 0.0, 0.0, 0.0, 1e6]
        else:
            # Covariances taken from here: https://github.com/MapaRobo/bno055/blob/master/src/imu_ros.py
            msg.orientation_covariance = [0.03, 0.0, 0.0, 0.0, 0.03, 0.0, 0.0, 0.0, 0.03]
            msg.angular_velocity_covariance = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            msg.linear_acceleration_covariance = [0.1, 0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.1]

        self.imu_pub.publish(msg)

        # GPS
        msg = NavSatFix()
        msg.header.frame_id = "gps_link"
        msg.header.stamp = self.get_clock().now().to_msg()

        satelites = data.get('satellites', -1)
        msg.status.status = NavSatStatus.STATUS_FIX if satelites > 0 else NavSatStatus.STATUS_NO_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS

        msg.latitude = data.get('lat', 0.0)
        msg.longitude = data.get('lon', 0.0)
        msg.altitude = data.get('alt', float('nan'))

        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_APPROXIMATED
        # Datasheet specifies 3m CEP, (3 / 1.1774)^2 = 6.5m^2 variance
        msg.position_covariance = [6.5, 0.0, 0.0, 0.0, 6.5, 0.0, 0.0, 0.0, 6.5]

        self.fix_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = Drive()

    node.get_logger().info(f"Nav/Ant. ready. Using serial port: {node.ser.name}")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser.is_open:
            node.ser.close()
