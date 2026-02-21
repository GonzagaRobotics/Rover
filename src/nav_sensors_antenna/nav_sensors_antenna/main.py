import json
import rclpy
import serial
from rclpy.node import Node, Subscription
from nav_interfaces.msg import CalibrationSetup, Location, Orientation


class Drive(Node):
    drive_sub: Subscription
    ser: serial.Serial

    def __init__(self):
        super().__init__('nav_sensors_antenna')

        self.ser = self.find_serial()

        self.calib_pub = self.create_publisher(CalibrationSetup, '/nav/calibration', 10)
        self.location_pub = self.create_publisher(Location, '/nav/location', 10)
        self.orient_pub = self.create_publisher(Orientation, '/nav/orient', 10)

        self.create_timer(1.0 / 2, self.timer_cb)

    def __del__(self):
        if self.ser.is_open:
            self.ser.close()

    def find_serial(self) -> serial.Serial:
        # TODO: We are likely going to have multiple potential
        # serial devices active, so we will also need some mechanism to find
        # the correct one.
        return serial.Serial('/dev/ttyUSB1', baudrate=115200)

    def timer_cb(self):
        if not self.ser.is_open:
            self.get_logger().error("Serial port is not open.")
            return

        line = self.ser.readline()

        if not line:
            return

        try:
            data = json.loads(line.decode())

            # TODO: Calibration data
            # TODO: Are all fields always present? Do we need to handle missing fields?

            location = Location()
            location.latitude = data.get('latitude', 0.0)
            location.longitude = data.get('longitude', 0.0)

            # TODO: Confirm the correct mapping of x/y/z to roll/pitch/yaw
            orientation = Orientation()
            orientation.roll = data.get('x', 0.0)
            orientation.pitch = data.get('y', 0.0)
            orientation.yaw = data.get('z', 0.0)

            self.location_pub.publish(location)
            self.orient_pub.publish(orientation)
        except json.JSONDecodeError as exc:
            self.get_logger().error(f"Failed to decode JSON: {exc}")
            return


def main(args=None):
    rclpy.init(args=args)

    node = Drive()

    node.get_logger().info(f"Nav/Ant. ready. Using serial port: {node.ser.name}")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
