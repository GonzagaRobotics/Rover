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

        self.had_fix = False

        self.calib_pub = self.create_publisher(CalibrationSetup, '/nav/calibration', 10)
        self.location_pub = self.create_publisher(Location, '/nav/location', 10)
        self.orient_pub = self.create_publisher(Orientation, '/nav/orient', 10)

        self.create_timer(1.0 / 10, self.timer_cb)

    def find_serial(self) -> serial.Serial:
        # TODO: We are likely going to have multiple potential
        # serial devices active, so we will also need some mechanism to find
        # the correct one.
        return serial.Serial('/dev/ttyCH341USB0', baudrate=115200)

    def timer_cb(self):
        if not self.ser.is_open:
            self.get_logger().error("Serial port is not open.")
            return

        line = self.ser.readline()

        if not line:
            return

        try:
            data: dict = json.loads(line.decode())

            calib = CalibrationSetup()
            try:
                calib.gps_fix = data.get('satelites', 0) > 0
                self.had_fix = calib.gps_fix
            except KeyError:
                calib.gps_fix = self.had_fix

            calib.gyro_calibrated = data.get('gyro', 0) == 3
            calib.accel_calibrated = data.get('accel', 0) == 3
            calib.mag_calibrated = data.get('mag', 0) == 3

            self.calib_pub.publish(calib)

            # Location data is not guaranteed to be present
            if 'lat' in data and 'long' in data:
                location = Location()
                location.latitude = data.get('lat')
                location.longitude = data.get('long')

                self.location_pub.publish(location)

            orientation = Orientation()
            orientation.roll = float(data.get('z'))
            orientation.pitch = float(data.get('y'))
            orientation.yaw = float(data.get('x'))

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
    finally:
        if node.ser.is_open:
            node.ser.close()
