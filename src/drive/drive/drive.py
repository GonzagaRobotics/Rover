import numpy as np
import rclpy
import serial
from rclpy.node import Node
from rclpy.logging import get_logger
from geometry_msgs.msg import Twist
from core_interfaces.msg import Killswitch


class Drive(Node):
    _ser = None

    def __init__(self):
        super().__init__('drive')

        ser_root = self.declare_parameter("ser_root", "/dev/ttyUSB").value
        self._ser = self.find_serial(ser_root)
        self._last_cmd = None
        self._kill = False

        self.create_subscription(
            Killswitch,
            "/killswitch",
            self.kill_cb,
            10
        )

        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.twist_cb,
            10
        )

        self.create_timer(0.25, self.timer_cb)

    def __del__(self):
        if self._ser is not None:
            self._ser.close()

    def find_serial(self, root: str) -> serial.Serial:
        for i in range(10):
            try:
                ser = serial.Serial(f"{root}{i}", 115200)
            except serial.SerialException:
                continue

            ser.write(bytes([0xff, 0x00]))
            if ser.readline() != b"DRIVE\n":
                ser.close()
                continue

            self.get_logger().info(f"Found drive system at {ser.name}")
            return ser

        raise serial.SerialException(f"Failed to find a valid drive system within {root}")

    def twist_cb(self, msg: Twist):
        self._last_cmd = msg

    def timer_cb(self):
        self._send_cmd()

    def _send_cmd(self):
        if self._last_cmd is None:
            left = 100
            right = 100
        else:
            left = int(np.clip(self._last_cmd.linear.x - self._last_cmd.angular.z, -1.0, 1.0) * 100 + 100)
            right = int(np.clip(self._last_cmd.linear.x + self._last_cmd.angular.z, -1.0, 1.0) * 100 + 100)

        try:
            self._ser.write(bytes([left, right]))
        except Exception as exc:
            self.get_logger().error(f"Failed to send command: {exc}")

    def kill_cb(self, msg: Killswitch):
        self._kill = msg.on
        if self._kill:
            self._last_cmd = None
            self._send_cmd()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = Drive()
    except Exception as exc:
        get_logger("global").error(str(exc))
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
