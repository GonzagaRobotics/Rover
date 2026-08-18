import time
import serial
import rclpy
from rclpy.node import Node
from rclpy.logging import get_logger
from std_msgs.msg import Float32, Int32, Bool


class Arm(Node):
    _ser = None

    def __init__(self):
        super().__init__('arm')

        ser_root = self.declare_parameter("ser_root", "/dev/ttyUSB").value
        self._ser = self.find_serial(ser_root)

        # These subscriptions follow the command order specified by the arm
        self.create_subscription(Float32, 'arm/base', self.base_cb, 10)
        self.create_subscription(Float32, 'arm/shoulder', self.shoulder_cb, 10)
        self.create_subscription(Float32, 'arm/forearm', self.forearm_cb, 10)
        self.create_subscription(Float32, 'arm/wrist', self.wrist_cb, 10)
        self.create_subscription(Int32, 'arm/minor/x', self.minor_x_cb, 10)
        self.create_subscription(Int32, 'arm/minor/rot', self.minor_rot_cb, 10)
        self.create_subscription(Bool, 'arm/minor/grab', self.minor_grab_cb, 10)

    def __del__(self):
        if self._ser is not None:
            self._ser.close()

    def find_serial(self, root: str) -> serial.Serial:
        for i in range(10):
            try:
                ser = serial.Serial(f"{root}{i}", 115200, timeout=1)
            except serial.SerialException:
                continue

            # The microcontroller needs time to reset
            time.sleep(2)

            ser.write(bytes([0xff, 0x00]))
            if ser.readline() != b"ARM\n":
                ser.close()
                continue

            self.get_logger().info(f"Found arm at {ser.name}")
            return ser

        raise serial.SerialException(f"Failed to find a valid arm within {root}")

    def base_cb(self, msg: Float32):
        self._send_dc(0, msg.data)

    def shoulder_cb(self, msg: Float32):
        self._send_dc(1, msg.data)

    def forearm_cb(self, msg: Float32):
        self._send_dc(2, msg.data)

    def wrist_cb(self, msg: Float32):
        self._send_dc(3, msg.data)

    def minor_x_cb(self, msg: Int32):
        self._send_stepper_servo(4, msg.data)

    def minor_rot_cb(self, msg: Int32):
        # TODO: Limit angles
        self._send_stepper_servo(5, msg.data)

    def minor_grab_cb(self, msg: Bool):
        self._send_stepper_servo(6, 180 if msg.data else 90)

    def _send_dc(self, cmd: int, val: float):
        # Byte order: [command, direction, speed]
        direction = 0 if val >= 0 else 1
        speed = int(min(abs(val) * 255, 255))

        buf = bytes([cmd, direction, speed])

        self._ser.write(buf)

    def _send_stepper_servo(self, cmd: int, target: int):
        # Byte order: [command, target (2 bytes)]
        buf = bytes([cmd]) + target.to_bytes(2, byteorder='little')

        self._ser.write(buf)


def main(args=None):
    rclpy.init(args=args)

    try:
        node = Arm()
    except Exception as exc:
        get_logger("global").error(str(exc))
        return

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
