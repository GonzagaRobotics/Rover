import pylibi2c
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, Bool

I2C_DEVICE = '/dev/i2c-0' # Check actual device path on the jetson
I2C_SLAVE_ADDRESS = 0x10
BUFFER_SIZE = 8

class Arm(Node):
    def __init__(self):
        super().__init__('arm')

        self._i2c = pylibi2c.I2CDevice(I2C_DEVICE, I2C_SLAVE_ADDRESS)

        # These subscriptions follow the command order specified by the arm
        self.create_subscription(Float32, 'arm/base', self.base_cb, 10)
        self.create_subscription(Float32, 'arm/shoulder', self.shoulder_cb, 10)
        self.create_subscription(Float32, 'arm/forearm', self.forearm_cb, 10)
        self.create_subscription(Float32, 'arm/wrist', self.wrist_cb, 10)
        self.create_subscription(Int32, 'arm/minor/x', self.minor_x_cb, 10)
        self.create_subscription(Int32, 'arm/minor/rot', self.minor_rot_cb, 10)
        self.create_subscription(Bool, 'arm/minor/grab/red', self.grab_red_cb, 10)
        self.create_subscription(Bool, 'arm/minor/grab/blk', self.grab_blk_cb, 10)

        self.get_logger().info('Ready')

    def base_cb(self, msg: Float32):
        self._send_dc(0, msg.data)

    def shoulder_cb(self, msg: Float32):
        self._send_dc(1, msg.data)

    def forearm_cb(self, msg: Float32):
        self._send_dc(2, msg.data)

    def wrist_cb(self, msg: Float32):
        self._send_dc(3, msg.data)

    def minor_x_cb(self, msg: Int32):
        self._send_stepper(4, msg.data)

    def minor_rot_cb(self, msg: Int32):
        # TODO: Limit angles
        self._send_servo(5, msg.data)

    def grab_red_cb(self, msg: Bool):
        self._send_servo(6, 180 if msg.data else 0)

    def grab_blk_cb(self, msg: Bool):
        self._send_servo(7, 180 if msg.data else 0)

    def _send_dc(self, cmd: int, val: float):
        # Byte order: [command, direction, speed]
        direction = 1 if val >= 0 else 0
        speed = int(min(abs(val) * 255, 255))

        buf = bytes(BUFFER_SIZE)
        buf[0] = cmd
        buf[1] = direction
        buf[2] = speed

        self._i2c.write(0x0, buf)

    def _send_stepper(self, cmd: int, target: int):
        # Byte order: [command, target (2 bytes)]
        buf = bytes(BUFFER_SIZE)
        buf[0] = cmd
        buf[1:3] = target.to_bytes(2, 'little')

        self._i2c.write(0x0, buf)

    def _send_servo(self, cmd: int, angle: int):
        # Byte order: [command, angle (2 bytes)]
        buf = bytes(BUFFER_SIZE)
        buf[0] = cmd
        buf[1:3] = angle.to_bytes(2, 'little')

        self._i2c.write(0x0, buf)


def main(args=None):
    rclpy.init(args=args)
    arm = Arm()
    
    try:
        rclpy.spin(arm)
    except KeyboardInterrupt:
        pass
    finally:
        arm._i2c.close()