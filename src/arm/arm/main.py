import smbus
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Int32, Bool

I2C_BUS_NAME = '/dev/i2c-1'
I2C_TARGET_ADDRESS = 0x10
BUFFER_SIZE = 8

class Arm(Node):
    def __init__(self):
        super().__init__('arm')

        self._bus = smbus.SMBus(1)

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
        self._send_stepper_servo(4, msg.data)

    def minor_rot_cb(self, msg: Int32):
        # TODO: Limit angles
        self._send_stepper_servo(5, msg.data)

    def grab_red_cb(self, msg: Bool):
        self._send_stepper_servo(6, 180 if msg.data else 0)

    def grab_blk_cb(self, msg: Bool):
        self._send_stepper_servo(7, 180 if msg.data else 0)

    def _send_dc(self, cmd: int, val: float):
        # Byte order: [command, direction, speed]
        direction = 1 if val >= 0 else 0
        speed = int(min(abs(val) * 255, 255))

        buf = bytes(BUFFER_SIZE - 1)
        buf[0] = direction
        buf[1] = speed

        self._bus.write_i2c_block_data(I2C_TARGET_ADDRESS, cmd, buf)

    def _send_stepper_servo(self, cmd: int, target: int):
        # Byte order: [command, target (2 bytes)]
        buf = bytes(BUFFER_SIZE - 1)
        buf[0:2] = target.to_bytes(2, 'little')

        self._bus.write_i2c_block_data(I2C_TARGET_ADDRESS, cmd, buf)


def main(args=None):
    rclpy.init(args=args)
    arm = Arm()
    
    try:
        rclpy.spin(arm)
    except KeyboardInterrupt:
        pass
    finally:
        arm._bus.close()