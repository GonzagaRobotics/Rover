import rclpy
import serial
from rclpy.node import Node, Subscription
from drive_interfaces.msg import DriveCommand


class Drive(Node):
    drive_sub: Subscription
    ser: serial.Serial

    def __init__(self):
        super().__init__('drive')

        self.ser = self.find_serial()

        self.drive_sub = self.create_subscription(
            DriveCommand,
            '/drive/command',
            self.drive_callback,
            10
        )

    def __del__(self):
        if self.ser.is_open:
            self.ser.close()

    def find_serial(self) -> serial.Serial:
        # TODO: We are likely going to have multiple potential
        # serial devices active, so we will also need some mechanism to find
        # the correct one.
        return serial.Serial('/dev/ttyS0')

    def drive_callback(self, msg):
        # Convert from doubles [-1, 1] to integers [0, 200]
        FB = round(msg.forward_backward * 100 + 100)
        LR = round(msg.left_right * 100 + 100)

        self.get_logger().info(f"Received command - FB: {FB} LR: {LR}")

        # TODO: Actually test this

        # Format the command and send it
        command = f"FB:{FB} LR:{LR} EN:1\n"

        try:
            self.ser.write(command.encode())
        except Exception as exc:
            self.get_logger().error(f"Failed to send command: {exc}")

        # TODO: Eventually we will want a response from the microcontroller


def main(args=None):
    rclpy.init(args=args)

    node = Drive()

    node.get_logger().info(
        f"Drive System ready. Using serial port: {node.ser.name}")

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
