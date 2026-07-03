import rclpy
from rclpy.node import Node


def main(args=None):
    rclpy.init(args=args)

    node = Node('coarse_node', namespace='auto')
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
