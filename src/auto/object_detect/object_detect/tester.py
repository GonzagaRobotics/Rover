import os
import glob
from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Image
from auto_msgs.msg import Detection
import cv2


class Tester(Node):
    def __init__(self):
        super().__init__('object_detect_tester_node')

        path = self.declare_parameter("path", "").value
        assert len(path) > 1, "Path cannot be empty"
        self.pub = self.create_publisher(Image, "/vision/main/image_rect_color", 1)
        self.create_subscription(Detection, "/auto/object/detect", self.detection_cb, 1)
        self.imgs = glob.glob(path)
        self.img_idx = 0
        self.timer = self.create_timer(2.0, self.timer_cb)

    def timer_cb(self):
        self.img = cv2.imread(self.imgs[self.img_idx])
        self.img_idx = (self.img_idx + 1) % len(self.imgs)

        msg = Image()
        msg.height = self.img.shape[0]
        msg.width = self.img.shape[1]
        msg.encoding = "bgr8"
        msg.step = self.img.shape[1] * 3
        msg.data.frombytes(self.img.data)
        self.pub.publish(msg)

        cv2.imshow("img", self.img)
        cv2.waitKey(1)

    def detection_cb(self, msg: Detection):
        if len(msg.ids) == 0:
            self.get_logger().info("No detections")
            return

        img = self.img.copy()
        p1 = (int(msg.rois[0].x_offset), int(msg.rois[0].y_offset))
        p2 = (int(msg.rois[0].x_offset + msg.rois[0].width), int(msg.rois[0].y_offset + msg.rois[0].height))
        cv2.rectangle(img, p1, p2, (0, 255, 0), 2)
        cv2.putText(img, f"{msg.ids[0]} {msg.confs[0]:.2f}", (p1[0], p1[1] - 10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)
        cv2.imshow("img", img)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)

    node = Tester()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
