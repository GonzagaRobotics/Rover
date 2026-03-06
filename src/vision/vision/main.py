import numpy as np
import cv2 as cv
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node, SetParametersResult
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image


class Vision(Node):
    def __init__(self):
        super().__init__("vision")

        cam_id = self.declare_parameter("camera_index", -1).value
        self._cam_name = self.declare_parameter("camera_name", "").value

        assert cam_id >= 0, "camera_index parameter must be set."
        assert self._cam_name != "", "camera_name parameter must be set."

        self.add_on_set_parameters_callback(self.set_params_cb)

        self._cap = cv.VideoCapture(cam_id)
        self._cap.set(cv.CAP_PROP_FRAME_WIDTH, 1280)
        self._cap.set(cv.CAP_PROP_FRAME_HEIGHT, 720)

        calib_dir = get_package_share_directory("vision") + "/calibrations/"

        self._cam_mtx = np.loadtxt(f"{calib_dir}{self._cam_name}_camera_matrix.txt")
        self._dist_coeffs = np.loadtxt(f"{calib_dir}{self._cam_name}_dist_coeffs.txt")

        self.create_publisher(Image, "/vision/image/clean", 10)

        self.create_timer(1.0 / 30, self.cam_cb)

        self.get_logger().info("Ready")

    def set_params_cb(self, params: list[Parameter]):
        for param in params:
            if param.name == "camera_index":
                return SetParametersResult(successful=False, reason="Camera index cannot be changed at runtime.")
            elif param.name == "camera_name":
                return SetParametersResult(successful=False, reason="Camera name cannot be changed at runtime.")

        return SetParametersResult(successful=True)

    def cam_cb(self):
        if not self._cap.isOpened():
            raise RuntimeError("Camera is not opened.")

        ret, frame = self._cap.retrieve()

        if not ret:
            return

        img = self._undistort_image(frame)
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.height = img.shape[0]
        msg.width = img.shape[1]
        msg.encoding = "bgr8"
        msg.data = img.tobytes()

        self.publish(msg)

    def _undistort_image(self, img: cv.Mat) -> cv.Mat:
        h,  w = img.shape[:2]
        newcameramtx, roi = cv.getOptimalNewCameraMatrix(self._cam_mtx, self._dist_coeffs, (w, h), 1, (w, h))

        dst = cv.undistort(img, self._cam_mtx, self._dist_coeffs, None, newcameramtx)

        # crop the image
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]

        return dst


def main(args=None):
    rclpy.init(args=args)

    node = Vision()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._cap.release()
