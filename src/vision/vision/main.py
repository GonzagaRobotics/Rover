import numpy as np
import cv2 as cv
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node, SetParametersResult
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image, CameraInfo


class Vision(Node):
    def __init__(self):
        super().__init__("vision_node")

        self._frame = None
        self._frame_stamp = None

        cam_id = self.declare_parameter("camera_index", 0).value
        self._cam_name = self.declare_parameter("camera_name", "").value

        self._fourcc = self.declare_parameter("fourcc", "MJPG").value
        self._fps = self.declare_parameter("fps", 30).value
        self._width = self.declare_parameter("width", 1280).value
        self._height = self.declare_parameter("height", 720).value
        self._send_fps = self.declare_parameter("send_fps", 30).value

        assert cam_id >= 0, "camera_index parameter must be non-negative"

        self.add_on_set_parameters_callback(self.set_params_cb)

        self._cap = cv.VideoCapture(cam_id, cv.CAP_V4L2)
        self._cap.set(cv.CAP_PROP_FOURCC, cv.VideoWriter_fourcc(*self._fourcc))
        self._cap.set(cv.CAP_PROP_FRAME_WIDTH, self._width)
        self._cap.set(cv.CAP_PROP_FRAME_HEIGHT, self._height)
        self._cap.set(cv.CAP_PROP_FPS, self._fps)

        self._raw_pub = self.create_publisher(Image, "/vision/main/image_raw", 10)

        if (self._cam_name != ""):
            calib_dir = get_package_share_directory("vision") + "/calibrations/"

            self._cam_mtx = np.loadtxt(f"{calib_dir}{self._cam_name}_camera_matrix.txt")
            self._dist_coeffs = np.loadtxt(f"{calib_dir}{self._cam_name}_dist_coeffs.txt")

            self._rect_pub = self.create_publisher(Image, "/vision/main/image_rect_color", 10)
            self._cam_info_pub = self.create_publisher(CameraInfo, "/vision/main/camera_info", 10)

        if self._cap.isOpened():
            w = int(self._cap.get(cv.CAP_PROP_FRAME_WIDTH))
            h = int(self._cap.get(cv.CAP_PROP_FRAME_HEIGHT))
            f = int(self._cap.get(cv.CAP_PROP_FPS))
            self.get_logger().info(f"Camera {cam_id} opened with {w}x{h}x{f}. Sending at {self._send_fps} FPS.")

        self.create_timer(1.0 / self._fps, self.cam_cb)
        self.create_timer(1.0 / self._send_fps, self.send_cb)

    def set_params_cb(self, params: list[Parameter]):
        for param in params:
            if param.name == "camera_index":
                return SetParametersResult(successful=False, reason="Camera index cannot be changed at runtime.")
            elif param.name == "camera_name":
                return SetParametersResult(successful=False, reason="Camera name cannot be changed at runtime.")

    def cam_cb(self):
        if not self._cap.isOpened():
            raise RuntimeError("Camera is not opened.")

        ret, frame = self._cap.read()

        if not ret:
            self.get_logger().error("Failed to read from camera.")
            return

        self._frame = frame
        self._frame_stamp = self.get_clock().now().to_msg()

    def send_cb(self):
        if self._frame is None:
            return

        msg = Image()
        msg.header.stamp = self._frame_stamp
        msg.header.frame_id = f"camera_{self._cam_name}" if self._cam_name != "" else "generic"
        msg.height = self._frame.shape[0]
        msg.width = self._frame.shape[1]
        msg.encoding = "bgr8"
        msg.step = self._frame.shape[1] * 3
        msg.data.frombytes(self._frame.data)
        self._raw_pub.publish(msg)

        if self._cam_name == "":
            self._frame = None
            return

        self._publish_camera_info()

        img = self._undistort_image(self._frame)
        msg = Image()
        msg.header.stamp = self._frame_stamp
        msg.header.frame_id = f"camera_{self._cam_name}" if self._cam_name != "" else "generic"
        msg.height = img.shape[0]
        msg.width = img.shape[1]
        msg.encoding = "bgr8"
        msg.step = img.shape[1] * 3
        # Since a ROI is used, the undistorted image is not contiguous in memory
        msg.data.frombytes(np.ascontiguousarray(img).data)

        self._rect_pub.publish(msg)

        self._frame = None

    def _undistort_image(self, img: np.ndarray) -> np.ndarray:
        h,  w = img.shape[:2]
        newcameramtx, roi = cv.getOptimalNewCameraMatrix(self._cam_mtx, self._dist_coeffs, (w, h), 1, (w, h))

        dst = cv.undistort(img, self._cam_mtx, self._dist_coeffs, None, newcameramtx)

        # crop the image
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]

        return dst

    def _publish_camera_info(self):
        msg = CameraInfo()
        msg.header.stamp = self._frame_stamp
        msg.header.frame_id = f"camera_{self._cam_name}" if self._cam_name != "" else "generic"
        msg.width = self._frame.shape[1]
        msg.height = self._frame.shape[0]
        msg.distortion_model = "plumb_bob"
        msg.d = self._dist_coeffs.flatten().tolist()
        msg.k = self._cam_mtx.flatten().tolist()
        msg.p = [float(self._cam_mtx[0, 0]), 0.0, float(self._cam_mtx[0, 2]), 0.0,
                 0.0, float(self._cam_mtx[1, 1]), float(self._cam_mtx[1, 2]), 0.0,
                 0.0, 0.0, 1.0, 0.0]

        self._cam_info_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node = Vision()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._cap.release()
