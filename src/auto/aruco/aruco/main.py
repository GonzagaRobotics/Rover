import numpy as np
import cv2
import rclpy
from rclpy.node import Node, Parameter, SetParametersResult
from sensor_msgs.msg import Image, RegionOfInterest, CameraInfo
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from auto_msgs.msg import Aruco as ArucoMsg


class ArucoDetector:
    def __init__(self, marker_size: float):
        self.params = cv2.aruco.DetectorParameters_create()
        self.params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        self.ar_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)

        self.object_points = self._object_points(marker_size)

    def set_marker_size(self, marker_size: float):
        self.object_points = self._object_points(marker_size)

    def detect(self, img: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        corners, ids, _ = cv2.aruco.detectMarkers(img, self.ar_dict, parameters=self.params)

        return corners, ids

    def _object_points(self, marker_size: float) -> np.ndarray:
        half = marker_size / 2

        return np.array([
            [-half, -half, 0],
            [half, -half, 0],
            [half, half, 0],
            [-half, half, 0]],
            dtype=np.float32)


class ArucoNode(Node):
    def __init__(self):
        super().__init__("aruco_node", namespace="auto")

        self._ready = False

        marker_size = self.declare_parameter("marker_size", 0.15).value

        self._last_img = None
        self._detector = ArucoDetector(marker_size)

        self._viz_pub = self.create_publisher(MarkerArray, 'aruco/viz', 10)
        self._aruco_pub = self.create_publisher(ArucoMsg, "aruco/detect", 10)

        self.create_subscription(Image, "/vision/main/image_rect_color", self.cam_cb, 1)
        self.create_subscription(CameraInfo, "/vision/main/camera_info", self.cam_cb, 1)

        self.add_on_set_parameters_callback(self.param_cb)

        self.create_timer(1.0 / 5, self.detect)

    def param_cb(self, params: list[Parameter]):
        for param in params:
            if param.name == "marker_size":
                if param.value <= 0:
                    return SetParametersResult(successful=False, reason="Marker size must be > 0.")

                self._detector.set_marker_size(param.value)
                return SetParametersResult(successful=True)

    def cam_cb(self, msg: Image | CameraInfo):
        if isinstance(msg, CameraInfo):
            self._cam_mtx = np.array(msg.k).reshape(3, 3)
            self._dist_coeffs = np.array(msg.d)

            if not self._ready and self._last_img is not None:
                self._ready = True
                self.get_logger().info("Ready")
        else:
            self._last_img = msg

    def detect(self):
        if not self._ready or self._last_img is None:
            return

        img = np.frombuffer(self._last_img.data, dtype=np.uint8).reshape(
            self._last_img.height, self._last_img.width, -1)

        corners, ids = self._detector.detect(img)

        msg = ArucoMsg()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()

        for i in range(len(corners)):
            # Reject markers that are not in the range of expected IDs (0-3)
            if ids[i][0] > 3:
                continue

            image_points = corners[i].reshape(-1, 2).astype(np.float32)
            _, _, t = cv2.solvePnP(self._detector.object_points, image_points, self._cam_mtx, self._dist_coeffs)

            roi = RegionOfInterest()
            roi.x_offset = int(image_points[:, 0].min())
            roi.y_offset = int(image_points[:, 1].min())
            roi.width = int(image_points[:, 0].max() - image_points[:, 0].min())
            roi.height = int(image_points[:, 1].max() - image_points[:, 1].min())

            msg.ids.append(ids[i][0])
            msg.translations.append(Vector3(x=t[2][0], y=-t[0][0], z=-t[1][0]))
            msg.rois.append(roi)

            marker_array = MarkerArray()
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = int(ids[i][0])
            marker.type = Marker.ARROW
            marker.action = Marker.ADD
            marker.points.append(Point(x=0.0, y=0.0, z=0.0))
            marker.points.append(Point(x=t[2][0], y=-t[0][0], z=-t[1][0]))
            marker.scale.x = 0.1
            marker.scale.y = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker_array.markers.append(marker)
            self._viz_pub.publish(marker_array)

        self._aruco_pub.publish(msg)

        self._last_img = None


def main(args=None):
    rclpy.init(args=args)

    node = ArucoNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
