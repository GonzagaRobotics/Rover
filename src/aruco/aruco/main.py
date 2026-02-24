import numpy as np
import cv2 as cv
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import RegionOfInterest
from geometry_msgs.msg import Point, Vector3
from visualization_msgs.msg import Marker, MarkerArray
from nav_interfaces.msg import Aruco as ArucoMsg

class Aruco(Node):
    def __init__(self):
        super().__init__("aruco")

        marker_size = self.declare_parameter("marker_size", 0.15).value
        cam_id = self.declare_parameter("camera_index", -1).value
        self._cam_name = self.declare_parameter("camera_name", "").value

        assert cam_id >= 0, "camera_index parameter must be set."
        assert self._cam_name != "", "camera_name parameter must be set."

        half = marker_size / 2
        self._object_points = np.array([
                [-half, -half, 0], 
                [half, -half, 0], 
                [half, half, 0], 
                [-half, half, 0]],
                dtype=np.float32)

        self._cap = cv.VideoCapture(cam_id)
        self._cap.set(cv.CAP_PROP_FRAME_WIDTH, 1280) 
        self._cap.set(cv.CAP_PROP_FRAME_HEIGHT, 720)
        self._last_img = None
        self._detector = self._make_detector()

        self._viz_pub = self.create_publisher(MarkerArray, '/aruco/viz', 10)
        self._aruco_pub = self.create_publisher(ArucoMsg, "/aruco/detect", 10)

        calib_dir = get_package_share_directory("aruco") + "/calibrations/"

        self._cam_mtx = np.loadtxt(f"{calib_dir}{self._cam_name}_camera_matrix.txt")
        self._dist_coeffs = np.loadtxt(f"{calib_dir}{self._cam_name}_dist_coeffs.txt")

        self.create_timer(1.0 / 30, self.cam_cb)
        self.create_timer(1.0 / 5, self.detect)

        self.get_logger().info("Ready")

    def cam_cb(self):
        ret = self._cap.grab()

        if not ret:
            raise RuntimeError("Failed to read from camera.")

    def detect(self):
        _, frame = self._cap.retrieve()

        img = self._undistort_image(frame.copy())
    
        corners, ids, _ = self._detector.detectMarkers(img)

        msg = ArucoMsg()
        msg.header.frame_id = self._cam_name
        msg.header.stamp = self.get_clock().now().to_msg()

        for i in range(len(corners)): 
            # Reject markers that are not in the range of expected IDs (0-3)
            if ids[i][0] > 3:
                continue

            image_points = corners[i].reshape(-1, 2).astype(np.float32)
            _, _, t = cv.solvePnP(self._object_points, image_points, self._cam_mtx, self._dist_coeffs)

            roi = RegionOfInterest()
            roi.x_offset = int(image_points[:, 0].min())
            roi.y_offset = int(image_points[:, 1].min())
            roi.width = int(image_points[:, 0].max() - image_points[:, 0].min())
            roi.height = int(image_points[:, 1].max() - image_points[:, 1].min())

            # TODO: Fully verify the coordinate transformation and sign conventions here

            msg.ids.append(ids[i][0])
            msg.translations.append(Vector3(x=t[2][0], y=-t[0][0], z=-t[1][0]))
            msg.rois.append(roi)

            marker_array = MarkerArray()
            marker = Marker()
            marker.header.frame_id = self._cam_name
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

    def _undistort_image(self, img: cv.Mat) -> cv.Mat:
        h,  w = img.shape[:2]
        newcameramtx, roi = cv.getOptimalNewCameraMatrix(self._cam_mtx, self._dist_coeffs, (w, h), 1, (w, h))

        dst = cv.undistort(img, self._cam_mtx, self._dist_coeffs, None, newcameramtx)

        # crop the image
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]

        return dst

    def _make_detector(self) -> cv.aruco.ArucoDetector:
        parameters = cv.aruco.DetectorParameters()
        parameters.cornerRefinementMethod = cv.aruco.CORNER_REFINE_SUBPIX

        aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)

        return cv.aruco.ArucoDetector(aruco_dict, parameters)


def main(args=None):
    rclpy.init(args=args)

    node = Aruco()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._cap.release()
