import numpy as np
import cv2 as cv
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

class Aruco(Node):
    def __init__(self):
        super().__init__("aruco")

        self._cap = cv.VideoCapture(2)
        self._cap.set(cv.CAP_PROP_FRAME_WIDTH, 1280) 
        self._cap.set(cv.CAP_PROP_FRAME_HEIGHT, 720)
        self._last_img = None
        self._detector = self._make_detector()

        self._viz_pub = self.create_publisher(MarkerArray, '/aruco/viz', 10)

        self._cam_mtx = np.loadtxt("/home/damon/robotics/aruco/camera_matrix.txt")
        self._dist_coeffs = np.loadtxt("/home/damon/robotics/aruco/dist_coeffs.txt")

        self.create_timer(1.0 / 30, self.cam_cb)
        self.create_timer(1.0 / 5, self.detect)

        self.get_logger().info("Aruco detector ready.")

    def cam_cb(self):
        ret = self._cap.grab()

        if not ret:
            raise RuntimeError("Failed to read from camera.")

    def detect(self):
        _, frame = self._cap.retrieve()

        img = self._undistort_image(frame.copy())

        corners, ids, _ = self._detector.detectMarkers(img)

        for i in range(len(corners)): 
            # Reject markers that are not the correct id (false positives)
            if ids[i][0] > 3:
                continue

            half = 0.015 / 2
            object_points = np.array([[-half, -half, 0], [half, -half, 0], [half, half, 0], [-half, half, 0]], dtype=np.float32)
            image_points = corners[i].reshape(-1, 2).astype(np.float32)
            _, r, t = cv.solvePnP(object_points, image_points, self._cam_mtx, self._dist_coeffs)

            # img = cv.drawFrameAxes(img, self._cam_mtx, self._dist_coeffs, r, t, 0.1)

            # cv.imshow("ar", cv.aruco.drawDetectedMarkers(img, corners, ids))
            # cv.waitKey(10)

            dist = np.linalg.norm(t)

            # Calculate the angles from the translation vector
            angle_x = np.arctan2(t[1], t[2]) * 180 / np.pi
            angle_y = np.arctan2(t[0], t[2]) * 180 / np.pi
            angle_z = np.arctan2(t[0], t[1]) * 180 / np.pi

            self.get_logger().info(f"ID: {ids[i][0]} @ {dist:0.2f} meters angle_x: {angle_x} angle_y: {angle_y} angle_z: {angle_z}")

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
            # marker.scale.z = 0.1
            marker.color.a = 1.0
            marker.color.r = 1.0
            marker_array.markers.append(marker)
            self._viz_pub.publish(marker_array)

    def _undistort_image(self, img):
        h,  w = img.shape[:2]
        newcameramtx, roi = cv.getOptimalNewCameraMatrix(self._cam_mtx, self._dist_coeffs, (w, h), 1, (w, h))

        # undistort
        dst = cv.undistort(img, self._cam_mtx, self._dist_coeffs, None, newcameramtx)

        # crop the image
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]

        return dst

    def _make_detector(self):
        parameters = cv.aruco.DetectorParameters()
        parameters.cornerRefinementMethod = cv.aruco.CORNER_REFINE_SUBPIX
        aruco_dict = cv.aruco.getPredefinedDictionary(cv.aruco.DICT_4X4_50)
        detector = cv.aruco.ArucoDetector(aruco_dict, parameters)

        return detector


def main(args=None):
    rclpy.init(args=args)

    node = Aruco()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._cap.release()
