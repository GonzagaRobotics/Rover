import numpy as np
import cv2 as cv
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class Aruco(Node):
    def __init__(self):
        super().__init__("aruco")

        # Camera matrix
        # TODO: Replace (damons) phone camera calibration with actual camera calibration
        self.cam_mtx = np.array([
            [7.571738678714283424e+02, 0.000000000000000000e+00,
                5.122508176676894891e+02],
            [0.000000000000000000e+00, 7.566823461234021124e+02,
                3.871533089401949610e+02],
            [0.000000000000000000e+00, 0.000000000000000000e+00,
                1.000000000000000000e+00]
        ])

        # Distortion coefficients
        self.dist_coeffs = np.array([2.240837462410895653e-01, -1.491022106094063382e+00,
                                    2.599097358123332721e-04, 2.269185491989064837e-04, 2.683566858009867495e+00])

        self.detector = self._make_detector()

        self.cam_sub = self.create_subscription(
            Image,
            "/image_raw",
            self.cam_callback, 10
        )
        self.last_img = None

        self.detect_timer = self.create_timer(1.0, self.detect)

        self.get_logger().info("Aruco detector ready.")

    def cam_callback(self, msg):
        # create a 1D uint8 view of the incoming bytes
        buf = np.frombuffer(msg.data, dtype=np.uint8)

        # cv.imshow("Camera", np.reshape(buf, (msg.height, msg.width, 3)))
        # cv.waitKey(1)
        raw = np.reshape(buf, (msg.height, msg.width, 3))
        raw = cv.resize(raw, (0, 0), fx=0.33, fy=0.33)
        cv.imshow("Camera", raw)
        cv.waitKey(10)

        h,  w = raw.shape[:2]
        newcameramtx, roi = cv.getOptimalNewCameraMatrix(
            self.cam_mtx, self.dist_coeffs, (w, h), 1, (w, h))

        print(newcameramtx)
        print(roi)

        # undistort
        dst = cv.undistort(raw, self.cam_mtx,
                           self.dist_coeffs, None, newcameramtx)

        # crop the image
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]
        cv.imshow("Undistorted", dst)
        cv.waitKey(10)

    def detect(self):
        if self.last_img is None:
            return

        corners, ids, rejected = self.detector.detectMarkers(self.last_img)

        for i in range(len(corners)):
            r, t, _ = cv.aruco.estimatePoseSingleMarkers(
                corners[i], 0.0889, self.cam_mtx, self.dist_coeffs)

            self.get_logger().info(
                f"ID: {ids[i][0]} @ {np.linalg.norm(t)} meters")

    def _undistort_image(self, img):
        h,  w = img.shape[:2]
        newcameramtx, roi = cv.getOptimalNewCameraMatrix(
            self.cam_mtx, self.dist_coeffs, (w, h), 1, (w, h))

        # undistort
        dst = cv.undistort(img, self.cam_mtx,
                           self.dist_coeffs, None, newcameramtx)

        # crop the image
        x, y, w, h = roi
        dst = dst[y:y+h, x:x+w]

        return dst

    def _make_detector(self):
        parameters = cv.aruco.DetectorParameters()
        parameters.cornerRefinementMethod = cv.aruco.CORNER_REFINE_SUBPIX
        aruco_dict = cv.aruco.getPredefinedDictionary(
            cv.aruco.DICT_4X4_50)
        detector = cv.aruco.ArucoDetector(aruco_dict, parameters)

        return detector


def main(args=None):
    rclpy.init(args=args)

    node = Aruco()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
