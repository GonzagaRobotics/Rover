#!/usr/bin/python3

"""
Calibrate camera using checkerboard images.

Usage:
    python3 calibrate.py <camera_id>
"""

import time
import sys
import cv2 as cv
import numpy as np

# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(8,5,0)
objp = np.zeros((9*6, 3), np.float32)
objp[:, :2] = np.mgrid[0:9, 0:6].T.reshape(-1, 2)

# Arrays to store object points and image points from all the images.
objpoints = []  # 3d point in real world space
imgpoints = []  # 2d points in image plane.

cap = cv.VideoCapture(int(sys.argv[1]))
cap.set(cv.CAP_PROP_FRAME_WIDTH, 1280) 
cap.set(cv.CAP_PROP_FRAME_HEIGHT, 720)

try:
    while True:
        start = time.time()
        ret, frame = cap.read()

        if not ret:
            print("Failed to read from camera.")
            sys.exit(1)

        img = frame.copy()
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

        # Find the chess board corners
        ret, corners = cv.findChessboardCorners(gray, (9, 6), None)

        # If found, add object points, image points (after refining them)
        if ret == True:
            objpoints.append(objp)

            corners2 = cv.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)

            # Draw and display the corners
            cv.drawChessboardCorners(img, (9, 6), corners2, ret)
            cv.imshow('img', img)
            cv.waitKey(10)
        else:
            cv.imshow('img', img)
            cv.waitKey(10)

        while time.time() - start < 0.5:
            cap.grab()
except KeyboardInterrupt:
    print("Calibration ended")

cv.destroyAllWindows()

if len(objpoints) == 0:
    print("No checkerboard corners were detected. Calibration failed.")
    sys.exit(1)

ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

np.savetxt("camera_matrix.txt", mtx)
np.savetxt("dist_coeffs.txt", dist)

mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv.projectPoints(
        objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv.norm(imgpoints[i], imgpoints2, cv.NORM_L2)/len(imgpoints2)
    mean_error += error

print("total error: {}".format(mean_error/len(objpoints)))
