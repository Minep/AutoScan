import numpy as np
import cv2
import sys

row_vert = 7
col_vert = 10

path = sys.argv[1]
sample_rate = 1 if len(sys.argv) < 3 else int(sys.argv[2])
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
objp = np.zeros((col_vert * row_vert,3), np.float32)
objp[:,:2] = np.mgrid[0:row_vert,0:col_vert].T.reshape(-1,2)
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

gst_string = "filesrc location={} ! qtdemux ! avdec_h264 ! videoconvert ! videorate ! video/x-raw, framerate={}/1 ! appsink"

gst = gst_string.format(path, sample_rate)
print(gst)
cvCap = cv2.VideoCapture(gst, cv2.CAP_GSTREAMER)

if not cvCap.isOpened():
    raise Exception("VideoCapture is not opened")

while True:
    ret, img = cvCap.read()
    if not ret:
        break
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv2.findChessboardCorners(gray, (row_vert, col_vert), None)
    # If found, add object points, image points (after refining them)
    if ret == True:
        corners2 = cv2.cornerSubPix(gray, corners, (5, 5), (-1,-1), criteria)
        objpoints.append(objp)
        imgpoints.append(corners2)
        cv2.drawChessboardCorners(img, (row_vert, col_vert), corners2, ret)
        cv2.imshow('img', img)
        cv2.waitKey(500)


ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
    mean_error += error

formatted = """
cx: {cx:.2f}
cy: {cy:.2f}
fx: {fx:.2f}
fy: {fy:.2f}
---
Camera1.cx: {cx:.2f}
Camera1.cy: {cy:.2f}
Camera1.fx: {fx:.2f}
Camera1.fy: {fy:.2f}
Camera1.k1: {k1:.2f}
Camera1.k2: {k2:.2f}
Camera1.p1: {p1:.2f}
Camera1.p2: {p2:.2f}
""".format_map({
    "cx": mtx[0, 2],
    "cy": mtx[1, 2],
    "fx": mtx[0, 0],
    "fy": mtx[1, 1],
    "k1": dist[0, 0],
    "k2": dist[0, 1],
    "p1": dist[0, 2],
    "p2": dist[0, 3],
})

import os

with open(path + ".txt", 'w') as f:
    f.write(formatted)
    
print(formatted)
print( "total error: {}".format(mean_error/len(objpoints)) )

cv2.destroyAllWindows()