import numpy as np
import cv2
import sys

row_vert = 10
col_vert = 7

path = sys.argv[1]
sample_rate = 1 if len(sys.argv) < 3 else int(sys.argv[2])
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
objp = np.zeros((col_vert * row_vert,3), np.float32)
objp[:,:2] = np.mgrid[0:col_vert,0:row_vert].T.reshape(-1,2)
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.

gst_string = "filesrc location={} ! decodebin ! videoconvert ! videorate ! video/x-raw, framerate={}/1 ! appsink"

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

print("""
cx: {cx},
cy: {cy},
fx: {fx},
fy: {fy},
distro: {dist}
""".format_map({
    "cx": mtx[0, 2],
    "cy": mtx[1, 2],
    "fx": mtx[0, 0],
    "fy": mtx[1, 1],
    "dist": str(dist)
}))

cv2.destroyAllWindows()