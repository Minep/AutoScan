import numpy as np
import cv2 as cv
import glob
import sys

def image_resize(image, width = None, height = None, inter = cv.INTER_AREA):
    # initialize the dimensions of the image to be resized and
    # grab the image size
    dim = None
    (h, w) = image.shape[:2]

    # if both the width and height are None, then return the
    # original image
    if width is None and height is None:
        return image

    # check to see if the width is None
    if width is None:
        # calculate the ratio of the height and construct the
        # dimensions
        r = height / float(h)
        dim = (int(w * r), height)

    # otherwise, the height is None
    elif height is None:
        # calculate the ratio of the width and construct the
        # dimensions
        r = width / float(w)
        dim = (width, int(h * r))
    else:
        dim = (width, height)

    # resize the image
    resized = cv.resize(image, dim, interpolation = inter)

    # return the resized image
    return resized

row_vert = 10
col_vert = 7

path = sys.argv[1]
cam_resW = 1280 if len(sys.argv) < 3 else int(sys.argv[2])
cam_resH = None if len(sys.argv) < 4 else int(sys.argv[3])
# termination criteria
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, like (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
objp = np.zeros((col_vert * row_vert,3), np.float32)
objp[:,:2] = np.mgrid[0:col_vert,0:row_vert].T.reshape(-1,2)
# Arrays to store object points and image points from all the images.
objpoints = [] # 3d point in real world space
imgpoints = [] # 2d points in image plane.
images = glob.glob('%s/*.jpg'%(path))


for fname in images:
    img = cv.imread(fname)
    img = image_resize(img, width=cam_resW, height=cam_resH)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
    # Find the chess board corners
    ret, corners = cv.findChessboardCorners(gray, (row_vert, col_vert), None)
    # If found, add object points, image points (after refining them)
    if ret == True:
        corners2 = cv.cornerSubPix(gray, corners, (5, 5), (-1,-1), criteria)
        objpoints.append(objp)
        imgpoints.append(corners2)
        cv.drawChessboardCorners(img, (row_vert, col_vert), corners2, ret)
        cv.imshow('img', img)
        cv.waitKey(500)


ret, mtx, dist, rvecs, tvecs = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

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

cv.destroyAllWindows()