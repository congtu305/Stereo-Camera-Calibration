import cv2
import os
import numpy as np
import glob
# Chess/checker board size, dimensions
CHESS_BOARD_DIM = (9, 6)
# The size of squares in the checker board design.
SQUARE_SIZE = 24  # millimeters (change it according to printed size)
# termination criteria
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# prepare object points, i.e. (0,0,0), (1,0,0), (2,0,0) ....,(6,5,0)
obj_3D = np.zeros((CHESS_BOARD_DIM[0] * CHESS_BOARD_DIM[1], 3), np.float32)
obj_3D[:, :2] = np.mgrid[0 : CHESS_BOARD_DIM[0], 0 : CHESS_BOARD_DIM[1]].T.reshape(
    -1, 2
)
obj_3D *= SQUARE_SIZE
# Arrays to store object points and image points from all the given images.
obj_points_3D = []  # 3d point in real world space
img_points_2D = []  # 2d points in image plane

#Load images
images = glob.glob('right*.png')
for img in images:
    # Load left and right images
    image = cv2.imread(img, cv2.IMREAD_GRAYSCALE)
    ret, corners = cv2.findChessboardCorners(image, CHESS_BOARD_DIM, None)
    if ret == True:
        obj_points_3D.append(obj_3D)
        corners2 = cv2.cornerSubPix(image, corners, (3, 3), (-1, -1), criteria)
        img_points_2D.append(corners2)

        img = cv2.drawChessboardCorners(image, CHESS_BOARD_DIM, corners2, ret)

cv2.destroyAllWindows()




# h, w = image.shape[:2]
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
    obj_points_3D, img_points_2D, image.shape[::-1], None, None
)
print("calibrated")

print("Matrix:",mtx)
print("dist:",dist)
print("rvecs:",rvecs)
print("tvecs:",tvecs)