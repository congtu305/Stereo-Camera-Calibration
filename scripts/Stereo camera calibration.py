import numpy as np
import cv2
import glob

# Define the size of the chessboard
chessboard_size = (9, 6)
# Define the size of each square on the chessboard in meters
square_size = 24
# Define the criteria for termination of the iterative process
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
# Prepare object points in a 3D pattern
objp = np.zeros((chessboard_size[0]*chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
objp *= square_size
# Create arrays to store object points and image points from all images
objpoints = []
imgpoints_left = []
imgpoints_right = []
# Load calibration images for both cameras
left_images = glob.glob('left*.png')
right_images = glob.glob('right*.png')

for left_img, right_img in zip(left_images, right_images):
    # Load the left and right images
    img_left = cv2.imread(left_img)
    img_right = cv2.imread(right_img)

    # Convert the images to grayscale
    gray_left = cv2.cvtColor(img_left, cv2.COLOR_BGR2GRAY)
    gray_right = cv2.cvtColor(img_right, cv2.COLOR_BGR2GRAY)

    # Find the chessboard corners in the images
    ret_left, corners_left = cv2.findChessboardCorners(gray_left, chessboard_size, None)
    ret_right, corners_right = cv2.findChessboardCorners(gray_right, chessboard_size, None)

    # If corners are found in both images, add object points and image points to the arrays
    if ret_left and ret_right:
        objpoints.append(objp)

        # Refine the corner positions to subpixel accuracy
        cv2.cornerSubPix(gray_left, corners_left, (11, 11), (-1, -1), criteria)
        cv2.cornerSubPix(gray_right, corners_right, (11, 11), (-1, -1), criteria)

        imgpoints_left.append(corners_left)
        imgpoints_right.append(corners_right)

#Calibrate left-right camera independently
ret_left, mtx_left, dist_left, rvecs_left, tvecs_left = cv2.calibrateCamera(
    objpoints, imgpoints_left, gray_left.shape[::-1], None, None
)
ret_right, mtx_right, dist_right, rvecs_right, tvecs_right = cv2.calibrateCamera(
    objpoints, imgpoints_right, gray_right.shape[::-1], None, None
)

print('K_left:\n', mtx_left)
print('D_left:\n', dist_left)
print('K_right:\n', mtx_right)
print('D_right:\n', dist_right)

# Calibrate stereo camera
ret, K_left, D_left, K_right, D_right, R, T, E, F = cv2.stereoCalibrate(
    objpoints, imgpoints_left, imgpoints_right,
    mtx_left, dist_left, mtx_right, dist_right,
    gray_left.shape[::-1],  # (width, height)
)
# Print calibration results
print('After stereo calibration:', ret)
print('K_left:\n', K_left)
print('D_left:\n', D_left)
print('K_right:\n', K_right)
print('D_right:\n', D_right)
print('R:\n', R)
print('T:\n', T)
print('E:\n', E)
print('F:\n', F)

# Stereo rectification
R1, R2, P1, P2, Q, _, _ = cv2.stereoRectify(
    K_left, D_left, K_right, D_right, gray_left.shape[::-1], R, T,
    alpha=0
)

# Save calibration results to a file
np.savez('calibration.npz', K_left=K_left, D_left=D_left, K_right=K_right, D_right=D_right, R=R, T=T, R1=R1, R2=R2, P1=P1, P2=P2, Q=Q)
