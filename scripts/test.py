import cv2
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
fs = cv2.FileStorage("C:/Users/HP/OneDrive/Desktop/test_left.yaml", cv2.FILE_STORAGE_READ)
fs_1 = cv2.FileStorage("C:/Users/HP/OneDrive/Desktop/test_right.yaml", cv2.FILE_STORAGE_READ)

# Load calibration files for both cameras
calib_left = cv2.FileStorage("C:/Users/HP/OneDrive/Desktop/test_left.yaml", cv2.FILE_STORAGE_READ)
calib_right = cv2.FileStorage("C:/Users/HP/OneDrive/Desktop/test_right.yaml", cv2.FILE_STORAGE_READ)
camera_matrix_left = calib_left.getNode('camera_matrix').mat()
dist_coeffs_left = calib_left.getNode('distortion_coefficients').mat()
camera_matrix_right = calib_right.getNode('camera_matrix').mat()
dist_coeffs_right = calib_right.getNode('distortion_coefficients').mat()

# Load stereo rectification parameters
R1 = calib_left.getNode('rectification_matrix').mat()
R2 = calib_right.getNode('rectification_matrix').mat()
P1 = calib_left.getNode('projection_matrix').mat()
P2 = calib_right.getNode('projection_matrix').mat()

# Load left and right images
img_left = cv2.imread("C:/Study/camera_project/Camera calib/images/left-0000.png",cv2.IMREAD_GRAYSCALE)
img_right = cv2.imread("C:/Study/camera_project/Camera calib/images/right-0000.png", cv2.IMREAD_GRAYSCALE)

# print(camera_matrix_left)
# print(camera_matrix_right)

# Apply undistortion and rectification to the images
img_left_rect = cv2.undistort(img_left, camera_matrix_left, dist_coeffs_left, None, P1)
img_right_rect = cv2.undistort(img_right, camera_matrix_right, dist_coeffs_right, None, P2)

# Compute disparity map using block matching
block_size = 5
min_disp = 0
num_disp = 64
stereo_bm = cv2.StereoBM_create(numDisparities=num_disp, blockSize=block_size)
disparity_map = stereo_bm.compute(img_left_rect, img_right_rect)

# Compute 3D coordinates of object using disparity and epipolar geometry
focal_length = P1[0, 0]
baseline = -P2[0, 3]/P2[0, 0]
x, y = np.meshgrid(range(disparity_map.shape[1]), range(disparity_map.shape[0]))
depth_map = (focal_length * baseline) / (disparity_map + np.finfo(float).eps)
X = (x - P1[0, 2]) * depth_map / focal_length
Y = (y - P1[1, 2]) * depth_map / focal_length
Z = depth_map

#------------------------Debug code------------------------------#
#depth_map = np.abs(depth_map)
#print("Baseline:",baseline)
#disparity_map = np.abs(disparity_map)
print(disparity_map)
print(depth_map)
# Display depth map
cv2.imshow('Dis Map', disparity_map)
cv2.imshow('Depth Map', depth_map)
cv2.imshow('left_rect',img_left_rect)
# concatenated_image = cv2.hconcat([img_left, img_right])
# concatenated_image_rect = cv2.hconcat([img_left_rect, img_right_rect])
# cv2.imshow("Raw images",concatenated_image)
# cv2.imshow("Rectificated images",concatenated_image_rect)
# cv2.imwrite("C:/Study/camera_project/Camera calib/images_rect/left_1.png",img_left_rect)
# cv2.imwrite("C:/Study/camera_project/Camera calib/images_rect/right_1.png",img_right_rect)
cv2.waitKey(0)
#----------------------------------------------------------------#