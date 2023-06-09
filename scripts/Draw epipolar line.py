import numpy as np
import cv2
from matplotlib import pyplot as plt
img_left = cv2.imread('C:/Study/camera_project/Camera calib/images_rect/left_1.png', cv2.IMREAD_GRAYSCALE)  #queryimage # left image
img_right = cv2.imread('C:/Study/camera_project/Camera calib/images_rect/right_1.png', cv2.IMREAD_GRAYSCALE) #trainimage # right image
sift = cv2.SIFT_create()
# find the keypoints and descriptors with SIFT
kp1, des1 = sift.detectAndCompute(img_left,None)
kp2, des2 = sift.detectAndCompute(img_right,None)
# FLANN parameters
FLANN_INDEX_KDTREE = 1
index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
search_params = dict(checks=50)
flann = cv2.FlannBasedMatcher(index_params,search_params)
matches = flann.knnMatch(des1,des2,k=2)
pts1 = []
pts2 = []
# ratio test as per Lowe's paper
for i,(m,n) in enumerate(matches):
    if m.distance < 0.8*n.distance:
        pts2.append(kp2[m.trainIdx].pt)
        pts1.append(kp1[m.queryIdx].pt)
pts1 = np.int32(pts1)
pts2 = np.int32(pts2)
F, mask = cv2.findFundamentalMat(pts1,pts2,cv2.FM_LMEDS)
#------------------------------------------------------------------------------#
# We select only inlier points
pts1 = pts1[mask.ravel()==1]
pts2 = pts2[mask.ravel()==1]
def drawlines(img1,img2,lines,pts1,pts2):
    ''' img1 - image on which we draw the epilines for the points in img2
        lines - corresponding epilines '''
    r,c = img1.shape
    img1 = cv2.cvtColor(img1,cv2.COLOR_GRAY2BGR)
    img2 = cv2.cvtColor(img2,cv2.COLOR_GRAY2BGR)
    for r,pt1,pt2 in zip(lines,pts1,pts2):
        color = tuple(np.random.randint(0,255,3).tolist())
        x0,y0 = map(int, [0, -r[2]/r[1] ])
        x1,y1 = map(int, [c, -(r[2]+r[0]*c)/r[1] ])
        img1 = cv2.line(img1, (x0,y0), (x1,y1), color,1)
        img1 = cv2.circle(img1,tuple(pt1),5,color,-1)
        img2 = cv2.circle(img2,tuple(pt2),5,color,-1)
    return img1,img2
# Find epilines corresponding to points in right image (second image) and
# drawing its lines on left image
lines1 = cv2.computeCorrespondEpilines(pts2.reshape(-1,1,2), 2,F)
lines1 = lines1.reshape(-1,3)
img5,img6 = drawlines(img_left,img_right,lines1,pts1,pts2)
# Find epilines corresponding to points in left image (first image) and
# drawing its lines on right image
lines2 = cv2.computeCorrespondEpilines(pts1.reshape(-1,1,2), 1,F)
lines2 = lines2.reshape(-1,3)
img3,img4 = drawlines(img_left,img_right,lines2,pts2,pts1)
plt.subplot(121),plt.imshow(img5)
plt.subplot(122),plt.imshow(img3)
plt.show()


















# # Define the point in the left image to draw epipolar lines for
# point = (100, 200)

# # Convert the point to homogeneous coordinates
# point_homogeneous = np.array([point[0], point[1], 1])

# # Compute the corresponding epipolar line in the right image
# epipolar_line = np.dot(F, point_homogeneous)
# print(epipolar_line)
# print(F)
# # Compute the two endpoints of the epipolar line
# endpoints = np.dot(np.array([[1, 0, 0], [0, 1, 0], [-epipolar_line[2], 0, epipolar_line[0]]]), np.array([0, img_right.shape[0], 1]))

# # Draw the epipolar line on the right image
# cv2.line(img_right, (int(endpoints[0]), int(endpoints[1])), (int(endpoints[0] + endpoints[2]), int(endpoints[1])), (0, 0, 255), 1)

# # Display the left and right images with the epipolar line
# cv2.imshow('Left Image', img_left)
# cv2.imshow('Right Image with Epipolar Line', img_right)

# # Wait for a key press and then close the windows
# cv2.waitKey(0)
# cv2.destroyAllWindows()