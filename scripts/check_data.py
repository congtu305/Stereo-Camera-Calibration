import cv2
import os
import glob
import cv2

Chess_Board_Dimensions = (9, 6)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

def detect_checker_board(image, criteria, boardDimension):
    ret, corners = cv2.findChessboardCorners(image, boardDimension)
    if ret == True:
        corners1 = cv2.cornerSubPix(image, corners, (3, 3), (-1, -1), criteria)
        image = cv2.drawChessboardCorners(image, boardDimension, corners1, ret)

    return image, ret

# Load images
left_images = glob.glob('left*.png')
right_images = glob.glob('right*.png')

ex = False
# Loop over each image pair
for left_img, right_img in zip(left_images, right_images):
    # Load left and right images
    img_left = cv2.imread(left_img, cv2.IMREAD_GRAYSCALE)
    img_right = cv2.imread(right_img, cv2.IMREAD_GRAYSCALE)

    img_left, ret = detect_checker_board(img_left,criteria,Chess_Board_Dimensions)
    img_right, ret = detect_checker_board(img_right,criteria,Chess_Board_Dimensions)

    cv2.imshow("left",img_left)
    cv2.imshow("right",img_right)
    if ex:
        break
    while (cv2.waitKey(0) != ord("d")):
        if cv2.waitKey(1) != ord("q"):
            ex = True
            break
