#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

cap_left = cv2.VideoCapture(0)
cap_right = cv2.VideoCapture("http://192.168.1.6:8080/video")
print(cap_left.isOpened())
print(cap_right.isOpened())
bridge_left = CvBridge()
bridge_right = CvBridge()

def talker():
   pub_left = rospy.Publisher('/my_stereo/left/image_raw', Image, queue_size = 1)
   pub_right = rospy.Publisher('/my_stereo/right/image_raw', Image, queue_size = 1)
   rate = rospy.Rate(10)
   while not rospy.is_shutdown():
     ret_left, frame_left = cap_left.read()
     ret_right, frame_right = cap_right.read()
     if not (ret_left and ret_right):
         break
     #height, width, channels = frame_right.shape
     #frame_left = cv2.resize(frame_left, (width, height))
     msg_left = bridge_left.cv2_to_imgmsg(frame_left,"bgr8")
     pub_left.publish(msg_left)
     msg_right = bridge_right.cv2_to_imgmsg(frame_right,"bgr8")
     pub_right.publish(msg_right)
     if cv2.waitKey(1) & 0xFF == ord('q'):
         break
     if rospy.is_shutdown():
         cap_left.release()
         cap_right.release()
if __name__ == '__main__':
   rospy.init_node('stereo_camera', anonymous=False)
   try:
     talker()
   except rospy.ROSInterruptException:
     pass

