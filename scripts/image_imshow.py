#!/usr/bin/env python3

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


cap = cv2.VideoCapture(0)
print(cap.isOpened())
bridge = CvBridge()

#cap.set(cv2.CAP_PROP_FRAME_WIDTH, 3840)
#cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 2160)
#cap.set(cv2.CAP_PROP_FPS, 30)

def talker():
   pub = rospy.Publisher('/image_raw', Image, queue_size = 1)
   rospy.init_node('image', anonymous = False)
   rate = rospy.Rate(10)
   while not rospy.is_shutdown():
     ret, frame = cap.read()
     if not ret:
         break
     msg = bridge.cv2_to_imgmsg(frame,"bgr8")
     pub.publish(msg)
     if cv2.waitKey(1) & 0xFF == ord('q'):
         break
     if rospy.is_shutdown():
         cap.release()
         
if __name__ == '__main__':
   try:
     talker()
   except rospy.ROSInterruptException:
     pass
