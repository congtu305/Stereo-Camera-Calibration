#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import CameraInfo
from sensor_msgs.srv import SetCameraInfo, SetCameraInfoResponse

def set_camera_info(req):
    # Store the received camera info message in a file
    file_path = "/home/tu/my_vision/camera_info.yaml"
    with open(file_path, "w") as f:
        f.write("# Camera calibration file\n")
        f.write("# Insert camera calibration parameters here\n")
        f.write(str(req.camera_info))

    # Send a response to the client
    res = SetCameraInfoResponse()
    res.success = True
    res.status_message = "Camera info set successfully"
    return res

if __name__ == "__main__":
    rospy.init_node("camera_info_publisher")

    # Create publishers for the left and right camera info topics
    left_pub = rospy.Publisher("/my_stereo/left/camera_info", CameraInfo, queue_size=10)
    right_pub = rospy.Publisher("/my_stereo/right/camera_info", CameraInfo, queue_size=10)

    # Create services for setting the left and right camera info
    left_service = rospy.Service("/my_stereo/left/set_camera_info", SetCameraInfo, set_camera_info)
    right_service = rospy.Service("/my_stereo/right/set_camera_info", SetCameraInfo, set_camera_info)

    # Publish camera info messages periodically
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown():
        # Create a dummy camera info message with calibration parameters
        camera_info_msg = CameraInfo()
        # Fill in the calibration parameters here
        camera_info_msg.header.stamp = rospy.Time.now()
        camera_info_msg.header.frame_id = "my_stereo_camera_frame"

        # Publish the camera info messages
        left_pub.publish(camera_info_msg)
        right_pub.publish(camera_info_msg)

        rate.sleep()

