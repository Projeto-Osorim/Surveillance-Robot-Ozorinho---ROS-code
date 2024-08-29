#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge

def image_callback(msg):
    try:
        bridge = CvBridge()
        # Converter mensagem ROS para imagem OpenCV
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        cv2.imshow("Received Image", cv_image)
        cv2.waitKey(1)
    except Exception as e:
        print(e)

def camera_subscriber():
    rospy.init_node('camera_subscriber', anonymous=True)
    rospy.Subscriber('camera_image', Image, image_callback)
    rospy.spin()

if __name__ == '_main_':
    try:
        camera_subscriber()
    except rospy.ROSInterruptException:
        pass