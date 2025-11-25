#!/usr/bin/env python3
# -*- coding: utf-8 -*-


import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

rospy.init_node('my_camera_pub')
pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
bridge = CvBridge()
cap = cv2.VideoCapture(0)

while not rospy.is_shutdown():
    ret, frame = cap.read()
    if ret:
        msg = bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        pub.publish(msg)
