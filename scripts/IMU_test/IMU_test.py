#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Imu
import tf.transformations as tft
import math

def callback(msg):
    q = msg.orientation
    quat = [q.x, q.y, q.z, q.w]
    roll, pitch, yaw = tft.euler_from_quaternion(quat)
    print(f"Roll: {math.degrees(roll):.2f}, Pitch: {math.degrees(pitch):.2f}, Yaw: {math.degrees(yaw):.2f}")

rospy.init_node("imu_listener")
rospy.Subscriber("/imu/data_raw", Imu, callback)
rospy.spin()
