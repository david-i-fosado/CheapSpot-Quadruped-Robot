#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
import math
from sensor_msgs.msg import Joy
from cheapspot.msg import GaitCommand

class PS4Teleop:
    def __init__(self):
        rospy.init_node('ps4_teleop')

        # Publisher
        self.pub = rospy.Publisher('/gait_cmd', GaitCommand, queue_size=1)

        # Suscripción al joystick
        rospy.Subscriber('/joy', Joy, self.joy_callback)

        rospy.loginfo("Teleop con control PS4 iniciado.")
        rospy.spin()

    def joy_callback(self, data: Joy):
        msg = GaitCommand()
        lx = data.axes[0] # izquierda/derecha
        ly = data.axes[1] # adelante/ atras
        rx = data.axes[3] # yaw

        # Mapeo de ejes (puedes ajustar según lo que quieras)
        # data.axes[1] -> stick izq vertical (adelante/atras)
        # data.axes[0] -> stick izq horizontal (izquierda/derecha)
        # data.axes[2] o data.axes[3] -> stick derecho para yaw
        msg.step_length = 0.03 * math.sqrt(lx**2 + ly**2)          # adelante/atrás
        msg.lateral_fraction = 1.3 * lx      # lateral
        msg.yaw_rate = 0.8 * rx              # giro

        self.pub.publish(msg)
        rospy.loginfo(f"Cmd -> step:{msg.step_length:.2f}, lat:{msg.lateral_fraction:.2f}, yaw:{msg.yaw_rate:.2f}")

if __name__ == "__main__":
    try:
        PS4Teleop()
    except rospy.ROSInterruptException:
        pass
