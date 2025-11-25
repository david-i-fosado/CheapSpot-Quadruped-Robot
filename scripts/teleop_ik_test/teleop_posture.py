#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Float32MultiArray
import sys, select, termios, tty

# Teclas que mapearemos
moveBindings = {
    'w': (0.0005, 0, 0, 0, 0, 0),   # +x
    's': (-0.0005, 0, 0, 0, 0, 0),  # -x
    'a': (0, 0.0005, 0, 0, 0, 0),   # +y
    'd': (0, -0.0005, 0, 0, 0, 0),  # -y
    'i': (0, 0, 0.0005, 0, 0, 0),   # +z
    'k': (0, 0, -0.0005, 0, 0, 0),  # -z
    'j': (0, 0, 0, 0.005, 0, 0),   # roll +
    'l': (0, 0, 0, -0.005, 0, 0),  # roll -
    'u': (0, 0, 0, 0, 0.005, 0),   # pitch +
    'o': (0, 0, 0, 0, -0.005, 0),  # pitch -
    'q': (0, 0, 0, 0, 0, 0.005),   # yaw +
    'e': (0, 0, 0, 0, 0, -0.005),  # yaw -
}

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('posture_teleop')
    pub = rospy.Publisher('/body_posture', Float32MultiArray, queue_size=1)

    x, y, z = 0.0, 0.0, 0.0
    roll, pitch, yaw = 0.0, 0.0, 0.0

    try:
        while not rospy.is_shutdown():
            key = getKey()
            if key in moveBindings.keys():
                dx, dy, dz, droll, dpitch, dyaw = moveBindings[key]
                x += dx
                y += dy
                z += dz
                roll += droll
                pitch += dpitch
                yaw += dyaw

                msg = Float32MultiArray()
                msg.data = [x, y, z, roll, pitch, yaw]
                rospy.loginfo(msg)
                pub.publish(msg)

                print(f"Postura -> x:{x:.2f}, y:{y:.2f}, z:{z:.2f}, roll:{roll:.2f}, pitch:{pitch:.2f}, yaw:{yaw:.2f}")

            elif key == '\x03': # CTRL-C
                break

    except Exception as e:
        print(e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
