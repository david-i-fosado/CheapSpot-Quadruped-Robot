#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from cheapspot.msg import GaitCommand
import sys, select, termios, tty

# Teclas → (step_length, lateral_fraction, yaw_rate)
moveBindings = {
	'w': (0.025, 0.0, 0.0),   # adelante
	's': (-0.025, 0.0, 0.0),  # atrás
	'a': (0.025, 1.3, 0.0),	# izquierda
	'd': (0.025, -1.3, 0.0),   # derecha
	'q': (0.005, 0.0, 0.8),	# giro izq
	'e': (0.005, 0.0, -0.8),   # giro der
}


def getKey(settings):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.05)  # 50 ms
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

if __name__=="__main__":
    settings = termios.tcgetattr(sys.stdin)
    rospy.init_node('gait_teleop')
    pub = rospy.Publisher('/gait_cmd', GaitCommand, queue_size=1)
    rate = rospy.Rate(20)  # 20 Hz

    try:
        while not rospy.is_shutdown():
            key = getKey(settings)
            msg = GaitCommand()

            if key in moveBindings:
                msg.step_length, msg.lateral_fraction, msg.yaw_rate = moveBindings[key]
            elif key == '\x03':  # CTRL-C
                break
            else:
                # si no hay tecla, todo a 0
                msg.step_length = 0.0
                msg.lateral_fraction = 0.0
                msg.yaw_rate = 0.0

            pub.publish(msg)
            rospy.loginfo(f"Cmd -> step:{msg.step_length:.2f}, lat:{msg.lateral_fraction:.2f}, yaw:{msg.yaw_rate:.2f}")
            rate.sleep()

    except Exception as e:
        print(e)
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
