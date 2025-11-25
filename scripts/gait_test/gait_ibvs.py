#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
import copy
#from cheapspot.msg import GaitCommand
from geometry_msgs.msg import Twist

from cheapspot.Kinematics.spot_ik import SpotModel
from cheapspot.GaitGenerator.Bezier import BezierGait
from cheapspot.OpenLoop.spotOL import BezierStepper

class BezierIKBridge:
    def __init__(self):
        rospy.init_node("bezier_ik_bridge", anonymous=True)

        # Modelo cinemático de Spot
        self.spot = SpotModel()
        self.T_bf0 = self.spot.WorldToFoot
        self.T_bf = copy.deepcopy(self.T_bf0)

        # Generador Bézier y Stepper
        self.dt = 0.04  # 20 Hz
        self.bzg = BezierGait(dt=self.dt, Tswing=0.23) #0.18 best foward
        self.bz_step = BezierStepper(dt=self.dt, mode=0)

        # Parámetros de prueba (fijos)
        self.StepLength = 0.0
        self.LateralFraction = 0.0
        self.YawRate = 0.0
        self.StepVelocity = 0.0001
        self.ClearanceHeight = 0.015 # 0.01 best foward
        self.PenetrationDepth = 0.0
        self.contacts = [1, 1, 1, 1]

        self.roll_cmd = 0.0
        self. pitch_cmd = 0.0

        self.roll_vel = 0.0
        self.pitch_vel = 0.0
        self.yaw_vel = 0.0

        # Se subscribe a GaitCommand
        #rospy.Subscriber("/gait_cmd", GaitCommand, self.gait_callback)
        rospy.Subscriber("/gait_cmd_twist", Twist, self.gait_callback)
        rospy.Subscriber("/balance/corrections", Float32MultiArray, self.balance_callback)

        # Publicador de posiciones de servos
        self.pub_servos = rospy.Publisher("/servo_positions", Float32MultiArray, queue_size=10)

        # Mapeo de servos como en tu código
        self.servo_map = {
            "FL": [10, 6, 2],
            "FR": [9, 5, 1],
            "BL": [12, 8, 4],
            "BR": [11, 7, 3]
        }

    def gait_callback(self, msg):
        #self.StepLength = msg.step_length
        #self.LateralFraction = msg.lateral_fraction
        #self.YawRate = msg.yaw_rate
        self.StepLength = msg.linear.x
        self.LateralFraction = msg.linear.y
        self.YawRate = msg.angular.z
        self.roll_vel = msg.angular.x
        self.pitch_vel = msg.angular.y
        #rospy.loginfo(f"X:{self.StepLength:.3f}")
        #rospy.loginfo(f"Y:{self.LateralFraction:3f}")
        #rospy.loginfo(self.YawRate)

    def balance_callback(self, msg):
        self.roll_cmd = msg.data[0]
        self.pitch_cmd = msg.data[1]

    def run(self):
        rate = rospy.Rate(1 / self.dt)
        rospy.loginfo("Nodo spot_gait iniciado. Publicando trayectoria...")

        while not rospy.is_shutdown():
            self.bz_step.ramp_up()

            self.roll_cmd += self.roll_vel * self.dt
            self.pitch_cmd += self.pitch_vel * self.dt
            max_angle = 0.35
            self.roll_cmd = np.clip(self.roll_cmd, -max_angle, max_angle)
            self.pitch_cmd = np.clip(self.pitch_cmd, -max_angle, max_angle)

            # Generar trayectoria de pies
            self.T_bf = self.bzg.GenerateTrajectory(
                self.StepLength,
                self.LateralFraction,
                self.YawRate,
                self.StepVelocity,
                self.T_bf0,
                self.T_bf,
                self.ClearanceHeight,
                self.PenetrationDepth,
                self.contacts
            )

            # Resolver IK
            joint_angles = self.spot.IK([self.roll_cmd, self.pitch_cmd, 0], [0, 0, 0], self.T_bf)
            joint_degrees = np.degrees(joint_angles)

            # Convertir a posiciones de servos
            servo_positions = [90.0] * 12
            for leg_idx, leg in enumerate(["FL", "FR", "BL", "BR"]):
                for joint_idx, servo_num in enumerate(self.servo_map[leg]):
                    raw = joint_degrees[leg_idx][joint_idx] + 90.0
                    servo_positions[servo_num - 1] = round(raw, 2)

            # Publicar
            msg_out = Float32MultiArray(data=servo_positions)
            #rospy.loginfo(f"Roll:{self.roll_cmd:.3f}, Pitch:{self.pitch_cmd:.3f}")
            #rospy.loginfo(servo_positions)
            self.pub_servos.publish(msg_out)
            rate.sleep()

if __name__ == "__main__":
    try:
        bridge = BezierIKBridge()
        bridge.run()
    except rospy.ROSInterruptException:
        pass
