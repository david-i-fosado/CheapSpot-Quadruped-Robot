#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
import copy

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
        self.bzg = BezierGait(dt=self.dt,Tswing=0.4)
        self.bz_step = BezierStepper(dt=self.dt, mode=0)

        # Parámetros de prueba (fijos)
        self.StepLength = 0.025
        self.LateralFraction = -0.9
        self.YawRate = 0.0
        self.StepVelocity = 0.00001
        self.ClearanceHeight = 0.03
        self.PenetrationDepth = 0.0
        self.contacts = [1, 1, 1, 1]

        # Publicador de posiciones de servos
        self.pub_servos = rospy.Publisher("/servo_positions", Float32MultiArray, queue_size=1)

        # Mapeo de servos como en tu código
        self.servo_map = {
            "FL": [10,6,2],
            "FR": [9,5,1],
            "BL": [12,8,4],
            "BR": [11,7,3]
        }

    def run(self):
        rate = rospy.Rate(1/self.dt)
        rospy.loginfo("Nodo spot_gait iniciado. Publicando trayectorias...")

        while not rospy.is_shutdown():
            self.bz_step.ramp_up()

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
            joint_angles = self.spot.IK([0,0,0], [0,0,0], self.T_bf)
            joint_degrees = np.degrees(joint_angles)

            # Convertir a posiciones de servos
            servo_positions = [90.0]*12
            for leg_idx, leg in enumerate(["FL","FR","BL","BR"]):
                for joint_idx, servo_num in enumerate(self.servo_map[leg]):
                    raw = joint_degrees[leg_idx][joint_idx] + 90.0
                    servo_positions[servo_num-1] = round(raw, 2)

            # Publicar
            msg_out = Float32MultiArray(data=servo_positions)
            rospy.loginfo(servo_positions)
            self.pub_servos.publish(msg_out)

            rate.sleep()

if __name__ == "__main__":
    try:
        bridge = BezierIKBridge()
        bridge.run()
    except rospy.ROSInterruptException:
        pass
