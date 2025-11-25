#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
from cheapspot.Kinematics.spot_ik import SpotModel   # <-- importa tu clase (ajusta el nombre del archivo si no se llama spot_model.py)

# Nodo puente IK: body_posture -> servo_positions
class IKBridge:
    def __init__(self):
        # Inicializa ROS
        rospy.init_node("ik_bridge", anonymous=True)

        # Crea el modelo de Spot
        self.spot = SpotModel()

        # Publicador de posiciones de servos
        self.pub_servos = rospy.Publisher("/servo_positions", Float32MultiArray, queue_size=1)

        # Suscriptor a posturas
        rospy.Subscriber("/body_posture", Float32MultiArray, self.callback)

        rospy.loginfo("Nodo ik_bridge iniciado. Escuchando /body_posture...")

    def callback(self, msg):
        """
        Convierte postura [x,y,z,roll,pitch,yaw] a posiciones de servos
        """
        try:
            # Extraer postura
            x, y, z, roll, pitch, yaw = msg.data
            pos = np.array([x, y, z])
            orn = np.array([roll, pitch, yaw])

            # Usar las posiciones base de los pies
            T_bf = self.spot.WorldToFoot

            # Resolver IK
            joint_angles = self.spot.IK(orn, pos, T_bf)

            # Convertir a grados y mapear a servos
            joint_degrees = np.degrees(joint_angles)

            servo_map = {
                "FL": [10,6,2],
                "FR": [9,5,1],
                "BL": [12,8,4],
                "BR": [11,7,3]
            }

            servo_positions = [90.0]*12  # Inicializar en 90°
            for leg_idx, leg in enumerate(["FL","FR","BL","BR"]):
                for joint_idx, servo_num in enumerate(servo_map[leg]):
                    raw = joint_degrees[leg_idx][joint_idx] + 90.0
                    servo_positions[servo_num-1] = round(raw, 2)

            # Publicar en ROS
            msg_out = Float32MultiArray(data=servo_positions)
            self.pub_servos.publish(msg_out)

            rospy.loginfo(f"Postura -> x:{x:.2f}, y:{y:.2f}, z:{z:.2f}, roll:{roll:.2f}, pitch:{pitch:.2f}, yaw:{yaw:.2f}")
            rospy.loginfo(f"Servos: {servo_positions}")

        except Exception as e:
            rospy.logerr(f"Error en callback IK: {e}")

    def spin(self):
        rospy.spin()


if __name__ == "__main__":
    node = IKBridge()
    node.spin()
