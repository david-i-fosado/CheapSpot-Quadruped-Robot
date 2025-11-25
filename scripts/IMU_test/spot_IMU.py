#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import math
import csv
import time
import tf.transformations as tft
from std_msgs.msg import Float32MultiArray
from cheapspot.Kinematics.spot_ik import SpotModel
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation as R
from simple_pid import PID



class BalanceController:
    def __init__(self):
        rospy.init_node("balance_controller", anonymous=True)

        # Modelo del robot
        self.spot = SpotModel()

        # Publisher hacia los servos
        self.pub_servos = rospy.Publisher("/servo_positions", Float32MultiArray, queue_size=1)

        # Subscriber a la IMU
        rospy.Subscriber("/imu/data_raw", Imu, self.imu_callback)

        # Referencia de postura (robot "derecho")
        self.ref_roll = 0.0
        self.ref_pitch = 0.0
        self.ref_yaw = 0.0   # lo podemos ignorar para balance
        self.roll_filtered = 0.0
        self.pitch_filtered = 0.0

        # PID para roll y pitch
        self.pid_roll = PID(1.8, 0.01, 0.01, setpoint=self.ref_roll)
        self.pid_pitch = PID(1.8, 0.01, 0.01, setpoint=self.ref_pitch)

        self.pid_roll.output_limits = (-0.25, 0.25)   # rad, ~±11°
        self.pid_pitch.output_limits = (-0.25, 0.25)

        # ===== CSV Logger =====
        self.csv_file = open("/home/david/catkin_ws/src/cheapspot/scripts/IMU_test/balance_log.csv", "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["time", "roll", "pitch", "roll_filtered", "pitch_filtered", "u_roll", "u_pitch"])
        self.t0 = time.time()

        rospy.loginfo("Nodo de balance iniciado")

    def imu_callback(self, msg):
        # Extraer orientación de la IMU (ya en radianes o convertir)
        q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        rpy = R.from_quat(q).as_euler('xyz', degrees=False)
        roll, pitch, yaw = rpy

        # --- Filtro pasa-bajas (exponencial) ---
        alpha = 0.2   # ajusta entre 0.05 y 0.2
        self.roll_filtered = alpha * roll + (1 - alpha) * self.roll_filtered
        self.pitch_filtered = alpha * pitch + (1 - alpha) * self.pitch_filtered

        # Error de balance
        u_roll = self.pid_roll(self.roll_filtered)
        u_pitch = self.pid_pitch(self.pitch_filtered)

        # Log en CSV
        t = time.time() - self.t0
        self.csv_writer.writerow([t, roll, pitch, self.roll_filtered, self.pitch_filtered, u_roll, u_pitch])

        # Postura corregida (base del cuerpo)
        x, y, z = 0.0, 0.0, 0.0  # cuerpo a 20cm de altura
        if abs(u_roll) > 0.02:
            roll_cmd = u_roll
        else:
            roll_cmd = 0
        if abs(u_pitch) > 0.02:
            pitch_cmd = u_pitch
        else:
            pitch_cmd = 0
        yaw_cmd = 0.0   # no corregimos yaw

        # Resolver IK con corrección
        T_bf = self.spot.WorldToFoot
        joint_angles = self.spot.IK([roll_cmd,pitch_cmd, 0], [x, y, z], T_bf)
        joint_degrees = np.degrees(joint_angles)

        # Mapear a servos
        servo_map = {
            "FL": [10,6,2],
            "FR": [9,5,1],
            "BL": [12,8,4],
            "BR": [11,7,3]
        }

        servo_positions = [90.0]*12
        for leg_idx, leg in enumerate(["FL","FR","BL","BR"]):
            for joint_idx, servo_num in enumerate(servo_map[leg]):
                raw = joint_degrees[leg_idx][joint_idx] + 90.0
                servo_positions[servo_num-1] = round(raw, 2)

        # Publicar en ROS
        self.pub_servos.publish(Float32MultiArray(data=servo_positions))

        rospy.loginfo(f"IMU roll:{roll:.5f} pitch:{pitch:.5f}")
        rospy.loginfo(f"Control roll:{u_roll:.5f}, pitch:{u_pitch:.5f}")
        rospy.loginfo(f"Servos: {servo_positions}")

    def spin(self):
       try:
           rospy.spin()
       finally:
           self.csv_file.close()

if __name__ == "__main__":
    node = BalanceController()
    node.spin()
