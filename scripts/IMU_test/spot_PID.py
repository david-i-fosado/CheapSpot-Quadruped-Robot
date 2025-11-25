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
        self.pub_corrections = rospy.Publisher("/balance/corrections", Float32MultiArray, queue_size=1)

        # Subscriber a la IMU
        rospy.Subscriber("/imu/data_raw", Imu, self.imu_callback)

        # Referencia de postura (robot "derecho")
        self.ref_roll = 0.0
        self.ref_pitch = 0.0
        self.ref_yaw = 0.0
        self.e_roll = 0.0
        self.e_pitch = 0.0

        # Filtros
        self.roll_filtered = 0.0
        self.pitch_filtered = 0.0

        # PID para roll y pitch
        self.pid_roll = PID(1.8, 0.4, 0.1, setpoint=self.ref_roll)
        self.pid_pitch = PID(1.8, 0.4, 0.1, setpoint=self.ref_pitch)
        self.pid_roll.output_limits = (-0.28, 0.28)   # radianes
        self.pid_pitch.output_limits = (-0.28, 0.28)  # radianes

        # CSV Logger
        self.csv_file = open("/home/david/catkin_ws/src/cheapspot/scripts/IMU_test/balance_log.csv", "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["time", "roll", "pitch", "roll_filtered", "pitch_filtered", "u_roll", "u_pitch","e_roll","e_pitch"])
        self.t0 = time.time()

        rospy.loginfo("Nodo de balance iniciado")

    def imu_callback(self, msg):
        # Extraer orientación de la IMU
        q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        try:
            rpy = R.from_quat(q).as_euler('xyz', degrees=False)
        except Exception:
            return
        roll, pitch, yaw = rpy

        # Filtro pasa-bajas exponencial
        alpha = 0.2
        self.roll_filtered = alpha * roll + (1 - alpha) * self.roll_filtered
        self.pitch_filtered = alpha * pitch + (1 - alpha) * self.pitch_filtered

        self.e_roll = self.pid_roll.setpoint - self.roll_filtered
        self.e_pitch = self.pid_pitch.setpoint - self.pitch_filtered

        # Control PID
        u_roll = self.pid_roll(self.roll_filtered)
        u_pitch = self.pid_pitch(self.pitch_filtered)

        # Log en CSV
        t = time.time() - self.t0
        self.csv_writer.writerow([t, roll, pitch, self.roll_filtered, self.pitch_filtered, u_roll, u_pitch, self.e_roll, self.e_pitch])
        self.csv_file.flush()

        # Deadband pequeño
        roll_cmd = u_roll if abs(u_roll) > 0.02 else 0.0
        pitch_cmd = u_pitch if abs(u_pitch) > 0.02 else 0.0
        yaw_cmd = 0.0

        corrections = Float32MultiArray(data=[roll_cmd, pitch_cmd])
        self.pub_corrections.publish(corrections)

        rospy.loginfo(f"IMU roll:{roll:.5f} pitch:{pitch:.5f}")
        rospy.loginfo(f"Control roll:{u_roll:.5f}, pitch:{u_pitch:.5f}")

    def spin(self):
        try:
            rospy.spin()
        finally:
            self.csv_file.close()

if __name__ == "__main__":
    node = BalanceController()
    node.spin()
