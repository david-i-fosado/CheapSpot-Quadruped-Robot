#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import csv
import time
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation as R
from simple_pid import PID

class PIDControlNode:
    def __init__(self):
        rospy.init_node("pid_control_node", anonymous=True)

        self.pub_corrections = rospy.Publisher("/balance/corrections", Float32MultiArray, queue_size=1)
        self.pub_state = rospy.Publisher("/balance/state", Float32MultiArray, queue_size=1)

        rospy.Subscriber("/imu/data_raw", Imu, self.imu_callback)
        rospy.Subscriber("/balance/fuzzy_tunings", Float32MultiArray, self.tunings_callback)

        self.imu_data_received = False
        
        self.ref_roll = 0.0
        self.ref_pitch = 0.0
        self.roll_filtered = 0.0
        self.pitch_filtered = 0.0

        self.initial_kp = 1.0
        self.initial_ki = 0.1
        self.initial_kd = 0.01
        
        self.pid_roll = PID(self.initial_kp, self.initial_ki, self.initial_kd, setpoint=self.ref_roll)
        self.pid_pitch = PID(self.initial_kp, self.initial_ki, self.initial_kd, setpoint=self.ref_pitch)
        
        self.pid_roll.output_limits = (-0.25, 0.25)
        self.pid_pitch.output_limits = (-0.25, 0.25)
        
        self.current_kp_roll = self.initial_kp
        self.current_kp_pitch = self.initial_kp
        self.current_ki_roll = self.initial_ki
        self.current_ki_pitch = self.initial_ki
        self.current_kd_roll = self.initial_kd
        self.current_kd_pitch = self.initial_kd

        # --- MODIFICADO: CSV Logger para TODAS las ganancias ---
        self.csv_file = open("/home/david/catkin_ws/src/cheapspot/scripts/IMU_test/fuzzypid_final.csv", "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow([
            "time", 
            "roll_raw", "pitch_raw", 
            "roll_filtered", "pitch_filtered", 
            "u_roll", "u_pitch", 
            "Kp_roll", "Ki_roll", "Kd_roll",
            "Kp_pitch", "Ki_pitch", "Kd_pitch"
        ])
        self.t0 = time.time()
        
        rospy.loginfo("Nodo de Control PID (Bucle Rápido) iniciado. Esperando datos de IMU...")


    def tunings_callback(self, msg):
        self.current_kp_roll = msg.data[0]
        self.current_kp_pitch = msg.data[1]
        self.current_ki_roll = msg.data[2]
        self.current_ki_pitch = msg.data[3]
        self.current_kd_roll = msg.data[4]
        self.current_kd_pitch = msg.data[5]

    def imu_callback(self, msg):
        if not self.imu_data_received:
            rospy.loginfo("Primer dato de IMU recibido. Iniciando publicación de estado.")
            self.imu_data_received = True

        # 1. Extracción y filtrado
        q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        try:
            rpy = R.from_quat(q).as_euler('xyz', degrees=False)
        except Exception:
            return
        roll, pitch, yaw = rpy # Estos son los datos "raw"
        alpha = 0.2
        self.roll_filtered = alpha * roll + (1 - alpha) * self.roll_filtered
        self.pitch_filtered = alpha * pitch + (1 - alpha) * self.pitch_filtered
        
        # 2. Publicar estado
        if self.imu_data_received:
            state_data = Float32MultiArray(data=[self.roll_filtered, self.pitch_filtered])
            self.pub_state.publish(state_data)

        # 3. Actualizar PID
        self.pid_roll.tunings = (self.current_kp_roll, self.current_ki_roll, self.current_kd_roll)
        self.pid_pitch.tunings = (self.current_kp_pitch, self.current_ki_pitch, self.current_kd_pitch)

        # 4. Calcular control
        u_roll = self.pid_roll(self.roll_filtered)
        u_pitch = self.pid_pitch(self.pitch_filtered)
         
        # 5. --- MODIFICADO: Log en CSV (¡SIN FLUSH!) ---
        t = time.time() - self.t0
        self.csv_writer.writerow([
            t, 
            roll, pitch, # Guardamos los datos raw de la IMU
            self.roll_filtered, self.pitch_filtered, 
            u_roll, u_pitch, 
            self.current_kp_roll, self.current_ki_roll, self.current_kd_roll,
            self.current_kp_pitch, self.current_ki_pitch, self.current_kd_pitch
        ])

        # 6. Publicar correcciones
        roll_cmd = u_roll if abs(u_roll) > 0.02 else 0.0
        pitch_cmd = u_pitch if abs(u_pitch) > 0.02 else 0.0
        corrections = Float32MultiArray(data=[roll_cmd, pitch_cmd])
        self.pub_corrections.publish(corrections)

    def run(self):
        try:
            rospy.spin()
        finally:
            self.csv_file.close()
            rospy.loginfo("Nodo de Control PID cerrado, CSV guardado.")

if __name__ == "__main__":
    node = PIDControlNode()
    node.run()
