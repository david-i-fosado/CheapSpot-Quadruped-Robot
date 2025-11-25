#!/usr/bin/env python3
from geometry_msgs.msg import Quaternion
import tf.transformations as tft
import rospy
import time
import smbus2
import numpy as np
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from ahrs.filters import Madgwick
from scipy.spatial.transform import Rotation as R
import math

# ===== Configuración MPU9250 =====
MPU_ADDR = 0x68
PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B
GYRO_XOUT_H  = 0x43

bus = smbus2.SMBus(1)
bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0)  # despertar sensor

# --- Funciones de lectura ---
def read_word(reg):
    high = bus.read_byte_data(MPU_ADDR, reg)
    low = bus.read_byte_data(MPU_ADDR, reg+1)
    val = (high << 8) + low
    if val >= 0x8000:
        val = -((65535 - val) + 1)
    return val

def read_accel():
    ax = read_word(ACCEL_XOUT_H) / 16384.0
    ay = read_word(ACCEL_XOUT_H+2) / 16384.0
    az = read_word(ACCEL_XOUT_H+4) / 16384.0
    return np.array([ax, ay, az])

def read_gyro():
    gx = read_word(GYRO_XOUT_H) / 131.0
    gy = read_word(GYRO_XOUT_H+2) / 131.0
    gz = read_word(GYRO_XOUT_H+4) / 131.0
    return np.radians(np.array([gx, gy, gz]))  # rad/s

# --- Nodo ROS ---
def imu_publisher():
    rospy.init_node("imu_node")
    pub = rospy.Publisher("/imu/data_raw", Imu, queue_size=1)
    rate = rospy.Rate(1/0.04)  # 20 Hz 1/0.04

    # ===== Calibración simple =====
    rospy.loginfo("Calibrando MPU9250, mantén el sensor quieto...")
    n_cal = 500
    acc_offset = np.zeros(3)
    gyro_offset = np.zeros(3)

    for _ in range(n_cal):
        acc_offset += read_accel()
        gyro_offset += read_gyro()
        time.sleep(0.01)

    acc_offset /= n_cal
    gyro_offset /= n_cal
    acc_offset[2] -= 1.0   # quitar 1g en Z

    rospy.loginfo(f"Accel offset: {acc_offset}")
    rospy.loginfo(f"Gyro offset: {gyro_offset}")

    # ===== Inicializar filtro AHRS =====
    madgwick = Madgwick(beta=0.1)  # filtro suave
    q = np.array([1.0, 0.0, 0.0, 0.0])  # cuaternión inicial

    # Promediar varias lecturas iniciales para referencia
    init_samples = 100
    for _ in range(init_samples):
        acc = read_accel() - acc_offset
        gyr = read_gyro() - gyro_offset
        q = madgwick.updateIMU(q, gyr=gyr, acc=acc)
        time.sleep(0.01)
    q_init = q.copy()  # cuaternión de referencia (inicio)

    rospy.loginfo("Orientación inicial tomada.")

    # Buffers para suavizado de roll/pitch
    roll_buffer, pitch_buffer = [], []

    while not rospy.is_shutdown():
        acc = read_accel() - acc_offset
        gyr = read_gyro() - gyro_offset

        # Actualizar filtro AHRS
        q = madgwick.updateIMU(q, gyr=gyr, acc=acc)

        # --- Convertir cuaternión actual a roll/pitch relativo ---
        R_curr = R.from_quat([q[1], q[2], q[3], q[0]])  # xyzw -> scipy xyzw
        R_init = R.from_quat([q_init[1], q_init[2], q_init[3], q_init[0]])
        R_rel = R_init.inv() * R_curr
        roll_rel, pitch_rel, _ = R_rel.as_euler('xyz', degrees=False)  # ignoramos yaw

        # Limitar ángulos a ±45°
        roll_rel = max(min(roll_rel, math.radians(45)), math.radians(-45))
        pitch_rel = max(min(pitch_rel, math.radians(45)), math.radians(-45))

        # Suavizado (promedio móvil 5 muestras)
        roll_buffer.append(roll_rel)
        pitch_buffer.append(pitch_rel)
        if len(roll_buffer) > 10:
            roll_buffer.pop(0)
            pitch_buffer.pop(0)
        roll_avg = sum(roll_buffer)/len(roll_buffer)
        pitch_avg = sum(pitch_buffer)/len(pitch_buffer)

        # Construir mensaje IMU
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "imu_link"
        imu_msg.linear_acceleration.x = acc[0]*9.81
        imu_msg.linear_acceleration.y = acc[1]*9.81
        imu_msg.linear_acceleration.z = acc[2]*9.81
        imu_msg.angular_velocity.x = gyr[0]
        imu_msg.angular_velocity.y = gyr[1]
        imu_msg.angular_velocity.z = gyr[2]

        # Orientación solo roll/pitch, yaw=0
        quat_msg = tft.quaternion_from_euler(roll_avg, pitch_avg, 0.0)
        imu_msg.orientation = Quaternion(*quat_msg)

        pub.publish(imu_msg)
        rate.sleep()

if __name__ == "__main__":
    try:
        imu_publisher()
    except rospy.ROSInterruptException:
        pass
