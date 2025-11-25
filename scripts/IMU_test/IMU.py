#!/usr/bin/env python3
import rospy
import math
import time
import smbus2
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
import tf.transformations as tft

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
    return ax, ay, az

def read_gyro():
    gx = read_word(GYRO_XOUT_H) / 131.0
    gy = read_word(GYRO_XOUT_H+2) / 131.0
    gz = read_word(GYRO_XOUT_H+4) / 131.0
    return gx, gy, gz

# --- Nodo ROS ---
def imu_publisher():
    rospy.init_node("imu_node")
    pub = rospy.Publisher("/imu/data_raw", Imu, queue_size=10)
    rate = rospy.Rate(20)  # 20 Hz

    # ===== Calibración automática =====
    rospy.loginfo("Calibrando MPU9250, mantén el sensor quieto...")
    n_cal = 200
    acc_offset = [0.0, 0.0, 0.0]
    gyro_offset = [0.0, 0.0, 0.0]

    for i in range(n_cal):
        ax, ay, az = read_accel()
        gx, gy, gz = read_gyro()
        acc_offset[0] += ax
        acc_offset[1] += ay
        acc_offset[2] += az
        gyro_offset[0] += gx
        gyro_offset[1] += gy
        gyro_offset[2] += gz
        time.sleep(0.01)

    # Promedio
    acc_offset = [acc_offset[0]/n_cal,
                  acc_offset[1]/n_cal,
                  acc_offset[2]/n_cal - 1.0]  # Z debe restar 1g
    gyro_offset = [gyro_offset[0]/n_cal,
                   gyro_offset[1]/n_cal,
                   gyro_offset[2]/n_cal]

    rospy.loginfo(f"Accel offset: {acc_offset}")
    rospy.loginfo(f"Gyro offset: {gyro_offset}")

    # ===== Filtro complementario y buffers =====
    alpha = 0.995  # más suave ante golpes
    dt = 0.05      # 20 Hz
    roll, pitch = 0.0, 0.0
    roll_buffer = []
    pitch_buffer = []

    while not rospy.is_shutdown():
        ax, ay, az = read_accel()
        gx, gy, gz = read_gyro()

        # Restar offsets
        ax -= acc_offset[0]
        ay -= acc_offset[1]
        az -= acc_offset[2]
        gx -= gyro_offset[0]
        gy -= gyro_offset[1]
        gz -= gyro_offset[2]

        # Integración giroscopio (rad)
        roll  += math.radians(gx) * dt
        pitch += math.radians(gy) * dt

        # Roll y pitch desde acelerómetro
        roll_acc  = math.atan2(ay, az)
        pitch_acc = math.atan2(-ax, math.sqrt(ay**2 + az**2))

        # Filtro complementario
        roll  = alpha * roll  + (1 - alpha) * roll_acc
        pitch = alpha * pitch + (1 - alpha) * pitch_acc

        # Limitar ángulos máximos
        roll  = max(min(roll, math.radians(45)), math.radians(-45))
        pitch = max(min(pitch, math.radians(45)), math.radians(-45))

        # Promedio móvil de 5 muestras
        roll_buffer.append(roll)
        pitch_buffer.append(pitch)
        if len(roll_buffer) > 5:
            roll_buffer.pop(0)
            pitch_buffer.pop(0)
        roll_avg  = sum(roll_buffer)/len(roll_buffer)
        pitch_avg = sum(pitch_buffer)/len(pitch_buffer)

        # Construir mensaje IMU
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "imu_link"
        imu_msg.linear_acceleration.x = ax * 9.81
        imu_msg.linear_acceleration.y = ay * 9.81
        imu_msg.linear_acceleration.z = az * 9.81
        imu_msg.angular_velocity.x = math.radians(gx)
        imu_msg.angular_velocity.y = math.radians(gy)
        imu_msg.angular_velocity.z = math.radians(gz)
        quat = tft.quaternion_from_euler(roll_avg, pitch_avg, 0.0)
        imu_msg.orientation = Quaternion(*quat)

        pub.publish(imu_msg)
        rate.sleep()

if __name__ == "__main__":
    try:
        imu_publisher()
    except rospy.ROSInterruptException:
        pass
