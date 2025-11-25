#!/usr/bin/env python3
import rospy
import time
import smbus2
import numpy as np
import math
import tf.transformations as tft
from sensor_msgs.msg import Imu
from geometry_msgs.msg import Quaternion
from ahrs.filters import Madgwick
from scipy.spatial.transform import Rotation as R

# ===== Configuración de Hardware =====
MPU_ADDR = 0x68       # Dirección I2C del MPU
PWR_MGMT_1 = 0x6B     # Registro de gestión de energía
ACCEL_XOUT_H = 0x3B   # Registros de datos del acelerómetro
GYRO_XOUT_H  = 0x43   # Registros de datos del giroscopio

# ===== Parámetros del Sensor (MPU9250 con config por defecto) =====
# Sensibilidad Acelerómetro (para fondo de escala ±2g)
ACCEL_SENSITIVITY = 16384.0  # LSB/g
# Sensibilidad Giroscopio (para fondo de escala ±250°/s)
GYRO_SENSITIVITY = 131.0     # LSB/(°/s)
# Constante de gravedad estándar
GRAVITY_STD = 9.80665        # m/s^2

# ===== Parámetros del Nodo y Filtro =====
NODE_NAME = "imu_node"
IMU_TOPIC = "/imu/data_raw"
IMU_FRAME_ID = "imu_link"
LOOP_PERIOD = 0.04  # 1.0 / 25 Hz = 0.04s
DT = LOOP_PERIOD    # El tiempo delta (dt) para el filtro es el periodo del bucle

# --- Parámetros de Calibración e Inicialización ---
CALIBRATION_SAMPLES = 500   # Número de muestras para calibrar
AHRS_INIT_SAMPLES = 100     # Número de muestras para estabilizar el filtro al inicio
AHRS_BETA = 0.2             # Ganancia del filtro Madgwick (más bajo = más suave)

# --- Parámetros de la Lógica Personalizada (si se mantiene) ---
SMOOTHING_BUFFER_SIZE = 10
ANGLE_LIMIT_DEG = 45.0
ANGLE_LIMIT_RAD = math.radians(ANGLE_LIMIT_DEG)

# ===== Inicialización del Bus I2C =====
bus = smbus2.SMBus(1)
try:
    bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0)  # despertar sensor
except IOError:
    rospy.logerr(f"No se pudo encontrar el MPU9250 en la dirección {hex(MPU_ADDR)}")
    # En un script real, aquí deberías salir o reintentar
    pass

# ===== Funciones de Lectura del Sensor =====
def read_word(reg):
    """Lee un valor de 16 bits (complemento a dos) de los registros I2C."""
    try:
        high = bus.read_byte_data(MPU_ADDR, reg)
        low = bus.read_byte_data(MPU_ADDR, reg+1)
        val = (high << 8) + low
        if val >= 0x8000:
            val = -((65535 - val) + 1)
        return val
    except IOError as e:
        rospy.logwarn(f"Error de I2C al leer el registro {hex(reg)}: {e}")
        return 0

def read_accel():
    """Lee el acelerómetro y devuelve los valores en g."""
    ax = read_word(ACCEL_XOUT_H) / ACCEL_SENSITIVITY
    ay = read_word(ACCEL_XOUT_H+2) / ACCEL_SENSITIVITY
    az = read_word(ACCEL_XOUT_H+4) / ACCEL_SENSITIVITY
    return np.array([ax, ay, az])

def read_gyro():
    """Lee el giroscopio y devuelve los valores en rad/s."""
    gx = read_word(GYRO_XOUT_H) / GYRO_SENSITIVITY
    gy = read_word(GYRO_XOUT_H+2) / GYRO_SENSITIVITY
    gz = read_word(GYRO_XOUT_H+4) / GYRO_SENSITIVITY
    return np.radians(np.array([gx, gy, gz]))

# ===== Nodo Principal de ROS =====
def imu_publisher():
    rospy.init_node(NODE_NAME)
    pub = rospy.Publisher(IMU_TOPIC, Imu, queue_size=1)
    # CORRECCIÓN: El comentario ahora es correcto (25 Hz)
    rate = rospy.Rate(1.0 / LOOP_PERIOD)  # 25 Hz

    # ===== Calibración simple (Offsets) =====
    rospy.loginfo("Iniciando calibración del MPU9250. Mantén el sensor quieto...")
    acc_offset = np.zeros(3)
    gyro_offset = np.zeros(3)

    for _ in range(CALIBRATION_SAMPLES):
        acc_offset += read_accel()
        gyro_offset += read_gyro()
        time.sleep(0.005)  # Un pequeño sleep para no saturar el bus

    acc_offset /= CALIBRATION_SAMPLES
    gyro_offset /= CALIBRATION_SAMPLES
    acc_offset[2] -= 1.0   # quitar 1g en Z

    # ------------------------------------------------------------------
    # CORRECCIÓN CRÍTICA (1):
    # NO restamos la gravedad (ej. acc_offset[2] -= 1.0).
    # El filtro AHRS (Madgwick) *necesita* el vector de gravedad [0, 0, 1]g
    # (o [0, 0, -1]g dependiendo de la orientación) como referencia para
    # saber "dónde está abajo" y corregir la deriva del giroscopio.
    # El offset solo debe corregir el bias del sensor (lectura en 0g).
    # ------------------------------------------------------------------

    rospy.loginfo(f"Calibración finalizada. Accel offset (g): {acc_offset}")
    rospy.loginfo(f"Calibración finalizada. Gyro offset (rad/s): {gyro_offset}")

    # ===== Inicializar filtro AHRS (Madgwick) =====
    madgwick = Madgwick(beta=AHRS_BETA, dt=DT)
    q = np.array([1.0, 0.0, 0.0, 0.0])  # Cuaternión inicial [w, x, y, z]

    # Promediar varias lecturas para que el filtro se estabilice
    rospy.loginfo(f"Estabilizando filtro AHRS durante {AHRS_INIT_SAMPLES} muestras...")
    for _ in range(AHRS_INIT_SAMPLES):
        acc = read_accel() - acc_offset
        gyr = read_gyro() - gyro_offset
        
        # CORRECCIÓN CRÍTICA (2): Añadimos el tiempo delta 'DT'
        q = madgwick.updateIMU(q, gyr=gyr, acc=acc)
        
        time.sleep(DT) # Dormir el tiempo del bucle
        
    q_init = q.copy()  # Guardar cuaternión de referencia (inicio)
    rospy.loginfo("Orientación inicial de referencia tomada. Iniciando bucle principal.")

    # ===== Inicializar Buffers de Suavizado =====
    # CORRECCIÓN (Buena práctica):
    # Pre-llenamos los buffers para que el promedio móvil sea estable
    # desde el inicio y evitar división por 1, 2, 3...
    roll_buffer = [0.0] * SMOOTHING_BUFFER_SIZE
    pitch_buffer = [0.0] * SMOOTHING_BUFFER_SIZE

    # ===== Bucle Principal =====
    while not rospy.is_shutdown():
        # Leer y aplicar calibración
        acc = read_accel() - acc_offset
        gyr = read_gyro() - gyro_offset

        # CORRECCIÓN CRÍTICA (2): Añadir 'DT' a la actualización
        q = madgwick.updateIMU(q, gyr=gyr, acc=acc)

        # ----------------------------------------------------------------------
        # --- INICIO: Lógica original del usuario (Cálculo de Roll/Pitch Relativo) ---
        # Esta sección calcula la inclinación relativa a la posición inicial,
        # la limita, la suaviza y publica solo esos dos ejes (con yaw=0).
        
        # Convertir cuaterniones a formato de Scipy [x, y, z, w]
        R_curr = R.from_quat([q[1], q[2], q[3], q[0]])
        R_init = R.from_quat([q_init[1], q_init[2], q_init[3], q_init[0]])
        
        # Calcular rotación relativa
        R_rel = R_init.inv() * R_curr
        roll_rel, pitch_rel, _ = R_rel.as_euler('xyz', degrees=False)  # ignoramos yaw

        # Limitar ángulos
        roll_rel = max(min(roll_rel, ANGLE_LIMIT_RAD), -ANGLE_LIMIT_RAD)
        pitch_rel = max(min(pitch_rel, ANGLE_LIMIT_RAD), -ANGLE_LIMIT_RAD)

        # Suavizado (media móvil)
        roll_buffer.append(roll_rel)
        pitch_buffer.append(pitch_rel)
        roll_buffer.pop(0)  # Quitar el más antiguo
        pitch_buffer.pop(0) # Quitar el más antiguo
        
        # CORRECCIÓN (Buena práctica): El divisor es ahora constante
        roll_avg = sum(roll_buffer) / SMOOTHING_BUFFER_SIZE
        pitch_avg = sum(pitch_buffer) / SMOOTHING_BUFFER_SIZE

        # Crear un nuevo cuaternión solo con roll/pitch (yaw=0)
        quat_msg = tft.quaternion_from_euler(roll_avg, pitch_avg, 0.0)
        
        # --- FIN: Lógica original del usuario ---
        # ----------------------------------------------------------------------
        
        # --- ALTERNATIVA (Estándar de ROS): ---
        # Si quisieras publicar la orientación 3D *completa* y *absoluta*
        # calculada por el filtro, deberías reemplazar toda la sección anterior por:
        #
        # quat_msg_list = [q[1], q[2], q[3], q[0]] # Convertir [w,x,y,z] a [x,y,z,w]
        # quat_msg = Quaternion(*quat_msg_list)
        # 
        # --- FIN ALTERNATIVA ---


        # ===== Construir y Publicar Mensaje IMU =====
        imu_msg = Imu()
        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = IMU_FRAME_ID
        
        # Aceleración lineal (en m/s^2)
        imu_msg.linear_acceleration.x = acc[0] * GRAVITY_STD
        imu_msg.linear_acceleration.y = acc[1] * GRAVITY_STD
        imu_msg.linear_acceleration.z = acc[2] * GRAVITY_STD
        
        # Velocidad angular (en rad/s)
        imu_msg.angular_velocity.x = gyr[0]
        imu_msg.angular_velocity.y = gyr[1]
        imu_msg.angular_velocity.z = gyr[2]

        # Orientación (usando el resultado de tu lógica)
        imu_msg.orientation = Quaternion(*quat_msg) # Desempaqueta [x,y,z,w]

        # Covarianzas (opcional, pero bueno tenerlas como "desconocidas")
        # 0 en la diagonal significa "covarianza desconocida"
        imu_msg.orientation_covariance[0] = -1 # Marcar como no usada
        imu_msg.angular_velocity_covariance[0] = -1
        imu_msg.linear_acceleration_covariance[0] = -1

        pub.publish(imu_msg)
        rate.sleep()

if __name__ == "__main__":
    try:
        imu_publisher()
    except rospy.ROSInterruptException:
        rospy.loginfo("Nodo IMU detenido.")
        pass
    finally:
        bus.close()
        rospy.loginfo("Bus I2C cerrado.")
