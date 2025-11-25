#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Int32MultiArray
import serial
import time

# -------------------------------
# CONFIGURACIÓN SERIAL
# -------------------------------
arduino = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
time.sleep(2)  # Espera que Arduino se inicialice

# Pose inicial (posición "neutral" del robot)
pose_actual = [90] * 12  

# -------------------------------
# FUNCIONES AUXILIARES
# -------------------------------
def ajustar_servos(comandos):
    """ Invierte los servos que están montados 'al revés'. """
    invertidos = [1,3,5,7,9,11]  # Cambia según tu montaje real
    for i in invertidos:
        comandos[i] = 180 - comandos[i]
    return comandos

def enviar_a_arduino(comandos):
    """ Envía los ángulos actuales al Arduino por serial. """
    msg = '<'+','.join(str(c) for c in comandos) + '>\n'
    arduino.write(msg.encode())

def mover_gradual(destino, pasos=35, delay=0.05):
    """ Mueve los servos de pose_actual hacia destino en pequeños pasos. """
    global pose_actual
    for step in range(1, pasos + 1):
        intermedia = [
            int(pose_actual[i] + (destino[i] - pose_actual[i]) * step / pasos)
            for i in range(len(pose_actual))
        ]
        enviar_a_arduino(intermedia)
        rospy.loginfo(intermedia)
        time.sleep(delay)
    pose_actual = destino[:]  # Guardar nueva posición como actual

# -------------------------------
# CALLBACK DEL TOPIC ROS
# -------------------------------
def callback(data):
    global pose_actual
    comandos = list(data.data)
    comandos = ajustar_servos(comandos)
    mover_gradual(comandos, pasos=35, delay=0.05)  
    # puedes ajustar pasos y delay según suavidad/velocidad

# -------------------------------
# NODO PRINCIPAL
# -------------------------------
def listener():
    rospy.init_node('servo_commander', anonymous=True)
    rospy.Subscriber('/servo_positions', Int32MultiArray, callback)
    rospy.loginfo("Nodo servo_commander iniciado y esperando comandos...")
    rospy.spin()

# -------------------------------
# EJECUCIÓN
# -------------------------------
if __name__ == '__main__':
    listener()
