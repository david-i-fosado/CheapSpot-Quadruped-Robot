#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy # Crear nodos, subscribirse a topics y publicar
from std_msgs.msg import Int32MultiArray # Mensaje ROS que contiene una lista de enteros
import serial # comunicacion serial
import time # esperas

# ---- Configuracion Serial ---- #
arduino = serial.Serial("/dev/ttyUSB0",9600, timeout=1)
time.sleep(2)

# ---- Funciones axiliares ---- #

def ajustar_servos(comandos):
    # Invierte la posicion de los servos pasan de 0 a 180 y de 180 a 0
    invertidos = [1,3,5,7,9,11]
    for i in invertidos:
        comandos[i] = 180 - comandos[i]
    return comandos

def enviar_arduino(comandos):
    # Convierte la lista de posiciones en un string para enviar a Arduino
    msg = '<' + ','.join(str(c) for c in comandos) + '>'
    arduino.write(msg.encode())

# ---- Callbacck del Topic ROS ---- #
def callback(data):
    # Se ejectuta cada vez que llega un mensaje al topic /servo_positions
    comandos = list(data.data) # Se extraen la lista de enteros
    comandos = ajustar_servos(comandos) # Se ajustan los servos
    enviar_arduino(comandos) # Se envian a Arduino
    while True:
        if arduino.in_waiting > 0:
            response = arduino.readline().decode().strip()
            if reponse == "OK":
                break

# ---- Nodo principal ---- #
def listener():
    # Inicializar el nodo ROS, se suscribe al topic y espera mensajes
    rospy.init_node('servo_commander', anonymous=True) # Se inicializa
    rospy.Subscriber('/servo_positions', Int32MultiArray, callback) # se Subscribe
    rospy.loginfo("Nodo servo_comamander iniciado y esperando comandos...")
    rospy.spin() # Mantiene el nodo en ejecucion

# Ejecucion

if __name__ == '__main__':
    listener()
