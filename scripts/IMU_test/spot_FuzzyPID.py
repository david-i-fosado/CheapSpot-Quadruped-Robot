#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
import math
import csv
import time
from std_msgs.msg import Float32MultiArray
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation as R
from simple_pid import PID

# --- Importaciones para Lógica Difusa ---
import skfuzzy as fuzz
from skfuzzy import control as ctrl
# --- IMPORTANTE: Para bloquear hilos (thread-safe) ---
from threading import Lock 


class BalanceController:
    def __init__(self):
        rospy.init_node("balance_controller", anonymous=True)

        # --- Publisher y Subscriber ---
        self.pub_corrections = rospy.Publisher("/balance/corrections", Float32MultiArray, queue_size=1)
        rospy.Subscriber("/imu/data_raw", Imu, self.imu_callback)

        # --- Referencia y Filtros ---
        self.ref_roll = 0.0
        self.ref_pitch = 0.0
        self.roll_filtered = 0.0
        self.pitch_filtered = 0.0

        # --- Valores iniciales del PID ---
        self.initial_kp = 1.8
        self.initial_ki = 0.01
        self.initial_kd = 0.01
         
        self.pid_roll = PID(self.initial_kp, self.initial_ki, self.initial_kd, setpoint=self.ref_roll)
        self.pid_pitch = PID(self.initial_kp, self.initial_ki, self.initial_kd, setpoint=self.ref_pitch)

        self.pid_roll.output_limits = (-0.25, 0.25)
        self.pid_pitch.output_limits = (-0.25, 0.25)

        # --- Variables de estado para el Sintonizador Difuso ---
        # Estas variables serán LEÍDAS por el callback_imu y ESCRITAS por el timer_fuzzy
        self.current_kp_roll = self.initial_kp
        self.current_kp_pitch = self.initial_kp
        self.kp_min = 0.5  
        self.kp_max = 2.0
        
        # Estas variables son USADAS SOLO por el timer_fuzzy
        self.prev_error_roll = 0.0
        self.prev_error_pitch = 0.0
        
        # --- NUEVO: Thread Lock ---
        # Previene que imu_callback lea una Kp mientras el timer la está escribiendo
        self.tuning_lock = Lock()

        # --- Sistema Difuso (Fuzzy) ---
        self.fuzzy_tuner_system = self._create_fuzzy_tuner_system()
        rospy.loginfo("Sistema difuso creado.")

        # --- CSV Logger ---
        self.csv_file = open("/home/david/catkin_ws/src/cheapspot/scripts/IMU_test/balance_log.csv", "w", newline="")
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(["time", "roll", "pitch", "roll_filtered", "pitch_filtered", "u_roll", "u_pitch", "Kp_roll", "Kp_pitch"])
        self.t0 = time.time()

        # --- NUEVO: Timer para el Sintonizador Difuso (10Hz) ---
        # Ejecutará la función _update_fuzzy_tunings cada 0.1 segundos (10Hz)
        # Esta es la misma velocidad que tenías, pero ahora no bloqueará el control principal
        self.fuzzy_update_rate = 0.1 # 10Hz
        rospy.Timer(rospy.Duration(self.fuzzy_update_rate), self.update_fuzzy_tunings)

        rospy.loginfo("Nodo de balance con Sintonizador Fuzzy-PID (Desacoplado) iniciado")
        rospy.loginfo(f"Bucle de control (IMU) a velocidad máxima.")
        rospy.loginfo(f"Bucle de sintonización (Fuzzy) a {1.0/self.fuzzy_update_rate} Hz.")


    # --- (Esta función no cambia) ---
    def _create_fuzzy_tuner_system(self):
        # ... (Todo el código de creación del sistema difuso sigue igual) ...
        self.error_universe = np.arange(-0.5, 0.51, 0.01)
        self.delta_error_universe = np.arange(-0.2, 0.21, 0.01)
        self.delta_kp_universe = np.arange(-0.1, 0.11, 0.01)
        error = ctrl.Antecedent(self.error_universe, 'error')
        delta_error = ctrl.Antecedent(self.delta_error_universe, 'delta_error')
        delta_Kp = ctrl.Consequent(self.delta_kp_universe, 'delta_Kp')
        nombres_mf = ['NG', 'NP', 'Z', 'PP', 'PG']
        error.automf(names=nombres_mf)
        delta_error.automf(names=nombres_mf)
        delta_Kp['NG'] = fuzz.trimf(delta_Kp.universe, [-0.1, -0.1, -0.05])
        delta_Kp['NP'] = fuzz.trimf(delta_Kp.universe, [-0.1, -0.05, 0])
        delta_Kp['Z']  = fuzz.trimf(delta_Kp.universe, [-0.05, 0, 0.05])
        delta_Kp['PP'] = fuzz.trimf(delta_Kp.universe, [0, 0.05, 0.1])
        delta_Kp['PG'] = fuzz.trimf(delta_Kp.universe, [0.05, 0.1, 0.1])
        rules_matrix = [
            ['PG', 'PG', 'PP', 'PP', 'Z' ],  # e = NG
            ['PG', 'PP', 'PP', 'Z',  'NP'],  # e = NP
            ['PP', 'PP', 'Z',  'NP', 'NP'],  # e = Z
            ['PP', 'Z',  'NP', 'NP', 'NG'],  # e = PP
            ['Z',  'NP', 'NP', 'NG', 'NG']   # e = PG
        ]
        rules = []
        for i_e, e_mf in enumerate(nombres_mf):
            for i_de, de_mf in enumerate(nombres_mf):
                output_mf = rules_matrix[i_e][i_de]
                rule = ctrl.Rule(error[e_mf] & delta_error[de_mf], delta_Kp[output_mf])
                rules.append(rule)
        kp_ctrl_sys = ctrl.ControlSystem(rules)
        kp_simulation = ctrl.ControlSystemSimulation(kp_ctrl_sys)
        return kp_simulation

    # --- (Esta función no cambia) ---
    def _compute_fuzzy_delta_kp(self, current_error, current_delta_error):
        # ... (Todo el código de cómputo difuso sigue igual) ...
        e_in = max(self.error_universe.min(), min(current_error, self.error_universe.max()))
        de_in = max(self.delta_error_universe.min(), min(current_delta_error, self.delta_error_universe.max()))
        self.fuzzy_tuner_system.input['error'] = e_in
        self.fuzzy_tuner_system.input['delta_error'] = de_in
        try:
            self.fuzzy_tuner_system.compute()
            delta_kp = self.fuzzy_tuner_system.output['delta_Kp']
        except Exception as e:
            rospy.logwarn_throttle(10.0, f"Error en cómputo difuso: {e}. dKp = 0.") # Evita spam de logs
            delta_kp = 0.0
        return delta_kp


    # --- NUEVA FUNCIÓN (Bucle Lento a 10Hz) ---
    def update_fuzzy_tunings(self, event):
        """
        Este es el BUCLE LENTO (ej. 10Hz).
        Calcula las nuevas Kp y las guarda de forma segura.
        """
        
        # 1. Calcular Error (e)
        # Nota: Usamos self.roll_filtered, que es actualizado por el imu_callback
        error_roll = self.pid_roll.setpoint - self.roll_filtered
        error_pitch = self.pid_pitch.setpoint - self.pitch_filtered

        # 2. Calcular Cambio de Error (de = e_k - e_k-1)
        delta_error_roll = error_roll - self.prev_error_roll
        delta_error_pitch = error_pitch - self.prev_error_pitch

        # 3. Obtener delta_Kp del sistema difuso (El cálculo pesado)
        delta_kp_roll = self._compute_fuzzy_delta_kp(error_roll, delta_error_roll)
        delta_kp_pitch = self._compute_fuzzy_delta_kp(error_pitch, delta_error_pitch)

        # 4. Calcular nueva Kp (Kp_k = Kp_k-1 + dKp)
        # Usamos las variables guardadas, no las del PID
        new_kp_roll = self.current_kp_roll + delta_kp_roll
        new_kp_pitch = self.current_kp_pitch + delta_kp_pitch

        # 5. Saturar/Limitar la Kp
        new_kp_roll = max(self.kp_min, min(new_kp_roll, self.kp_max))
        new_kp_pitch = max(self.kp_min, min(new_kp_pitch, self.kp_max))
         
        # 6. Guardar estado actual para el próximo ciclo (LENTO)
        self.prev_error_roll = error_roll
        self.prev_error_pitch = error_pitch

        # 7. --- GUARDADO SEGURO (Thread-Safe) ---
        # Adquirimos el bloqueo para escribir las nuevas Kp
        with self.tuning_lock:
            self.current_kp_roll = new_kp_roll
            self.current_kp_pitch = new_kp_pitch
        
        # 8. Log (opcional, ahora no bloquea el control)
        rospy.loginfo(f"Fuzzy Kp_r:{new_kp_roll:.3f} Kp_p:{new_kp_pitch:.3f}")


    # --- FUNCIÓN MODIFICADA (Bucle Rápido, ~25Hz+) ---
    def imu_callback(self, msg):
        """
        Este es el BUCLE RÁPIDO (velocidad de la IMU).
        NO hace cálculos difusos. Solo lee Kp, calcula PID y publica.
        """
        
        # 1. Extracción de RPY y filtro pasa-bajas (Rápido)
        q = [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w]
        try:
            rpy = R.from_quat(q).as_euler('xyz', degrees=False)
        except Exception as e:
            return 
            
        roll, pitch, yaw = rpy
        alpha = 0.2
        self.roll_filtered = alpha * roll + (1 - alpha) * self.roll_filtered
        self.pitch_filtered = alpha * pitch + (1 - alpha) * self.pitch_filtered

        # --- LÓGICA DE CONTROL RÁPIDA ---
        
        # 2. --- LECTURA SEGURA (Thread-Safe) ---
        # Leemos las Kp más recientes que calculó el bucle lento
        with self.tuning_lock:
            kp_r = self.current_kp_roll
            kp_p = self.current_kp_pitch
        
        # 3. Actualizar los sintonizadores del PID (Rápido)
        self.pid_roll.tunings = (kp_r, self.pid_roll.Ki, self.pid_roll.Kd)
        self.pid_pitch.tunings = (kp_p, self.pid_pitch.Ki, self.pid_pitch.Kd)

        # 4. Calcular la salida de control (Rápido)
        u_roll = self.pid_roll(self.roll_filtered)
        u_pitch = self.pid_pitch(self.pitch_filtered)
         
        # 5. Log en CSV (¡SIN FLUSH!)
        t = time.time() - self.t0
        self.csv_writer.writerow([t, roll, pitch, self.roll_filtered, self.pitch_filtered, u_roll, u_pitch, kp_r, kp_p])
        # self.csv_file.flush() # <-- ¡ELIMINADO! ¡NO MÁS BLOQUEO DE DISCO!

        # 6. Publicar correcciones (Rápido)
        roll_cmd = u_roll if abs(u_roll) > 0.02 else 0.0
        pitch_cmd = u_pitch if abs(u_pitch) > 0.02 else 0.0
        corrections = Float32MultiArray(data=[roll_cmd, pitch_cmd])
        self.pub_corrections.publish(corrections)

        # 7. Logs de ROS (¡ELIMINADOS!)
        # Comenta esto para máxima velocidad. Descomenta SOLO si necesitas depurar.
        rospy.loginfo(f"IMU r:{roll:.4f} p:{pitch:.4f} | Control u_r:{u_roll:.4f} u_p:{u_pitch:.4f}")

    def spin(self):
       try:
           rospy.spin()
       finally:
           # Cuando el nodo se cierra, cerramos el archivo de forma segura
           rospy.loginfo("Cerrando nodo y guardando CSV...")
           self.csv_file.close()
           rospy.loginfo("CSV guardado.")

if __name__ == "__main__":
    node = BalanceController()
    node.spin()
