#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from std_msgs.msg import Float32MultiArray
import skfuzzy as fuzz
from skfuzzy import control as ctrl

class FuzzyPIDHybridNode:
    def __init__(self):
        rospy.init_node("fuzzy_pid_hybrid_node", anonymous=True)

        self.pub_tunings = rospy.Publisher("/balance/fuzzy_tunings", Float32MultiArray, queue_size=1)
        rospy.Subscriber("/balance/state", Float32MultiArray, self.state_callback)

        # --- ¡AÑADIR ESTA LÍNEA AQUÍ! ---
        self.state_data_received = False

        self.current_roll = 0.0
        self.current_pitch = 0.0
        self.prev_error_roll = 0.0
        self.prev_error_pitch = 0.0

        # --- Rangos de Ganancias ---
        self.kp_range = [1.0, 3.0]  # Kp [Min, Max]
        self.ki_range = [0.1, 0.4]  # Ki [Min, Max]
        self.kd_range = [0.01, 0.1] # Kd [Min, Max]
        
        # --- Valor Base (Reposo) para Kp ---
        self.kp_base = 1.8 # ¡Este es el valor "quieto" de Kp!
        
        # --- Valores iniciales (para Ki y Kd) ---
        self.current_ki_roll = self.ki_range[0]
        self.current_ki_pitch = self.ki_range[0]
        self.current_kd_roll = self.kd_range[0]
        self.current_kd_pitch = self.kd_range[0]

        # --- Rangos de "Volumen" de Sintonización ---
        self.delta_ki_max = 0.01
        self.delta_kd_max = 0.01

        # --- Universos de Entrada ---
        self.abs_e_universe_max = 0.7
        self.abs_de_universe_max = 0.1
        
        # --- ¡VALORES DE ZONA MUERTA (Sintoniza esto!) ---
        self.limite_ruido_e = 0.02   # (ej. ~1.15 grados)
        self.limite_ruido_de = 0.001 # (Cambio de error)
        
        # --- Crear los 3 Sistemas Difusos ---
        self.scheduler_kp = self._create_scheduler_kp()
        self.tuner_ki = self._create_tuner_ki()
        self.tuner_kd = self._create_tuner_kd()
        
        self.fuzzy_update_rate = 0.1
        rospy.Timer(rospy.Duration(self.fuzzy_update_rate), self.update_fuzzy_tunings)
        
        rospy.loginfo("Nodo Sintonizador Híbrido (con Zonas Muertas) iniciado.")
        rospy.loginfo(f"Zona muerta de |e|: {self.limite_ruido_e} rad")
        rospy.loginfo(f"Zona muerta de |de|: {self.limite_ruido_de} rad/ciclo")


    # --- 1. PLANIFICADOR DE Kp (con Zona Muerta) ---
    def _create_scheduler_kp(self):
        
        # Universo de Salida Kp ahora tiene 3 niveles de picos
        Kp = ctrl.Consequent(np.arange(self.kp_range[0], self.kp_range[1] + 0.1, 0.1), 'Kp')
        Kp['Pequeña'] = fuzz.trimf(Kp.universe, [self.kp_range[0], self.kp_range[0], self.kp_base])
        Kp['Base']    = fuzz.trimf(Kp.universe, [self.kp_range[0], self.kp_base, self.kp_range[1]])
        Kp['Grande']  = fuzz.trimf(Kp.universe, [self.kp_base, self.kp_range[1], self.kp_range[1]])
        
        # Universo de Entrada |e| tiene 3 niveles
        abs_error = ctrl.Antecedent(np.arange(0, self.abs_e_universe_max + 0.1, 0.01), 'abs_error')
        limite_medio_e = 0.3 # Límite entre "Medio" y "Grande"
        
        abs_error['Ruido']  = fuzz.trimf(abs_error.universe, [0, 0, self.limite_ruido_e])
        abs_error['Medio']  = fuzz.trimf(abs_error.universe, [self.limite_ruido_e, limite_medio_e, self.abs_e_universe_max])
        abs_error['Grande'] = fuzz.trimf(abs_error.universe, [limite_medio_e, self.abs_e_universe_max, self.abs_e_universe_max])

        # --- NUEVAS REGLAS para Kp ---
        # 1. Si el error es "Ruido", Kp = Base (¡Tu Zona Muerta!)
        rule1 = ctrl.Rule(abs_error['Ruido'], Kp['Base'])
        
        # 2. Si el error es "Medio", Kp = Grande (Rígido cerca del setpoint)
        rule2 = ctrl.Rule(abs_error['Medio'], Kp['Grande'])
        
        # 3. Si el error es "Grande", Kp = Pequeña (Suave lejos del setpoint)
        rule3 = ctrl.Rule(abs_error['Grande'], Kp['Pequeña'])
        
        return ctrl.ControlSystemSimulation(ctrl.ControlSystem([rule1, rule2, rule3]))

    # --- 2. SINTONIZADOR DE Ki (con Zona Muerta) ---
    def _create_tuner_ki(self):
        abs_error = ctrl.Antecedent(np.arange(0, self.abs_e_universe_max + 0.1, 0.01), 'abs_error')
        delta_Ki = ctrl.Consequent(np.arange(-self.delta_ki_max, self.delta_ki_max + 0.01, 0.01), 'delta_Ki')
        
        # MFs de Entrada |e|
        limite_medio_e = 0.05
        abs_error['Ruido']  = fuzz.trimf(abs_error.universe, [0, 0, 0.03])
        abs_error['Medio']  = fuzz.trimf(abs_error.universe, [0.03, 0.04, 0.05])
        abs_error['Grande'] = fuzz.trimf(abs_error.universe, [0.05, self.abs_e_universe_max, self.abs_e_universe_max])

        # MFs de Salida dKi
        delta_Ki['Negativo'] = fuzz.trimf(delta_Ki.universe, [-self.delta_ki_max, -self.delta_ki_max, 0])
        delta_Ki['Cero']     = fuzz.trimf(delta_Ki.universe, [-0.01, 0, 0.01])
        delta_Ki['Positivo'] = fuzz.trimf(delta_Ki.universe, [0, self.delta_ki_max, self.delta_ki_max])
        
        # --- REGLAS CORREGIDAS para Ki ---
        # 1. Si el error es "Ruido", dKi = Cero (¡Tu Zona Muerta!)
        rule1 = ctrl.Rule(abs_error['Ruido'], delta_Ki['Cero'])
        
        # 2. Si el error es "Medio", dKi = Positivo (Sube Ki para eliminar error)
        rule2 = ctrl.Rule(abs_error['Medio'], delta_Ki['Positivo'])
        
        # 3. Si el error es "Grande", dKi = Negativo (Baja Ki para no oscilar)
        rule3 = ctrl.Rule(abs_error['Grande'], delta_Ki['Negativo'])
        
        return ctrl.ControlSystemSimulation(ctrl.ControlSystem([rule1, rule2, rule3]))

    # --- 3. SINTONIZADOR DE Kd (con Zona Muerta) ---
    def _create_tuner_kd(self):
        abs_delta_error = ctrl.Antecedent(np.arange(0, self.abs_de_universe_max + 0.1, 0.01), 'abs_delta_error')
        delta_Kd = ctrl.Consequent(np.arange(-self.delta_kd_max, self.delta_kd_max + 0.01, 0.01), 'delta_Kd')
        
        # MFs de Entrada |de|
        limite_medio_de = 0.004
        abs_delta_error['Ruido'] = fuzz.trimf(abs_delta_error.universe, [0, 0, 0.0001])
        abs_delta_error['Medio'] = fuzz.trimf(abs_delta_error.universe, [0.0001, 0.005, 0.01])
        abs_delta_error['Grande'] = fuzz.trimf(abs_delta_error.universe, [0.01, self.abs_de_universe_max, self.abs_de_universe_max])
        
        # MFs de Salida dKd
        delta_Kd['Negativo'] = fuzz.trimf(delta_Kd.universe, [-self.delta_kd_max, -self.delta_kd_max, 0])
        delta_Kd['Cero']     = fuzz.trimf(delta_Kd.universe, [-0.01, 0, 0.01])
        delta_Kd['Positivo'] = fuzz.trimf(delta_Kd.universe, [0, self.delta_kd_max, self.delta_kd_max])
        
        # --- REGLAS CORREGIDAS para Kd ---
        # 1. Si el cambio es "Ruido", dKd = Cero (¡Tu Zona Muerta!)
        rule1 = ctrl.Rule(abs_delta_error['Ruido'], delta_Kd['Cero'])
        
        # 2. Si el cambio es "Medio", dKd = Negativo (Tu lógica original "bajar Kd")
        rule2 = ctrl.Rule(abs_delta_error['Medio'], delta_Kd['Negativo'])
        
        # 3. Si el cambio es "Grande", dKd = Positivo (Tu lógica original "subir Kd")
        rule3 = ctrl.Rule(abs_delta_error['Grande'], delta_Kd['Positivo'])
        
        return ctrl.ControlSystemSimulation(ctrl.ControlSystem([rule1, rule2, rule3]))
    
    # --- (El resto del código (compute, callback, update) es idéntico al anterior) ---
    
    def _compute_fuzzy_kp(self, e_in):
        self.scheduler_kp.input['abs_error'] = max(0, min(e_in, self.abs_e_universe_max))
        try:
            self.scheduler_kp.compute()
            return self.scheduler_kp.output['Kp']
        except: 
            # Si no hay reglas activas (poco probable), vuelve a Base
            return self.kp_base 

    def _compute_delta_ki(self, e_in):
        self.tuner_ki.input['abs_error'] = max(0, min(e_in, self.abs_e_universe_max))
        try:
            self.tuner_ki.compute()
            val = self.tuner_ki.output['delta_Ki']
            return val if val is not None else 0.0
        except: return 0.0

    def _compute_delta_kd(self, de_in):
        self.tuner_kd.input['abs_delta_error'] = max(0, min(de_in, self.abs_de_universe_max))
        try:
            self.tuner_kd.compute()
            val = self.tuner_kd.output['delta_Kd']
            return val if val is not None else 0.0
        except: return 0.0
    
    
    def state_callback(self, msg):
        self.current_roll = msg.data[0]
        self.current_pitch = msg.data[1]
        if not self.state_data_received:
            self.state_data_received = True
            rospy.loginfo("Primer dato de estado recibido. Iniciando sintonización.")

    def update_fuzzy_tunings(self, event):
        if not self.state_data_received:
            return 

        # Calcular errores
        error_roll = 0.0 - self.current_roll
        delta_error_roll = error_roll - self.prev_error_roll
        abs_e_roll = abs(error_roll)
        abs_de_roll = abs(delta_error_roll)
        
        error_pitch = 0.0 - self.current_pitch
        delta_error_pitch = error_pitch - self.prev_error_pitch
        abs_e_pitch = abs(error_pitch)
        abs_de_pitch = abs(delta_error_pitch)

        # Calcular 3 ganancias para ROLL
        new_kp_roll = self._compute_fuzzy_kp(abs_e_roll) 
        delta_ki_roll = self._compute_delta_ki(abs_e_roll)
        new_ki_roll = self.current_ki_roll + delta_ki_roll
        delta_kd_roll = self._compute_delta_kd(abs_de_roll)
        new_kd_roll = self.current_kd_roll + delta_kd_roll

        # Calcular 3 ganancias para PITCH
        new_kp_pitch = self._compute_fuzzy_kp(abs_e_pitch)
        delta_ki_pitch = self._compute_delta_ki(abs_e_pitch)
        new_ki_pitch = self.current_ki_pitch + delta_ki_pitch
        delta_kd_pitch = self._compute_delta_kd(abs_de_pitch)
        new_kd_pitch = self.current_kd_pitch + delta_kd_pitch

        # Saturar Ki y Kd
        new_ki_roll = max(self.ki_range[0], min(new_ki_roll, self.ki_range[1]))
        new_ki_pitch = max(self.ki_range[0], min(new_ki_pitch, self.ki_range[1]))
        new_kd_roll = max(self.kd_range[0], min(new_kd_roll, self.kd_range[1]))
        new_kd_pitch = max(self.kd_range[0], min(new_kd_pitch, self.kd_range[1]))

        # Guardar estado
        self.prev_error_roll = error_roll
        self.prev_error_pitch = error_pitch
        self.current_ki_roll = new_ki_roll
        self.current_ki_pitch = new_ki_pitch
        self.current_kd_roll = new_kd_roll
        self.current_kd_pitch = new_kd_pitch

        # Publicar
        tunings_data = Float32MultiArray(data=[
            new_kp_roll, new_kp_pitch, 
            new_ki_roll, new_ki_pitch, 
            new_kd_roll, new_kd_pitch
        ])
        self.pub_tunings.publish(tunings_data)
        
        rospy.loginfo_throttle(1.0, f"Kp_r={new_kp_roll:.2f}, Ki_r={new_ki_roll:.2f}, Kd_r={new_kd_roll:.3f}")
        rospy.loginfo_throttle(1.0, f"Kp_p={new_kp_pitch:.2f}, Ki_p={new_ki_pitch:.2f}, Kd_p={new_kd_pitch:.3f}")
        rospy.loginfo_throttle(1.0, f"Delta_error| Roll:{abs_de_roll:.6f}, Pitch{abs_de_pitch:.6f}")

    def run(self):
        rospy.spin()

if __name__ == "__main__":
    node = FuzzyPIDHybridNode()
    node.run()
