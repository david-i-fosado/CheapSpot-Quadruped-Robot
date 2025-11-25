#!/usr/bin/env python

import numpy as np


class LegIK():
    def __init__(self, legtype="RIGHT", 
                 shoulder_length=0.04, 
                 elbow_length=0.115, 
                 wrist_length=0.125, 
                 hip_lim=[-0.548, 0.548], 
                 shoulder_lim=[-2.17, 0.97], 
                 leg_lim=[-0.1, 2.59], 
                 angle_offsets=[0.0,0.0,-np.pi/2]):
        self.legtype = legtype
        self.shoulder_length = shoulder_length
        self.elbow_length = elbow_length
        self.wrist_length = wrist_length
        self.hip_lim = hip_lim
        self.shoulder_lim = shoulder_lim
        self.leg_lim = leg_lim
        self.angle_offsets = angle_offsets

    def get_domain(self, x, y, z):
        """
        Calculates the leg's Domain and caps it in case of a breach

        :param x,y,z: hip-to-foot distances in each dimension
        :return: Leg Domain D
        """
        D = (y**2 + (-z)**2 - self.shoulder_length**2 +
             (-x)**2 - self.elbow_length**2 - self.wrist_length**2) / (
                 2 * self.wrist_length * self.elbow_length)
        if D > 1 or D < -1:
            # DOMAIN BREACHED
            # print("---------DOMAIN BREACH---------")
            D = np.clip(D, -1.0, 1.0)
            return D
        else:
            return D

    def solve(self, xyz_coord):
        """
        Generic Leg Inverse Kinematics Solver

        :param xyz_coord: hip-to-foot distances in each dimension
        :return: Joint Angles required for desired position
        """
        x = xyz_coord[0]
        y = xyz_coord[1]
        z = xyz_coord[2]
        D = self.get_domain(x, y, z)
        if self.legtype == "RIGHT":
            return self.RightIK(x, y, z, D)
        else:
            return self.LeftIK(x, y, z, D)

    def RightIK(self, x, y, z, D):
        """
        Right Leg Inverse Kinematics Solver

        :param x,y,z: hip-to-foot distances in each dimension
        :param D: leg domain
        :return: Joint Angles required for desired position
        """
        wrist_angle = np.arctan2(-np.sqrt(1 - D**2), D)
        sqrt_component = y**2 + (-z)**2 - self.shoulder_length**2
        if sqrt_component < 0.0:
            # print("NEGATIVE SQRT")
            sqrt_component = 0.0
        shoulder_angle = -np.arctan2(z, y) - np.arctan2(
            np.sqrt(sqrt_component), -self.shoulder_length)
        elbow_angle = np.arctan2(-x, np.sqrt(sqrt_component)) - np.arctan2(
            self.wrist_length * np.sin(wrist_angle),
            self.elbow_length + self.wrist_length * np.cos(wrist_angle))
        joint_angles = np.array([-shoulder_angle, elbow_angle, wrist_angle])
        return -joint_angles + self.angle_offsets

    def LeftIK(self, x, y, z, D):
        """
        Left Leg Inverse Kinematics Solver

        :param x,y,z: hip-to-foot distances in each dimension
        :param D: leg domain
        :return: Joint Angles required for desired position
        """
        # y = -y
        wrist_angle = np.arctan2(-np.sqrt(1 - D**2), D)
        sqrt_component = y**2 + (-z)**2 - self.shoulder_length**2
        if sqrt_component < 0.0:
            print("NEGATIVE SQRT")
            sqrt_component = 0.0
        shoulder_angle = -np.arctan2(z, y) - np.arctan2(
            np.sqrt(sqrt_component), self.shoulder_length)
        elbow_angle = np.arctan2(-x, np.sqrt(sqrt_component)) - np.arctan2(
            self.wrist_length * np.sin(wrist_angle),
            self.elbow_length + self.wrist_length * np.cos(wrist_angle))
        joint_angles = np.array([shoulder_angle, elbow_angle, wrist_angle]) # Cambiar signo de shoulder(- -> +)
        return -joint_angles + self.angle_offsets
    
def main():
    # Instancia para pierna derecha e izquierda
    ik_r = LegIK(legtype="RIGHT")
    ik_l = LegIK(legtype="LEFT")

    # Lista de posiciones de prueba [x, y, z] (m)
    test_positions = [
    [0.0,-0.04,-0.240], # Pierna estirada hacia abajo
    [0.125,-0.04,-0.115], # Pierna en pose L
    [0.0,-0.04,-0.17], # Pierna doblada pero en x=0
    [0.240,-0.04,0.0], # Pierna estirada hacia el frente
    [0.125,-0.05,-0.1] # Pierna en L pero abriendo el hombro
    ]

    indice = 4

    x,y,z = test_positions[indice]

    angles_right = ik_r.solve([x,y,z])
    angles_left = ik_l.solve([x,-y,z])

    if indice == 0:
        print("--- Resultados Pierna estirada hacia abajo ---")
    elif indice == 1:
        print("--- Resultados Pierna en pose L ---")
    elif indice == 2:
        print("--- Resultados Pierna doblada pero en x=0 ---")
    elif indice == 3:
        print("--- Resultados Pierna estirada hacia el frente ---")
    else:
        print("--- Resultados Pierna en L pero abriendo el hombro ---")

    print("\n--- Resultados para pierna Derecha ---")
    print(np.degrees(angles_right))
    print("\n--- Resultados para pierna Izquierda ---")
    print(np.degrees(angles_left))

if __name__ == "__main__":
    main()
