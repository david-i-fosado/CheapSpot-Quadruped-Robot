#!/usr/bin/env python

import numpy as np
from cheapspot.Kinematics.leg_ik import LegIK
from cheapspot.Kinematics.lieAlgebra import RpToTrans, TransToRp, TransInv, RPY, TransformVector
from collections import OrderedDict

class SpotModel:
    def __init__(self,
                 shoulder_length=0.04,
                 elbow_length=0.115,
                 wrist_length=0.125,
                 hip_x=0.18,
                 hip_y=0.08,
                 foot_x=0.18,
                 foot_y=0.16, #0.16
                 height=0.19,
                 com_offset=-0.034, #0.016  -0.032
                 shoulder_lim=[-0.548, 0.548],
                 elbow_lim=[-2.17, 0.97],
                 wrist_lim=[-0.1, 2.59]):
        """
        Spot Micro Kinematics
        """
        # COM offset in x direction
        self.com_offset = com_offset

        # Leg Parameters
        self.shoulder_length = shoulder_length
        self.elbow_length = elbow_length
        self.wrist_length = wrist_length

        # Leg Vector desired_positions

        # Distance Between Hips
        # Length
        self.hip_x = hip_x
        # Width
        self.hip_y = hip_y

        # Distance Between Feet
        # Length
        self.foot_x = foot_x
        # Width
        self.foot_y = foot_y

        # Body Height
        self.height = height

        # Joint Parameters
        self.shoulder_lim = shoulder_lim
        self.elbow_lim = elbow_lim
        self.wrist_lim = wrist_lim

        # Dictionary to store Leg IK Solvers
        self.Legs = OrderedDict()
        self.Legs["FL"] = LegIK("LEFT", self.shoulder_length,
                                self.elbow_length, self.wrist_length,
                                self.shoulder_lim, self.elbow_lim,
                                self.wrist_lim)
        self.Legs["FR"] = LegIK("RIGHT", self.shoulder_length,
                                self.elbow_length, self.wrist_length,
                                self.shoulder_lim, self.elbow_lim,
                                self.wrist_lim)
        self.Legs["BL"] = LegIK("LEFT", self.shoulder_length,
                                self.elbow_length, self.wrist_length,
                                self.shoulder_lim, self.elbow_lim,
                                self.wrist_lim)
        self.Legs["BR"] = LegIK("RIGHT", self.shoulder_length,
                                self.elbow_length, self.wrist_length,
                                self.shoulder_lim, self.elbow_lim,
                                self.wrist_lim)

        # Dictionary to store Hip and Foot Transforms

        # Transform of Hip relative to world frame
        # With Body Centroid also in world frame
        Rwb = np.eye(3)
        self.WorldToHip = OrderedDict()

        self.ph_FL = np.array([self.hip_x / 2.0, self.hip_y / 2.0, 0])
        self.WorldToHip["FL"] = RpToTrans(Rwb, self.ph_FL)

        self.ph_FR = np.array([self.hip_x / 2.0, -self.hip_y / 2.0, 0])
        self.WorldToHip["FR"] = RpToTrans(Rwb, self.ph_FR)

        self.ph_BL = np.array([-self.hip_x / 2.0, self.hip_y / 2.0, 0])
        self.WorldToHip["BL"] = RpToTrans(Rwb, self.ph_BL)

        self.ph_BR = np.array([-self.hip_x / 2.0, -self.hip_y / 2.0, 0])
        self.WorldToHip["BR"] = RpToTrans(Rwb, self.ph_BR)

        # Transform of Foot relative to world frame
        # With Body Centroid also in world frame
        self.WorldToFoot = OrderedDict()

        self.pf_FL = np.array(
            [self.foot_x / 2.0, self.foot_y / 2.0, -self.height])
        self.WorldToFoot["FL"] = RpToTrans(Rwb, self.pf_FL)

        self.pf_FR = np.array(
            [self.foot_x / 2.0, -self.foot_y / 2.0, -self.height])
        self.WorldToFoot["FR"] = RpToTrans(Rwb, self.pf_FR)

        self.pf_BL = np.array(
            [-self.foot_x / 2.0, self.foot_y / 2.0, -self.height])
        self.WorldToFoot["BL"] = RpToTrans(Rwb, self.pf_BL)

        self.pf_BR = np.array(
            [-self.foot_x / 2.0, -self.foot_y / 2.0, -self.height])
        self.WorldToFoot["BR"] = RpToTrans(Rwb, self.pf_BR)

    def HipToFoot(self, orn, pos, T_bf):
        """
        Converts a desired position and orientation wrt Spot's
        home position, with a desired body-to-foot Transform
        into a body-to-hip Transform, which is used to extract
        and return the Hip To Foot Vector.

        :param orn: A 3x1 np.array([]) with Spot's Roll, Pitch, Yaw angles
        :param pos: A 3x1 np.array([]) with Spot's X, Y, Z coordinates
        :param T_bf: Dictionary of desired body-to-foot Transforms.
        :return: Hip To Foot Vector for each of Spot's Legs.
        """

        # Following steps in attached document: SpotBodyIK.
        # TODO: LINK DOC

        # Only get Rot component
        Rb, _ = TransToRp(RPY(orn[0], orn[1], orn[2]))
        pb = pos
        T_wb = RpToTrans(Rb, pb)

        # Dictionary to store vectors
        HipToFoot_List = OrderedDict()

        for i, (key, T_wh) in enumerate(self.WorldToHip.items()):
            # ORDER: FL, FR, FR, BL, BR

            # Extract vector component
            _, p_bf = TransToRp(T_bf[key])

            # Step 1, get T_bh for each leg
            T_bh = np.dot(TransInv(T_wb), T_wh)

            # Step 2, get T_hf for each leg

            # VECTOR ADDITION METHOD
            _, p_bh = TransToRp(T_bh)
            p_hf0 = p_bf - p_bh

            # TRANSFORM METHOD
            T_hf = np.dot(TransInv(T_bh), T_bf[key])
            _, p_hf1 = TransToRp(T_hf)

            # They should yield the same result
            if p_hf1.all() != p_hf0.all():
                print("NOT EQUAL")

            p_hf = p_hf1

            HipToFoot_List[key] = p_hf

        return HipToFoot_List

    def IK(self, orn, pos, T_bf):
        """
        Uses HipToFoot() to convert a desired position
        and orientation wrt Spot's home position into a
        Hip To Foot Vector, which is fed into the LegIK solver.

        Finally, the resultant joint angles are returned
        from the LegIK solver for each leg.

        :param orn: A 3x1 np.array([]) with Spot's Roll, Pitch, Yaw angles
        :param pos: A 3x1 np.array([]) with Spot's X, Y, Z coordinates
        :param T_bf: Dictionary of desired body-to-foot Transforms.
        :return: Joint angles for each of Spot's joints.
        """

        # Following steps in attached document: SpotBodyIK.
        # TODO: LINK DOC

        # Modify x by com offset
        pos[0] += self.com_offset

        # 4 legs, 3 joints per leg
        joint_angles = np.zeros((4, 3))

        # print("T_bf: {}".format(T_bf))

        # Steps 1 and 2 of pipeline here
        HipToFoot = self.HipToFoot(orn, pos, T_bf)

        for i, (key, p_hf) in enumerate(HipToFoot.items()):
            # ORDER: FL, FR, FR, BL, BR

            # print("LEG: {} \t HipToFoot: {}".format(key, p_hf))

            # Step 3, compute joint angles from T_hf for each leg
            joint_angles[i, :] = self.Legs[key].solve(p_hf)
        #print(joint_angles)

        # print("-----------------------------")

        return joint_angles
    
def main():
    # Crear modelo del robot
    spot = SpotModel()

    # Orientación del cuerpo [roll, pitch, yaw] en radianes
    orn = np.array([0.0, 0.0, 0.0])  # sin inclinación

    # Posición del cuerpo [x, y, z] en metros
    pos = np.array([0.0, 0.0, 0.0])  # en el origen

    # Posiciones deseadas de las patas (usar las que trae el modelo)
    T_bf = spot.WorldToFoot

    # Llamada al solver y obtención de ángulos
    joint_angles = spot.IK(orn, pos, T_bf)

    # Impresión de resultados en grados
    print("Ángulos de las 4 patas en Home Pose:")
    for (leg, _), angles in zip(spot.Legs.items(), joint_angles):
        a_deg = np.degrees(angles)
        print(f" {leg}: shoulder={a_deg[0]:6.1f}°, "
              f"elbow={a_deg[1]:6.1f}°, "
              f"wrist={a_deg[2]:6.1f}°")

    servo_map = {
        "FL": [10,6,2],
        "FR": [9,5,1],
        "BL": [12,8,4],
        "BR": [11,7,3]
    }

    servo_positions = [0]*12

    joint_degrees = np.degrees(joint_angles)

    for leg_idx, leg in enumerate(["FL","FR","BL","BR"]):
        for joint_idx, servo_num in enumerate(servo_map[leg]):
            raw = joint_degrees[leg_idx][joint_idx] + 90
            servo_positions[servo_num-1] = int(round(raw))

    print("--- Servo Position ---\n",servo_positions)
    print(f'rostopic pub /servo_positions std_msgs/Int32MultiArray "data: {servo_positions}"')

if __name__ == "__main__":
    main()
