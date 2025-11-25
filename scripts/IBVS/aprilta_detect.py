#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import cv2
import numpy as np
import apriltag

class PoseEstimatorNode:
    def __init__(self):
        rospy.init_node("ibvs_pose_estimator", anonymous=True)

        # Publicador hacia la Raspberry
        self.gait_pub = rospy.Publisher('/gait_cmd_twist', Twist, queue_size=10,latch=True)

        self.control = 1

        self.sd = np.array([-0.2017, -0.2249, 0.0236, -0.2315,  0.0299, -0.0048, -0.1879,  0.0046]).reshape(-1,1)
        # --- 1) Matriz antisimétrica y constantes (como en tu MATLAB)
        self.cRp = np.array([[0, -1, 0],
                             [0,  0, -1],
                             [1,  0,  0]], dtype=float)
        tx = 0.07 # tx = tz
        ty = -0.145 # ty = -tx
        tz = 0.0 # tz = ty
        self.ctp_x = np.array([[0, -tz, ty],
                               [tz, 0, -tx],
                               [-ty, tx, 0]], dtype=float)

        # --- 2) Cargar intrínsecos
        # Se asume un archivo calibration_data.npz con 'mtx' o cameraParams.npz
        datafile = rospy.get_param("~calib_file", "/home/david/Desktop/Modular/Raspberry_Pruebas/camera_calibration/calibration_data.npz")
        data = np.load(datafile, allow_pickle=True)
        # Preferencia por claves comunes
        if "mtx" in data:
            self.K = data["mtx"]
        elif "cameraMatrix" in data:
            self.K = data["cameraMatrix"]
        else:
            # si guardaste MATLAB cameraParams, ajusta la carga
            raise RuntimeError("No pude encontrar 'mtx' o 'cameraMatrix' en " + datafile)

        rospy.loginfo("K cargada:\n%s", str(self.K))

        # --- 3) Tag size y puntos 3D en el marco del tag (X)
        self.tagSize = rospy.get_param("~tag_size", 0.13)  # 0.13 m por defecto
        h = self.tagSize / 2.0
        # Orden: [(-h, h), (h, h), (h, -h), (-h, -h)] (igual que tu MATLAB)
        self.obj_points = np.array([[-h,  h, 0.0],
                                    [ h,  h, 0.0],
                                    [ h, -h, 0.0],
                                    [-h, -h, 0.0]], dtype=np.float32)

        # Detector Apriltag
        self.detector = apriltag.Detector()

        # cv_bridge + subscripción
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber("usb_cam/image_raw", Image, self.callback, queue_size=1, buff_size=2**24)

        # variables de uso
        self.last_frame = None

        rospy.loginfo("ibvs_pose_estimator iniciado.")
        rospy.spin()
    
    def callback(self, img_msg):
        # Convertir a OpenCV
        try:
            frame = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr("CvBridge error: %s", str(e))
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self.lost_counter = getattr(self, "lost_counter", 0)

        # Detectar tags
        detections = self.detector.detect(gray)
        if len(detections) == 0:
            self.lost_counter += 1
            if self.lost_counter >=15:
                cmd = Twist()
                self.gait_pub.publish(cmd)
            # opcional: mostrar
            cv2.imshow("Frame", frame)
            cv2.waitKey(1)
            return
        else:
            self.lost_counter = 0

        # Tomamos la primera detección (ajusta si quieres controlar por id)
        d = detections[0]

        # apriltag python devuelve 'corners' como 4x2 float in image coordinates.
        # IMPORTANTE: verificar orden. En la práctica apriltag devuelve esquinas en
        # orden: [pt0,pt1,pt2,pt3] con orientación CCW, pero conviene mapear
        # al orden que definiste en obj_points.
        corners = np.array(d.corners, dtype=np.float32)  # shape (4,2)

        # Si es necesario reordenar para que coincida con obj_points, hazlo aquí.
        # La siguiente asunción: apriltag devuelve [top-left, top-right, bottom-right, bottom-left].
        # Esa coincide con tu orden en MATLAB: [-h,h], [h,h], [h,-h], [-h,-h].
        image_points = corners.reshape(-1, 2)

        # Estimación de pose con solvePnP (usa los 4 puntos 3D y sus corners 2D)
        # Nota: solvePnP requiere punto3D shape (N,3) float32 y punto2D (N,2) float32
        success, rvec, tvec = cv2.solvePnP(self.obj_points,
                                           image_points,
                                           self.K,
                                           None,
                                           flags=cv2.SOLVEPNP_IPPE_SQUARE)  # IPPE_SQUARE good for planar square

        if not success:
            rospy.logwarn("solvePnP falló.")
            return

        # Convertir rvec -> R
        R, _ = cv2.Rodrigues(rvec)  # R: 3x3
        t = tvec.reshape(3,)

        # Construir Rt.A (3x4) como tu Rt.A en MATLAB
        #RtA = np.hstack((R, t.reshape(3,1)))  # 3x4

        # Calcular Z para cada corner: Z = Rt.A * X_homog (ver MATLAB)
        #X_hom = np.hstack((self.obj_points, np.ones((4,1), dtype=np.float32))).T  # 4x4 -> transpose -> 4x4, but we want 4xN?
        # mejor: multiplicar R*[X] + t para cada punto
        Zs = (R @ self.obj_points.T + t.reshape(3,1))[2, :]  # vector de 4 elementos (profundidades)

        # Calcular s normalizado (como en tu Proyection_2D)
        fx = self.K[0,0]; fy = self.K[1,1]
        cx = self.K[0,2]; cy = self.K[1,2]
        s_list = []
        for (u,v) in image_points:
            x_norm = (u - cx) / fx
            y_norm = (v - cy) / fy
            s_list.append([x_norm, y_norm])
        s = np.array(s_list).reshape(-1,1)  # 8x1 (4 puntos -> 8 entradas)
        #sdiff = s-self.sd
        #rospy.loginfo_throttle(1.0, "sdiff:%s",np.array2string(sdiff, precision=4))
        L_s = interaction_matrix(s, Zs)
        #rospy.loginfo_throttle(1.0, "Matriz de interaccion L_s:\n%s", np.array2string(L_s, precision=4))

        vc = -self.control * np.linalg.pinv(L_s) @ (s - self.sd)
        rospy.loginfo_throttle(1.0, "vc: %s", np.array2string(vc, precision=4))

        # 2) Matriz de transformación V (6x6)
        V = np.block([
            [self.cRp, self.ctp_x @ self.cRp],
            [np.zeros((3,3)), self.cRp]
        ])

        # 3) Velocidad del robot / marco base
        vp = -self.control * np.linalg.pinv(L_s @ V) @ (s - self.sd)

        rospy.loginfo_throttle(1.0, "vp:%s", np.array2string(vp, precision=4))

        cmd = Twist()

         # Escala (ajústala según respuesta del robot)
        scale_step = 0.3
        scale_lat = 0.8
        scale_yaw = 0.5

        cmd.linear.x = float(np.clip(vp[0] * scale_step, -0.03, 0.03))
        cmd.linear.y = float(np.clip(vp[1] * scale_lat, -1.8, 1.8))
        #cmd.angular.z = float(np.clip(vp[5] * scale_yaw, -0.4, 0.4)) 
        dead_zone_x = 0.006
        dead_zone_y = 0.04
        if abs(cmd.linear.x) < dead_zone_x:
            cmd.linear.x = 0
            if abs(cmd.linear.y) > dead_zone_y:
                cmd.linear.x = 0.015
                cmd.linear.y = np.sign(cmd.linear.y) * (abs(cmd.linear.y) + 1)
        else:
            # Resta el umbral gradualmente para evitar el salto
            cmd.linear.x = np.sign(cmd.linear.x) * (abs(cmd.linear.x) - dead_zone_x)
            cmd.linear.y = np.sign(cmd.linear.y) * (abs(cmd.linear.y) - dead_zone_y)
        # Limitar frecuencia de publicación
        now = rospy.Time.now()
        if not hasattr(self, "last_pub") or (now - self.last_pub).to_sec() > 0.25:  # 10 Hz
            self.gait_pub.publish(cmd)
            self.last_pub = now

        self.gait_pub.publish(cmd)

        # Log / visualizar resultados equivalentes a tu bloque MATLAB
        #rospy.loginfo_throttle(1.0, "rvec: %s", np.array2string(rvec.reshape(3,), precision=4))
        #rospy.loginfo_throttle(1.0, "tvec (m): %s", np.array2string(t, precision=4))
        #rospy.loginfo_throttle(1.0, "Z per corner (m): %s", np.array2string(Zs, precision=4))
        rospy.loginfo_throttle(1.0, "s (normalized): %s", np.array2string(s.reshape(-1,), precision=4))
        #"""
        # Dibujar corners detectados y eje para debug
        for (u,v) in image_points:
            cv2.circle(frame, (int(u), int(v)), 4, (0,255,0), -1)

        # Opcional: proyectar origen del tag y ejes 3D para verificar (como en MATLAB)
        # puntos eje en mm o m (0.5*tagSize para visibilidad)
        axis = np.float32([[0,0,0],[self.tagSize/2,0,0],[0,self.tagSize/2,0],[0,0,self.tagSize/2]])
        imgpts, _ = cv2.projectPoints(axis, rvec, tvec, self.K, None)
        imgpts = imgpts.reshape(-1,2).astype(int)
        origin = tuple(imgpts[0])
        cv2.line(frame, origin, tuple(imgpts[1]), (0,0,255), 3)  # x (red)
        cv2.line(frame, origin, tuple(imgpts[2]), (0,255,0), 3)  # y (green)
        cv2.line(frame, origin, tuple(imgpts[3]), (255,0,0), 3)  # z (blue)

        cv2.imshow("IBVS pose", frame)
        cv2.waitKey(1)
        #"""

def interaction_matrix(s, Zs):
        """
        s: array de 8x1 = [x1, y1, x2, y2, x3, y3, x4, y4]^T
        Zs: array de 4 elementos = [Z1, Z2, Z3, Z4]
        retorna L_s: 8x6
        """
        n_points = len(Zs)
        L_list = []

        for i in range(n_points):
            x = s[2*i,0]   # x_i
            y = s[2*i+1,0] # y_i
            Z = Zs[i]

            L_i = np.array([
                [-1/Z,     0,  x/Z,  x*y,  -(1+x**2),  y],
                [ 0,    -1/Z,  y/Z, 1+y**2,  -x*y,   -x]
            ])
            L_list.append(L_i)

        L_s = np.vstack(L_list)  # 8x6
        return L_s


if __name__ == "__main__":
    try:
        PoseEstimatorNode()
    except rospy.ROSInterruptException:
        pass

