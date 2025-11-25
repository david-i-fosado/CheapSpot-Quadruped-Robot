#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Twist
import cv2
import numpy as np
import apriltag
import csv
import os

class PoseEstimatorNode:
    def __init__(self):
        rospy.init_node("ibvs_pose_estimator", anonymous=True)

        # Publicador
        self.gait_pub = rospy.Publisher('/gait_cmd_twist', Twist, queue_size=10,latch=True)

        self.control = 1
        self.sd = np.array([-0.2017, -0.2249, 0.0236, -0.2315,  0.0299, -0.0048, -0.1879,  0.0046]).reshape(-1,1)
        self.sd = np.array([-0.1342, -0.076,   0.0361, -0.0803,  0.0402,  0.0973, -0.1337,  0.1008]).reshape(-1,1)
        
        
        # ... (Matrices cRp, ctp_x sin cambios) ...
        self.cRp = np.array([[0, -1, 0], [0,  0, -1], [1,  0,  0]], dtype=float)
        tx = 0.07; ty = -0.145; tz = 0.0
        self.ctp_x = np.array([[0, -tz, ty], [tz, 0, -tx], [-ty, tx, 0]], dtype=float)

        # ... (Carga de K sin cambios) ...
        datafile = rospy.get_param("~calib_file", "/home/david/Desktop/Modular/Raspberry_Pruebas/camera_calibration/calibration_data.npz")
        data = np.load(datafile, allow_pickle=True)
        if "mtx" in data: self.K = data["mtx"]
        elif "cameraMatrix" in data: self.K = data["cameraMatrix"]
        else: raise RuntimeError("No pude encontrar 'mtx' o 'cameraMatrix' en " + datafile)
        rospy.loginfo("K cargada:\n%s", str(self.K))

        # ... (Puntos 3D del tag sin cambios) ...
        self.tagSize = rospy.get_param("~tag_size", 0.13)
        h = self.tagSize / 2.0
        self.obj_points = np.array([[-h,  h, 0.0], [ h,  h, 0.0], [ h, -h, 0.0], [-h, -h, 0.0]], dtype=np.float32)

        # ... (Detector, bridge, subscripción sin cambios) ...
        self.detector = apriltag.Detector()
        self.bridge = CvBridge()
        # --- MODIFICADO: Veo que cambiaste el tópico de la cámara. ¡Perfecto! ---
        self.sub = rospy.Subscriber("usb_cam/image_raw", Image, self.callback, queue_size=1, buff_size=2**24)
        self.last_frame = None

        # --- 4) Configuración del archivo CSV --- MODIFICADO ---
        log_filename = os.path.join(os.path.expanduser('~'), "ibvs_log_full.csv")
        rospy.loginfo(f"Guardando log completo en: {log_filename}")
        try:
            self.logfile = open(log_filename, 'w', newline='')
            self.csv_writer = csv.writer(self.logfile)
            
            # Escribir la cabecera (MODIFICADA para incluir vp y cmd)
            self.csv_writer.writerow([
                "timestamp",
                "vc_x", "vc_y", "vc_z",  "wc_x", "wc_y", "wc_z", # Velocidad cámara
                "error_norm", 
                "e_x1", "e_y1", "e_x2", "e_y2", 
                "e_x3", "e_y3", "e_x4", "e_y4",
                # --- NUEVAS COLUMNAS ---
                "vp_x", "vp_y", "vp_z", "wp_x", "wp_y", "wp_z", # Velocidad robot (deseada)
                "cmd_linear_x", "cmd_linear_y", "cmd_angular_z" # Comando final (enviado)
            ])
        except IOError as e:
            rospy.logerr(f"No se pudo abrir el archivo CSV: {e}")
            self.csv_writer = None

        rospy.on_shutdown(self.close_logfile)
        # --- FIN DE SECCIÓN CSV ---

        rospy.loginfo("ibvs_pose_estimator iniciado.")
        rospy.spin()
    
    
    def close_logfile(self):
        if hasattr(self, 'logfile') and self.logfile:
            rospy.loginfo("Cerrando archivo CSV de log...")
            self.logfile.close()

    
    def callback(self, img_msg):
        # ... (Conversión de imagen, detección de tag, solvePnP sin cambios) ...
        try:
            frame = self.bridge.imgmsg_to_cv2(img_msg, desired_encoding="bgr8")
        except Exception as e:
            rospy.logerr("CvBridge error: %s", str(e))
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self.lost_counter = getattr(self, "lost_counter", 0)
        detections = self.detector.detect(gray)

        if len(detections) == 0:
            # ... (lógica de pérdida de tag sin cambios) ...
            self.lost_counter += 1
            if self.lost_counter >=15:
                cmd = Twist(); self.gait_pub.publish(cmd)
            #cv2.imshow("Frame", frame); cv2.waitKey(1)
            return
        else:
            self.lost_counter = 0

        d = detections[0]
        image_points = np.array(d.corners, dtype=np.float32).reshape(-1, 2)

        success, rvec, tvec = cv2.solvePnP(self.obj_points, image_points, self.K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not success:
            rospy.logwarn("solvePnP falló."); return

        R, _ = cv2.Rodrigues(rvec)
        t = tvec.reshape(3,)
        Zs = (R @ self.obj_points.T + t.reshape(3,1))[2, :]

        # ... (Cálculo de 's' sin cambios) ...
        fx = self.K[0,0]; fy = self.K[1,1]; cx = self.K[0,2]; cy = self.K[1,2]
        s_list = []
        for (u,v) in image_points:
            x_norm = (u - cx) / fx; y_norm = (v - cy) / fy
            s_list.append([x_norm, y_norm])
        s = np.array(s_list).reshape(-1,1)
        
        L_s = interaction_matrix(s, Zs)

        # --- Cálculo del Error ---
        error_vec = s - self.sd
        error_norm = np.linalg.norm(error_vec)
        rospy.loginfo_throttle(1.0, "Error Norm: %.4f", error_norm)

        # Velocidad de la cámara
        vc = -self.control * np.linalg.pinv(L_s) @ (error_vec) 
        rospy.loginfo_throttle(1.0, "vc: %s", np.array2string(vc, precision=4))

        # --- ATENCIÓN: El bloque de guardado CSV se ha movido más abajo ---


        # 2) Matriz de transformación V (6x6)
        V = np.block([
            [self.cRp, self.ctp_x @ self.cRp],
            [np.zeros((3,3)), self.cRp]
        ])

        # 3) Velocidad del robot / marco base (ESTA ES vp)
        vp = -self.control * np.linalg.pinv(L_s @ V) @ (error_vec) 
        rospy.loginfo_throttle(1.0, "vp:%s", np.array2string(vp, precision=4))

        cmd = Twist()
        
        # ... (Lógica de escalado y clipping) ...
        scale_step = 0.3; scale_lat = 0.8; scale_yaw = 0.5
        cmd.linear.x = float(np.clip(vp[0] * scale_step, -0.03, 0.03))
        cmd.linear.y = float(np.clip(vp[1] * scale_lat, -1.8, 1.8))
        # (El usuario escribió cmd.angular.w, pero en Twist es .z)
        cmd.angular.z= float(np.clip(vp[5] * scale_yaw, -0.6, 0.6))
        
        # ... (Lógica de dead_zone comentada, la dejo como está) ...
        """
        dead_zone_x = 0.006; dead_zone_y = 0.04
        if abs(cmd.linear.x) < dead_zone_x:
            cmd.linear.x = 0
            if abs(cmd.linear.y) > dead_zone_y:
                cmd.linear.x = 0.015
                cmd.linear.y = np.sign(cmd.linear.y) * (abs(cmd.linear.y) + 1)
        else:
            cmd.linear.x = np.sign(cmd.linear.x) * (abs(cmd.linear.x) - dead_zone_x)
            cmd.linear.y = np.sign(cmd.linear.y) * (abs(cmd.linear.y) - dead_zone_y)
        """

        # --- MODIFICADO: Bloque de guardado CSV (movido y actualizado) ---
        # Lo ponemos aquí para asegurarnos de que 'vc', 'error_vec', 'vp' y 'cmd'
        # ya han sido calculados.
        if self.csv_writer:
            try:
                log_time = rospy.Time.now().to_sec()
                
                # Aplanar todos los vectores
                vc_flat = vc.flatten()
                error_flat = error_vec.flatten()
                vp_flat = vp.flatten() # --- NUEVO ---
                
                # Crear la fila completa
                row_data = [log_time] + \
                           list(vc_flat) + \
                           [error_norm] + \
                           list(error_flat) + \
                           list(vp_flat) + \
                           [cmd.linear.x, cmd.linear.y, cmd.angular.z] # --- NUEVO ---
                
                self.csv_writer.writerow(row_data)
                
            except Exception as e:
                rospy.logwarn_throttle(10.0, f"Error al escribir en CSV: {e}")
        # --- FIN DE GUARDAR ---

        # Publicación del comando
        now = rospy.Time.now()
        if not hasattr(self, "last_pub") or (now - self.last_pub).to_sec() > 0.25:
            self.gait_pub.publish(cmd)
            self.last_pub = now

        # ... (Visualización de CV2 sin cambios) ...
        rospy.loginfo_throttle(1.0, "s (normalized): %s", np.array2string(s.reshape(-1,), precision=4))
        for (u,v) in image_points:
            cv2.circle(frame, (int(u), int(v)), 4, (0,255,0), -1)
        axis = np.float32([[0,0,0],[self.tagSize/2,0,0],[0,self.tagSize/2,0],[0,0,self.tagSize/2]])
        imgpts, _ = cv2.projectPoints(axis, rvec, tvec, self.K, None)
        imgpts = imgpts.reshape(-1,2).astype(int)
        origin = tuple(imgpts[0])
        cv2.line(frame, origin, tuple(imgpts[1]), (0,0,255), 3)
        cv2.line(frame, origin, tuple(imgpts[2]), (0,255,0), 3) # Y
        cv2.line(frame, origin, tuple(imgpts[3]), (255,0,0), 3) # Z
        cv2.imshow("IBVS pose", frame)
        cv2.waitKey(1)

# ... (función interaction_matrix sin cambios) ...
def interaction_matrix(s, Zs):
    n_points = len(Zs)
    L_list = []
    for i in range(n_points):
        x = s[2*i,0]; y = s[2*i+1,0]; Z = Zs[i]
        L_i = np.array([
            [-1/Z,     0,  x/Z,  x*y,  -(1+x**2),  y],
            [ 0,    -1/Z,  y/Z, 1+y**2,  -x*y,   -x]
        ])
        L_list.append(L_i)
    L_s = np.vstack(L_list)
    return L_s


if __name__ == "__main__":
    try:
        PoseEstimatorNode()
    except rospy.ROSInterruptException:
        pass
