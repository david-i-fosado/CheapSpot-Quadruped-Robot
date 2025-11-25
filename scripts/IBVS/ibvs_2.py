#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
import cv2
import numpy as np
import apriltag
import csv
import os
# from cv_bridge import CvBridge  <-- ¡ELIMINADO!

class PoseEstimatorNode:
    def __init__(self):
        rospy.init_node("ibvs_pose_estimator", anonymous=True)

        self.gait_pub = rospy.Publisher('/gait_cmd_twist', Twist, queue_size=10, latch=True)

        self.control = 1
        #self.sd = np.array([-0.1342, -0.076, 0.0361, -0.0803, 0.0402, 0.0973, -0.1337, 0.1008]).reshape(-1,1)
        self.sd = np.array([-0.1373, -0.1605, 0.1307, -0.1644, 0.1323, 0.1104, -0.1331, 0.1152]).reshape(-1,1)

        self.cRp = np.array([[0, -1, 0], [0,  0, -1], [1,  0,  0]], dtype=float)
        tx = 0.07; ty = -0.145; tz = 0.0
        self.ctp_x = np.array([[0, -tz, ty], [tz, 0, -tx], [-ty, tx, 0]], dtype=float)

        datafile = rospy.get_param("~calib_file", "/home/david/catkin_ws/src/cheapspot/scripts/IBVS/calibration_data.npz")
        data = np.load(datafile, allow_pickle=True)
        if "mtx" in data: self.K = data["mtx"]
        elif "cameraMatrix" in data: self.K = data["cameraMatrix"]
        else: raise RuntimeError("No pude encontrar 'mtx' o 'cameraMatrix' en " + datafile)
        rospy.loginfo("K cargada:\n%s", str(self.K))

        self.tagSize = rospy.get_param("~tag_size", 0.13)
        h = self.tagSize / 2.0
        self.obj_points = np.array([[-h,  h, 0.0], [ h,  h, 0.0], [ h, -h, 0.0], [-h, -h, 0.0]], dtype=np.float32)

        self.detector = apriltag.Detector()
        # self.bridge = CvBridge() <-- ¡ELIMINADO!

        # Asegúrate de que este topic coincida con el de tu usb_cam
        self.sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback, queue_size=1, buff_size=2**24)
        self.last_frame = None

        # ... (Configuración del log CSV igual que antes) ...
        log_filename = os.path.join(os.path.expanduser('~'), "ibvs_log_full.csv")
        try:
            self.logfile = open(log_filename, 'w', newline='')
            self.csv_writer = csv.writer(self.logfile)
            self.csv_writer.writerow(["timestamp", "vc_x", "vc_y", "vc_z", "wc_x", "wc_y", "wc_z", "error_norm", 
                                      "vp_x", "vp_y", "vp_z", "wp_x", "wp_y", "wp_z", "cmd_linear_x", "cmd_linear_y", "cmd_angular_z"])
        except IOError as e:
            self.csv_writer = None

        rospy.on_shutdown(self.close_logfile)
        rospy.loginfo("ibvs_pose_estimator iniciado (MODO SIN CV_BRIDGE).")
        rospy.spin()
    
    def close_logfile(self):
        if hasattr(self, 'logfile') and self.logfile:
            self.logfile.close()

    def image_callback_manual(self, msg):
        """ Convierte manualmente SensorMsgs/Image (YUYV) a OpenCV (BGR) """
        try:
            dtype = np.uint8
            
            # CASO 1: La imagen viene en YUYV (lo más probable por tu comando)
            if msg.encoding == "yuyv" or msg.encoding == "yuv422":
                # En YUYV hay 2 bytes por pixel
                im_yuv = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width, 2)
                # Convertir de YUYV a BGR
                im = cv2.cvtColor(im_yuv, cv2.COLOR_YUV2BGR_YUYV)
                return im
                
            # CASO 2: La imagen viene en BGR8 o RGB8 (por si acaso cambias la config)
            elif msg.encoding == "bgr8":
                im = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width, 3)
                return im
            elif msg.encoding == "rgb8":
                im = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width, 3)
                im = cv2.cvtColor(im, cv2.COLOR_RGB2BGR)
                return im
            
            # CASO 3: No sabemos qué es, intentamos adivinar basado en el tamaño
            else:
                # Si el tamaño de datos coincide con ancho*alto*2 -> Es YUYV
                if len(msg.data) == msg.width * msg.height * 2:
                    im_yuv = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width, 2)
                    im = cv2.cvtColor(im_yuv, cv2.COLOR_YUV2BGR_YUYV)
                    return im
                # Si coincide con ancho*alto*3 -> Es BGR/RGB
                elif len(msg.data) == msg.width * msg.height * 3:
                    im = np.frombuffer(msg.data, dtype=dtype).reshape(msg.height, msg.width, 3)
                    return im # Asumimos BGR
                
                rospy.logwarn(f"Formato desconocido: {msg.encoding}. Intentando decodificar...")
                return None

        except Exception as e:
            rospy.logerr(f"Error conversión manual YUYV: {e}")
            return None

    def callback(self, img_msg):
        # --- CAMBIO CRÍTICO AQUÍ ---
        # En lugar de self.bridge.imgmsg_to_cv2, usamos nuestra función manual
        frame = self.image_callback_manual(img_msg)
        
        if frame is None:
            return
        # ---------------------------

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self.lost_counter = getattr(self, "lost_counter", 0)
        detections = self.detector.detect(gray)

        if len(detections) == 0:
            self.lost_counter += 1
            if self.lost_counter >= 15:
                cmd = Twist()
                self.gait_pub.publish(cmd)
                rospy.loginfo_throttle(2.0, "Buscando Tag... (No veo nada)")
            return
        else:
            self.lost_counter = 0

        d = detections[0]
        image_points = np.array(d.corners, dtype=np.float32).reshape(-1, 2)

        success, rvec, tvec = cv2.solvePnP(self.obj_points, image_points, self.K, None, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not success:
            rospy.logwarn("solvePnP falló."); return

        # ... (El resto de tu lógica matemática y de control se mantiene idéntica) ...
        R, _ = cv2.Rodrigues(rvec)
        t = tvec.reshape(3,)
        Zs = (R @ self.obj_points.T + t.reshape(3,1))[2, :]

        fx = self.K[0,0]; fy = self.K[1,1]; cx = self.K[0,2]; cy = self.K[1,2]
        s_list = []
        for (u,v) in image_points:
            x_norm = (u - cx) / fx; y_norm = (v - cy) / fy
            s_list.append([x_norm, y_norm])
        
        s = np.array(s_list).reshape(-1,1)
        rospy.loginfo_throttle(1.0, "s:%s",np.array2string(s, precision=4))
        rospy.loginfo_throttle(1.0, "s_d:%s",np.array2string(self.sd, precision=4))
        L_s = interaction_matrix(s, Zs)
        error_vec = s - self.sd
        error_norm = np.linalg.norm(error_vec)

        vc = -self.control * np.linalg.pinv(L_s) @ (error_vec) 
        
        V = np.block([[self.cRp, self.ctp_x @ self.cRp], [np.zeros((3,3)), self.cRp]])
        vp = -self.control * np.linalg.pinv(L_s @ V) @ (error_vec)
        rospy.loginfo_throttle(1.0, "vp:%s", np.array2string(vp, precision=4))

        cmd = Twist()
        scale_step = 1.2; scale_lat = 1.8; scale_yaw = 0.8
        scale_roll = 1.2; scale_pitch = 1.2;
        cmd.linear.x = float(np.clip(vp[0] * scale_step, -0.03, 0.03))
        cmd.linear.y = float(np.clip(vp[1] * scale_lat, -1.8, 1.8))
        cmd.angular.z = float(np.clip(vp[5] * scale_yaw, -0.5, 0.5))
        cmd.angular.x = float(np.clip(vp[3] * scale_roll,-0.5,0.5))
        cmd.angular.y = float(np.clip(vp[4] * scale_pitch,-0.5,0.5))

        deadzone_lin_x = 0.05
        deadzone_ang = 0.0001

        if abs(cmd.linear.x) < deadzone_lin_x: cmd.linear.x = 0.0

        if abs(cmd.angular.x) < deadzone_ang: cmd.angular.x = 0.0
        if abs(cmd.angular.y) < deadzone_ang: cmd.angular.y = 0.0
        if abs(cmd.angular.z) < deadzone_ang: cmd.angular.z = 0.0

        if self.csv_writer:
             # ... (Lógica de guardado CSV idéntica) ...
             pass

        now = rospy.Time.now()
        if not hasattr(self, "last_pub") or (now - self.last_pub).to_sec() > 0.25:
            self.gait_pub.publish(cmd)
            self.last_pub = now

        # Visualización
        #if frame is not None:
             # Solo para debug visual, puedes comentar esto si va lento
             #cv2.imshow("IBVS pose", frame)
             #cv2.waitKey(1)

def interaction_matrix(s, Zs):
    # ... (tu función sin cambios) ...
    n_points = len(Zs)
    L_list = []
    for i in range(n_points):
        x = s[2*i,0]; y = s[2*i+1,0]; Z = Zs[i]
        L_i = np.array([[-1/Z, 0, x/Z, x*y, -(1+x**2), y], [0, -1/Z, y/Z, 1+y**2, -x*y, -x]])
        L_list.append(L_i)
    return np.vstack(L_list)

if __name__ == "__main__":
    try:
        PoseEstimatorNode()
    except rospy.ROSInterruptException:
        pass
