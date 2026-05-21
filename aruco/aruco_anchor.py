import rclpy
from rclpy.node import Node
import cv2
import numpy as np
import json
import os
import tf_transformations
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class ArucoAnchor(Node):
    def __init__(self):
        super().__init__('aruco_anchor')
        self.get_logger().info("--- Dynamic Camera System Started (With Memory) ---")

        # --- ΡΥΘΜΙΣΕΙΣ ---
        self.ANCHOR_ID = 1
        self.MARKER_SIZE = 0.03 
        
        # --- ΝΕΟ: ΜΝΗΜΗ ΤΩΝ MARKERS ---
        # Εδώ θα αποθηκεύουμε την τελευταία γνωστή θέση: { marker_id : (rvec, tvec) }
        self.last_known_poses = {}

        # Φόρτωση Παραμέτρων
        json_file = "camera_params.json"
        if not os.path.exists(json_file):
            self.get_logger().error("Το αρχείο camera_params.json δεν βρέθηκε!")
            exit()

        with open(json_file, "r") as f:
            data = json.load(f)
        
        params = data["camera_parameters"]["4"]
        self.camera_matrix = np.array(params["camera_matrix"])
        self.dist_coeffs = np.array(params["dist_coeffs"])

        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.parameters = cv2.aruco.DetectorParameters()

        self.cap = cv2.VideoCapture(0)
        
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
        
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(0.05, self.loop)

    def loop(self):
        ret, frame = self.cap.read()
        if not ret: return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = cv2.aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        # 1. Ενημέρωση της Μνήμης (αν βλέπει markers τώρα)
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(corners, self.MARKER_SIZE, self.camera_matrix, self.dist_coeffs)

            for i in range(len(ids)):
                current_id = ids[i][0]
                
                # Αποθηκεύουμε/Ανανεώνουμε την τελευταία γνωστή θέση στη μνήμη
                self.last_known_poses[current_id] = (rvecs[i], tvecs[i])

                try:
                    cv2.drawFrameAxes(frame, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.03)
                except AttributeError:
                    cv2.aruco.drawAxis(frame, self.camera_matrix, self.dist_coeffs, rvecs[i], tvecs[i], 0.03)

        # 2. Εκπομπή όλων των γνωστών θέσεων (από τη μνήμη) στο ROS
        for m_id, (rvec, tvec) in self.last_known_poses.items():
            if m_id == self.ANCHOR_ID:
                self.broadcast_camera_pose(rvec, tvec)
            else:
                self.broadcast_target_pose(m_id, rvec, tvec)

        cv2.imshow('Dynamic Camera View', frame)
        cv2.waitKey(1)

    def broadcast_camera_pose(self, rvec, tvec):
        R, _ = cv2.Rodrigues(rvec)
        T = tvec[0]
        R_inv = R.T
        T_inv = -np.dot(R_inv, T)
        
        transform_matrix = np.eye(4)
        transform_matrix[0:3, 0:3] = R_inv
        quat = tf_transformations.quaternion_from_matrix(transform_matrix)
        
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "marker_base"   
        t.child_frame_id = "camera_link"    
        
        t.transform.translation.x = T_inv[0]
        t.transform.translation.y = T_inv[1]
        t.transform.translation.z = T_inv[2]
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]
        
        self.tf_broadcaster.sendTransform(t)

    def broadcast_target_pose(self, marker_id, rvec, tvec):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "camera_link"
        t.child_frame_id = f"marker_{marker_id}"

        t.transform.translation.x = tvec[0][0]
        t.transform.translation.y = tvec[0][1]
        t.transform.translation.z = tvec[0][2]

        rot_matrix, _ = cv2.Rodrigues(rvec)
        transform_matrix = np.eye(4)
        transform_matrix[0:3, 0:3] = rot_matrix
        quat = tf_transformations.quaternion_from_matrix(transform_matrix)
        
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.tf_broadcaster.sendTransform(t)

def main():
    rclpy.init()
    node = ArucoAnchor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
