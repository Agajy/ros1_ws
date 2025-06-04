#!/usr/bin/env python
import rospy
import tf
from std_msgs.msg import Float64, Float32, Bool
from geometry_msgs.msg import PoseStamped, Vector3, Pose, Point, PoseArray
from visualization_msgs.msg import Marker
import cv2
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np
from skimage.morphology import skeletonize
from scipy.signal import savgol_filter
from math import *


class LineAndArucoDetector:
    def __init__(self):
        rospy.init_node('line_and_aruco_detector', anonymous=True)
        self.bridge = CvBridge()
        
        # Subscribers
        self.image_sub = rospy.Subscriber('/image_tcp_client/image_gray', Image, self.image_callback)
        
        # Publishers
        self.original_pub = rospy.Publisher('/original_curve', Marker, queue_size=10)
        self.filtered_pub = rospy.Publisher('/filtered_curve', Marker, queue_size=10)
        self.aruco_pose_pub = rospy.Publisher('/aruco_pose', PoseStamped, queue_size=10)
        self.aruco_corner_pub = rospy.Publisher('/aruco_corners', PoseArray, queue_size=10)
        self.aruco_detected_pub = rospy.Publisher('/aruco_detected', Bool, queue_size=10)

        # Paramètres image
        self.largeur = 640
        self.hauteur = 480

        # Configuration ArUco
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
        self.aruco_params = aruco.DetectorParameters()
        
        # Paramètres caméra
        self.camera_matrix = np.array([[314.9937259674355, 0, 146.24621020976434],
                                     [0, 314.5862387824696, 128.53019044749945],
                                     [0, 0, 1]], dtype=np.float32)
        self.dist_coeffs = np.zeros((4, 1))  # Coefficients de distorsion
        
        # Taille réelle du marqueur ArUco en mètres
        self.marker_size = 0.2  # 10cm
        
        rospy.loginfo("Line and ArUco detector initialized.")
        rospy.spin()
    
    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        except Exception as e:
            rospy.logerr("CV Bridge error: %s", e)
            return
            
        # Mettre à jour les dimensions d'image
        self.hauteur, self.largeur = cv_image.shape[:2]
        
        # Créer une version couleur pour l'affichage
        cv_display = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
        
        # Traitement des lignes (votre code existant)
        self.process_lines(cv_image, cv_display)
        
        # Détection ArUco améliorée
        self.detect_aruco(cv_image, cv_display)
        
        # Affichage
        cv2.imshow("Lines and ArUco Detection", cv_display)
        cv2.waitKey(1)

    def process_lines(self, cv_image, cv_display):
        """Traitement des lignes (code existant optimisé)"""
        # Seuillage + ouverture morphologique
        _, thresh = cv2.threshold(cv_image, 200, 255, cv2.THRESH_BINARY)
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, np.ones((3,3), np.uint8))
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        nombre_lignes = 0
        all_skeleton_points = []
        
        for contour in contours:
            if len(contour) < 50:
                continue
                
            # Masque et ROI
            mask = np.zeros_like(cv_image, dtype=np.uint8)
            cv2.drawContours(mask, [contour], 0, 255, 0)
            x, y, w, h = cv2.boundingRect(contour)
            
            skeleton = skeletonize(mask[y:y+h, x:x+w])
            skeleton_coords = np.column_stack(np.where(skeleton))
            
            if skeleton_coords.size == 0:
                continue
                
            nombre_lignes += 1
            
            # Affichage des points
            coords = []
            for j, i in skeleton_coords:
                cv_display[y + j, x + i] = (0, 0, 255)
                coords.append((x + i, y + j))
            
            all_skeleton_points.append(coords)
        
        # Calcul de la ligne centrale
        if len(all_skeleton_points) >= 2:
            sorted_skeletons = sorted(all_skeleton_points, key=len, reverse=True)
            skeleton_1 = np.array(sorted_skeletons[0])
            skeleton_2 = np.array(sorted_skeletons[1])
            
            if len(skeleton_1) > 0 and len(skeleton_2) > 0:
                iteration_min = min(len(skeleton_1), len(skeleton_2))
                central_curve_points = []
                
                for idx in range(1, iteration_min):
                    x_center = (skeleton_1[-idx,0] + skeleton_2[-idx,0]) / 2
                    y_center = (skeleton_1[-idx,1] + skeleton_2[-idx,1]) / 2
                    central_curve_points.append((x_center, y_center, 0.0))
                
                if central_curve_points:
                    central_curve = np.array(central_curve_points)
                    original_marker = self.create_line_marker(central_curve, 0, 1.0, 0.0, 0.0, "original_curve")
                    self.original_pub.publish(original_marker)

    def detect_aruco(self, cv_image, cv_display):
        """Détection ArUco améliorée"""
        # Détection des marqueurs
        corners, ids, _ = aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None and len(ids) > 0:
            # Dessiner les marqueurs détectés
            aruco.drawDetectedMarkers(cv_display, corners, ids)
            
            # Estimation de pose si les paramètres de caméra sont disponibles
            rvecs, tvecs, _ = aruco.estimatePoseSingleMarkers(
                corners, self.marker_size, self.camera_matrix, self.dist_coeffs)
            
            # Traiter chaque marqueur
            for i in range(len(ids)):
                marker_id = ids[i][0]
                corner = corners[i][0]
                
                # Dessiner les axes du marqueur
                cv2.drawFrameAxes(cv_display, self.camera_matrix, self.dist_coeffs, 
                                rvecs[i], tvecs[i], 0.05)
                
                # Publier les coins du marqueur
                self.publish_aruco_corners(corner, marker_id)
                
                # Publier la pose du marqueur
                self.publish_aruco_pose(rvecs[i], tvecs[i], marker_id)
                
                # Calculer et afficher des informations
                center_x = np.mean(corner[:, 0])
                center_y = np.mean(corner[:, 1])
                
                # Calculer l'angle d'orientation
                dx = corner[1][0] - corner[0][0]
                dy = corner[1][1] - corner[0][1]
                angle = np.arctan2(dy, dx)
                
                rospy.loginfo_throttle(1.0, 
                    f"ArUco {marker_id} détecté: centre=({center_x:.1f},{center_y:.1f}), "
                    f"angle={np.degrees(angle):.1f}°, distance={np.linalg.norm(tvecs[i]):.2f}m")
            
            # Publier l'état de détection
            self.aruco_detected_pub.publish(Bool(data=True))
            
        else:
            # Aucun marqueur détecté
            self.aruco_detected_pub.publish(Bool(data=False))

    def publish_aruco_corners(self, corners, marker_id):
        """Publie les coins du marqueur ArUco"""
        pose_array = PoseArray()
        pose_array.header.frame_id = "camera_frame"
        pose_array.header.stamp = rospy.Time.now()
        
        # Ordre des coins: top-left, top-right, bottom-right, bottom-left
        corner_order = [0, 1, 2, 3]  # ArUco retourne déjà dans le bon ordre
        
        for idx in corner_order:
            corner_pose = Pose()
            
            # Coordonnées en pixels
            x_pixel = corners[idx][0]
            y_pixel = corners[idx][1]
            
            # IMPORTANT: Correction de la normalisation
            # Les coordonnées doivent être centrées sur l'image
            corner_pose.position.x = x_pixel
            corner_pose.position.y = y_pixel
            corner_pose.position.z = 0.0
            
            # Orientation par défaut
            corner_pose.orientation.x = 0.0
            corner_pose.orientation.y = 0.0
            corner_pose.orientation.z = 0.0
            corner_pose.orientation.w = 1.0
            
            pose_array.poses.append(corner_pose)
        
        self.aruco_corner_pub.publish(pose_array)

    def publish_aruco_pose(self, rvec, tvec, marker_id):
        """Publie la pose 3D du marqueur ArUco"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "camera_frame"
        
        # Position (vecteur de translation)
        pose_msg.pose.position.x = tvec[0][0]
        pose_msg.pose.position.y = tvec[0][1]
        pose_msg.pose.position.z = tvec[0][2]
        
        # Orientation (conversion du vecteur de rotation en quaternion)
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        
        # Conversion de la matrice de rotation en quaternion
        # Utilisation de tf pour la conversion
        quaternion = tf.transformations.quaternion_from_matrix(
            np.vstack([np.hstack([rotation_matrix, [[0], [0], [0]]]), [0, 0, 0, 1]]))
        
        pose_msg.pose.orientation.x = quaternion[0]
        pose_msg.pose.orientation.y = quaternion[1]
        pose_msg.pose.orientation.z = quaternion[2]
        pose_msg.pose.orientation.w = quaternion[3]
        
        self.aruco_pose_pub.publish(pose_msg)

    def create_line_marker(self, points, marker_id, r, g, b, ns="curve"):
        """Crée un marqueur de ligne pour RViz"""
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1
        
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 1.0
        
        # Centrer les points
        marker.points = [Point(x=p[0]-self.largeur/2, y=p[1]-self.hauteur/2, z=p[2] if len(p) > 2 else 0.0) 
                        for p in points]
        
        return marker


if __name__ == '__main__':
    try:
        LineAndArucoDetector()
    except rospy.ROSInterruptException:
        rospy.loginfo("Detector node interrupted")
        pass
    cv2.destroyAllWindows()