#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Point
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
        self.image_sub = rospy.Subscriber('/image_tcp_client/image_gray', Image, self.image_callback)
        self.original_pub = rospy.Publisher('/original_curve', Marker, queue_size=10)
        self.filtered_pub = rospy.Publisher('/filtered_curve', Marker, queue_size=10)
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)
        self.aruco_params = aruco.DetectorParameters()
        rospy.loginfo("Line and ArUco detector initialized.")
        rospy.spin()
    
    def image_callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        except Exception as e:
            rospy.logerr("CV Bridge error: %s", e)
            return
        
        # Create a color version of the image for drawing
        cv_lines = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
        
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
            rospy.loginfo(f"Ligne {nombre_lignes} : {len(skeleton_coords)} points")
            
            # Affichage des points
            coords = []
            for j, i in skeleton_coords:
                cv_lines[y + j, x + i] = (0, 0, 255)
                coords.append((x + i, y + j))
            
            all_skeleton_points.append(coords)
        
        rospy.loginfo(f"Nombre de lignes détectées : {nombre_lignes}")
        
        # === Calcul de la ligne centrale entre les deux plus grandes courbes ===
        if len(all_skeleton_points) >= 2:
            # Trier les squelettes par taille (du plus grand au plus petit)
            sorted_skeletons = sorted(all_skeleton_points, key=len, reverse=True)
            
            # Prendre les deux plus grandes courbes
            skeleton_1 = np.array(sorted_skeletons[0])
            skeleton_2 = np.array(sorted_skeletons[1])
            
            # Vérifier que les deux squelettes sont valides
            if len(skeleton_1) > 0 and len(skeleton_2) > 0:

                itérationmin = min(len(skeleton_1), len(skeleton_2))
                               
                central_curve_points = []
                
                for idx in range(1, itérationmin):
                    # Calculer le point médian entre les deux courbes
                    x_center = ((skeleton_1[-idx,0] + skeleton_2[-idx,0]) / 2)
                    y_center = ((skeleton_1[-idx,1] + skeleton_2[-idx,1]) / 2)
                    central_curve_points.append((x_center, y_center,0.0))
                
                # Courbe filtrée
                if central_curve_points:
                    central_curve = np.array(central_curve_points)
                    # window_size = len(skeleton_1)/5 
                    # poly_order = 5
                    # filtered_curve = self.filter_curve(central_curve, window_size, poly_order)
                    
                    # Créer et publier les marqueurs
                    original_marker = self.create_line_marker(central_curve, 0, 1.0, 0.0, 0.0, "original_curve")
                    self.original_pub.publish(original_marker)
        
                    # filtered_marker = self.create_line_marker(filtered_curve, 0, 0.0, 0.0, 1.0, "filtered_curve")
                    # self.filtered_pub.publish(filtered_marker)
       
        # Détection ArUco
        corners, ids, _ = aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.aruco_params)
        
        if ids is not None:
            aruco.drawDetectedMarkers(cv_lines, corners, ids)
        
        cv2.imshow("Lines and ArUco Detection", cv_lines)
        cv2.waitKey(1)

    def create_line_marker(self,points, marker_id, r, g, b, ns="curve"):
        """Crée un marqueur de ligne pour RViz."""
        marker = Marker()
        marker.header.frame_id = "map"  # Ajustez selon votre système de coordonnées
        marker.header.stamp = rospy.Time.now()
        
        marker.ns = ns
        marker.id = marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.1  # Épaisseur de la ligne
        
        marker.color.r = r
        marker.color.g = g
        marker.color.b = b
        marker.color.a = 1.0
        
        # Convertir les points en messages Point
        marker.points = [Point(x=p[0], y=p[1], z=p[2] if len(p) > 2 else 0.0) for p in points]
        
        return marker

    # def filter_curve(self,central_curve, window_size=11, poly_order=3):
    #     """
    #     Filtre une courbe en utilisant le filtre Savitzky-Golay.
        
    #     Args:
    #         central_curve: Liste ou tableau de points [x, y, z]
    #         window_size: Taille de la fenêtre pour le filtre (doit être impair)
    #         poly_order: Ordre du polynôme pour l'ajustement
        
    #     Returns:
    #         Tableau numpy de points filtrés
    #     """
    #     # Conversion en tableau numpy si ce n'est pas déjà fait
    #     curve = np.array(central_curve)
        
    #     # Vérifier la dimension des points
    #     dim = curve.shape[1]
        
    #     # Filtrer chaque dimension séparément
    #     filtered_curve = np.zeros_like(curve)
    #     for i in range(dim):
    #         filtered_curve[:, i] = savgol_filter(curve[:, i], window_size, poly_order)
        
    #     return filtered_curve


if __name__ == '__main__':
    try:
        LineAndArucoDetector()
    except rospy.ROSInterruptException:
        pass
    cv2.destroyAllWindows()