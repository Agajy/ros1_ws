#!/usr/bin/env python3

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge, CvBridgeError
from math import atan, pi
from std_msgs.msg import Float32
from geometry_msgs.msg import PoseArray, PoseStamped, Pose, Point, Quaternion
from sensor_msgs.msg import Image

class TrajectoryErrorNode:
    def __init__(self):
        rospy.init_node('trajectory_error_node', anonymous=True)
        
        # Paramètres caméra 
        self.focal_x = 314.9937259674355
        self.focal_y = 314.5862387824696
        self.center_x = 146.24621020976434 
        self.center_y = 128.53019044749945
        self.image_width = 320
        self.image_height = 240
        
        # Variables d'état
        self.line_detected = False
        self.current_trajectory = None
        self.current_image = None
        self.bridge = CvBridge()
        
        # Point de référence (centre de l'image)
        self.reference_point = (int(self.image_width/2), int(self.image_height/2))
        
        # Subscribers
        self.line_detected_sub = rospy.Subscriber('/line_detected', Float32, self.line_detected_callback)
        self.line_sub = rospy.Subscriber('/line_path', PoseArray, self.line_path_callback)
        self.image_sub = rospy.Subscriber('/image_tcp_client/image_gray', Image, self.image_callback)
        
        # Publishers
        self.error_pub = rospy.Publisher('/target_pose', PoseStamped, queue_size=10)
        
        # Timer pour le calcul périodique des erreurs
        self.timer = rospy.Timer(rospy.Duration(0.1), self.compute_error_callback)

        self.gain_x = 0.1
        self.gain_y = 0.5
        self.gain_z = 0.5
        
        rospy.loginfo("Trajectory Error Node initialized")

    def line_detected_callback(self, msg):
        """Callback pour la détection de ligne"""
        self.line_detected = (msg.data > 0.0)  # Seuil pour considérer la ligne comme détectée
        
    def line_path_callback(self, msg):
        """Callback pour recevoir la trajectoire"""
        self.current_trajectory = msg
        
    def image_callback(self, msg):
        """Callback pour recevoir l'image (pour visualisation)"""
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "mono8")
        except CvBridgeError as e:
            rospy.logerr(f"Erreur de conversion d'image: {e}")
    
    def pixel_to_camera_coordinates(self, u, v):
        """Convertit les coordonnées pixel en coordonnées caméra normalisées"""
        x_cam = u / self.focal_x
        y_cam = v / self.focal_y
        return x_cam, y_cam
    
    def find_closest_trajectory_point(self, reference_point):
        """Trouve le point de trajectoire le plus proche du point de référence"""
        if not self.current_trajectory or len(self.current_trajectory.poses) == 0:
            return None
            
        min_distance = float('inf')
        closest_point = None
        idx = 0
        min_idx=0
        
        for pose in self.current_trajectory.poses:
            # Convertir la pose 3D en coordonnées image (projection simple)
            # Ici on suppose que les coordonnées de la trajectoire sont déjà en pixels
            # Si elles sont en coordonnées monde, il faudrait appliquer la transformation appropriée
            traj_x = pose.position.x
            traj_y = pose.position.y
            
            # Calculer la distance euclidienne
            distance = np.sqrt((traj_x - reference_point[0])**2 + (traj_y - reference_point[1])**2)
            idx += 1
            if distance < min_distance:
                min_distance = distance
                closest_point = (traj_x, traj_y)
                min_idx = idx
                
        return closest_point, min_idx
    
    def compute_error_callback(self, event):
        """Calcule et publie les erreurs de trajectoire"""
        if not self.line_detected or not self.current_trajectory:
            return
            
        # Trouver le point de trajectoire le plus proche du centre de l'image
        reference_point = (int(self.image_width/2), 0)#(int(self.image_width/2), int(self.image_height/2))

        closest_point,idx = self.find_closest_trajectory_point(reference_point)


        closest_point_m,idx_m = self.find_closest_trajectory_point(self.reference_point)
        
        if closest_point is None:
            return
            
        # Calculer les erreurs en pixels
        error_x_pixels = closest_point[0] - self.reference_point[0]
        error_y_pixels = closest_point[1] - self.reference_point[1]

        error_xm_pixels = closest_point_m[0] - self.reference_point[0]
        error_ym_pixels = closest_point_m[1] - self.reference_point[1]

        if idx_m+1 >= len(self.current_trajectory.poses):
            n_x_pixels = closest_point[0]- self.current_trajectory.poses[idx_m-1].position.x
            n_y_pixels = closest_point[1] - self.current_trajectory.poses[idx_m-1].position.y
        else:
            n_x_pixels = self.current_trajectory.poses[idx_m+1].position.x - closest_point[0]
            n_y_pixels = self.current_trajectory.poses[idx_m+1].position.y - closest_point[1]
 
        
        # Convertir les erreurs en coordonnées caméra normalisées
        error_x_cam, error_y_cam = self.pixel_to_camera_coordinates(
            error_x_pixels,
            error_y_pixels
        )

        error_xm_cam, error_ym_cam = self.pixel_to_camera_coordinates(
            error_xm_pixels,
            error_ym_pixels
        )
        
        # Coordonnées caméra du point de référence
        ref_x_cam, ref_y_cam = self.pixel_to_camera_coordinates(
            self.reference_point[0], self.reference_point[1]
        )

        n_x_pixels, n_y_pixels = self.pixel_to_camera_coordinates(
            n_x_pixels,
            n_y_pixels
        )
        
        
        # Créer le message PoseStamped avec les erreurs
        target_pose = PoseStamped()
        target_pose.header.stamp = rospy.Time.now()
        target_pose.header.frame_id = "camera_frame"

        
        
        # Utiliser les erreurs comme position cible
        target_pose.pose.position.x = -1.0 - error_ym_cam
        target_pose.pose.position.y = error_xm_cam

        
        # Orientation neutre
        target_pose.pose.orientation.x = 0.0
        target_pose.pose.orientation.y = 0.0
        target_pose.pose.orientation.z = np.arctan2( -n_x_pixels, n_y_pixels) #np.arctan2(n_x_pixels, n_y_pixels)
        target_pose.pose.orientation.w = 1.0
        

        # Publier les erreurs
        target_pose = self.publish_errors(target_pose)

        # Log pour debug
        rospy.loginfo_throttle(1.0, f"Erreurs - X: {target_pose.pose.position.x:.4f}, Y: {target_pose.pose.position.y:.4f}")
        
        # Visualisation optionnelle
        self.visualize_error(closest_point, closest_point_m, target_pose.pose.position.x, target_pose.pose.position.y, np.degrees(target_pose.pose.orientation.z))
    

        
      

    def publish_errors(self, target_pose):
        target_pose.header.stamp = rospy.Time.now()
        target_pose.header.frame_id = "camera_frame"

        target_pose.pose.position.x = target_pose.pose.position.x *self.gain_x
        target_pose.pose.position.y = target_pose.pose.position.y *self.gain_y
        target_pose.pose.orientation.z = target_pose.pose.orientation.z *self.gain_z

        if target_pose.pose.position.x>0.05:
            target_pose.pose.position.x = 0.05
        elif target_pose.pose.position.x<-0.05:
            target_pose.pose.position.x = -0.05

        if target_pose.pose.position.y>0.05:
            target_pose.pose.position.y = 0.05
        elif target_pose.pose.position.y<-0.05:
            target_pose.pose.position.y = -0.05

        
        if abs(target_pose.pose.orientation.z)>pi/60:
            if target_pose.pose.orientation.z>0.0:
                target_pose.pose.orientation.z = pi/60
            else:
                target_pose.pose.orientation.z = -pi/60
        
       
        self.error_pub.publish(target_pose)

        return target_pose


    def visualize_error(self, closest_point, closest_point_m, error_x, error_y, angle):
        """Visualise les erreurs sur l'image (optionnel)"""
        if self.current_image is None:
            return
            
        # Créer une copie de l'image pour la visualisation
        vis_image = cv2.cvtColor(self.current_image.copy(), cv2.COLOR_GRAY2BGR)
        
        # Dessiner le point de référence (centre)
        cv2.circle(vis_image, self.reference_point, 5, (0, 255, 0), -1)
        
        # Dessiner le point de trajectoire le plus proche
        if closest_point:
            cv2.circle(vis_image, (int(closest_point[0]), int(closest_point[1])), 5, (255, 0, 0), -1)
            cv2.circle(vis_image, (int(closest_point_m[0]), int(closest_point_m[1])), 5, (0, 0, 255), -1)

            
            # Dessiner la ligne d'erreur
            cv2.line(vis_image, self.reference_point, 
                    (int(closest_point[0]), int(closest_point[1])), (0, 0, 255), 2)
            
            cv2.line(vis_image, self.reference_point, 
                    (int(closest_point_m[0]), int(closest_point_m[1])), (0, 0, 255), 2)
            
            # Afficher les valeurs d'erreur
            error_text = f"Ex: {error_x:.4f}, Ey: {error_y:.4f}"
            angle_error_text =f"Angle: {angle:.4f}"
            cv2.putText(vis_image, error_text, (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(vis_image, angle_error_text, (10, 80), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Afficher l'image (optionnel, peut être commenté en production)
        cv2.imshow("Trajectory Error Visualization", vis_image)
        cv2.waitKey(1)

    def run(self):
        """Boucle principale du nœud"""
        rospy.loginfo("Trajectory Error Node is running...")
        rospy.spin()

if __name__ == '__main__':
    try:
        node = TrajectoryErrorNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Trajectory Error Node stopped.")
    finally:
        cv2.destroyAllWindows()