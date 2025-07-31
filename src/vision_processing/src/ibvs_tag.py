#!/usr/bin/env python

"""
======================================================
 Fichier     : ibvs_tag.py
 Auteur      : Aurélien Garreau
 Créé en     : 2025
 Description : Noeud ROS pour suivre un tag ArUco avec IBVS
======================================================
"""
import rospy
import tf
from std_msgs.msg import Float64, Float32, Bool
from geometry_msgs.msg import PoseStamped, Vector3, Pose, Point, PoseArray
from visualization_msgs.msg import Marker
import numpy as np
from math import *


class IBVS:
    def __init__(self):
        rospy.init_node('ibvs_tag_node', anonymous=True)

        # Subscribers
        self.original_sub = rospy.Subscriber('/original_curve', Marker, self.curve_callback)
        self.aruco_pose_sub = rospy.Subscriber('/aruco_pose', PoseStamped, self.aruco_pose_callback)
        self.aruco_detected_sub = rospy.Subscriber('/aruco_detected', Bool, self.aruco_detected_callback)
        self.aruco_corner_sub = rospy.Subscriber('/aruco_corners', PoseArray, self.aruco_corner_callback)
        
        # Publishers
        self.error_tag = rospy.Publisher('/target_pose_aruco', PoseStamped, queue_size=10)

        # État du système
        self.pose_tag_camera = Pose()
        self.curve = None
        self.corner = None
        self.aruco_state = False
        self.init_aruco = False
        
        # Paramètres IBVS
        l = 0.15  # Taille du tag ArUco (en mètres)
        z_star = 2.0  
        a = l / (2 * z_star)
        
        # Matrice d'interaction et positions désirées
        self._L_matrix = self.compute_C_matrix(l, z_star)
        
        # Positions désirées des 4 coins dans l'image (coordonnées normalisées)
        # Tag centré dans l'image avec une taille apparente correspondant à z_star
        desired_size = l / z_star  # Taille apparente désirée
        half_size = desired_size / 2.0
        
        # Coins désirés: top-left, top-right, bottom-right, bottom-left
        self._s_star_matrix = np.array([
            [-half_size, half_size, half_size, -half_size, half_size, half_size, -half_size, -half_size]    
        ], dtype=np.float32).T  # (8,1)
        self._s_matrix = np.zeros((8, 1), dtype=np.float32)

        # Paramètres caméra 
        self.focal_x = 314.9937259674355
        self.focal_y = 314.5862387824696
        self.center_x = 146.24621020976434 
        self.center_y = 128.53019044749945
        self.image_width = 640
        self.image_height = 480

        # Gain pour le contrôle
        self.lambda_gain = 1.0

        self.yaw = 0.0  # Initialisation de l'angle de lacet

        # Timer pour le contrôle
        rospy.Timer(rospy.Duration(0.1), self.control_loop)

        rospy.loginfo("IBVS controller initialized.")
        rospy.spin()

    def compute_C_matrix(self, l, z_star):
        """Calcule la matrice d'interaction C pour IBVS avec 4 points"""
        # Pour un tag ArUco carré avec 4 coins
        # Coordonnées des coins dans le repère du tag (en mètres)
        half_size = l / 2.0
        
        # Coins du tag: top-left, top-right, bottom-right, bottom-left
        X = np.array([
            [-half_size, -half_size, half_size, half_size],  # coordonnées X
            [half_size, -half_size, -half_size, half_size],  # coordonnées Y  
            [0, 0, 0, 0]  # coordonnées Z (tag plan)
        ])
        
        # Matrice d'interaction pour 4 points (8x6)
        L = np.zeros((8, 6), dtype=np.float32)
        
        for i in range(4):
            # Coordonnées 3D du point i
            x, y, z = X[0, i], X[1, i], X[2, i]
            
            # Approximation: on suppose que z ≈ z_star (distance désirée)
            Z = z_star
            
            # Matrice d'interaction pour le point i
            # Ligne pour coordonnée x du point i
            L[i, 0] = -1.0 / Z                   
            L[i, 1] = 0.0                         
            L[i, 2] = x / Z                       
            L[i, 3] = x * y                       
            L[i, 4] = -(1 + x*x)                 
            L[i, 5] = y                           
            
            # Ligne pour coordonnée y du point i
            L[i+4, 0] = 0.0                       
            L[i+4, 1] = -1.0 / Z                  
            L[i+4, 2] = y / Z                      
            L[i+4, 3] = 1 + y*y                   
            L[i+4, 4] = -x * y                    
            L[i+4, 5] = -x                        
        
        return L
    
    def aruco_detected_callback(self, data):
        """Callback pour l'état de détection ArUco"""
        self.aruco_state = data.data
        if self.aruco_state:
            self.init_aruco = True
        else:
            self.init_aruco = False

    def aruco_corner_callback(self, data):
        """Callback pour les coins du tag ArUco"""
        if len(data.poses) < 4:
            rospy.logwarn("Nombre insuffisant de coins détectés")
            return
            
        self._s_matrix = np.zeros((8, 1), dtype=np.float32)
        
        self.yaw = data.poses[0].orientation.z  # Récupérer l'orientation du tag ArUco

        # Conversion des coordonnées pixel en coordonnées normalisées métriques
        for i in range(4):
            # Récupérer les coordonnées en pixels
            x_pixel = data.poses[i].position.x
            y_pixel = data.poses[i].position.y
            
            # Convertir en coordonnées normalisées métriques (repère caméra)
            # Formule: (pixel - centre) / focale
            x_norm = (x_pixel - self.center_x) / self.focal_x
            y_norm = (y_pixel - self.center_y) / self.focal_y
            
            self._s_matrix[i, 0] = x_norm      # x des coins
            self._s_matrix[i+4, 0] = y_norm    # y des coins
        
        rospy.logdebug(f"Coins normalisés: x={self._s_matrix[:4].flatten()}, y={self._s_matrix[4:].flatten()}")
        rospy.loginfo(f"ArUco coins détectés: {len(data.poses)} coins")

    def aruco_pose_callback(self, data):
        """Callback pour la pose du tag ArUco"""
        self.pose_tag_camera = data.pose

    def curve_callback(self, data):
        """Callback pour la courbe détectée"""
        self.curve = np.array([[point.x, point.y, point.z] for point in data.points])

    def control_loop(self, event):
        """Boucle de contrôle principale"""
        if not self.init_aruco:
            return

        try:
            # Calculer la commande de contrôle
            control_command = self.compute_control_command()
            
            if control_command is not None:
                # Créer et publier le message de commande
                command_msg = PoseStamped()
                command_msg.header.stamp = rospy.Time.now()
                command_msg.header.frame_id = "drone_frame"
                command_msg.pose = control_command
                
                self.error_tag.publish(command_msg)
                
                # Debug
                rospy.loginfo_throttle(1.0, f"Commande publiée: x={control_command.position.x:.3f}, "
                                      f"y={control_command.position.y:.3f}, z={control_command.position.z:.3f}")
                
        except Exception as e:
            rospy.logerr(f"Erreur dans la boucle de contrôle: {e}")

    def compute_control_command(self):
        """Calcule la commande de contrôle IBVS"""
        if np.all(self._s_matrix == 0):
            rospy.logwarn("Matrice s vide, pas de commande générée")
            return None
            
        # Calculer l'erreur visuelle
        error = self._s_matrix - self._s_star_matrix
        
        # Vérifier les dimensions
        rospy.logdebug(f"Dimensions - L_matrix: {self._L_matrix.shape}, error: {error.shape}")
        
        # Calculer la commande dans le repère caméra
        # velocity_camera = -lambda * L^+ * error
        # où L^+ est la pseudo-inverse de L
        L_pinv = np.linalg.pinv(self._L_matrix)
        velocity_camera = -self.lambda_gain * (L_pinv @ error)
        
        rospy.logdebug(f"Erreur visuelle: {error.flatten()}")
        rospy.logdebug(f"Vitesse caméra: {velocity_camera.flatten()}")
        
        # Transformer dans le repère drone
        drone_command = self.transform_camera_to_drone(velocity_camera)
        
        return drone_command

    def transform_camera_to_drone(self, velocity_camera):
        """Transforme les vitesses du repère caméra vers le repère drone"""
        # Matrice de transformation caméra vers drone
        # Ajustez cette matrice selon l'orientation de votre caméra sur le drone
        camera_to_drone = np.array([
            [0, 1, 0, -0.06],
            [-1, 0, 0, 0], 
            [0, 0, 1, 0.08],
            [0, 0, 0, 1]
        ], dtype=np.float32)

        # Extraire la matrice de rotation
        rotation_matrix = camera_to_drone[:3, :3]

        # Transformer les vitesses de translation
        translation_velocities = rotation_matrix @ velocity_camera[:3].flatten()
        
        # Transformer les vitesses de rotation
        rotation_velocities = rotation_matrix @ velocity_camera[3:].flatten()

        # Créer le message de pose
        command_pose = Pose()
        
        # Les vitesses deviennent des positions relatives
        command_pose.position.x = translation_velocities[0]
        command_pose.position.y = translation_velocities[1]
        command_pose.position.z = translation_velocities[2]
        
        # Convertir les vitesses angulaires en quaternion
        # Note: Pour de petites rotations, on peut approximer
        roll = rotation_velocities[0]
        pitch = rotation_velocities[1] 
        yaw = rotation_velocities[2]
        
        # Limiter les rotations pour éviter les instabilités
        max_rotation = 0.1  # radians
        # roll = np.clip(roll, -max_rotation, max_rotation)
        # pitch = np.clip(pitch, -max_rotation, max_rotation)
        # yaw = np.clip(yaw, -max_rotation, max_rotation)
        
        q = tf.transformations.quaternion_from_euler(roll, pitch, yaw)
        command_pose.orientation.x = 0.0
        command_pose.orientation.y = 0.0
        command_pose.orientation.z = yaw
        command_pose.orientation.w = 0.0

        return command_pose


if __name__ == '__main__':
    try:
        IBVS()
    except rospy.ROSInterruptException:
        rospy.loginfo("IBVS node interrupted")
        pass