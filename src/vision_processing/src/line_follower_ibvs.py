#!/usr/bin/env python

"""
======================================================
 Fichier     : line_follower.py
 Auteur      : Aurélien Garreau
 Créé en     : 2025
 Description : Noeud ROS pour suivre une ligne avec IBVS
======================================================
"""
import rospy
import tf
from std_msgs.msg import Float64, Float32, Bool
from geometry_msgs.msg import PoseStamped, Vector3, Pose, Point, PoseArray
from visualization_msgs.msg import Marker
import numpy as np
from math import *
from angles import normalize_angle


class IBVS:
    def __init__(self):
        rospy.init_node('ibvs_line_node', anonymous=True)

        # Subscribers
        self.line_detected_sub = rospy.Subscriber('/line_detected', Float32, self.line_detected_callback)
        self.line_sub = rospy.Subscriber('/line_path', PoseArray, self.line_path_callback)
        
        # Publishers
        self.error_tag = rospy.Publisher('/target_pose_lines', PoseStamped, queue_size=10)

        # État du système
        self.pose_tag_camera = Pose()
        self.curve = None
        self.corner = None
        self.aruco_state = False
        self.init_aruco = False
        self.line_state = False
        self.line_state_past = self.line_state
        self.direction = 0.0
        
        # Paramètres IBVS
        l = 0.15  # Taille du tag ArUco (en mètres)
        z_star = 2.0  
        self.a = l / (2 * z_star)
        
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
        self.current_trajectory = None

        # Paramètres caméra 
        self.focal_x = 314.9937259674355
        self.focal_y = 314.5862387824696
        self.center_x = 146.24621020976434 
        self.center_y = 128.53019044749945
        self.image_width = 320
        self.image_height = 240

        # Gain pour le contrôle
        self.lambda_gain = 1.0
        self.gain_z = 0.5

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
    
    def line_detected_callback(self, data):
        self.line_state_past = self.line_state
        self.line_state = True if data.data > 0 else False

    def pixel_to_camera_coordinates(self, u, v):
        """Convertit les coordonnées pixel en coordonnées caméra normalisées"""
        x_cam = u / self.focal_x
        y_cam = v / self.focal_y
        return x_cam, y_cam

    def control_loop(self, event):
        """Boucle de contrôle principale"""
        if not self.line_state:
            return

        try:
            # Calculer la commande de contrôle
            control_command = self.compute_control_command()
            
            if control_command is not None:
                # Créer et publier le message de commande
                command_msg = PoseStamped()
                command_msg.header.stamp = rospy.Time.now()
                command_msg.header.frame_id = "drone_frame"

                command_msg.pose.position.x = control_command.position.x
                command_msg.pose.position.y = control_command.position.y
                command_msg.pose.orientation.z = self.gain_z  * control_command.orientation.z

                command_msg.pose.position.x = np.clip(command_msg.pose.position.x, -0.1, 0.1)
                command_msg.pose.position.y = np.clip(command_msg.pose.position.y, -0.1, 0.1)
                command_msg.pose.orientation.z = np.clip(command_msg.pose.orientation.z, -pi/60, pi/60)
                
                self.error_tag.publish(command_msg)
                
                # Afficher les commandes dans les logs
                rospy.loginfo(f"Commande publiée: x={control_command.position.x:.3f}, y={control_command.position.y:.3f}, yaw={control_command.orientation.z:.3f}")
                
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

        
        # Commande d'orientation
        reference_point = (int(self.image_width/2), int(self.image_height/2))
        closest_point,idx = self.find_closest_trajectory_point(reference_point)

        if idx+1 >= len(self.current_trajectory.poses):
            n_x_pixels = closest_point[0]- self.current_trajectory.poses[idx-1].position.x
            n_y_pixels = closest_point[1] - self.current_trajectory.poses[idx-1].position.y
        else:
            n_x_pixels = self.current_trajectory.poses[idx+1].position.x - closest_point[0]
            n_y_pixels = self.current_trajectory.poses[idx+1].position.y - closest_point[1]

        n_x_pixels, n_y_pixels = self.pixel_to_camera_coordinates(
            n_x_pixels,
            n_y_pixels
        )

        drone_command.orientation.z = np.arctan2( -n_x_pixels, n_y_pixels) #np.arctan2(n_x_pixels, n_y_pixels)
        
        return drone_command
    
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
    
    def line_path_callback(self, data):
        
        if not self.line_state:
            self.current_trajectory = None
            return
        self.current_trajectory = data
        
        # Calcul du point appartenant à la courbe le plus proche du centre de l'image
        list_position = [(data.poses[i].position.x, data.poses[i].position.y) for i in range(len(data.poses))]

        # CORRECTION: Cohérence x/y avec width/height
        center_x_img = self.image_width / 2
        center_y_img = self.image_height / 2
        
        list_dist_middle_position = [sqrt((center_x_img - list_position[i][0])**2 + 
                                        (center_y_img - list_position[i][1])**2) 
                                    for i in range(len(list_position))]
        id_cm_point = np.argmin(list_dist_middle_position)
        cm_point = list_position[id_cm_point]

        # Choix de la direction du sens de suivi de trajectoire
        if not self.line_state_past:
            # Point en bas au milieu de l'image pour référence
            pmb = (center_x_img, self.image_height)  # CORRECTION: x, y cohérents
            
            # Calculer les distances correctement
            dist_to_first = sqrt((pmb[0] - list_position[0][0])**2 + (pmb[1] - list_position[0][1])**2)
            dist_to_last = sqrt((pmb[0] - list_position[-1][0])**2 + (pmb[1] - list_position[-1][1])**2)
            
            # Choisir la direction initiale
            if dist_to_first > dist_to_last:
                # Suivre dans le sens normal
                if id_cm_point + 1 < len(list_position):
                    dx = list_position[id_cm_point + 1][0] - list_position[id_cm_point][0]
                    dy = list_position[id_cm_point + 1][1] - list_position[id_cm_point][1]
                    self.direction = atan2(dy, dx)
            else:
                # Inverser la liste et suivre
                list_position = list_position[::-1]
                list_dist_middle_position = [sqrt((center_x_img - list_position[i][0])**2 + 
                                                (center_y_img - list_position[i][1])**2) 
                                            for i in range(len(list_position))]
                id_cm_point = np.argmin(list_dist_middle_position)
                if id_cm_point + 1 < len(list_position):
                    dx = list_position[id_cm_point + 1][0] - list_position[id_cm_point][0]
                    dy = list_position[id_cm_point + 1][1] - list_position[id_cm_point][1]
                    self.direction = atan2(dy, dx)
        else:
            # Maintenir la continuité de direction
            if id_cm_point + 1 < len(list_position) and id_cm_point - 1 >= 0:
                # Calculer les deux directions possibles
                dx_forward = list_position[id_cm_point + 1][0] - list_position[id_cm_point][0]
                dy_forward = list_position[id_cm_point + 1][1] - list_position[id_cm_point][1]
                angle_forward = normalize_angle(atan2(dy_forward, dx_forward))
                
                dx_backward = list_position[id_cm_point - 1][0] - list_position[id_cm_point][0]
                dy_backward = list_position[id_cm_point - 1][1] - list_position[id_cm_point][1]
                angle_backward = normalize_angle(atan2(dy_backward, dx_backward))
                
                # Choisir la direction la plus cohérente avec la direction précédente
                diff_forward = abs(normalize_angle(angle_forward - self.direction))
                diff_backward = abs(normalize_angle(angle_backward - self.direction))
                
                if diff_forward < diff_backward:
                    self.direction = angle_forward
                else:
                    # Inverser la liste si nécessaire
                    list_position = list_position[::-1]
                    list_dist_middle_position = [sqrt((center_x_img - list_position[i][0])**2 + 
                                                    (center_y_img - list_position[i][1])**2) 
                                                for i in range(len(list_position))]
                    id_cm_point = np.argmin(list_dist_middle_position)
                    if id_cm_point + 1 < len(list_position):
                        dx = list_position[id_cm_point + 1][0] - list_position[id_cm_point][0]
                        dy = list_position[id_cm_point + 1][1] - list_position[id_cm_point][1]
                        self.direction = atan2(dy, dx)

        # Calcul du point désiré
        idx_offset = 20
        if id_cm_point + idx_offset >= len(list_position):
            idx_offset = len(list_position) - id_cm_point - 1
            if idx_offset <= 0:
                rospy.logwarn("Pas assez de points pour calculer le point désiré")
                return
        
        tag_desired = list_position[id_cm_point + idx_offset]
        tag_x, tag_y = tag_desired[0], tag_desired[1]
        
        # CORRECTION: Création correcte du carré virtuel orienté
        # Vecteurs perpendiculaires pour créer le carré
        cos_dir = cos(self.direction)
        sin_dir = sin(self.direction)
        
        # Vecteur tangent (direction de la ligne)
        tangent_x = cos_dir
        tangent_y = sin_dir
        
        # Vecteur normal (perpendiculaire à la ligne)
        normal_x = -sin_dir
        normal_y = cos_dir
        
        # Créer un carré orienté selon la direction de la ligne
        half_size = self.a
        
        # Les 4 coins du carré virtuel (dans l'ordre: TL, TR, BR, BL)
        point1 = (tag_x + normal_x * half_size - tangent_x * half_size,  # Top-left
                tag_y + normal_y * half_size - tangent_y * half_size)
        point2 = (tag_x + normal_x * half_size + tangent_x * half_size,  # Top-right  
                tag_y + normal_y * half_size + tangent_y * half_size)
        point3 = (tag_x - normal_x * half_size + tangent_x * half_size,  # Bottom-right
                tag_y - normal_y * half_size + tangent_y * half_size)
        point4 = (tag_x - normal_x * half_size - tangent_x * half_size,  # Bottom-left
                tag_y - normal_y * half_size - tangent_y * half_size)

        # Ordre cohérent avec la matrice d'interaction
        list_position = [point1, point2, point3, point4]
        
        self._s_matrix = np.zeros((8, 1), dtype=np.float32)

        # Conversion des coordonnées pixel en coordonnées normalisées métriques
        for i in range(4):
            x_pixel, y_pixel = list_position[i][0], list_position[i][1]
            
            # Convertir en coordonnées normalisées métriques (repère caméra)
            x_norm = (x_pixel - self.center_x) / self.focal_x
            y_norm = (y_pixel - self.center_y) / self.focal_y
            
            self._s_matrix[i, 0] = x_norm      # x des coins
            self._s_matrix[i+4, 0] = y_norm    # y des coins


    def transform_camera_to_drone(self, velocity_camera):
        """Transforme les vitesses du repère caméra vers le repère drone"""
        # CORRECTION: Ajustement de la transformation selon votre configuration
        camera_to_drone = np.array([
            [0, 1, 0, -0.06],
            [-1, 0, 0, 0], 
            [0, 0, 1, 0.08],
            [0, 0, 0, 1]
        ], dtype=np.float32)

        translation_velocities = camera_to_drone[:3, :3] @ velocity_camera[:3].flatten()
        
        command_pose = Pose()
        command_pose.position.x = translation_velocities[0]
        command_pose.position.y = translation_velocities[1]
        command_pose.position.z = translation_velocities[2]
        
        return command_pose
    



if __name__ == '__main__':
    try:
        IBVS()
    except rospy.ROSInterruptException:
        rospy.loginfo("IBVS node interrupted")
        pass