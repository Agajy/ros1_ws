#!/usr/bin/env python
import math
import numpy
from excel_reader_class import ExcelReader
from math import *
import rospy
import tf
from std_msgs.msg import Float64, Float32, Bool
from geometry_msgs.msg import PoseStamped, Vector3, Pose
import tf.transformations
import socket
import sys
import time
import threading
import tkinter as tk
from tkinter import ttk
from tkinter import messagebox

class Decision(object):
    def __init__(self):
        rospy.init_node("decision")
        self.object_uav = rospy.get_param('~uav')
        self.object_ugv = rospy.get_param('~ugv')
        self.in_simu = rospy.get_param('~in_simu')

        self._state_uav = False
        self._state_ugv = False

        # Modes de contrôle
        self.uav_mode = "stop"  # "follow_lines", "follow_aruco", "follow_path_optitrack", "stop"
        self.ugv_mode = "stop"  # "follow_path_optitrack", "controlled_by_image", "stop"
        self.uav_ugv_mode = "stop"  # "follow_path_optitrack", "stop"

        # Subscribers existants
        self._sub_vrpn_uav = rospy.Subscriber(f"/vrpn_client_node/{self.object_uav}/pose", PoseStamped, self.vrpn_client_uav_callback, queue_size=1)
        self._sub_vrpn_ugv = rospy.Subscriber(f"/vrpn_client_node/{self.object_ugv}/pose", PoseStamped, self.vrpn_client_ugv_callback, queue_size=1)

        self._sub_state_uav = rospy.Subscriber(f"/state/{self.object_uav}", Bool, self.state_uav_callback, queue_size=1)

        if self.in_simu:
            self._sub_state_ugv = rospy.Subscriber(f"/state/{self.object_ugv}", Bool, self.state_ugv_callback, queue_size=1)
        else:
            self._state_ugv = True 

        # Publishers
        self._pub_error_uav = rospy.Publisher(f"/error_pose/{self.object_uav}/pose", PoseStamped, queue_size=1)
        self._pub_error_ugv = rospy.Publisher(f"/error_pose/{self.object_ugv}/pose", PoseStamped, queue_size=1)
        self.no_optitrack_pub = rospy.Publisher('/no_optitrack', Bool, queue_size=1)


        # Sub pour les poses cibles
        self.target_pose_lines_sub = rospy.Subscriber('/target_pose_lines', PoseStamped, self.target_pose_lines_callback, queue_size=1)
        self.target_pose_aruco_sub = rospy.Subscriber('/target_pose_aruco', PoseStamped, self.target_pose_aruco_callback, queue_size=1)
        self.target_pose_ugv_sub = rospy.Subscriber('/target_pose_ugv', PoseStamped, self.target_pose_ugv_callback, queue_size=1)

        # Sub pour les détections
        self.aruco_detected_sub = rospy.Subscriber('/aruco_detected', Bool, self.aruco_detected_callback, queue_size=1)
        self.line_detected_sub = rospy.Subscriber('/line_detected', Float32, self.line_detected_callback)       

        # Référence de trajectoire
        path_excel_file = rospy.get_param('path_excel_file')
        self.reference = ExcelReader.extract_data_excel_to_array(path_excel_file).T

        # Poses
        self._pose_uav = Pose()
        self._pose_ugv = Pose()
        self._target_pose_lines = Pose()
        self._target_pose_aruco = Pose()
        self._target_pose_ugv = Pose()

        # Variables de contrôle
        self.detection_aruco = False
        self.line_state = False
        self.way_init_uav = False
        self.way_init_ugv = False

        self._k_uav = 0
        self._k_ugv = 0
        
        # Variable pour stocker le point désiré de l'UAV
        self._uav_desired_pose = Pose()
        
        # Timer pour la logique principale
        rospy.Timer(rospy.Duration(1.0/40), self.logic)

        # Créer l'interface graphique
        self.gui_thread = threading.Thread(target=self.create_and_run_gui)
        self.gui_thread.daemon = True
        self.gui_thread.start()

        
        rospy.loginfo("decision.py : decision initialized with GUI")

    def create_gui(self):
        """Créer l'interface graphique"""
        self.root = tk.Tk()
        self.root.title("Contrôle UAV/UGV")
        self.root.geometry("800x1200")
        
        # Frame pour UAV
        uav_frame = ttk.LabelFrame(self.root, text="Contrôle UAV", padding="10")
        uav_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")
        
        self.uav_mode_var = tk.StringVar(value="stop")
        ttk.Radiobutton(uav_frame, text="Follow Lines", variable=self.uav_mode_var, 
                       value="follow_lines", command=self.update_uav_mode).grid(row=0, column=0, sticky="w")
        ttk.Radiobutton(uav_frame, text="Follow ArUco", variable=self.uav_mode_var, 
                       value="follow_aruco", command=self.update_uav_mode).grid(row=1, column=0, sticky="w")
        ttk.Radiobutton(uav_frame, text="Follow Path OptiTrack", variable=self.uav_mode_var, 
                       value="follow_path_optitrack", command=self.update_uav_mode).grid(row=2, column=0, sticky="w")
        ttk.Radiobutton(uav_frame, text="Stop", variable=self.uav_mode_var, 
                       value="stop", command=self.update_uav_mode).grid(row=3, column=0, sticky="w")
        
        # Frame pour UGV
        ugv_frame = ttk.LabelFrame(self.root, text="Contrôle UGV", padding="10")
        ugv_frame.grid(row=1, column=0, padx=10, pady=10, sticky="nsew")
        
        self.ugv_mode_var = tk.StringVar(value="stop")
        ttk.Radiobutton(ugv_frame, text="Follow Path OptiTrack", variable=self.ugv_mode_var, 
                       value="follow_path_optitrack", command=self.update_ugv_mode).grid(row=0, column=0, sticky="w")
        ttk.Radiobutton(ugv_frame, text="Controlled by Image", variable=self.ugv_mode_var, 
                       value="controlled_by_image", command=self.update_ugv_mode).grid(row=1, column=0, sticky="w")
        ttk.Radiobutton(ugv_frame, text="Stop", variable=self.ugv_mode_var, 
                       value="stop", command=self.update_ugv_mode).grid(row=2, column=0, sticky="w")
        
        # Frame pour UAV-UGV combiné
        combined_frame = ttk.LabelFrame(self.root, text="Contrôle UAV-UGV", padding="10")
        combined_frame.grid(row=2, column=0, padx=10, pady=10, sticky="nsew")
        
        self.combined_mode_var = tk.StringVar(value="stop")
        ttk.Radiobutton(combined_frame, text="UGV suit position actuelle UAV", variable=self.combined_mode_var, 
                       value="follow_uav_current", command=self.update_combined_mode).grid(row=0, column=0, sticky="w")
        ttk.Radiobutton(combined_frame, text="UGV suit point désiré UAV", variable=self.combined_mode_var, 
                       value="follow_uav_desired", command=self.update_combined_mode).grid(row=1, column=0, sticky="w")
        ttk.Radiobutton(combined_frame, text="Stop", variable=self.combined_mode_var, 
                       value="stop", command=self.update_combined_mode).grid(row=2, column=0, sticky="w")
        
        # Frame pour les status
        status_frame = ttk.LabelFrame(self.root, text="Status", padding="10")
        status_frame.grid(row=3, column=0, padx=10, pady=10, sticky="nsew", columnspan=2)
        
        self.status_text = tk.Text(status_frame, height=8, width=70)
        self.status_text.grid(row=0, column=0)
        
        scrollbar = ttk.Scrollbar(status_frame, orient="vertical", command=self.status_text.yview)
        scrollbar.grid(row=0, column=1, sticky="ns")
        self.status_text.configure(yscrollcommand=scrollbar.set)
        
        # Bouton d'urgence
        emergency_button = tk.Button(self.root, text="ARRÊT D'URGENCE", bg="red", fg="white", 
                                   font=("Arial", 14, "bold"), command=self.emergency_stop)
        emergency_button.grid(row=4, column=0, padx=10, pady=10, sticky="ew")
        
        # Configuration de la grille
        self.root.columnconfigure(0, weight=1)
        for i in range(5):
            self.root.rowconfigure(i, weight=1)
    
    def create_and_run_gui(self):
        self.create_gui()
        self.root.mainloop()  # boucle d’événements dédiée


    def update_status(self, message):
        """Mettre à jour le status dans l'interface"""
        self.status_text.insert(tk.END, f"{rospy.Time.now()}: {message}\n")
        self.status_text.see(tk.END)

    def update_uav_mode(self):
        """Mettre à jour le mode UAV"""
        self.uav_mode = self.uav_mode_var.get()
        self.way_init_uav = False
        self._k_uav = 0
        self.update_status(f"UAV mode changed to: {self.uav_mode}")

    def update_ugv_mode(self):
        """Mettre à jour le mode UGV"""
        self.ugv_mode = self.ugv_mode_var.get()
        self.way_init_ugv = False
        self._k_ugv = 0
        self.update_status(f"UGV mode changed to: {self.ugv_mode}")

    def update_combined_mode(self):
        """Mettre à jour le mode combiné"""
        self.uav_ugv_mode = self.combined_mode_var.get()
        self.update_status(f"Combined mode changed to: {self.uav_ugv_mode}")

    def emergency_stop(self):
        """Arrêt d'urgence"""
        self.uav_mode = "stop"
        self.ugv_mode = "stop"
        self.uav_ugv_mode = "stop"
        self.uav_mode_var.set("stop")
        self.ugv_mode_var.set("stop")
        self.combined_mode_var.set("stop")
        self.update_status("EMERGENCY STOP ACTIVATED!")

    def state_uav_callback(self, msg):
        self._state_uav = msg.data
        if not msg.data:
            self.way_init_uav = False

    def state_ugv_callback(self, msg):
        self._state_ugv = msg.data


    def euler_from_quaternion(self, x, y, z, w):
        """Convert a quaternion into euler angles (roll, pitch, yaw)"""
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
     
        return roll_x, pitch_y, yaw_z

    def nearest_point(self, point, reference):
        """Find the nearest point in the reference trajectory to the given point"""
        min_dist = float('inf')
        nearest_index = -1
        for i in range(len(reference[0])):
            dist = sqrt((point.position.x - reference[0][i])**2 + (point.position.y - reference[1][i])**2)
            if dist < min_dist:
                min_dist = dist
                nearest_index = i
        return nearest_index





    def target_pose_lines_callback(self, msg):
        self._target_pose_lines = msg.pose



    def target_pose_aruco_callback(self, msg):
        self._target_pose_aruco = msg.pose



    def target_pose_ugv_callback(self, msg):
        self._target_pose_ugv = msg.pose
        self._target_pose_ugv.position.x= -msg.pose.position.z
        self._target_pose_ugv.position.y= msg.pose.position.x
        self._target_pose_ugv.position.z= -msg.pose.position.y
        self._target_pose_ugv.orientation.x = -msg.pose.orientation.z
        self._target_pose_ugv.orientation.y = msg.pose.orientation.x
        self._target_pose_ugv.orientation.z = -msg.pose.orientation.y
        self._target_pose_ugv.orientation.w = msg.pose.orientation.w



    def line_detected_callback(self, data):
        self.line_state = True if data.data > 0 else False



    def aruco_detected_callback(self, msg):
        self.detection_aruco = msg.data



    def vrpn_client_uav_callback(self, msg):
        self._pose_uav = msg.pose

    def vrpn_client_ugv_callback(self, msg):
        self._pose_ugv = msg.pose


    def create_pose_from_reference(self, k, vehicle_pose):
        """Créer une pose à partir de la référence"""
        pose = Pose()
        next_k = (k + 1) % len(self.reference[0, :])
        pose.position.x = self.reference[0, k]
        pose.position.y = self.reference[1, k]
        pose.position.z = self.reference[2, k]

        # Calculate orientation based on the next point
        dx = self.reference[0, next_k] - self.reference[0, k]
        dy = self.reference[1, next_k] - self.reference[1, k]
        yaw = math.atan2(dy, dx)
        yaw_vehicle = self.euler_from_quaternion(vehicle_pose.orientation.x,
                                               vehicle_pose.orientation.y,
                                               vehicle_pose.orientation.z,
                                               vehicle_pose.orientation.w)[2]
        
        # Handle yaw wrapping
        if ((yaw_vehicle >= -math.pi) and (yaw_vehicle <= -math.pi/2) and (yaw <= math.pi) and (yaw >= math.pi/2)):
            yaw -= 2*math.pi
        if ((yaw >= -math.pi) and (yaw <= -math.pi/2) and (yaw_vehicle <= math.pi) and (yaw_vehicle >= math.pi/2)):
            yaw += 2*math.pi
        
        pose.orientation.z = yaw

        return pose
    

    def create_error_pose(self, current_pose, target_pose, target_yaw=None):
        """Créer un message d'erreur de pose"""
        error_pose = PoseStamped()
        error_pose.header.stamp = rospy.Time.now()
        error_pose.header.frame_id = "map"
        error_pose.pose.position.x = (current_pose.position.x - target_pose.position.x)
        error_pose.pose.position.y = (current_pose.position.y - target_pose.position.y)
        error_pose.pose.position.z = (current_pose.position.z - target_pose.position.z)

        rospy.loginfo(f"Current Pose: {current_pose.position.x}, {current_pose.position.y}, {current_pose.position.z}")
        rospy.loginfo(f"Target Pose: {target_pose.position.x}, {target_pose.position.y}, {target_pose.position.z}")

        if target_yaw is not None:
            error_pose.pose.orientation.x = 0.0
            error_pose.pose.orientation.y = 0.0
            error_pose.pose.orientation.z = target_yaw
            error_pose.pose.orientation.w = 0.0
        else:
            yaw_vehicle = self.euler_from_quaternion(current_pose.orientation.x,
                                        -current_pose.orientation.y,
                                        current_pose.orientation.z,
                                        current_pose.orientation.w)[2]
        

            error_pose.pose.orientation.x = 0.0
            error_pose.pose.orientation.y = 0.0
            error_pose.pose.orientation.z = 0.0 #-(yaw_vehicle -target_pose.orientation.z)
            error_pose.pose.orientation.w = 0.0

        return error_pose

    def logic(self, event=None):
        """Logique principale de contrôle"""
        # Contrôle UAV
        if self._state_uav and self.uav_mode != "stop":
            if self.uav_mode == "follow_lines":
                rospy.loginfo("decision.py : UAV follow lines mode")
                self.handle_uav_follow_lines()
            elif self.uav_mode == "follow_aruco":
                self.handle_uav_follow_aruco()
            elif self.uav_mode == "follow_path_optitrack":
                self.handle_uav_follow_path_optitrack()
        else:
            self.way_init_uav = False

        # Contrôle UGV
        if self._state_ugv and self.ugv_mode != "stop":
            if self.ugv_mode == "follow_path_optitrack":
                self.handle_ugv_follow_path_optitrack()
            elif self.ugv_mode == "controlled_by_image":
                self.handle_ugv_controlled_by_image()
        else:
            self.way_init_ugv = False

        # Contrôle combiné UAV-UGV
        if self.uav_ugv_mode == "follow_uav_current":
            self.handle_combined_follow_uav_current()
        elif self.uav_ugv_mode == "follow_uav_desired":
            self.handle_combined_follow_uav_desired()

    def handle_uav_follow_lines(self):
        """Gérer le mode suivi de lignes pour l'UAV"""
        self.no_optitrack_pub.publish(Bool(data=True if self.line_state else False))
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose = self._target_pose_lines
        self._pub_error_uav.publish(pose_stamped)

    def handle_uav_follow_aruco(self):
        """Gérer le mode suivi ArUco pour l'UAV"""
        self.no_optitrack_pub.publish(Bool(data=True if self.detection_aruco else False))
        pose_stamped = PoseStamped()
        pose_stamped.header.stamp = rospy.Time.now()
        pose_stamped.header.frame_id = "map"
        pose_stamped.pose = self._target_pose_aruco
        self._pub_error_uav.publish(pose_stamped)
 

    def handle_uav_follow_path_optitrack(self):
        """Gérer le mode suivi de trajectoire OptiTrack pour l'UAV"""
        self.no_optitrack_pub.publish(Bool(data=False))
        if not self.way_init_uav:
            self._k_uav = self.nearest_point(self._pose_uav, self.reference)
            self.way_init_uav = True
        else:
            if self._k_uav < len(self.reference[0, :]):
                target_pose = self.create_pose_from_reference(self._k_uav, self._pose_uav)
                
                # Stocker le point désiré pour l'UGV
                self._uav_desired_pose = target_pose
                
                error_pose = self.create_error_pose(self._pose_uav, target_pose, target_pose.orientation.z)
                self._pub_error_uav.publish(error_pose)
                self._k_uav += 1
            else:
                self._k_uav = 0

    def handle_ugv_follow_path_optitrack(self):
        """Gérer le mode suivi de trajectoire OptiTrack pour l'UGV"""
        if not self.way_init_ugv:
            self._k_ugv = self.nearest_point(self._pose_ugv, self.reference)
            self.way_init_ugv = True
        else:
            if self._k_ugv < len(self.reference[0, :]):
                target_pose = self.create_pose_from_reference(self._k_ugv, self._pose_ugv)

                error_pose = self.create_error_pose(self._pose_ugv, target_pose)
                self._pub_error_ugv.publish(error_pose)
                self._k_ugv += 1
            else:
                self._k_ugv = 0

    def handle_ugv_controlled_by_image(self):
        """Gérer le mode contrôle par image pour l'UGV"""
        error_pose = self._target_pose_ugv
        self._pub_error_ugv.publish(error_pose)

    def handle_combined_follow_uav_current(self):
        """L'UGV suit la position actuelle de l'UAV"""
        uav_target_pose = Pose()
        uav_target_pose.position.x = self._pose_uav.position.x
        uav_target_pose.position.y = self._pose_uav.position.y
        uav_target_pose.position.z = 0.0  # UGV reste au sol
        uav_target_pose.orientation = self._pose_uav.orientation

        error_pose = self.create_error_pose(self._pose_ugv, uav_target_pose)
        self._pub_error_ugv.publish(error_pose)

    def handle_combined_follow_uav_desired(self):
        """L'UGV suit le point désiré de l'UAV (sans décalage)"""
        if hasattr(self, '_uav_desired_pose'):
            ugv_target_pose = Pose()
            ugv_target_pose.position.x = self._uav_desired_pose.position.x
            ugv_target_pose.position.y = self._uav_desired_pose.position.y
            ugv_target_pose.position.z = 0.0  # UGV reste au sol
            ugv_target_pose.orientation = self._uav_desired_pose.orientation

            error_pose = self.create_error_pose(self._pose_ugv, ugv_target_pose)
            self._pub_error_ugv.publish(error_pose)

    def spin(self):
        rospy.spin()

def main():
    try:
        node = Decision()
        node.spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as e:
        rospy.logerr(f"Error in main: {e}")


# main
if __name__ == "__main__":
    main()