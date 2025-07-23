#!/usr/bin/env python3
from casadi import *
import rospy
import numpy as np
from numpy import linalg as la
from std_msgs.msg import String, Float32
from geometry_msgs.msg import PoseStamped, Vector3, Pose
from trajectory.srv import GetPose, GetPoseResponse, GetTwist, GetTwistResponse  
import time
from controller_frac import Controller
from scipy.spatial.transform import Rotation as Rot



from tf.transformations import euler_from_quaternion

class SteeringCarMainNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('steering_car_main_node')
        rospy.loginfo('steering_car_controller.py : steering_car_main_node successfully initialized!')

        # Initialize the subscriber to the Optitrack
        rospy.Subscriber("/vrpn_client_node/target/pose", PoseStamped, self.__callback_vrpn)

        # Initialize the subscriber to the STOP message
        rospy.Subscriber("/steering_car/STOP",String,self.__callback_stop_emergency)
        
        # Initialize the publisherer of debug topic
        self.debug_state_pub = rospy.Publisher("/steering_car/current_state",Vector3,queue_size=10)
        self.debug_ref_pub = rospy.Publisher("/steering_car/reference",Vector3,queue_size=10)

        self.current_vrpn_msg = PoseStamped()
        self.current_orientation = 0
        self.pose_reference = np.zeros(3)
        self.last_pose_reference = np.zeros(3)
        self.pose=np.zeros(3)
        self.last_pose=np.zeros(3)
        # test=0
        # # Initialize first desired position
        # self.pose_reference_init = False
        # while self.pose_reference_init==False:
        #     test+=1
        #     rospy.loginfo(f"steering_car_controller.py : in the boucle: {test}")
        #     try :
        #         self.pose_reference = self.__state_vector(self.current_vrpn_msg)
        #         self.pose = self.__state_vector(self.current_vrpn_msg)
        #     except:
        #         continue

        # Sampling time and timer
        self.Ts = rospy.get_param('Ts')
        rospy.Timer(rospy.Duration(self.Ts), self.reference_callback)
        rospy.Timer(rospy.Duration(self.Ts), self.control_callback)

        # Distance between front and rear wheels
        self.L = rospy.get_param('L')
                
        # Initialize the controller
        self.steering_msg = Float32()
        self.throttle_msg = Float32()
        self.steering = 0.0
        self.vel=sqrt((pow(self.pose[0]-self.last_pose[0],2)+pow(self.pose[1]-self.last_pose[1],2)))/self.Ts
        
        # Initialize the controls' publisher
        self.steering_pub = rospy.Publisher("/steering", Float32, queue_size=10)
        self.throttle_pub = rospy.Publisher("/throttle", Float32, queue_size=10) 
        self.offset = 0.25
        self.im_number = 0
        self.started = False
        self.emergency_stop = False
        self.t_prev = None

        # longitudinal ugv controller param
        alpha_x = 0.9
        ws12, ws22 = 1.0, 1.0 #  0.1, 0.1
        alpha12 = 0.9
        k2 = 0.8

        simulation = False
        if simulation:
            # lateral ugv controller param
            alpha_y, alpha_psi = 0.99, 0.0 # 
            w_y, w_psi, sign_psi = 1.0, 0.0, 1.1 # 
            ws1, ws2 = 1.0, 0.5 # 0.5 3
            alpha1 = 0.999 #
            k = 10.0 #

        else:
            # lateral controller param
            alpha_y, alpha_psi = 0.99, 0.0 # 
            w_y, w_psi, sign_psi = 1.0, 0.1, 1.1 # 
            ws1, ws2 = 0.1, 1.0 # 0.5 3
            alpha1 = 0.99 #
            k = 10.0 #

        self.ugv_controller = Controller(alpha_x, alpha_y, alpha_psi,
                 w_y, w_psi, sign_psi,
                ws1, ws2, alpha1, k,
                ws12, ws22, alpha12, k2)
               

                
        # Simulation horizon
        self.stop_time = rospy.get_param('stop_time')
        
        # Flag to STOP since simulation has been completed
        self.stop_time_reached = False
        
        # Flag for emergency STOP 
        self.stop_emergency = False

        # Initial control
        # self.throttle_msg.data, self.steering_msg.data=self.ugvControl(0,0,0,self.Ts)

        # self.throttle_pub.publish(self.throttle_msg)
        # self.steering_pub.publish(self.steering_msg)
    
    def ugvControl(self, dx_, dy_, dpsi, dt):
        # invert y, and switch x-y axis (ugv frame to dynamic model frame)
        dx = dy_
        dy = -dx_
        # Get controller output
        controller_ac, controller_steering = self.ugv_controller.run(dx, dy, dpsi, self.steering, self.vel, dt)
        controller_ac *= 0.32
        controller_ac = np.clip(controller_ac, -0.3,0.3)

        self.steering = controller_steering

        # clip values and change its range (according to jetson racer robot commands input)
        s_max = 0.7
        ac_max = 1.0
        steering_cmd = np.clip(np.clip(controller_steering, -s_max, s_max)/s_max, -0.95,0.95)
        ac_cmd = np.clip(controller_ac/ac_max, -1.0,1.0)

        controller_ac = np.clip(controller_ac,0,0.3)
        ac_cmd = abs(ac_cmd)*0.8
        return -ac_cmd, -steering_cmd+self.offset



    # Modify to change the controller
    def control_callback(self,event):
        self.pose=self.__state_vector(self.current_vrpn_msg)
        dx=(self.pose_reference[0]-self.pose[0])
        dy=(self.pose_reference[1]-self.pose[1])
        dpsi=self.angle_between_vectors_xy(self.pose_reference-self.last_pose_reference, self.pose[2])

        p_pose_reference_dif = self.pose_reference-self.last_pose_reference
        dpsi_pose_reference = np.arctan2( p_pose_reference_dif[1],  p_pose_reference_dif[0])
        p_pose_dif = self.pose-self.last_pose
        dpsi_ugv=np.arctan2( p_pose_dif[1],  p_pose_dif[0])
        # print("angles: ", np.rad2deg(dpsi_ugv_target), np.rad2deg(dpsi_feature_target))
        #
        dpsi =  dpsi_pose_reference - dpsi_ugv



        self.vel=sqrt((pow(self.pose[0]-self.last_pose[0],2)+pow(self.pose[1]-self.last_pose[1],2)))/self.Ts
   
        self.throttle_msg.data, self.steering_msg.data=self.ugvControl(dx,dy,dpsi,self.Ts)
        self.throttle_pub.publish(self.throttle_msg)
        self.steering_pub.publish(self.steering_msg)
        self.last_pose=self.pose
        self.last_pose_reference=self.pose_reference
        u_k=np.array([self.throttle_msg,self.steering_msg])
        rospy.loginfo(f"steering_car_controller.py : receive :\n   position {self.pose},\n   position_desired {self.pose_reference},\n error:\n  {dx},{dy},{dpsi},\n   send:\n   {u_k}")

    def angle_between_vectors_xy(self,v1, v2):
        # Projection des vecteurs sur le plan XY
        v1_xy = np.array([v1[0], v1[1]])
        v2_xy = np.array([0, 1])
        
        # Calcul du produit scalaire
        dot_product = np.dot(v1_xy, v2_xy)
        
        # Calcul des normes
        norm_v1 = np.linalg.norm(v1_xy)
        norm_v2 = np.linalg.norm(v2_xy)
        
        # Éviter la division par zéro
        if norm_v1 == 0:
            norm_v1=1.0
        
        # Calcul de l'angle en radians
        cos_theta = np.clip(dot_product / (norm_v1 * norm_v2), -1.0, 1.0)
        angle = np.arccos(cos_theta)
        
        # Conversion en degrés
        return angle-v2

    
    def reference_callback(self,event):
        service_name="u_position_d"
        rospy.wait_for_service(service_name)
        rospy.loginfo("steering_car_controller.py : new_pose")
        try:
            service_proxy = rospy.ServiceProxy(service_name, GetPose)
            reponse = service_proxy()
            pose_reference=reponse.pose
            # rospy.loginfo(f"steering_car_controller.py : Response from {service_name}: {self.pose_reference}")
            self.pose_reference = np.array([pose_reference.position.x, pose_reference.position.y, pose_reference.position.z])
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
        
    def __callback_stop_emergency(self, msg):
        self.stop_emergency = True
        
    def __callback_vrpn(self, msg):
        self.pose_reference_init=True
        self.current_vrpn_msg = msg
             
    def __state_vector(self, msg):
        # The Optitrack measures the center of the steering car
        x_c = msg.pose.position.x
        y_c = -msg.pose.position.z
        
        orientation_list = [msg.pose.orientation.x, -msg.pose.orientation.z, msg.pose.orientation.y, msg.pose.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
                    
        # The model considers the center of the robot
        x_k = np.array([x_c, y_c, yaw])  
        
        return x_k 

    def run(self):
        rospy.init_node("steering_car_main_node")
        rospy.spin()

if __name__ == "__main__":
    steering_car_main_node=SteeringCarMainNode()
    steering_car_main_node.run()

