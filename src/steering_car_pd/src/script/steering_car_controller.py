#!/usr/bin/env python3
from casadi import *
import rospy
import numpy as np

from steering_car_publisher_controls_class import PublisherControls
from steering_car_PD_class import SteeringCarPD
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Vector3, Pose
from trajectory.srv import GetPose, GetPoseResponse, GetTwist, GetTwistResponse  


from tf.transformations import euler_from_quaternion

class SteeringCarMainNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('steering_car_main_node')
        rospy.loginfo('steering_car_main_node successfully initialized!')

        # Initialize the subscriber to the Optitrack
        rospy.Subscriber("/vrpn_client_node/target/pose", PoseStamped, self.__callback_vrpn)
        rospy.Subscriber("/orientation/euler",Vector3, self.__callback_orientation)

        # Initialize the subscriber to the STOP message
        rospy.Subscriber("/steering_car/STOP",String,self.__callback_stop_emergency)
        
        # Initialize the publisherer of debug topic
        self.debug_state_pub = rospy.Publisher("/steering_car/current_state",Vector3,queue_size=10)
        self.debug_ref_pub = rospy.Publisher("/steering_car/reference",Vector3,queue_size=10)

        self.current_vrpn_msg = PoseStamped()
        self.current_orientation = 0
        self.pose_reference = np.zeros(3)
        self.pose=np.zeros(3)
        # test=0
        # # Initialize first desired position
        # self.pose_reference_init = False
        # while self.pose_reference_init==False:
        #     test+=1
        #     rospy.loginfo(f"in the boucle: {test}")
        #     try :
        #         self.pose_reference = self.__calculateRearPoint(self.current_vrpn_msg)
        #         self.pose = self.__calculateRearPoint(self.current_vrpn_msg)
        #     except:
        #         continue

        # Sampling time and timer
        self.Ts = rospy.get_param('Ts')
        rospy.Timer(rospy.Duration(self.Ts), self.reference_callback)
        rospy.Timer(rospy.Duration(self.Ts), self.control_callback)

        # Distance between front and rear wheels
        self.L = rospy.get_param('L')
                
        # Initialize the controller
        Kp_v = rospy.get_param('Kp_v')
        Kd_v = rospy.get_param('Kd_v')
        Kp_psi = rospy.get_param('Kp_psi')
        Kd_psi = rospy.get_param('Kd_psi')
        self.my_steering_car_pd = SteeringCarPD(self.Ts, Kp_v, Kd_v, Kp_psi, Kd_psi)
        
        # Initialize the controls' publisher
        steering_gain = rospy.get_param('steering/gain')
        steering_offset = rospy.get_param('steering/offset')
        throttle_gain = rospy.get_param('throttle/gain')
        self.my_controls_publisher = PublisherControls(steering_gain, steering_offset, throttle_gain)
               
        # Initial control
        steering_0 = rospy.get_param('u_vector_0/steering')
        throttle_0 = rospy.get_param('u_vector_0/throttle')
        u_0 = np.array([steering_0,throttle_0])
                
        # Simulation horizon
        self.stop_time = rospy.get_param('stop_time')
        
        # Flag to STOP since simulation has been completed
        self.stop_time_reached = False
        
        # Flag for emergency STOP 
        self.stop_emergency = False
               
        self.my_controls_publisher.publish_control_inputs(u_0)


    # Modify to change the controller
    def control_callback(self,event):
        self.pose=self.__calculateRearPoint(self.current_vrpn_msg)
        u_k = self.my_steering_car_pd.compute_control_action(self.pose, self.pose_reference)
        self.my_controls_publisher.publish_control_inputs(u_k)
        rospy.loginfo(f"receive :\n   position {self.pose},\n   position_desired {self.pose_reference},\n send:\n   {u_k}")


    
    def reference_callback(self,event):
        service_name="u_position_d"
        rospy.wait_for_service(service_name)
        rospy.loginfo("new_pose")
        try:
            service_proxy = rospy.ServiceProxy(service_name, GetPose)
            reponse = service_proxy()
            pose_reference=reponse.pose
            # rospy.loginfo(f"Response from {service_name}: {self.pose_reference}")
            self.pose_reference = np.array([pose_reference.position.x, pose_reference.position.y, pose_reference.position.z])
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
        
    def __callback_stop_emergency(self, msg):
        self.stop_emergency = True
        
    def __callback_vrpn(self, msg):
        self.pose_reference_init=True
        self.current_vrpn_msg = msg
        
    def __callback_orientation(self, msg):
        self.current_orientation = msg.y
       
    def __calculateRearPoint(self, msg):
        # The Optitrack measures the center of the steering car
        x_c = msg.pose.position.x
        y_c = -msg.pose.position.z
        
        orientation_list = [msg.pose.orientation.x, -msg.pose.orientation.z, msg.pose.orientation.y, msg.pose.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
                    
        # The model considers the center of the rear axis
        x = x_c - (self.L/2) * np.cos(yaw)
        y = y_c - (self.L/2) * np.sin(yaw)
        x_k = np.array([x, y, yaw])  
        
        return x_k 

    def run(self):
        rospy.init_node("steering_car_main_node")
        rospy.spin()

if __name__ == "__main__":
    steering_car_main_node=SteeringCarMainNode()
    steering_car_main_node.run()

