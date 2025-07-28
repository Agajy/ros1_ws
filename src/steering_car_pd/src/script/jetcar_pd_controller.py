#!/usr/bin/env python3
from casadi import *
import rospy
import numpy as np

from steering_car_publisher_controls_class import PublisherControls
from steering_car_PD import SteeringCarPD
from excel_reader_class import ExcelReader
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, Vector3

from tf.transformations import euler_from_quaternion

class SteeringCarMainNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('steering_car_main_node')
        rospy.loginfo('steering_car_main_node.py : successfully initialized!')

        self.object_ugv= rospy.get_param('~ugv')
        self.in_simu= rospy.get_param('~in_simu')
        while self.in_simu:
            continue
        # Sampling time
        self.Ts = 0.1
        
        # Distance between front and rear wheels
        self.L = 0.334
        
        self.error_msg = PoseStamped()
        self.uav_vrpn_msg = PoseStamped()
        self.rec_destination = PoseStamped()
          
        # Initialize the controller
        Kp_v = 1.0
        Kd_v = 0.1
        Kp_psi = 1.0
        Kd_psi = 0.1
        self.my_steering_car_pd = SteeringCarPD(self.Ts, Kp_v, Kd_v, Kp_psi, Kd_psi)
        
        # Initialize the controls' publisher
        steering_gain = 0.65
        steering_offset = 0.0
        throttle_gain = 1.0
        self.my_controls_publisher = PublisherControls(steering_gain, steering_offset, throttle_gain)
        
        # Initial control
        steering_0 = 0.0
        throttle_0 = 0.0
        u_0 = np.array([[steering_0],[throttle_0]])
        
        # Initialize the subscriber to the Optitrack
        rospy.Subscriber(f"/error_pose/{self.object_ugv}/pose", PoseStamped, self.__error_callback)

        self.publish_rec = rospy.Publisher(f"/rec_command/{self.object_ugv}/command", PoseStamped, queue_size=10)
                
        self.my_controls_publisher.publish_control_inputs(u_0)

    def run(self):
        rospy.sleep(10)
                
        # Main loop of the node
        rate_Hz = int(1/self.Ts) # 10 Hz
        rate = rospy.Rate(rate_Hz)
        
        while (not rospy.is_shutdown()): #(not self.stop_time_reached) and
            
            u_k = np.zeros((2,1))          
            x_k = self.__calculateRearPoint(self.error_msg)                

            self.rec_destination.pose.position.x = x_k[0]
            self.rec_destination.pose.position.y = x_k[1]
            self.rec_destination.pose.position.z = 0.0
            self.rec_destination.header.stamp = rospy.Time.now()
            rospy.loginfo(f"steering_car_main_node.py : x= {self.rec_destination.pose.position.x}, y={self.rec_destination.pose.position.y}, z={self.rec_destination.pose.position.z}")

            self.publish_rec.publish(self.rec_destination)
                                  
            u_k = self.my_steering_car_pd.compute_control_action(x_k)   
            
            # Publish the control inputs
            self.my_controls_publisher.publish_control_inputs(u_k)
            
            rate.sleep()
        
        
    def __error_callback(self, msg):
        self.error_msg = msg
    
    def __calculateRearPoint(self, msg):
        # The Optitrack measures the center of the steering car
        x_c = msg.pose.position.x
        y_c = msg.pose.position.y
        yaw = msg.pose.orientation.z
                            
        # The model considers the center of the rear axis
        
        x = x_c - (self.L/2) * np.cos(yaw)
        y = y_c - (self.L/2) * np.sin(yaw)
        if x_c==0.0 and y_c==0.0 :
            x=0.0
            y=0.0
        x_k = np.array([x, y, yaw])  
        return x_k 


if __name__ == "__main__":
    steering_car_main_node = SteeringCarMainNode()
    steering_car_main_node.run()

