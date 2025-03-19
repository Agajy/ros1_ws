#!/usr/bin/env python3
from casadi import *
import rospy
import numpy as np

from steering_car_publisher_controls_class import PublisherControls
from steering_car_PD_class import SteeringCarPD
from excel_reader_class import ExcelReader
from std_msgs.msg import String, Int32
from geometry_msgs.msg import PoseStamped, Vector3

from tf.transformations import euler_from_quaternion

class SteeringCarMainNode:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('steering_car_main_node')
        rospy.loginfo('steering_car_main_node successfully initialized!')
        
        # Initialize time of the simulation
        self.t = 0.0
        
        # Initialize the discrete time index
        self._k = 0
        
        # Sampling time
        self.Ts = rospy.get_param('Ts')
        
        # Distance between front and rear wheels
        self.L = rospy.get_param('L')        
        
        self.current_vrpn_msg = PoseStamped()
        self.old_vrpn_msg = PoseStamped()
        self.current_orientation = 0
        
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
        
        # Store reference read from Excel
        path_excel_file = rospy.get_param('path_excel_file')
        self.reference = ExcelReader.extract_data_excel_to_array(path_excel_file).T
        
        # Initial control
        steering_0 = rospy.get_param('u_vector_0/steering')
        throttle_0 = rospy.get_param('u_vector_0/throttle')
        u_0 = np.array([[steering_0],[throttle_0]])
                
        # Simulation horizon
        self.stop_time = rospy.get_param('stop_time')
        
        # Flag to STOP since simulation has been completed
        self.stop_time_reached = False
        
        # Flag for emergency STOP 
        self.stop_emergency = False
        
        # Time of the first lost data
        self.start_losses_time = rospy.get_param('start_losses_time')
        
        self.stop_losses_time = rospy.get_param('stop_losses_time') #round(self.start_losses_time + self.Ts * self.n_losses, 1)
        
        self.first_loss = True
        
        # Initialize the subscriber to the Optitrack
        rospy.Subscriber("/vrpn_client_node/target/pose", PoseStamped, self.__callback_vrpn)
        rospy.Subscriber("/orientation/euler",Vector3, self.__callback_orientation)

        # Initialize the listener to the STOP message
        rospy.Subscriber("/steering_car/STOP",String,self.__callback_stop_emergency)
        
        self.debug_state_pub = rospy.Publisher("/steering_car/current_state",Vector3,queue_size=10)
        self.debug_ref_pub = rospy.Publisher("/steering_car/reference",Vector3,queue_size=10)
        self.debug_loss_pub = rospy.Publisher("/steering_car/lost",Int32,queue_size=10)
        
        self.my_controls_publisher.publish_control_inputs(u_0)

    def run(self):
        # Sleep 1s before starting the simulation
        rospy.sleep(1)
                
        # Main loop of the node
        rate_Hz = int(1/self.Ts) # 10 Hz
        rate = rospy.Rate(rate_Hz)
        
        while (not self.stop_time_reached) and (not rospy.is_shutdown()):
            
            u_k = np.zeros((2,1))          
            
            # If the simulation is not finished ...
            if (not self.stop_emergency) and (self.t <= self.stop_time):  
                
                rospy.loginfo("[%s] k = %d, t = %.1f", rospy.get_name(), self._k, self.t)
                
                # print(self.t)
                
                msg = PoseStamped()
                
                # (for debugging estimated state)
                
                msg = self.current_vrpn_msg
                x_k = self.__calculateRearPoint(msg)
                current_state = Vector3()
                current_state.x = x_k[0,0]
                current_state.y = x_k[1,0]
                current_state.z = x_k[2,0]
                self.debug_state_pub.publish(current_state)
                
                # If there is no loss (i.e. not in the losses' time interval)
                if (self.t < self.start_losses_time) or (self.t > self.stop_losses_time):
                    # store the current posestamped message for the next iteration
                    self.old_vrpn_msg = msg
                    loss_msg = Int32()
                    loss_msg.data = 0
                    self.debug_loss_pub.publish(loss_msg)                                    
                else:
                    print("lost data !")
                    msg = self.old_vrpn_msg
                    # The state should be the old one (not the actual one)
                    x_k = self.__calculateRearPoint(msg)
                    loss_msg = Int32()
                    loss_msg.data = 1
                    self.debug_loss_pub.publish(loss_msg)
                                    
                # The reference is a trajectory
                ref_k = np.array([[self.reference[0,self._k]], 
                                  [self.reference[1,self._k]], 
                                  [self.reference[2,self._k]]])
                                  
                # (for debugging reference)
                reference = Vector3()
                reference.x = ref_k[0,0]
                reference.y = ref_k[1,0]
                reference.z = ref_k[2,0]
                self.debug_ref_pub.publish(reference)
                
                u_k = self.my_steering_car_pd.compute_control_action(x_k, ref_k)
                
            # If the simulation is finished
            else:                
                # Update to stop the simulation at the next iteration
                self.stop_time_reached = True
        
            
            # Publish the computed control input (if the simulation is finished it will be both values equal to 0)
            self.my_controls_publisher.publish_control_inputs(u_k)
            
            # Update t for the next iteration
            self.t = round(self.t + self.Ts,1)
            
            # Update k for next iteration
            self._k = self._k + 1
            
            rate.sleep()
        
    def __callback_stop_emergency(self, msg):
        self.stop_emergency = True
        
    def __callback_vrpn(self, msg):
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
        x_k = np.array([[x], [y], [yaw]])  
        
        return x_k
    

if __name__ == "__main__":
    steering_car_main_node = SteeringCarMainNode()
    steering_car_main_node.run()

