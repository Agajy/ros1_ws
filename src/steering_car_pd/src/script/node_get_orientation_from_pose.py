#!/usr/bin/env python3

import rospy
import numpy as np

import tf as transform
from geometry_msgs.msg import PoseStamped, Quaternion, Vector3


from scipy.spatial.transform import Rotation as R

THRESHOLD_BIG_VALUE = 0.8 * np.pi

class DirectionExtractor: 

    def __init__(self,publisher_euler,publisher_quaternion):
        self._publisher_euler = publisher_euler
        self._publisher_quaternion = publisher_quaternion

    def callback(self,msg):
        (roll, pitch , yaw) = transform.transformations.euler_from_quaternion(
                                [msg.pose.orientation.x, 
                                msg.pose.orientation.y, 
                                msg.pose.orientation.z, 
                                msg.pose.orientation.w])

        
        r = R.from_quat( [msg.pose.orientation.x, 
                                msg.pose.orientation.y, 
                                msg.pose.orientation.z, 
                                msg.pose.orientation.w])
        
        pitch_fix = r.as_rotvec()[1]
        if(pitch_fix > np.pi): 
            pitch_fix = 2*np.pi - pitch_fix
        elif(pitch_fix < -np.pi): 
            pitch_fix = 2*np.pi + pitch_fix

        if(pitch_fix < 0):
            pitch_fix = np.pi + (np.pi + pitch_fix)
        
        new_roll = 0
        new_pitch = pitch_fix
        new_yaw = 0

        # build euler msg
        angle_euler = Vector3()
        angle_euler.x = new_roll
        angle_euler.y = new_pitch
        angle_euler.z = new_yaw

        # build quaternion msg from euler definition
        quaternion_array = transform.transformations.quaternion_from_euler(new_roll,new_pitch,new_yaw)
        angle_quaternion = Quaternion()
        angle_quaternion.x = quaternion_array[0]
        angle_quaternion.y = quaternion_array[1]
        angle_quaternion.z = quaternion_array[2]
        angle_quaternion.w = quaternion_array[3]

        # publish messages
        self._publisher_euler.publish(angle_euler)
        self._publisher_quaternion.publish(angle_quaternion)


if __name__=="__main__":
    rospy.init_node("node_get_direction_from_pose")

    pub_euler = rospy.Publisher("orientation/euler",Vector3,queue_size=10)
    pub_quaternion = rospy.Publisher("orientation/quaternion",Quaternion,queue_size=10)
    direction = DirectionExtractor(publisher_euler=pub_euler,
                                   publisher_quaternion=pub_quaternion)
    rospy.Subscriber("/vrpn_client_node/target/pose",PoseStamped,direction.callback)

    rospy.spin()
