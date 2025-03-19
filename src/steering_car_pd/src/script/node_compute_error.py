#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped, Vector3

class DifferenceNode:
    def __init__(self):
        self.ref = None
        self.state = None

        rospy.init_node('difference_node', anonymous=True)

        # subscribe to reference
        self.sub_ref = rospy.Subscriber("/steering_car/reference", Vector3, self.callback_reference)
        # subscribe to the position 
        self.sub_state = rospy.Subscriber("/steering_car/current_state", Vector3, self.callback_optitrack)
        # publisher of the error
        self.pub_error = rospy.Publisher('/steering_car/error', Vector3, queue_size=10)

        rospy.spin()

    def callback_reference(self, msg):
        self.ref = np.array([[msg.x],
                             [msg.y],
                             [msg.z]])
        self.publish_difference()

    def callback_optitrack(self, msg):
        self.state = np.array([[msg.x],
                               [msg.y],
                               [msg.z]])
        self.publish_difference()

    def publish_difference(self):
        if self.state is not None and self.ref is not None:
            error = self.state - self.ref
            error_msg = Vector3()
            error_msg.x = error[0,0]
            error_msg.y = error[1,0]
            error_msg.z = error[2,0]
            self.pub_error.publish(error_msg)

if __name__ == '__main__':
    try:
        DifferenceNode()
    except rospy.ROSInterruptException:
        pass
