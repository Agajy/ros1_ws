#!/usr/bin/env python
import rospy
import numpy as np
import scipy.signal
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class GaussianFilterNode:
    def __init__(self):
        rospy.init_node('gaussian_filter_node', anonymous=True)
        self.filtered_pub = rospy.Publisher('/filtered_curve', Marker, queue_size=10)
        self.original_sub = rospy.Subscriber('/original_curve', Marker, self.original_callback)
        rospy.loginfo("Gaussian Filter Node Initialized")
    
    def gaussian_filter(self,x, P_gauss=3, epsilon=0.01):
        sigma = P_gauss / np.sqrt(-2.0 * np.log(epsilon))
        b_gauss = np.array([np.exp(-(k - P_gauss) ** 2 / (2 * sigma ** 2)) for k in range(2 * P_gauss + 1)])
        b_gauss /= np.sum(b_gauss)  # Normalize the filter
        y = scipy.signal.convolve(x, b_gauss, mode='valid')
        return y, P_gauss

    def original_callback(self, msg):
        # Extract x, y points from the Marker message
        x = np.array([point.x for point in msg.points])
        y = np.array([point.y for point in msg.points])

        # Apply Gaussian filter to y values
        filtered_y, P_gauss = self.gaussian_filter(y)

        # Create a new Marker message for the filtered curve
        filtered_marker = Marker()
        filtered_marker.header = msg.header
        filtered_marker.type = Marker.LINE_STRIP
        filtered_marker.scale.x = 0.05
        filtered_marker.color.r = 1.0
        filtered_marker.color.g = 0.0
        filtered_marker.color.b = 0.0
        filtered_marker.color.a = 1.0

        # Adjust x values to match the filtered y values
        filtered_x = x[P_gauss:len(x) - P_gauss]
        filtered_marker.points = [Point(x=filtered_x[i], y=filtered_y[i], z=0) for i in range(len(filtered_y))]

        # Publish the filtered curve
        self.filtered_pub.publish(filtered_marker)

if __name__ == '__main__':   
    try:
        node = GaussianFilterNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
