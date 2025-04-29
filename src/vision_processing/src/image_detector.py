#!/usr/bin/env python

import rospy
import cv2
import cv2.aruco as aruco
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import numpy as np

class LineAndArucoDetector:
    def __init__(self):
        rospy.init_node('line_and_aruco_detector', anonymous=True)

        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/image_tcp_client/image_gray', Image, self.image_callback)

        # ArUco dictionary
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

        self.aruco_params = aruco.DetectorParameters()

        rospy.loginfo("Line and ArUco detector initialized.")
        rospy.spin()

    def image_callback(self, data):
        try:
            # Convert ROS Image to OpenCV grayscale image
            cv_image = self.bridge.imgmsg_to_cv2(data, desired_encoding='passthrough')
        except Exception as e:
            rospy.logerr("CV Bridge error: %s", e)
            return

        # Detect white lines (simple threshold + contour detection)
        _, thresh = cv2.threshold(cv_image, 200, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv_lines = cv2.cvtColor(cv_image, cv2.COLOR_GRAY2BGR)
        cv2.drawContours(cv_lines, contours, -1, (0, 255, 0), 2)

        # Detect ArUco markers
        corners, ids, _ = aruco.detectMarkers(cv_image, self.aruco_dict, parameters=self.aruco_params)
        if ids is not None:
            aruco.drawDetectedMarkers(cv_lines, corners, ids)

        # Show result (for debugging)
        cv2.imshow("Lines and ArUco Detection", cv_lines)
        cv2.waitKey(1)

if __name__ == '__main__':
    try:
        LineAndArucoDetector()
    except rospy.ROSInterruptException:
        pass
    cv2.destroyAllWindows()
