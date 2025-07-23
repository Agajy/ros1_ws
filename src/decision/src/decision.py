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

class Decision(object):
    def __init__(self):
        rospy.init_node("decision")
        self.object_uav= rospy.get_param('~uav')
        self.object_ugv= rospy.get_param('~ugv')
        self.in_simu= rospy.get_param('~in_simu')

        self._state_uav = False
        self._state_ugv = False

        self._sub_vrpn_uav = rospy.Subscriber(f"/vrpn_client_node/{self.object_uav}/pose", PoseStamped, self.vrpn_client_uav_callback, queue_size=1)
        self._sub_vrpn_ugv = rospy.Subscriber(f"/vrpn_client_node/{self.object_ugv}/pose", PoseStamped, self.vrpn_client_ugv_callback, queue_size=1)

        self._sub_state_uav = rospy.Subscriber(f"/state/{self.object_uav}", Bool, self.state_uav_callback, queue_size=1)

        if self.in_simu:
            self._sub_state_ugv = rospy.Subscriber(f"/state/{self.object_ugv}", Bool, self.state_ugv_callback, queue_size=1)
        else:
            self._state_ugv = True 

        self._pub_error_uav = rospy.Publisher(f"/error_pose/{self.object_uav}/pose", PoseStamped, queue_size=1)
        self._pub_error_ugv = rospy.Publisher(f"/error_pose/{self.object_ugv}/pose", PoseStamped, queue_size=1)

        self.aruco_pose_sub = rospy.Subscriber('/target_pose_lines', PoseStamped, self.aruco_pose_callback, queue_size=1)
        self.aruco_detected_sub = rospy.Subscriber('/aruco_detected', Bool, self.aruco_detected_callback, queue_size=1)
        self.line_detected_sub = rospy.Subscriber('/line_detected', Float32, self.line_detected_callback)       

        self.no_optitrack_pub = rospy.Publisher('/no_optitrack', Bool, queue_size=1)


        path_excel_file = rospy.get_param('path_excel_file')
        self.reference = ExcelReader.extract_data_excel_to_array(path_excel_file).T

        self._pose_uav = Pose()
        self._pose_ugv = Pose()
        self._pose_aruco = Pose()


        rospy.Timer(rospy.Duration(1.0/40), self.logic)
        self.detection_aruco = False
        self.line_state = False
        self.way_init_uav = False

        self._k_uav = 0
        self._k_ugv = 0
        
        rospy.loginfo("decision.py : decision initialized")

    def line_detected_callback(self, data):
        self.line_state = True if data.data > 0 else False
    
    def aruco_detected_callback(self, msg):
        self.detection_aruco = msg.data

    def euler_from_quaternion(self,x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
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
     
        return roll_x, pitch_y, yaw_z # in radians

    def aruco_pose_callback(self, msg):
        self._pose_aruco = msg.pose
      

    def vrpn_client_uav_callback(self, msg):
        self._pose_uav = msg.pose


    def vrpn_client_ugv_callback(self, msg):
        self._pose_ugv = msg.pose


    def state_uav_callback(self, msg):
        self._state_uav = msg.data
        if not msg.data:
            self.way_init_uav = False


    def state_ugv_callback(self, msg):
        self._state_ugv = msg.data


    def nearest_point(self, point, reference):
        """
        Find the nearest point in the reference trajectory to the given point.
        """
        min_dist = float('inf')
        nearest_index = -1
        for i in range(len(reference[0])):
            dist = sqrt((point.position.x - reference[0][i])**2 + (point.position.y - reference[1][i])**2)
            if dist < min_dist:
                min_dist = dist
                nearest_index = i
        return nearest_index


    def logic(self,event=None):
        if self._state_uav:
            if not self.way_init_uav:
                self._k_uav = self.nearest_point(self._pose_uav, self.reference)
                self.way_init_uav = True
            else:
                if self._k_uav < len(self.reference[0,:]):
                    if not self.detection_aruco and not self.line_state :
                        pose = Pose()
                        next_k = (self._k_uav + 1) % len(self.reference[0, :])
                        pose.position.x = self.reference[0, self._k_uav]
                        pose.position.y = self.reference[1, self._k_uav]
                        pose.position.z = self.reference[2, self._k_uav]

                        # Calculate orientation based on the next point
                        dx = self.reference[0, next_k] - self.reference[0, self._k_uav]
                        dy = self.reference[1, next_k] - self.reference[1, self._k_uav]
                        yaw = math.atan2(dy, dx)
                        yaw_uav = self.euler_from_quaternion(self._pose_uav.orientation.x,
                                                        self._pose_uav.orientation.y,
                                                        self._pose_uav.orientation.z,
                                                        self._pose_uav.orientation.w)[2]
                        if ((yaw_uav>=-math.pi) and (yaw_uav<=-math.pi/2) and (yaw<=math.pi)and (yaw>=math.pi/2)):
                            yaw -= 2*math.pi
                        if ((yaw>=-math.pi) and (yaw<=-math.pi/2) and (yaw_uav<=math.pi)and (yaw_uav>=math.pi/2)):
                            yaw += 2*math.pi
                        quaternion = tf.transformations.quaternion_from_euler(0.0, 0.0, yaw)

                        pose.orientation.x = quaternion[0]
                        pose.orientation.y = quaternion[1]
                        pose.orientation.z = quaternion[2]
                        pose.orientation.w = quaternion[3]

                        error_pose = PoseStamped()
                        error_pose.header.stamp = rospy.Time.now()
                        error_pose.header.frame_id = "map"
                        error_pose.pose.position.x = self._pose_uav.position.x - pose.position.x
                        error_pose.pose.position.y = self._pose_uav.position.y - pose.position.y
                        error_pose.pose.position.z = self._pose_uav.position.z - pose.position.z


                        # on envoie pas un quaternion mais un angle ce n'est pas tres propre mais ca permet d'avoir la continuité des angles
                        error_pose.pose.orientation.x = 0.0
                        error_pose.pose.orientation.y = 0.0
                        error_pose.pose.orientation.z = yaw
                        error_pose.pose.orientation.w = 0.0
                        self._k_uav += 1

                    else :
                        error_pose = PoseStamped()
                        error_pose.header.stamp = rospy.Time.now()
                        error_pose.header.frame_id = "map"
                        error_pose.pose.position.x = self._pose_aruco.position.x
                        error_pose.pose.position.y = self._pose_aruco.position.y
                        error_pose.pose.position.z = self._pose_aruco.position.z


                        # roll,pitch,yaw = self.euler_from_quaternion(self._pose_aruco.orientation.x,
                        #                                 self._pose_aruco.orientation.y,
                        #                                 self._pose_aruco.orientation.z,
                        #                                 self._pose_aruco.orientation.w)


                        error_pose.pose.orientation.x = self._pose_aruco.orientation.x
                        error_pose.pose.orientation.y = self._pose_aruco.orientation.y
                        error_pose.pose.orientation.z = self._pose_aruco.orientation.z
                        error_pose.pose.orientation.w = self._pose_aruco.orientation.w
                       

                        rospy.loginfo(f"decision.py : IBVS en fonctionnement")

                    self.no_optitrack_pub.publish(Bool(data=True if self.line_state else False))
                    self._pub_error_uav.publish(error_pose)
                    rospy.loginfo(f"decision.py : error pose uav: {self._k_uav}")
                
                else:
                    self._k_uav =0
                    rospy.loginfo("decision.py : Reached end of trajectory, resetting...")
        else:
            self.way_init_uav = False

    
    def spin(self):
        rospy.spin()


def main():
    node = Decision()
    node.spin()


# main
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass