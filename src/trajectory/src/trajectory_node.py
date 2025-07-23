#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Pose, Twist
from trajectory.srv import GetPose, GetPoseResponse, GetTwist, GetTwistResponse  # Remplace "your_package" par le nom de ton package
from excel_reader_class import ExcelReader

class TrajectoryNode:
    def __init__(self):
        rospy.init_node('trajectory_node')
        
        path_excel_file = rospy.get_param('path_excel_file')
        self.reference = ExcelReader.extract_data_excel_to_array(path_excel_file).T

        self.pose_service = rospy.Service('u_position_d', GetPose, self.handle_position)
        self.velocity_service = rospy.Service('u_vel_d', GetTwist, self.handle_velocity)
        self.acceleration_service = rospy.Service('u_acc_d', GetTwist, self.handle_acceleration)
        
        rospy.loginfo("trajectory_node.py : Service node is ready.")
        self._k = 0
    
    def handle_position(self, req):
        pose = Pose()
        try:
            pose.position.x = self.reference[0, self._k]
            pose.position.y = self.reference[1, self._k]
            pose.position.z = self.reference[2, self._k]
            self._k += 1
        except:
            pose.position.x = self.reference[0, -1]
            pose.position.y = self.reference[1, -1]
            pose.position.z = self.reference[2, -1]

        rospy.loginfo(f"trajectory_node.py : Position sent: x={pose.position.x}, y={pose.position.y}, z={pose.position.z}")
        return GetPoseResponse(pose=pose)
    
    def handle_velocity(self, req):
        velocity = Twist()
        velocity.linear.x = 0.0
        velocity.linear.y = 0.0
        velocity.linear.z = 0.0
        velocity.angular.x = 0.0
        velocity.angular.y = 0.0
        velocity.angular.z = 0.0

        rospy.loginfo(f"trajectory_node.py : Velocity sent: {velocity}")
        return GetTwistResponse(twist=velocity)

    def handle_acceleration(self, req):
        acceleration = Twist()
        acceleration.linear.x = 0.0
        acceleration.linear.y = 0.0
        acceleration.linear.z = 0.0
        acceleration.angular.x = 0.0
        acceleration.angular.y = 0.0
        acceleration.angular.z = 0.0

        rospy.loginfo(f"trajectory_node.py : Acceleration sent: {acceleration}")
        return GetTwistResponse(twist=acceleration)
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    node = TrajectoryNode()
    node.run()
