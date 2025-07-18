#!/usr/bin/env python3

import rospy
import socket
import threading
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_matrix
from std_msgs.msg import Bool


class RecPosition:
    def __init__(self):
        rospy.init_node('position_subscriber')
        
        # TCP parameters
        self.in_simu = rospy.get_param('~in_simu', False)
        self.TCP_IP = '127.0.0.1' if self.in_simu else rospy.get_param('~host', '172.26.209.28')
        self.TCP_PORT = rospy.get_param('~port', 5000)

        self.object = rospy.get_param('~object', "uav")
                
        self._sub_state_object = rospy.Subscriber(f"/state/{self.object}", Bool, self.state_object_callback, queue_size=1)
        self.publishers_object = rospy.Publisher(f'/rec_command/{self.object}/command', PoseStamped, queue_size=10)
        self.command_topic = f"/error_pose/{self.object}/pose"
        self._error_pose_sub = rospy.Subscriber(self.command_topic, PoseStamped, self.publish_pose_rec_by_ros, queue_size=1)
        self._state_object = False

        self.running = True
        
        # Start TCP client thread
        self.client_thread = threading.Thread(target=self.tcp_client)
        self.client_thread.start()
    

    def publish_pose_rec_by_tcp(self, object_name, x, y, z, qx, qy, qz, qw):   
        if not self.in_simu and self.object == "ugv":
            return # Skip if not in simulation and object is UGV
        elif self._state_object:
            return # Skip if the state of the object is active  
        
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = object_name
        
        pose_msg.pose.position.x = float(x)
        pose_msg.pose.position.y = float(y)
        pose_msg.pose.position.z = float(z)
        
        pose_msg.pose.orientation.x = float(qx)
        pose_msg.pose.orientation.y = float(qy)
        pose_msg.pose.orientation.z = float(qz)
        pose_msg.pose.orientation.w = float(qw)
        
        self.publishers_object.publish(pose_msg)
    
    def publish_pose_rec_by_ros(self, msg):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = msg.header.frame_id
        
        pose_msg.pose.position.x = msg.pose.position.x
        pose_msg.pose.position.y = msg.pose.position.y
        pose_msg.pose.position.z = msg.pose.position.z
        
        pose_msg.pose.orientation.x = msg.pose.orientation.x
        pose_msg.pose.orientation.y = msg.pose.orientation.y
        pose_msg.pose.orientation.z = msg.pose.orientation.z
        pose_msg.pose.orientation.w = msg.pose.orientation.w
        
        self.publishers_object.publish(pose_msg)

    
    def state_object_callback(self, msg):
        self._state_object = msg.data


    def tcp_client(self):
        while not rospy.is_shutdown() and self.running:
            try:
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.connect((self.TCP_IP, self.TCP_PORT))
                
                while not rospy.is_shutdown() and self.running:
                    # First line contains number of objects
                    data = sock.recv(4096).decode('utf-8')
                    if not data:
                        break
                    
                    lines = data.strip().split('\n')
                    num_objects = int(lines[0])
                    
                    # Process each object's position
                    for i in range(num_objects):
                        if i + 1 >= len(lines):
                            break
                            
                        parts = lines[i + 1].strip().split(';')
                        if len(parts) == 8:
                            obj_name = parts[0]
                            x, y, z = float(parts[1]), float(parts[2]), float(parts[3])
                            qx, qy, qz, qw = float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])
                            
                            self.publish_pose_rec_by_tcp(obj_name, x, y, z, qx, qy, qz, qw)
                
            except socket.error as e:
                rospy.logwarn(f"Socket error: {e}. Retrying in 5 seconds...")
                rospy.sleep(5.0)
            finally:
                sock.close()

    def shutdown(self):
        self.running = False
        self.client_thread.join()

if __name__ == '__main__':
    try:
        subscriber = RecPosition()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        if 'subscriber' in locals():
            subscriber.shutdown()