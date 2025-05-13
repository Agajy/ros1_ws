#!/usr/bin/env python3

import rospy
import socket
import threading
from geometry_msgs.msg import PoseStamped
from tf.transformations import quaternion_matrix

class PositionSubscriber:
    def __init__(self):
        rospy.init_node('position_subscriber')
        
        # TCP parameters
        self.TCP_IP = rospy.get_param('~host', 'localhost')
        self.TCP_PORT = rospy.get_param('~port', 5000)
        
        self.publishers = {}
        self.running = True
        
        # Start TCP client thread
        self.client_thread = threading.Thread(target=self.tcp_client)
        self.client_thread.start()

    def create_publisher(self, object_name):
        if object_name not in self.publishers:
            self.publishers[object_name] = rospy.Publisher(
                f'/vrpn_client_node/{object_name}/pose',
                PoseStamped,
                queue_size=10
            )

    def publish_pose(self, object_name, x, y, z, qx, qy, qz, qw):
        self.create_publisher(object_name)
        
        pose_msg = PoseStamped()
        pose_msg.header.stamp = rospy.Time.now()
        pose_msg.header.frame_id = "map"
        
        pose_msg.pose.position.x = float(x)
        pose_msg.pose.position.y = float(y)
        pose_msg.pose.position.z = float(z)
        
        pose_msg.pose.orientation.x = float(qx)
        pose_msg.pose.orientation.y = float(qy)
        pose_msg.pose.orientation.z = float(qz)
        pose_msg.pose.orientation.w = float(qw)
        
        self.publishers[object_name].publish(pose_msg)

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
                            
                            self.publish_pose(obj_name, x, y, z, qx, qy, qz, qw)
                
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
        subscriber = PositionSubscriber()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        if 'subscriber' in locals():
            subscriber.shutdown()