#!/usr/bin/env python
import math
import numpy
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

class SendTCPCommand(object):
    def __init__(self):
        rospy.init_node("Send_by TCP_Command")
        self.in_simu = rospy.get_param('~in_simu', False)
        self.host = '127.0.0.1' if self.in_simu else rospy.get_param('~host', '172.26.209.13')
        self.port = rospy.get_param('~port', 62732)
        
        self.object_to_command = rospy.get_param('~object_to_command')
        self.command_topic = f"/error_pose/{self.object_to_command}/pose"
        self._error_pose_sub = rospy.Subscriber(self.command_topic, PoseStamped, self.error_pose_callback, queue_size=1)

        self._publisher_state = rospy.Publisher(f"/state/{self.object_to_command}", Bool, queue_size=1)
        
        self._ex = 0.0
        self._ey = 0.0
        self._eyaw = 0.0
        self._connected = False
        self.sock = None
        
        # Paramètre pour la fréquence d'envoi (0.05s = 20Hz)
        self.send_rate = 0.05
        
        # Démarrer un thread séparé pour gérer la connexion et l'envoi
        self.connection_thread = threading.Thread(target=self.connection_manager)
        self.connection_thread.daemon = True
        self.connection_thread.start()
        
        rospy.loginfo("Send TCP Command initialized")
    
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
          
    def error_pose_callback(self, msg):
        self._ex = msg.pose.position.x
        self._ey = msg.pose.position.y
        # roll_x, pitch_y, self._eyaw = self.euler_from_quaternion(
        #     msg.pose.orientation.x,msg.pose.orientation.y,msg.pose.orientation.z,msg.pose.orientation.w
        #     )
        self._eyaw = msg.pose.orientation.z
    def connection_manager(self):
        """Thread qui gère la connexion et la reconnexion TCP"""
        while not rospy.is_shutdown():
            if not self._connected:
                self.connect()
            
            if self._connected:
                self.send_message_loop()
            
            # Attendre un peu avant de tenter une reconnexion
            time.sleep(1.0)
    
    def connect(self):
        """Établit une connexion TCP avec le serveur"""
        try:
            if self.sock:
                self.sock.close()
            
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.sock.connect((self.host, self.port))
            self._connected = True
            rospy.loginfo(f"Connected to {self.host}:{self.port}")
        except Exception as e:
            self._connected = False
            rospy.logerr(f"Connection failed: {e}")
    
    def send_message_loop(self):
        """Envoie les données à intervalle régulier"""
        try:
            # Définir le socket en mode non-bloquant pour éviter les blocages pendant l'envoi
            self.sock.settimeout(1.0)
            
            while self._connected and not rospy.is_shutdown():
                message = f'{self._ex};{self._ey};{self._eyaw};'
                try:
                    self.sock.sendall(message.encode())
                    rospy.logdebug(f"Sent: {message}")
                    
                    # Facultatif: recevoir une réponse
                    try:
                        data = self.sock.recv(16)
                        if data:
                            response = data.decode()
                            if response == "True":
                                rospy.loginfo("Command accepted and will be executed")
                                self._publisher_state.publish(True)
                            elif response == "False":
                                rospy.logwarn("Command rejected")
                                self._publisher_state.publish(False)
                            else:
                                rospy.logwarn(f"Unexpected response: {response}")
                                self._publisher_state.publish(False)
                        else:
                            # Connection fermée par le serveur
                            raise Exception("Connection closed by server")
                    except socket.timeout:
                        # Pas de réponse, mais ce n'est pas un problème
                        pass
                        
                except (socket.error, socket.timeout) as e:
                    rospy.logerr(f"Socket error: {e}")
                    self._connected = False
                    break
                
                # Attendre pour respecter la fréquence d'envoi
                time.sleep(self.send_rate)
                
        except Exception as e:
            rospy.logerr(f"Error in send_message_loop: {e}")
        finally:
            self._connected = False
            if self.sock:
                try:
                    self.sock.close()
                except:
                    pass
            rospy.loginfo("Connection closed")
    
    def spin(self):
        rospy.spin()

def main():
    node = SendTCPCommand()
    node.spin()

# main
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass