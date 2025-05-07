#!/usr/bin/env python
import math
import numpy
from math import *
import rospy
import tf
from std_msgs.msg import Float64, Float32
from geometry_msgs.msg import PoseStamped, Vector3, Pose
import tf.transformations
import socket
import sys
import time
import threading

class SimuSendUGVCommand(object):
    def __init__(self):
        rospy.init_node("Simu_Send_UGV_Command")
        self.in_simu = rospy.get_param('~in_simu', False)
        self.host = '127.0.0.1' if self.in_simu else rospy.get_param('~host', '172.26.209.13')
        self.port = rospy.get_param('~port', 62733)
        
        self._steering_cmd_sub = rospy.Subscriber("steering", Float32, self.steering_cmd_callback, queue_size=1)
        self._throttle_cmd_sub = rospy.Subscriber("throttle", Float32, self.throttle_cmd_callback, queue_size=1)
        
        self._speed = 0.0
        self._steer_ang = 0.0
        self._connected = False
        self.sock = None
        
        # Paramètre pour la fréquence d'envoi (0.05s = 20Hz)
        self.send_rate = 0.05
        
        # Démarrer un thread séparé pour gérer la connexion et l'envoi
        self.connection_thread = threading.Thread(target=self.connection_manager)
        self.connection_thread.daemon = True
        self.connection_thread.start()
        
        rospy.loginfo("SimuSendUGVCommand initialized")
    
    def throttle_cmd_callback(self, msg):
        self._speed = -msg.data
        
    def steering_cmd_callback(self, msg):
        self._steer_ang = -msg.data
    
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
                message = f'{self._steer_ang} - {self._speed}'
                try:
                    self.sock.sendall(message.encode())
                    rospy.logdebug(f"Sent: {message}")
                    
                    # Facultatif: recevoir une réponse
                    try:
                        data = self.sock.recv(16)
                        if data:
                            rospy.logdebug(f"Received: {data.decode()}")
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
    node = SimuSendUGVCommand()
    node.spin()

# main
if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass