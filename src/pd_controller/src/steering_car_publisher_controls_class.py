#!/usr/bin/env python

"""
======================================================
 Fichier     : steering_car_publisher_controls_class.py
 Auteur      : Alessandra Elisa Sindi Morando
 Créé en     : ?
 Description : Fonction pour le PD controleur de la jetracer pour contrôler l'UGV dans la volière.
======================================================
"""
import rospy
from std_msgs.msg import Float32


class PublisherControls:
    def __init__(self, steering_gain, steering_offset, throttle_gain):
        # Gains and offsets of the steering car
        self.__steering_gain = steering_gain
        self.__steering_offset = steering_offset
        self.__throttle_gain = throttle_gain
        
        # Initialize the Publisher
        # https://stackoverflow.com/questions/57370573/why-is-ros-publisher-not-publishing-first-message
        self.steering_pub = rospy.Publisher("/steering", Float32, queue_size=10)
        self.throttle_pub = rospy.Publisher("/throttle", Float32, queue_size=10)           

    def publish_control_inputs(self, u_k):
            # https://www.waveshare.com/wiki/JetRacer_AI_Kit                        
            # y = a * x + b -> x = (y - b) / a
            steering_y = - u_k[0]
            steering_x = (steering_y - self.__steering_offset)/self.__steering_gain
            
            # y = a * x -> x = y / a 
            throttle_y = - u_k[1]
            throttle_x = throttle_y/self.__throttle_gain

            # Publish Control inputs
            throttle_msg = Float32()
            throttle_msg.data = throttle_x
            
            steering_msg = Float32()
            steering_msg.data = steering_x            
            
            self.throttle_pub.publish(throttle_msg)
            self.steering_pub.publish(steering_msg)
    