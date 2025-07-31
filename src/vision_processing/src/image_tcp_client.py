#!/usr/bin/env python

"""
======================================================
 Fichier     : image_tcp_client.py
 Auteur      : Aurélien Garreau
 Créé en     : 2025
 Description : Noeud ROS pour recevoir un flux d'images via TCP et les publier sur des topics ROS
              dans différents formats (GRAY, RGB, RGBA, YUV, CMYK).
======================================================
"""

import rospy
import socket
import struct
import time
import os
import numpy as np
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

HEADER_FORMAT = "!IIIIII"  # 6 uint32_t : magic, width, height, channels, frameNumber, timestamp
HEADER_SIZE = struct.calcsize(HEADER_FORMAT)
MAGIC_NUMBER = 0x12345678

class TcpImageStreamNode:
    def __init__(self):
        rospy.init_node('tcp_image_stream_client', anonymous=True)
        
        # Get parameters
        self.in_simu = rospy.get_param('~in_simu', False)
        self.host = '127.0.0.1' if self.in_simu else rospy.get_param('~host', '172.26.209.28')
        self.port = rospy.get_param('~port', 62734)
        self.save = rospy.get_param('~save', False)
        self.save_dir = rospy.get_param('~save_dir', 'frames')
        self.display = rospy.get_param('~display', True)
        
        # Initialize CV bridge for ROS image conversion
        self.bridge = CvBridge()
        
        # Create image publishers for different formats
        self.gray_pub = rospy.Publisher('~image_gray', Image, queue_size=10)
        self.rgb_pub = rospy.Publisher('~image_rgb', Image, queue_size=10)
        self.rgba_pub = rospy.Publisher('~image_rgba', Image, queue_size=10)
        self.yuv_pub = rospy.Publisher('~image_yuv', Image, queue_size=10)
        self.cmyk_pub = rospy.Publisher('~image_cmyk', Image, queue_size=10)
        
        rospy.loginfo(f"TCP Image Stream Client connecting to {self.host}:{self.port}")
        rospy.loginfo(f"Running in simulation mode: {self.in_simu}")
        
    def receive_exact(self, sock, n):
        data = b''
        while len(data) < n:
            packet = sock.recv(n - len(data))
            if not packet:
                return None
            data += packet
        return data
    
    def try_decode_and_show(self, image_data, width, height, frame_number, timestamp):
        attempts = [
            ("GRAY", 1, lambda img: img, self.gray_pub),
            ("RGB", 3, lambda img: cv2.cvtColor(img, cv2.COLOR_RGB2BGR), self.rgb_pub),
            ("RGBA", 4, lambda img: cv2.cvtColor(img, cv2.COLOR_RGBA2BGRA), self.rgba_pub),
            ("YUV", 3, lambda img: cv2.cvtColor(img, cv2.COLOR_YUV2BGR), self.yuv_pub),
            ("CMYK", 4, self.cmyk_to_bgr, self.cmyk_pub),
        ]
        
        for name, ch, converter, publisher in attempts:
            try:
                img = np.frombuffer(image_data, dtype=np.uint8).reshape((height, width, ch))
                result = converter(img)
                
                # Publish to ROS topic
                if name == "GRAY":
                    # Special handling for grayscale
                    if ch == 1:
                        ros_image = self.bridge.cv2_to_imgmsg(img, encoding="mono8")
                    else:
                        continue
                else:
                    ros_image = self.bridge.cv2_to_imgmsg(result, encoding="bgr8")
                
                ros_image.header.stamp = rospy.Time.from_sec(timestamp / 1000.0)
                ros_image.header.frame_id = f"frame_{frame_number}"
                publisher.publish(ros_image)
                
                # Display if enabled
                if self.display:
                    window_name = f"{name} format"
                    cv2.imshow(window_name, result)
            except Exception as e:
                pass  # Silent if format error
    
    def cmyk_to_bgr(self, img_array):
        cmyk = img_array.astype(np.float32) / 255.0
        C, M, Y, K = cmyk[..., 0], cmyk[..., 1], cmyk[..., 2], cmyk[..., 3]
        R = (1.0 - np.minimum(1.0, C * (1 - K) + K))
        G = (1.0 - np.minimum(1.0, M * (1 - K) + K))
        B = (1.0 - np.minimum(1.0, Y * (1 - K) + K))
        bgr = np.stack((B, G, R), axis=-1) * 255
        return bgr.astype(np.uint8)
    
    def run(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        
        try:
            sock.connect((self.host, self.port))
            rospy.loginfo("Connected. Receiving stream...")
            
            frame_count = 0
            start_time = time.time()
            
            while not rospy.is_shutdown():
                header_data = self.receive_exact(sock, HEADER_SIZE)
                if not header_data:
                    rospy.logwarn("Connection closed by server (header)")
                    break
                
                magic, width, height, channels, frame_number, timestamp = struct.unpack(HEADER_FORMAT, header_data)
                
                if magic != MAGIC_NUMBER:
                    rospy.logerr(f"Invalid magic number: {magic:x}")
                    break
                
                image_size = width * height * channels
                image_data = self.receive_exact(sock, image_size)
                if not image_data:
                    rospy.logwarn("Connection closed (image)")
                    break
                
                self.try_decode_and_show(image_data, width, height, frame_number, timestamp)
                
                if self.save and frame_count % 30 == 0:
                    os.makedirs(self.save_dir, exist_ok=True)
                    path = f"{self.save_dir}/frame_{frame_count:06d}.bin"
                    with open(path, "wb") as f:
                        f.write(image_data)
                    rospy.loginfo(f"Frame saved: {path}")
                
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    rospy.loginfo("Exit requested by user")
                    break
                
                frame_count += 1
                if frame_count % 30 == 0:
                    elapsed = time.time() - start_time
                    fps = 30 / elapsed if elapsed > 0 else 0
                    start_time = time.time()
                    rospy.loginfo(f"Frame {frame_count} | {width}x{height} | {channels} channels | FPS: {fps:.2f}")
        
        except rospy.ROSInterruptException:
            rospy.loginfo("Interrupted by ROS")
        except KeyboardInterrupt:
            rospy.loginfo("Interrupted by user")
        except Exception as e:
            rospy.logerr(f"Error: {e}")
        finally:
            sock.close()
            cv2.destroyAllWindows()
            rospy.loginfo("Client terminated.")

if __name__ == "__main__":
    node = TcpImageStreamNode()
    node.run()