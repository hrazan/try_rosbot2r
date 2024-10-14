#!/usr/bin/env python3
import rclpy
import math
import numpy as np
import cv2
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class MyNode(Node):
    def __init__(self):
        super().__init__("camera_control_node")
        self.depthImage = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depthImage_callback,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        self.__input = self.create_subscription(
            String,
            '/control_input',
            self.__input_callback,
            10
        )
        self.__visualization_pub = self.create_publisher(Image, '/camera_visualization', 10)
        self.__visualization_debug_pub = self.create_publisher(Image, '/camera_visualization_debug', 10)
        self.publisher_ = self.create_publisher(Twist, '/camera_cmd_vel', 10)

        self.__debug = False
        self.get_logger().info("camera_control_node started")


    def __input_callback(self, msg):
        if msg.data == "debug_on":
            self.__debug = True
        elif msg.data == "debug_off":
            self.__debug = False
        else:
            self.get_logger().info(f"input error: {msg}")
            
            
    def depthImage_callback(self, msg):
        cv_bridge = CvBridge()        
        #""" GREY method
        frame_    = cv_bridge.imgmsg_to_cv2(msg, "16UC1")
        frame     = ((frame_ / np.max(frame_)) * 255).astype("uint8")
        ret, mask = cv2.threshold(frame, 25, 255, cv2.THRESH_TOZERO_INV)
        ret, mask = cv2.threshold(mask, 1, 255, cv2.THRESH_BINARY)
        cnts, hei = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        max_area  = 0
        x         = 0
        y         = 0
        w         = 0
        h         = 0
        approx    = []
        
        for c in cnts:
            area = cv2.contourArea(c)
            if area > 300 and area > max_area:
                max_area = area
                peri     = cv2.arcLength(c, True)
                x,y,w,h  = cv2.boundingRect(c)
                #self.get_logger().info(f"area: {area}")
                
        vel_msg  = Twist()
        is_found = False
        if w > 0 and h > 0:
            if self.__debug:
                cv2.putText(frame, f"x: {x}, y: {y}", (x, y+h+20), cv2.FONT_HERSHEY_COMPLEX, 0.7, 255, 2)
                cv2.putText(frame, f"w: {w}, h: {h}", (x, y+h+40), cv2.FONT_HERSHEY_COMPLEX, 0.7, 255, 2)
                cv2.rectangle(frame, (x, y), (x+w, y+h), 255, 2)
            obj_x_center      = x + (w / 2)
            obj_y_center      = y + (h / 2)
            px_to_center      = (msg.width / 2) - obj_x_center
            if (y + h) > (msg.height / 2):
                vel_msg.angular.z = (1 - abs(px_to_center / (msg.width / 2))) * (px_to_center / abs(px_to_center))
        
        self.publisher_.publish(vel_msg)
        if self.__debug:
            self.get_logger().info(f"wz: {vel_msg.angular.z}") #, y+h: {y + h}, h/2: {msg.height / 2}")
        #"""
        
        #""" DEBUG
        if self.__debug:
            mask_img_msg = cv_bridge.cv2_to_imgmsg(mask, "mono8")
            self.__visualization_debug_pub.publish(mask_img_msg)
            img_msg = cv_bridge.cv2_to_imgmsg(frame, "8UC1") #8UC3
            self.__visualization_pub.publish(img_msg)
        #"""


def main(args=None):
    rclpy.init(args=args)
    my_node = MyNode()
    rclpy.spin(my_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
