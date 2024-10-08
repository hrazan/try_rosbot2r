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
        self.MIN_ANG_VEL  = 0.15
        self.MAX_ANG_VEL  = 0.5
        self.ANGULAR_GAIN = 1.7
    	
        super().__init__("tracker_node")
        self.__img_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.__image_callback,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        self.__input = self.create_subscription(
            String,
            '/tracker_input',
            self.__input_callback,
            10
        )
        self.__visualization_pub = self.create_publisher(Image, '/visualization', 10)
        self.__visualization_debug_pub = self.create_publisher(Image, '/visualization_debug', 10)
        self.__vel_pub = self.create_publisher(Twist, '/tracker_cmd_vel', 10) #'/tracker_cmd_vel'
        self.__debug = False
        #RGB (255, 0, 255) PURPLE
        self.__hue_min   = 150
        self.__hue_max   = 170
        self.__sat_min   = 100
        self.__sat_max   = 255
        self.__val_min   = 100
        self.__val_max   = 255
        self.get_logger().info("tracker_node started")
    
    
    def __input_callback(self, msg):
        if msg.data == "debug_on":
            self.__debug = True
        elif msg.data == "debug_off":
            self.__debug = False
        else:
            self.get_logger().info(f"input error: {msg}")
    
    
    def __image_callback(self, msg):
        cv_bridge = CvBridge()
        frame     = cv_bridge.imgmsg_to_cv2(msg, "bgr8")
        hsv       = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hue_min   = self.__hue_min
        hue_max   = self.__hue_max
        sat_min   = self.__sat_min
        sat_max   = self.__sat_max
        val_min   = self.__val_min
        val_max   = self.__val_max
        lower     = np.array([hue_min, sat_min, val_min])
        upper     = np.array([hue_max, sat_max, val_max])
        mask      = cv2.inRange(hsv, lower, upper)
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
                approx   = cv2.approxPolyDP(c, 0.02*peri, True)
                x,y,w,h  = cv2.boundingRect(c)
                #self.get_logger().info(f"area: {area}")
        
        vel_msg  = Twist()
        is_found = False
        if w > 0 and h > 0:
            if len(approx) == 10:
                is_found = True
                cv2.putText(frame, f"x: {x}, y: {y}", (x, y+h+40), cv2.FONT_HERSHEY_COMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(frame, f"w: {w}, y: {h}", (x, y+h+60), cv2.FONT_HERSHEY_COMPLEX, 0.7, (0, 255, 0), 2)
                obj_x_center      = x + (w / 2)
                px_to_center      = (msg.width / 2) - obj_x_center
                vel_msg.angular.z = px_to_center / (msg.width / 2)
                vel_msg.angular.x = 0.0
                
            cv2.putText(frame, f"Points: {len(approx)}", (x, y+h+20), cv2.FONT_HERSHEY_COMPLEX, 0.7, (0, 255*is_found, 255*(not is_found)), 2)
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255*is_found, 255*(not is_found)), 2)
        
        if not is_found:
            vel_msg.angular.z = 0.0
            vel_msg.angular.x = 1.0
        
        self.__vel_pub.publish(vel_msg)
        #self.get_logger().info(f"ang_vel_x: {vel_msg.angular.x}, ang_vel_z: {vel_msg.angular.z}")
        
        #""" DEBUG
        if self.__debug:
            mask_img_msg = cv_bridge.cv2_to_imgmsg(mask, "mono8")
            #cv2.imshow("Mask", mask
            self.__visualization_debug_pub.publish(mask_img_msg)
        #"""
        #"""
        img_msg = cv_bridge.cv2_to_imgmsg(frame, "bgr8")
        self.__visualization_pub.publish(img_msg)
        #"""


def main(args=None):
    rclpy.init(args=args)
    my_node = MyNode()
    rclpy.spin(my_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
