#!/usr/bin/env python3
import rclpy
import math
import numpy as np
import cv2
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, Range, Image, PointCloud2
from geometry_msgs.msg import Twist


class MyNode(Node):
    def __init__(self):
        super().__init__("camera_control_node")
        self.depthImage = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depthImage_callback,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        self.depthPoints = self.create_subscription(
            PointCloud2,
            '/camera/depth/points',
            self.depthPoints_callback,
            QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        self.__visualization_pub = self.create_publisher(Image, '/visualization', 10)
        self.__visualization_debug_pub = self.create_publisher(Image, '/visualization_debug', 10)
        self.publisher_ = self.create_publisher(Twist, '/camera_cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.__debug = True
        #RGB DARK GREY
        self.__hue_min = 0
        self.__hue_max = 0
        self.__sat_min = 0
        self.__sat_max = 0
        self.__val_min = 1
        self.__val_max = 15
        self.get_logger().info("camera_control_node started")


    def timer_callback(self):
        control_msg = Twist()
        self.publisher_.publish(control_msg)
        #self.get_logger().info("fr: "+ str(self.ranges[0]) + " ,fl: "+ str(self.ranges[1]) + " ,rr: "+ str(self.ranges[2]) + " ,rl: "+ str(self.ranges[3]) +" ,vx: " + str(control_msg.linear.x))


    def depthImage_callback(self, msg):
        cv_bridge = CvBridge()
        """ HSV method
        frame__   = cv_bridge.imgmsg_to_cv2(msg, "16UC1")
        frame_    = ((frame__ / np.max(frame__)) * 255).astype("uint8")
        frame     = cv2.cvtColor(frame_, cv2.COLOR_GRAY2BGR)
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
                x,y,w,h  = cv2.boundingRect(c)
                #self.get_logger().info(f"area: {area}")
                
        vel_msg  = Twist()
        is_found = False
        if w > 0 and h > 0:
            is_found = True
            cv2.putText(frame, f"x: {x}, y: {y}", (x, y+h+20), cv2.FONT_HERSHEY_COMPLEX, 0.7, (0, 0, 255), 2)
            cv2.putText(frame, f"w: {w}, y: {h}", (x, y+h+40), cv2.FONT_HERSHEY_COMPLEX, 0.7, (0, 0, 255), 2)
            obj_x_center      = x + (w / 2)
            px_to_center      = (msg.width / 2) - obj_x_center
            vel_msg.angular.z = px_to_center / (msg.width / 2)
            vel_msg.angular.x = 0.0
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 0, 255), 2)
        
        if not is_found:
            vel_msg.angular.z = 0.0
            vel_msg.angular.x = 1.0
        
        self.publisher_.publish(vel_msg)
        #self.get_logger().info(f"ang_vel_x: {vel_msg.angular.x}, ang_vel_z: {vel_msg.angular.z}")
        """
        
        #""" GREY method
        frame_    = cv_bridge.imgmsg_to_cv2(msg, "16UC1")
        frame     = ((frame_ / np.max(frame_)) * 255).astype("uint8")
        ret, mask = cv2.threshold(frame, 15, 255, cv2.THRESH_TOZERO_INV)
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
            is_found = True
            cv2.putText(frame, f"x: {x}, y: {y}", (x, y+h+20), cv2.FONT_HERSHEY_COMPLEX, 0.7, 255, 2)
            cv2.putText(frame, f"w: {w}, y: {h}", (x, y+h+40), cv2.FONT_HERSHEY_COMPLEX, 0.7, 255, 2)
            obj_x_center      = x + (w / 2)
            px_to_center      = (msg.width / 2) - obj_x_center
            vel_msg.angular.z = px_to_center / (msg.width / 2)
            vel_msg.angular.x = 0.0
            cv2.rectangle(frame, (x, y), (x+w, y+h), 255, 2)
        
        if not is_found:
            vel_msg.angular.z = 0.0
            vel_msg.angular.x = 1.0
        
        self.publisher_.publish(vel_msg)
        #"""
        
        #""" DEBUG
        if self.__debug:
            mask_img_msg = cv_bridge.cv2_to_imgmsg(mask, "mono8")
            self.__visualization_debug_pub.publish(mask_img_msg)
        #"""
        #"""
        img_msg = cv_bridge.cv2_to_imgmsg(frame, "8UC1") #8UC3
        self.__visualization_pub.publish(img_msg)
        #"""


    def depthPoints_callback(self, msg):
        x = 0
        #img_points = np.array([msg.data])
        #self.get_logger().info("pts_shape: " + str(img_points.shape) + " " + str(min(msg.data)) + " " + str(max(msg.data)))


def main(args=None):
    rclpy.init(args=args)
    my_node = MyNode()
    rclpy.spin(my_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
