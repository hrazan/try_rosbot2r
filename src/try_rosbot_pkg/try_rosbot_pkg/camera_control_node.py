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
        self.publisher_ = self.create_publisher(Twist, '/camera_cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("camera_control_node started")


    def timer_callback(self):
        control_msg = Twist()
        self.publisher_.publish(control_msg)
        #self.get_logger().info("fr: "+ str(self.ranges[0]) + " ,fl: "+ str(self.ranges[1]) + " ,rr: "+ str(self.ranges[2]) + " ,rl: "+ str(self.ranges[3]) +" ,vx: " + str(control_msg.linear.x))


    def depthImage_callback(self, msg):
        bridge = CvBridge()
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        #self.get_logger().info(str(cv_img.shape) + " ,[240, 320]: " + str(cv_img[240, 320]))


    def depthPoints_callback(self, msg):
        img_points = np.array([msg.data])
        #self.get_logger().info("pts_shape: " + str(img_points.shape) + " " + str(len(msg.data)))


def main(args=None):
    rclpy.init(args=args)
    my_node = MyNode()
    rclpy.spin(my_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
