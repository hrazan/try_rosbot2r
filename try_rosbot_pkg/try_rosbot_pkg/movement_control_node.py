#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, Range
from geometry_msgs.msg import Twist


class MyNode(Node):
    def __init__(self):
        super().__init__("movement_control_node")
        self.rangeCmd = self.create_subscription(
            Twist,
            '/range_cmd_vel',
            self.rangeCmd_callback,
            10
        )
        self.lidarCmd = self.create_subscription(
            Twist,
            '/lidar_cmd_vel',
            self.lidarCmd_callback,
            10
        )
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.rangeCmd_msg = Twist()
        self.lidarCmd_msg = Twist()
        self.maxLinearVelocity  = 0.25
        self.maxAngularVelocity = 2.00
        self.get_logger().info("movement_control_node started")


    def timer_callback(self):
        control_msg = Twist()
        if self.rangeCmd_msg.linear.x == 0.0 or self.lidarCmd_msg.linear.x == 0.0:
             control_msg.linear.x  = 0.0
             control_msg.angular.z = 1.0 * self.maxAngularVelocity
        else:
             control_msg.linear.x  = ((self.rangeCmd_msg.linear.x + self.lidarCmd_msg.linear.x) / 2) * self.maxLinearVelocity
             control_msg.angular.z = self.lidarCmd_msg.angular.z * self.maxAngularVelocity

        self.publisher_.publish(control_msg)
        self.get_logger().info("rvx: " + str(self.rangeCmd_msg.linear.x) + " ,rwz: " + str(self.rangeCmd_msg.angular.z) + " ,lvx: " + str(self.lidarCmd_msg.linear.x) + " ,lwz: " + str(self.lidarCmd_msg.angular.z) + " ,vx: " + str(control_msg.linear.x) + " ,wz: " + str(control_msg.angular.z))


    def rangeCmd_callback(self, msg):
        self.rangeCmd_msg = msg


    def lidarCmd_callback(self, msg):
        self.lidarCmd_msg = msg


def main(args=None):
    rclpy.init(args=args)
    my_node = MyNode()
    rclpy.spin(my_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
