#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, Range
from geometry_msgs.msg import Twist


class MyNode(Node):
    def __init__(self):
        super().__init__("lidar_control_node")
        self.frontLeft = self.create_subscription(
            Range,
            '/camera',
            self.fl_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        self.rearRight = self.create_subscription(
            Range,
            '/range/rr',
            self.rr_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        self.publisher_ = self.create_publisher(Twist, '/range_cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)

        self.ranges = [0, 0, 0, 0] #[fr, fl, rr, rl]
        self.get_logger().info("range_control_node started")


    def timer_callback(self):
        control_msg = Twist()
        if (self.ranges[0] < 0.3) or (self.ranges[1] < 0.3):
            control_msg.linear.x = 0.0
        else:
            control_msg.linear.x = min((self.ranges[0] + self.ranges[1]) / 2, 0.7) / 0.7
        self.publisher_.publish(control_msg)
        self.get_logger().info("fr: "+ str(self.ranges[0]) + " ,fl: "+ str(self.ranges[1]) + " ,rr: "+ str(self.ranges[2]) + " ,rl: "+ str(self.ranges[3]) +" ,vx: " + str(control_msg.linear.x))


    def fr_callback(self, msg):
        if math.isnan(msg.range):
            self.ranges[0] = msg.max_range
        else:
            self.ranges[0] = msg.range


    def fl_callback(self, msg):
        if math.isnan(msg.range):
            self.ranges[1] = msg.max_range
        else:
            self.ranges[1] = msg.range


    def rr_callback(self, msg):
        if math.isnan(msg.range):
            self.ranges[2] = msg.max_range
        else:
            self.ranges[2] = msg.range


    def rl_callback(self, msg):
        if math.isnan(msg.range):
            self.ranges[3] = msg.max_range
        else:
            self.ranges[3] = msg.range


def main(args=None):
    rclpy.init(args=args)
    my_node = MyNode()
    rclpy.spin(my_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
