#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan, Range
from geometry_msgs.msg import Twist
from std_msgs.msg import String


class MyNode(Node):
    def __init__(self):
        super().__init__("range_control_node")
        self.frontRight = self.create_subscription(
            Range,
            '/range/fr',
            self.fr_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        self.frontLeft = self.create_subscription(
            Range,
            '/range/fl',
            self.fl_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        self.rearRight = self.create_subscription(
            Range,
            '/range/rr',
            self.rr_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        self.rearLeft = self.create_subscription(
            Range,
            '/range/rl',
            self.rl_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        self.__input = self.create_subscription(
            String,
            '/control_input',
            self.__input_callback,
            10
        )
        self.publisher_ = self.create_publisher(Twist, '/range_cmd_vel', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.__free_mode = False

        self.ranges = [0, 0, 0, 0] #[fr, fl, rr, rl]
        self.get_logger().info("range_control_node started")


    def __input_callback(self, msg):
        if msg.data == "free_mode":
            self.__free_mode = True
        elif msg.data == "pursue_mode":
            self.__free_mode = False
        else:
            self.get_logger().info(f"input error: {msg}")
            
            
    def timer_callback(self):
        control_msg = Twist()
        if self.__free_mode:
            if (self.ranges[0] < 0.3) or (self.ranges[1] < 0.3):
                control_msg.linear.x = 0.0
            else:
                control_msg.linear.x = min((self.ranges[0] + self.ranges[1]) / 2, 0.7) / 0.7
        else:
            if (self.ranges[0] < 0.3) or (self.ranges[1] < 0.3):
                control_msg.angular.x = 1.0
        self.publisher_.publish(control_msg)
        #self.get_logger().info("fr: "+ str(self.ranges[0]) + " ,fl: "+ str(self.ranges[1]) + " ,rr: "+ str(self.ranges[2]) + " ,rl: "+ str(self.ranges[3]) +" ,vx: " + str(control_msg.linear.x))


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
