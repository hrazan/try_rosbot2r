#!/usr/bin/env python3
import rclpy
import math
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class MyNode(Node):
    def __init__(self):    	
        super().__init__("tracker_input_node")
        self.publisher_ = self.create_publisher(String, '/tracker_input', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        #RGB (255, 0, 255) PURPLE
        self.__hue_min   = 150
        self.__hue_max   = 170
        self.__sat_min   = 100
        self.__sat_max   = 255
        self.__val_min   = 100
        self.__val_max   = 255
        self.get_logger().info("tracker_input_node started")

    
    def timer_callback(self):
        send_data = String()
        input_data = input('"debug_on" or "debug_off"?\n')
        if input_data == "debug_on":
            self.__debug = True
        elif input_data == "debug_off":
            self.__debug = False
        else:
            self.get_logger().info(f"input error")
            return
        
        send_data.data = input_data
        self.publisher_.publish(send_data)



def main(args=None):
    rclpy.init(args=args)
    my_node = MyNode()
    rclpy.spin(my_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
