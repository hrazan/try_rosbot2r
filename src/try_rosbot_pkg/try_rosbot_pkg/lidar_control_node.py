#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class MyNode(Node):
    def __init__(self):
        super().__init__("lidar_control_node")
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10
        )
        self.publisher_ = self.create_publisher(Twist, '/lidar_cmd_vel', 10)

        self.lidar_points   = 5
        self.range_total   = []
        self.range_number  = []
        self.range_average = []
        for i in range(self.lidar_points):
            self.range_total.append(0)
            self.range_number.append(0)
            self.range_average.append(0)

        self.get_logger().info("lidar_control_node started")


    def listener_callback(self, msg):
        linear_x = 0.0
        angular_z = 0.0
        lidar_data_pack = int(240.0/self.lidar_points) * 2
        data = 0.0
        for i in range(self.lidar_points):
            self.range_total[i]   = 0
            self.range_number[i]  = 0
            self.range_average[i] = 0
            for j in range(lidar_data_pack):
                k = 480 + (lidar_data_pack * i) + j
                if k >= 720:
                    k -= 720

                data = msg.ranges[k]
                if math.isfinite(data):
                    self.range_total[i]  += data
                    self.range_number[i] += 1
                else:
                    self.range_total[i]  += 0
                    self.range_number[i] += 0

            if self.range_number[i] != 0:
                self.range_average[i] = self.range_total[i] / self.range_number[i]

            if i == 0:
                self.range_average[i] = min(self.range_average[i], 0.5)
                angular_z += -(self.range_average[i] / 0.5) * 0.4
            elif i == 1:
                self.range_average[i] = min(self.range_average[i], 1.0)
                linear_x  +=  (self.range_average[i] / 1.0) * 0.25
                angular_z += -(self.range_average[i] / 1.0) * 0.6
            elif i == 2:
                self.range_average[i] = min(self.range_average[i], 2.0)
                linear_x += (self.range_average[i] / 2.0) * 0.5
            elif i == 3:
                self.range_average[i] = min(self.range_average[i], 1.0)
                linear_x  += (self.range_average[i] / 1.0) * 0.25
                angular_z += (self.range_average[i] / 1.0) * 0.6
            elif i == 4:
                self.range_average[i] = min(self.range_average[i], 0.5)
                angular_z += (self.range_average[i] / 0.5) * 0.4

        control_msg = Twist()
        control_msg.linear.x  = linear_x
        control_msg.angular.z = angular_z
        if self.range_average[2] < 0.4:
            control_msg.linear.x  = 0
            control_msg.angular.z = 1

        self.publisher_.publish(control_msg)
        #self.get_logger().info("L: " + str(self.range_average[4]) + " ,DL: " + str(self.range_average[3]) + " ,C: " + str(self.range_average[2]) + " ,DR: " + str(self.range_average[1]) + " ,R: " + str(self.range_average[0]) + " ,vx: " + str(control_msg.linear.x) + " ,wz: " + str(control_msg.angular.z))



def main(args=None):
    rclpy.init(args=args)
    my_node = MyNode()
    rclpy.spin(my_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
