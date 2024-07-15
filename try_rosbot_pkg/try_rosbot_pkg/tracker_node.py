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
        self.__visualization_pub = self.create_publisher(Image, '/visualization', 10)
        self.__vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.__is_tracker_initialized = False
        self.tracker = None

        self.get_logger().info("tracker_node started")


    def __image_callback(self, msg):
        cv_bridge = CvBridge()
        cv_image  = cv_bridge.imgmsg_to_cv2(msg, "bgr8") #desired_encoding="passthrough")
        frame     = cv_image
        obj       = None        
        vel_msg   = Twist()
        
        if not self.__is_tracker_initialized:
            obj, self.tracker = self.__init_tracker(frame)
        
        ok, obj = self.tracker.update(frame)
        
        if ok:
            self.get_logger().info(str(obj))
            vel_msg = self.__designate_control(vel_msg, obj, msg.width);
            self.get_logger().info(f"Angular velocity: {vel_msg.angular.z}")
        else:
            self.get_logger().info("Tracking failure detected. Stop vehicle!")
            cv2.putText(frame, "tracking failure detected", (100, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
        	
        self.__vel_pub.publish(vel_msg)
        cv2.rectangle(frame, (int(obj[0]), int(obj[1])), (int(obj[0]+obj[2]), int(obj[1]+obj[3])), (255, 0, 0), 2)
        
        cv_image = frame
        img_msg = cv_bridge.cv2_to_imgmsg(cv_image, "bgr8")
        self.__visualization_pub.publish(img_msg)
        #self.get_logger().info(str(cv_img.shape) + " ,[240, 320]: " + str(cv_img[240, 320]))


    def __init_tracker(self, frame):
        _obj = cv2.selectROI("ROI selector", frame, False)
        _tracker = cv2.legacy.TrackerKCF_create()
        _tracker.init(frame, _obj)
        self.__is_tracker_initialized = True;
        cv2.destroyWindow("ROI selector")
        cv2.waitKey(1)
        return _obj, _tracker
        #self.get_logger().info("pts_shape: " + str(img_points.shape) + " " + str(len(msg.data)))
        

    def __designate_control(self, vel_msg, obj, img_width):
        obj_x_center = obj[0] + obj[2] / 2
        px_to_center = img_width / 2 - obj_x_center
        ang_vel = self.ANGULAR_GAIN * px_to_center / float(img_width)
        
        if (ang_vel >= -self.MAX_ANG_VEL and ang_vel <= -self.MIN_ANG_VEL) or (ang_vel >= self.MIN_ANG_VEL and ang_vel <= self.MAX_ANG_VEL):
            vel_msg.angular.z = ang_vel;
        
        return vel_msg



def main(args=None):
    rclpy.init(args=args)
    my_node = MyNode()
    rclpy.spin(my_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
