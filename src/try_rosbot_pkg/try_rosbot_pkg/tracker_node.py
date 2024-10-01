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
        self.__vel_pub = self.create_publisher(Twist, '/tracker_cmd_vel', 10)
        self.__is_tracker_initialized = False
        self.tracker = None
        self.__lastTrack = False
        self.get_logger().info("tracker_node started")
    
    
    def __empty(self, img):
        pass


    def __image_callback0(self, msg):
        cv_bridge = CvBridge()
        cv_image  = cv_bridge.imgmsg_to_cv2(msg, "bgr8") #desired_encoding="passthrough")
        frame     = cv_image
        obj       = None        
        vel_msg   = Twist()
        
        if not self.__is_tracker_initialized:
            obj, self.tracker = self.__init_tracker(frame)
        
        if self.__lastTrack == False:
            self.tracker.init(frame, obj)
        
        ok, obj = self.tracker.update(frame)
        
        if ok:
            self.__lastTrack = True
            vel_msg = self.__designate_control(vel_msg, obj, msg.width);
            self.get_logger().info(f"Angular velocity: {vel_msg.angular.z}, Obj: {obj}")
        else:
            self.__lastTrack = False
            self.get_logger().info("Tracking failure detected. Stop vehicle!")
            cv2.putText(frame, "tracking failure detected", (100, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 0, 255), 2)
        	
        self.__vel_pub.publish(vel_msg)
        cv2.rectangle(frame, (int(obj[0]), int(obj[1])), (int(obj[0]+obj[2]), int(obj[1]+obj[3])), (255, 0, 0), 2)
        
        cv_image = frame
        img_msg = cv_bridge.cv2_to_imgmsg(cv_image, "bgr8")
        self.__visualization_pub.publish(img_msg)
        #self.get_logger().info(str(cv_img.shape) + " ,[240, 320]: " + str(cv_img[240, 320]))
    
    
    def __image_callback(self, msg):
        cv_bridge = CvBridge()
        cv_image  = cv_bridge.imgmsg_to_cv2(msg, "bgr8") #desired_encoding="passthrough")
        frame     = cv_image
        hsv       = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        hue_min   = 135
        hue_max   = 175
        sat_min   = 100
        sat_max   = 255
        val_min   = 100
        val_max   = 255
        lower     = np.array([hue_min, sat_min, val_min])
        upper     = np.array([hue_max, sat_max, val_max])
        mask      = cv2.inRange(hsv, lower, upper)
        cnts, hei = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
        for c in cnts:
            area = cv2.contourArea(c)
            if area > 300:
                self.get_logger().info(f"area: {area}")
                x, y, w, h = cv2.boundingRect(c)
                cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
        
        obj       = None        
        vel_msg   = Twist()
        
        """
        cv_image = mask
        mask_img_msg = cv_bridge.cv2_to_imgmsg(cv_image, "mono8")
        cv2.imshow("Mask", mask)
        """
        cv_image = frame
        img_msg = cv_bridge.cv2_to_imgmsg(cv_image, "bgr8")
        self.__visualization_pub.publish(img_msg)
        
        #self.get_logger().info(str(cv_img.shape) + " ,[240, 320]: " + str(cv_img[240, 320]))


    def __init_tracker(self, frame):
        _obj = cv2.selectROI("ROI selector", frame, False)
        #""" KCF
        _tracker = cv2.legacy.TrackerKCF_create()
        #"""
        """ DaSiamRPN: takes too much time (2 sec)
        _params = cv2.TrackerDaSiamRPN_Params()
        _params.model = "/home/husarion/model/DaSiamRPN/dasiamrpn_model.onnx"
        _params.kernel_r1 = "/home/husarion/model/DaSiamRPN/dasiamrpn_kernel_r1.onnx"
        _params.kernel_cls1 = "/home/husarion/model/DaSiamRPN/dasiamrpn_kernel_cls1.onnx"
        _tracker = cv2.TrackerDaSiamRPN_create(_params)
        """
        """ MOSSE: no good precision
        _tracker = cv2.legacy.TrackerMOSSE_create()
        """
        """ CSRT: hmmmmm
        _tracker = cv2.TrackerCSRT_create()
        """
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
