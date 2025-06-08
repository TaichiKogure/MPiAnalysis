#!/usr/bin/env python3
# encoding: utf-8

import rclpy
import cv2
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Header

class RobotControlNode(Node):
    """
    Robot control node for MentorPi robot.
    
    This node handles remote control commands and provides access to sensor data.
    It subscribes to the camera and LiDAR topics, compresses the data, and republishes
    it on topics that can be accessed by the PC control node.
    """
    
    def __init__(self):
        super().__init__('robot_control')
        
        # Declare parameters
        self.declare_parameter('enable_camera', True)
        self.declare_parameter('enable_lidar', True)
        self.declare_parameter('camera_topic', '/depth_cam/rgb/image_raw')
        self.declare_parameter('lidar_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/controller/cmd_vel')
        
        # Get parameter values
        self.enable_camera = self.get_parameter('enable_camera').value
        self.enable_lidar = self.get_parameter('enable_lidar').value
        self.camera_topic = self.get_parameter('camera_topic').value
        self.lidar_topic = self.get_parameter('lidar_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Create publishers
        self.camera_pub = self.create_publisher(Image, '~/camera', 10)
        self.lidar_pub = self.create_publisher(LaserScan, '~/lidar', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        
        # Create subscriptions
        if self.enable_camera:
            self.camera_sub = self.create_subscription(
                Image,
                self.camera_topic,
                self.camera_callback,
                10)
        
        if self.enable_lidar:
            self.lidar_sub = self.create_subscription(
                LaserScan,
                self.lidar_topic,
                self.lidar_callback,
                10)
        
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '~/cmd_vel',
            self.cmd_vel_callback,
            10)
        
        self.get_logger().info('Robot control node initialized')
    
    def camera_callback(self, msg):
        """
        Process camera data and republish it.
        
        This function converts the camera image to a compressed format
        to reduce bandwidth usage.
        """
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Resize image to reduce bandwidth
            resized_image = cv2.resize(cv_image, (320, 240))
            
            # Convert back to ROS Image message
            ros_image = self.bridge.cv2_to_imgmsg(resized_image, encoding='bgr8')
            ros_image.header = msg.header
            
            # Publish compressed image
            self.camera_pub.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f'Error processing camera data: {e}')
    
    def lidar_callback(self, msg):
        """
        Process LiDAR data and republish it.
        
        This function passes through the LiDAR data without modification.
        """
        try:
            # Republish LiDAR data
            self.lidar_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error processing LiDAR data: {e}')
    
    def cmd_vel_callback(self, msg):
        """
        Process command velocity messages and forward them to the robot.
        
        This function passes through the command velocity messages without modification.
        """
        try:
            # Forward command velocity to the robot
            self.cmd_vel_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f'Error processing command velocity: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = RobotControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()