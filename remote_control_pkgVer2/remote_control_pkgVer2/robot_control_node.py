#!/usr/bin/env python3
# encoding: utf-8

"""
Robot control node for MentorPi robot (Version 2).

This node handles remote control commands and provides access to sensor data.
It subscribes to the camera and LiDAR topics, processes the data, and republishes
it on topics that can be accessed by the PC control node.

Improvements over Version 1:
- Better support for depth camera data
- Improved error handling and recovery
- Better QoS settings for reliable communication
- Support for status reporting and diagnostics
- Ensuring compatibility with existing MentorPi functionality
"""

import rclpy
import cv2
import numpy as np
import time
import threading
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, LaserScan, CompressedImage
from geometry_msgs.msg import Twist
from std_msgs.msg import Header, String, Float32
from builtin_interfaces.msg import Time

class RobotControlNode(Node):
    """
    Robot control node for MentorPi robot.
    
    This node handles remote control commands and provides access to sensor data.
    It subscribes to the camera and LiDAR topics, processes the data, and republishes
    it on topics that can be accessed by the PC control node.
    """
    
    def __init__(self):
        super().__init__('robot_control')
        
        # Declare parameters
        self.declare_parameter('enable_camera', True)
        self.declare_parameter('enable_lidar', True)
        self.declare_parameter('enable_depth_camera', False)
        self.declare_parameter('rgb_camera_topic', '/ascamera/camera_publisher/rgb0/image')
        self.declare_parameter('depth_camera_topic', '/ascamera/camera_publisher/depth/image')
        self.declare_parameter('lidar_topic', '/scan')
        self.declare_parameter('cmd_vel_topic', '/controller/cmd_vel')
        self.declare_parameter('camera_fps', 10)  # Target FPS for camera republishing
        self.declare_parameter('lidar_fps', 5)    # Target FPS for LiDAR republishing
        self.declare_parameter('camera_width', 320)  # Resized width for bandwidth reduction
        self.declare_parameter('camera_height', 240)  # Resized height for bandwidth reduction
        self.declare_parameter('use_compressed_image', True)  # Use compressed image format
        
        # Get parameter values
        self.enable_camera = self.get_parameter('enable_camera').value
        self.enable_lidar = self.get_parameter('enable_lidar').value
        self.enable_depth_camera = self.get_parameter('enable_depth_camera').value
        self.rgb_camera_topic = self.get_parameter('rgb_camera_topic').value
        self.depth_camera_topic = self.get_parameter('depth_camera_topic').value
        self.lidar_topic = self.get_parameter('lidar_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.camera_fps = self.get_parameter('camera_fps').value
        self.lidar_fps = self.get_parameter('lidar_fps').value
        self.camera_width = self.get_parameter('camera_width').value
        self.camera_height = self.get_parameter('camera_height').value
        self.use_compressed_image = self.get_parameter('use_compressed_image').value
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Create callback groups for parallel execution
        self.camera_callback_group = MutuallyExclusiveCallbackGroup()
        self.lidar_callback_group = MutuallyExclusiveCallbackGroup()
        self.cmd_vel_callback_group = MutuallyExclusiveCallbackGroup()
        self.timer_callback_group = MutuallyExclusiveCallbackGroup()
        
        # Create QoS profiles
        self.sensor_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.reliable_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Create publishers
        if self.use_compressed_image:
            self.rgb_camera_pub = self.create_publisher(
                CompressedImage, '~/rgb_camera/compressed', 10)
            if self.enable_depth_camera:
                self.depth_camera_pub = self.create_publisher(
                    CompressedImage, '~/depth_camera/compressed', 10)
        else:
            self.rgb_camera_pub = self.create_publisher(
                Image, '~/rgb_camera', 10)
            if self.enable_depth_camera:
                self.depth_camera_pub = self.create_publisher(
                    Image, '~/depth_camera', 10)
        
        self.lidar_pub = self.create_publisher(
            LaserScan, '~/lidar', self.reliable_qos)
        self.cmd_vel_pub = self.create_publisher(
            Twist, self.cmd_vel_topic, self.reliable_qos)
        self.status_pub = self.create_publisher(
            String, '~/status', self.reliable_qos)
        self.fps_pub = self.create_publisher(
            Float32, '~/fps', 10)
        
        # Create subscriptions
        if self.enable_camera:
            self.rgb_camera_sub = self.create_subscription(
                Image,
                self.rgb_camera_topic,
                self.rgb_camera_callback,
                self.sensor_qos,
                callback_group=self.camera_callback_group)
            
            if self.enable_depth_camera:
                self.depth_camera_sub = self.create_subscription(
                    Image,
                    self.depth_camera_topic,
                    self.depth_camera_callback,
                    self.sensor_qos,
                    callback_group=self.camera_callback_group)
        
        if self.enable_lidar:
            self.lidar_sub = self.create_subscription(
                LaserScan,
                self.lidar_topic,
                self.lidar_callback,
                self.sensor_qos,
                callback_group=self.lidar_callback_group)
        
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '~/cmd_vel',
            self.cmd_vel_callback,
            self.reliable_qos,
            callback_group=self.cmd_vel_callback_group)
        
        # Create timers for rate limiting and status reporting
        self.status_timer = self.create_timer(
            1.0,  # 1 Hz
            self.publish_status,
            callback_group=self.timer_callback_group)
        
        # Initialize data storage
        self.latest_rgb_image = None
        self.latest_depth_image = None
        self.latest_lidar_scan = None
        self.rgb_frame_count = 0
        self.depth_frame_count = 0
        self.lidar_frame_count = 0
        self.rgb_last_time = time.time()
        self.depth_last_time = time.time()
        self.lidar_last_time = time.time()
        self.rgb_fps = 0.0
        self.depth_fps = 0.0
        self.lidar_fps = 0.0
        
        # Initialize error counters
        self.rgb_error_count = 0
        self.depth_error_count = 0
        self.lidar_error_count = 0
        self.cmd_vel_error_count = 0
        
        self.get_logger().info('Robot control node initialized')
        self.publish_status()  # Publish initial status
    
    def rgb_camera_callback(self, msg):
        """
        Process RGB camera data and republish it.
        
        This function converts the camera image to a compressed format
        to reduce bandwidth usage.
        """
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Resize image to reduce bandwidth
            resized_image = cv2.resize(cv_image, (self.camera_width, self.camera_height))
            
            # Update frame count and calculate FPS
            self.rgb_frame_count += 1
            current_time = time.time()
            elapsed = current_time - self.rgb_last_time
            
            if elapsed >= 1.0:
                self.rgb_fps = self.rgb_frame_count / elapsed
                self.rgb_frame_count = 0
                self.rgb_last_time = current_time
                self.get_logger().debug(f'RGB Camera FPS: {self.rgb_fps:.1f}')
                
                # Publish FPS information
                fps_msg = Float32()
                fps_msg.data = float(self.rgb_fps)
                self.fps_pub.publish(fps_msg)
            
            # Store the latest image
            self.latest_rgb_image = resized_image
            
            # Publish image based on format preference
            if self.use_compressed_image:
                # Convert to compressed image
                compressed_msg = CompressedImage()
                compressed_msg.header = msg.header
                compressed_msg.format = "jpeg"
                compressed_msg.data = np.array(cv2.imencode(
                    '.jpg', resized_image, [cv2.IMWRITE_JPEG_QUALITY, 80])[1]).tobytes()
                
                # Publish compressed image
                self.rgb_camera_pub.publish(compressed_msg)
            else:
                # Convert back to ROS Image message
                ros_image = self.bridge.cv2_to_imgmsg(resized_image, encoding='bgr8')
                ros_image.header = msg.header
                
                # Publish image
                self.rgb_camera_pub.publish(ros_image)
                
        except CvBridgeError as e:
            self.rgb_error_count += 1
            self.get_logger().error(f'CV Bridge error processing RGB camera data: {e}')
        except Exception as e:
            self.rgb_error_count += 1
            self.get_logger().error(f'Error processing RGB camera data: {e}')
    
    def depth_camera_callback(self, msg):
        """
        Process depth camera data and republish it.
        
        This function processes depth data and converts it to a format
        suitable for visualization.
        """
        if not self.enable_depth_camera:
            return
            
        try:
            # Convert ROS Image message to OpenCV image (depth data)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # Resize image to reduce bandwidth
            resized_image = cv2.resize(cv_image, (self.camera_width, self.camera_height))
            
            # Normalize depth data for visualization (0-10m mapped to 0-255)
            depth_array = np.array(resized_image, dtype=np.float32)
            cv_image_normalized = np.clip(depth_array / 10.0 * 255, 0, 255).astype(np.uint8)
            
            # Apply colormap for better visualization
            depth_colormap = cv2.applyColorMap(cv_image_normalized, cv2.COLORMAP_JET)
            
            # Update frame count and calculate FPS
            self.depth_frame_count += 1
            current_time = time.time()
            elapsed = current_time - self.depth_last_time
            
            if elapsed >= 1.0:
                self.depth_fps = self.depth_frame_count / elapsed
                self.depth_frame_count = 0
                self.depth_last_time = current_time
                self.get_logger().debug(f'Depth Camera FPS: {self.depth_fps:.1f}')
            
            # Store the latest depth image
            self.latest_depth_image = depth_colormap
            
            # Publish image based on format preference
            if self.use_compressed_image:
                # Convert to compressed image
                compressed_msg = CompressedImage()
                compressed_msg.header = msg.header
                compressed_msg.format = "jpeg"
                compressed_msg.data = np.array(cv2.imencode(
                    '.jpg', depth_colormap, [cv2.IMWRITE_JPEG_QUALITY, 80])[1]).tobytes()
                
                # Publish compressed image
                self.depth_camera_pub.publish(compressed_msg)
            else:
                # Convert back to ROS Image message
                ros_image = self.bridge.cv2_to_imgmsg(depth_colormap, encoding='bgr8')
                ros_image.header = msg.header
                
                # Publish image
                self.depth_camera_pub.publish(ros_image)
                
        except CvBridgeError as e:
            self.depth_error_count += 1
            self.get_logger().error(f'CV Bridge error processing depth camera data: {e}')
        except Exception as e:
            self.depth_error_count += 1
            self.get_logger().error(f'Error processing depth camera data: {e}')
    
    def lidar_callback(self, msg):
        """
        Process LiDAR data and republish it.
        
        This function filters and processes LiDAR data before republishing.
        """
        try:
            # Store the latest LiDAR scan
            self.latest_lidar_scan = msg
            
            # Update frame count and calculate FPS
            self.lidar_frame_count += 1
            current_time = time.time()
            elapsed = current_time - self.lidar_last_time
            
            if elapsed >= 1.0:
                self.lidar_fps = self.lidar_frame_count / elapsed
                self.lidar_frame_count = 0
                self.lidar_last_time = current_time
                self.get_logger().debug(f'LiDAR FPS: {self.lidar_fps:.1f}')
            
            # Filter out invalid readings (optional)
            # ranges = list(msg.ranges)
            # for i in range(len(ranges)):
            #     if np.isinf(ranges[i]) or ranges[i] <= 0.0:
            #         ranges[i] = 0.0
            # msg.ranges = tuple(ranges)
            
            # Republish LiDAR data
            self.lidar_pub.publish(msg)
            
        except Exception as e:
            self.lidar_error_count += 1
            self.get_logger().error(f'Error processing LiDAR data: {e}')
    
    def cmd_vel_callback(self, msg):
        """
        Process command velocity messages and forward them to the robot.
        
        This function validates and forwards command velocity messages.
        """
        try:
            # Validate velocity commands (optional)
            # Limit maximum linear and angular velocities for safety
            max_linear_speed = 0.5  # m/s
            max_angular_speed = 1.0  # rad/s
            
            validated_msg = Twist()
            validated_msg.linear.x = max(-max_linear_speed, min(max_linear_speed, msg.linear.x))
            validated_msg.linear.y = max(-max_linear_speed, min(max_linear_speed, msg.linear.y))
            validated_msg.linear.z = 0.0  # No vertical movement
            validated_msg.angular.x = 0.0  # No roll
            validated_msg.angular.y = 0.0  # No pitch
            validated_msg.angular.z = max(-max_angular_speed, min(max_angular_speed, msg.angular.z))
            
            # Forward command velocity to the robot
            self.cmd_vel_pub.publish(validated_msg)
            
            # Log command at debug level
            self.get_logger().debug(
                f'CMD_VEL: linear=({validated_msg.linear.x:.2f}, {validated_msg.linear.y:.2f}, '
                f'{validated_msg.linear.z:.2f}), angular=({validated_msg.angular.x:.2f}, '
                f'{validated_msg.angular.y:.2f}, {validated_msg.angular.z:.2f})'
            )
            
        except Exception as e:
            self.cmd_vel_error_count += 1
            self.get_logger().error(f'Error processing command velocity: {e}')
    
    def publish_status(self):
        """
        Publish status information about the node.
        
        This function publishes information about the node's status,
        including error counts and FPS rates.
        """
        try:
            status_msg = String()
            status_msg.data = (
                f"Status: RGB Camera: {'Enabled' if self.enable_camera else 'Disabled'}, "
                f"Depth Camera: {'Enabled' if self.enable_depth_camera else 'Disabled'}, "
                f"LiDAR: {'Enabled' if self.enable_lidar else 'Disabled'}\n"
                f"FPS: RGB={self.rgb_fps:.1f}, Depth={self.depth_fps:.1f}, LiDAR={self.lidar_fps:.1f}\n"
                f"Errors: RGB={self.rgb_error_count}, Depth={self.depth_error_count}, "
                f"LiDAR={self.lidar_error_count}, CMD_VEL={self.cmd_vel_error_count}"
            )
            self.status_pub.publish(status_msg)
            
            # Reset error counters after reporting
            if self.rgb_error_count > 0 or self.depth_error_count > 0 or \
               self.lidar_error_count > 0 or self.cmd_vel_error_count > 0:
                self.get_logger().warn(status_msg.data)
                
            self.rgb_error_count = 0
            self.depth_error_count = 0
            self.lidar_error_count = 0
            self.cmd_vel_error_count = 0
            
        except Exception as e:
            self.get_logger().error(f'Error publishing status: {e}')

def main(args=None):
    rclpy.init(args=args)
    
    # Create and initialize node
    node = RobotControlNode()
    
    # Use a multithreaded executor for better performance
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        # Spin the node in the executor
        executor.spin()
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Unhandled exception: {e}')
    finally:
        # Clean up
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()