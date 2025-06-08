#!/usr/bin/env python3
# encoding: utf-8

import rclpy
import cv2
import numpy as np
import matplotlib.pyplot as plt
import threading
import time
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from std_msgs.msg import Header
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class PCControlNode(Node):
    """
    PC control node for MentorPi robot.
    
    This node provides a user interface for controlling the robot and visualizing
    sensor data. It subscribes to the camera and LiDAR topics from the robot control
    node and displays the data in a GUI. It also provides keyboard control for the robot.
    """
    
    def __init__(self):
        super().__init__('pc_control')
        
        # Declare parameters
        self.declare_parameter('robot_ip', '192.168.149.1')
        self.declare_parameter('enable_camera_view', True)
        self.declare_parameter('enable_lidar_view', True)
        self.declare_parameter('camera_topic', '/robot_control/camera')
        self.declare_parameter('lidar_topic', '/robot_control/lidar')
        self.declare_parameter('cmd_vel_topic', '/robot_control/cmd_vel')
        
        # Get parameter values
        self.robot_ip = self.get_parameter('robot_ip').value
        self.enable_camera_view = self.get_parameter('enable_camera_view').value
        self.enable_lidar_view = self.get_parameter('enable_lidar_view').value
        self.camera_topic = self.get_parameter('camera_topic').value
        self.lidar_topic = self.get_parameter('lidar_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Create reliable QoS profile for better network performance
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create publishers
        self.cmd_vel_pub = self.create_publisher(Twist, self.cmd_vel_topic, 10)
        
        # Create subscriptions
        if self.enable_camera_view:
            self.camera_sub = self.create_subscription(
                Image,
                self.camera_topic,
                self.camera_callback,
                qos_profile)
        
        if self.enable_lidar_view:
            self.lidar_sub = self.create_subscription(
                LaserScan,
                self.lidar_topic,
                self.lidar_callback,
                qos_profile)
        
        # Initialize visualization variables
        self.camera_image = None
        self.lidar_data = None
        self.last_key = None
        
        # Create a figure for visualization
        if self.enable_camera_view or self.enable_lidar_view:
            plt.ion()  # Enable interactive mode
            self.fig = plt.figure(figsize=(12, 6))
            
            if self.enable_camera_view and self.enable_lidar_view:
                self.camera_ax = self.fig.add_subplot(1, 2, 1)
                self.lidar_ax = self.fig.add_subplot(1, 2, 2, projection='polar')
            elif self.enable_camera_view:
                self.camera_ax = self.fig.add_subplot(1, 1, 1)
            elif self.enable_lidar_view:
                self.lidar_ax = self.fig.add_subplot(1, 1, 1, projection='polar')
            
            self.fig.canvas.mpl_connect('key_press_event', self.key_press_callback)
        
        # Start visualization thread
        self.running = True
        self.viz_thread = threading.Thread(target=self.visualization_loop)
        self.viz_thread.daemon = True
        self.viz_thread.start()
        
        self.get_logger().info(f'PC control node initialized, connecting to robot at {self.robot_ip}')
    
    def camera_callback(self, msg):
        """
        Process camera data from the robot.
        
        This function converts the ROS Image message to an OpenCV image for display.
        """
        try:
            # Convert ROS Image message to OpenCV image
            self.camera_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Error processing camera data: {e}')
    
    def lidar_callback(self, msg):
        """
        Process LiDAR data from the robot.
        
        This function stores the LiDAR data for display.
        """
        try:
            self.lidar_data = msg
        except Exception as e:
            self.get_logger().error(f'Error processing LiDAR data: {e}')
    
    def key_press_callback(self, event):
        """
        Handle key press events for robot control.
        
        This function maps key presses to robot commands:
        - Arrow keys: Move the robot
        - Space: Stop the robot
        - Escape: Exit the program
        """
        self.last_key = event.key
        
        twist = Twist()
        
        if event.key == 'up':
            twist.linear.x = 0.2  # Forward
        elif event.key == 'down':
            twist.linear.x = -0.2  # Backward
        elif event.key == 'left':
            twist.angular.z = 0.5  # Turn left
        elif event.key == 'right':
            twist.angular.z = -0.5  # Turn right
        elif event.key == ' ':
            pass  # Stop (all values are 0 by default)
        elif event.key == 'escape':
            self.running = False
            plt.close()
            return
        
        self.cmd_vel_pub.publish(twist)
    
    def visualization_loop(self):
        """
        Main visualization loop.
        
        This function updates the visualization with the latest sensor data.
        """
        while self.running:
            try:
                if not plt.fignum_exists(self.fig.number):
                    self.running = False
                    break
                
                # Update camera view
                if self.enable_camera_view and self.camera_image is not None:
                    self.camera_ax.clear()
                    self.camera_ax.imshow(cv2.cvtColor(self.camera_image, cv2.COLOR_BGR2RGB))
                    self.camera_ax.set_title('Camera View')
                    self.camera_ax.axis('off')
                
                # Update LiDAR view
                if self.enable_lidar_view and self.lidar_data is not None:
                    self.lidar_ax.clear()
                    angles = np.linspace(
                        self.lidar_data.angle_min,
                        self.lidar_data.angle_max,
                        len(self.lidar_data.ranges)
                    )
                    
                    # Filter out invalid readings
                    ranges = np.array(self.lidar_data.ranges)
                    valid_indices = np.isfinite(ranges)
                    
                    if np.any(valid_indices):
                        self.lidar_ax.scatter(angles[valid_indices], ranges[valid_indices], s=2)
                        self.lidar_ax.set_rmax(10)  # Set maximum range to 10 meters
                        self.lidar_ax.set_title('LiDAR View')
                        self.lidar_ax.grid(True)
                
                # Add key command help text
                if self.enable_camera_view:
                    self.camera_ax.text(
                        0.5, 0.02,
                        'Controls: Arrow keys to move, Space to stop, Esc to exit',
                        horizontalalignment='center',
                        verticalalignment='bottom',
                        transform=self.camera_ax.transAxes,
                        color='white',
                        bbox=dict(facecolor='black', alpha=0.5)
                    )
                
                # Update the figure
                self.fig.canvas.draw()
                self.fig.canvas.flush_events()
                
                # Sleep to reduce CPU usage
                time.sleep(0.1)
            
            except Exception as e:
                self.get_logger().error(f'Error in visualization loop: {e}')
                time.sleep(1)  # Sleep longer on error
    
    def destroy_node(self):
        """
        Clean up resources when the node is destroyed.
        """
        self.running = False
        if self.viz_thread.is_alive():
            self.viz_thread.join(timeout=1.0)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = PCControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()