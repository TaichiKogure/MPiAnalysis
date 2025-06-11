#!/usr/bin/env python3
# encoding: utf-8

"""
PC control node for MentorPi robot (Version 2).

This node provides a user interface for controlling the robot and visualizing
sensor data. It subscribes to the camera and LiDAR topics from the robot control
node and displays the data in a GUI. It also provides keyboard control for the robot.

Improvements over Version 1:
- Better support for depth camera data visualization
- Improved LiDAR data visualization with obstacle detection
- More interactive controls and visualization options
- Status monitoring and display
- Better error handling and recovery
- Support for both compressed and raw images
"""

import rclpy
import cv2
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.widgets import Button, Slider
import threading
import time
import argparse
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, LaserScan, CompressedImage
from geometry_msgs.msg import Twist
from std_msgs.msg import Header, String, Float32
from builtin_interfaces.msg import Time

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
        self.declare_parameter('enable_rgb_view', True)
        self.declare_parameter('enable_depth_view', False)
        self.declare_parameter('enable_lidar_view', True)
        self.declare_parameter('rgb_camera_topic', '/robot_control/rgb_camera')
        self.declare_parameter('depth_camera_topic', '/robot_control/depth_camera')
        self.declare_parameter('lidar_topic', '/robot_control/lidar')
        self.declare_parameter('cmd_vel_topic', '/robot_control/cmd_vel')
        self.declare_parameter('status_topic', '/robot_control/status')
        self.declare_parameter('use_compressed_image', True)
        self.declare_parameter('linear_speed', 0.2)  # Default linear speed (m/s)
        self.declare_parameter('angular_speed', 0.5)  # Default angular speed (rad/s)
        self.declare_parameter('obstacle_threshold', 0.5)  # Obstacle detection threshold (m)
        
        # Get parameter values
        self.robot_ip = self.get_parameter('robot_ip').value
        self.enable_rgb_view = self.get_parameter('enable_rgb_view').value
        self.enable_depth_view = self.get_parameter('enable_depth_view').value
        self.enable_lidar_view = self.get_parameter('enable_lidar_view').value
        self.rgb_camera_topic = self.get_parameter('rgb_camera_topic').value
        self.depth_camera_topic = self.get_parameter('depth_camera_topic').value
        self.lidar_topic = self.get_parameter('lidar_topic').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.status_topic = self.get_parameter('status_topic').value
        self.use_compressed_image = self.get_parameter('use_compressed_image').value
        self.linear_speed = self.get_parameter('linear_speed').value
        self.angular_speed = self.get_parameter('angular_speed').value
        self.obstacle_threshold = self.get_parameter('obstacle_threshold').value
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Create callback groups for parallel execution
        self.camera_callback_group = MutuallyExclusiveCallbackGroup()
        self.lidar_callback_group = MutuallyExclusiveCallbackGroup()
        self.status_callback_group = MutuallyExclusiveCallbackGroup()
        
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
        self.cmd_vel_pub = self.create_publisher(
            Twist, self.cmd_vel_topic, self.reliable_qos)
        
        # Create subscriptions
        if self.enable_rgb_view:
            if self.use_compressed_image:
                self.rgb_camera_sub = self.create_subscription(
                    CompressedImage,
                    self.rgb_camera_topic + '/compressed',
                    self.rgb_camera_compressed_callback,
                    self.sensor_qos,
                    callback_group=self.camera_callback_group)
            else:
                self.rgb_camera_sub = self.create_subscription(
                    Image,
                    self.rgb_camera_topic,
                    self.rgb_camera_callback,
                    self.sensor_qos,
                    callback_group=self.camera_callback_group)
        
        if self.enable_depth_view:
            if self.use_compressed_image:
                self.depth_camera_sub = self.create_subscription(
                    CompressedImage,
                    self.depth_camera_topic + '/compressed',
                    self.depth_camera_compressed_callback,
                    self.sensor_qos,
                    callback_group=self.camera_callback_group)
            else:
                self.depth_camera_sub = self.create_subscription(
                    Image,
                    self.depth_camera_topic,
                    self.depth_camera_callback,
                    self.sensor_qos,
                    callback_group=self.camera_callback_group)
        
        if self.enable_lidar_view:
            self.lidar_sub = self.create_subscription(
                LaserScan,
                self.lidar_topic,
                self.lidar_callback,
                self.sensor_qos,
                callback_group=self.lidar_callback_group)
        
        # Subscribe to status messages
        self.status_sub = self.create_subscription(
            String,
            self.status_topic,
            self.status_callback,
            self.reliable_qos,
            callback_group=self.status_callback_group)
        
        # Initialize visualization variables
        self.rgb_image = None
        self.depth_image = None
        self.lidar_data = None
        self.status_text = "Connecting to robot..."
        self.last_key = None
        self.recording = False
        self.record_frames = []
        self.obstacle_detected = False
        self.obstacle_direction = None
        
        # Control state
        self.current_linear_speed = self.linear_speed
        self.current_angular_speed = self.angular_speed
        
        # Create a figure for visualization
        self.setup_visualization()
        
        # Start visualization thread
        self.running = True
        self.viz_thread = threading.Thread(target=self.visualization_loop)
        self.viz_thread.daemon = True
        self.viz_thread.start()
        
        self.get_logger().info(f'PC control node initialized, connecting to robot at {self.robot_ip}')
    
    def setup_visualization(self):
        """
        Set up the visualization figure and axes.
        """
        if not (self.enable_rgb_view or self.enable_depth_view or self.enable_lidar_view):
            return
            
        plt.ion()  # Enable interactive mode
        self.fig = plt.figure(figsize=(15, 8))
        self.fig.canvas.manager.set_window_title('MentorPi Remote Control Ver2')
        
        # Create grid layout based on enabled views
        if self.enable_rgb_view and self.enable_depth_view and self.enable_lidar_view:
            # All three views
            gs = gridspec.GridSpec(2, 2, height_ratios=[3, 1])
            self.rgb_ax = plt.subplot(gs[0, 0])
            self.depth_ax = plt.subplot(gs[0, 1])
            self.lidar_ax = plt.subplot(gs[1, :], projection='polar')
            self.status_ax = None  # Status will be shown as text in the figure
        elif self.enable_rgb_view and self.enable_depth_view:
            # RGB and depth views
            gs = gridspec.GridSpec(2, 2, height_ratios=[3, 1])
            self.rgb_ax = plt.subplot(gs[0, 0])
            self.depth_ax = plt.subplot(gs[0, 1])
            self.lidar_ax = None
            self.status_ax = plt.subplot(gs[1, :])
        elif self.enable_rgb_view and self.enable_lidar_view:
            # RGB and LiDAR views
            gs = gridspec.GridSpec(2, 1, height_ratios=[3, 1])
            self.rgb_ax = plt.subplot(gs[0])
            self.depth_ax = None
            self.lidar_ax = plt.subplot(gs[1], projection='polar')
            self.status_ax = None
        elif self.enable_depth_view and self.enable_lidar_view:
            # Depth and LiDAR views
            gs = gridspec.GridSpec(2, 1, height_ratios=[3, 1])
            self.rgb_ax = None
            self.depth_ax = plt.subplot(gs[0])
            self.lidar_ax = plt.subplot(gs[1], projection='polar')
            self.status_ax = None
        elif self.enable_rgb_view:
            # Only RGB view
            gs = gridspec.GridSpec(2, 1, height_ratios=[3, 1])
            self.rgb_ax = plt.subplot(gs[0])
            self.depth_ax = None
            self.lidar_ax = None
            self.status_ax = plt.subplot(gs[1])
        elif self.enable_depth_view:
            # Only depth view
            gs = gridspec.GridSpec(2, 1, height_ratios=[3, 1])
            self.rgb_ax = None
            self.depth_ax = plt.subplot(gs[0])
            self.lidar_ax = None
            self.status_ax = plt.subplot(gs[1])
        elif self.enable_lidar_view:
            # Only LiDAR view
            gs = gridspec.GridSpec(2, 1, height_ratios=[3, 1])
            self.rgb_ax = None
            self.depth_ax = None
            self.lidar_ax = plt.subplot(gs[0], projection='polar')
            self.status_ax = plt.subplot(gs[1])
        
        # Add control buttons
        self.fig.subplots_adjust(bottom=0.2)
        
        # Speed control sliders
        ax_linear = plt.axes([0.25, 0.1, 0.5, 0.03])
        ax_angular = plt.axes([0.25, 0.05, 0.5, 0.03])
        self.linear_slider = Slider(ax_linear, 'Linear Speed (m/s)', 0.0, 1.0, valinit=self.linear_speed)
        self.angular_slider = Slider(ax_angular, 'Angular Speed (rad/s)', 0.0, 2.0, valinit=self.angular_speed)
        self.linear_slider.on_changed(self.update_linear_speed)
        self.angular_slider.on_changed(self.update_angular_speed)
        
        # Record button
        ax_record = plt.axes([0.8, 0.05, 0.1, 0.075])
        self.record_button = Button(ax_record, 'Record')
        self.record_button.on_clicked(self.toggle_recording)
        
        # Connect key press event
        self.fig.canvas.mpl_connect('key_press_event', self.key_press_callback)
        
        # Set up titles
        if self.rgb_ax:
            self.rgb_ax.set_title('RGB Camera View')
            self.rgb_ax.axis('off')
        if self.depth_ax:
            self.depth_ax.set_title('Depth Camera View')
            self.depth_ax.axis('off')
        if self.lidar_ax:
            self.lidar_ax.set_title('LiDAR View')
            self.lidar_ax.grid(True)
        if self.status_ax:
            self.status_ax.set_title('Status')
            self.status_ax.axis('off')
    
    def update_linear_speed(self, val):
        """
        Update the linear speed value from the slider.
        """
        self.current_linear_speed = val
        self.get_logger().info(f'Linear speed updated to {val:.2f} m/s')
    
    def update_angular_speed(self, val):
        """
        Update the angular speed value from the slider.
        """
        self.current_angular_speed = val
        self.get_logger().info(f'Angular speed updated to {val:.2f} rad/s')
    
    def toggle_recording(self, event):
        """
        Toggle recording of camera frames.
        """
        self.recording = not self.recording
        if self.recording:
            self.record_frames = []
            self.record_button.label.set_text('Stop')
            self.get_logger().info('Recording started')
        else:
            self.record_button.label.set_text('Record')
            self.get_logger().info(f'Recording stopped, {len(self.record_frames)} frames captured')
            if len(self.record_frames) > 0:
                timestamp = time.strftime("%Y%m%d-%H%M%S")
                filename = f'mentorpi_recording_{timestamp}.avi'
                self.save_recording(filename)
    
    def save_recording(self, filename):
        """
        Save recorded frames to a video file.
        """
        if not self.record_frames:
            self.get_logger().warn('No frames to save')
            return
            
        try:
            height, width = self.record_frames[0].shape[:2]
            fourcc = cv2.VideoWriter_fourcc(*'XVID')
            out = cv2.VideoWriter(filename, fourcc, 10.0, (width, height))
            
            for frame in self.record_frames:
                out.write(frame)
                
            out.release()
            self.get_logger().info(f'Recording saved to {filename}')
        except Exception as e:
            self.get_logger().error(f'Error saving recording: {e}')
    
    def rgb_camera_callback(self, msg):
        """
        Process RGB camera data from the robot.
        
        This function converts the ROS Image message to an OpenCV image for display.
        """
        try:
            # Convert ROS Image message to OpenCV image
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Record frame if recording is enabled
            if self.recording and self.rgb_image is not None:
                self.record_frames.append(self.rgb_image.copy())
                
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error processing RGB camera data: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing RGB camera data: {e}')
    
    def rgb_camera_compressed_callback(self, msg):
        """
        Process compressed RGB camera data from the robot.
        
        This function converts the ROS CompressedImage message to an OpenCV image for display.
        """
        try:
            # Convert compressed image to OpenCV image
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.rgb_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            # Record frame if recording is enabled
            if self.recording and self.rgb_image is not None:
                self.record_frames.append(self.rgb_image.copy())
                
        except Exception as e:
            self.get_logger().error(f'Error processing compressed RGB camera data: {e}')
    
    def depth_camera_callback(self, msg):
        """
        Process depth camera data from the robot.
        
        This function converts the ROS Image message to an OpenCV image for display.
        """
        try:
            # Convert ROS Image message to OpenCV image
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                
        except CvBridgeError as e:
            self.get_logger().error(f'CV Bridge error processing depth camera data: {e}')
        except Exception as e:
            self.get_logger().error(f'Error processing depth camera data: {e}')
    
    def depth_camera_compressed_callback(self, msg):
        """
        Process compressed depth camera data from the robot.
        
        This function converts the ROS CompressedImage message to an OpenCV image for display.
        """
        try:
            # Convert compressed image to OpenCV image
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.depth_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
                
        except Exception as e:
            self.get_logger().error(f'Error processing compressed depth camera data: {e}')
    
    def lidar_callback(self, msg):
        """
        Process LiDAR data from the robot.
        
        This function stores the LiDAR data for display and performs obstacle detection.
        """
        try:
            self.lidar_data = msg
            
            # Perform obstacle detection
            ranges = np.array(msg.ranges)
            valid_indices = np.isfinite(ranges) & (ranges > 0.0)
            
            if np.any(valid_indices):
                # Check for obstacles within threshold
                obstacle_indices = np.where((ranges < self.obstacle_threshold) & valid_indices)[0]
                
                if len(obstacle_indices) > 0:
                    # Calculate the average angle of obstacles
                    angles = np.linspace(msg.angle_min, msg.angle_max, len(msg.ranges))
                    obstacle_angles = angles[obstacle_indices]
                    mean_angle = np.mean(obstacle_angles)
                    
                    # Determine direction (front, left, right)
                    if -0.5 < mean_angle < 0.5:  # Front
                        direction = "front"
                    elif mean_angle >= 0.5:  # Left
                        direction = "left"
                    else:  # Right
                        direction = "right"
                    
                    self.obstacle_detected = True
                    self.obstacle_direction = direction
                    self.get_logger().warn(f'Obstacle detected {direction}, distance: {np.min(ranges[obstacle_indices]):.2f}m')
                else:
                    self.obstacle_detected = False
                    self.obstacle_direction = None
            
        except Exception as e:
            self.get_logger().error(f'Error processing LiDAR data: {e}')
    
    def status_callback(self, msg):
        """
        Process status messages from the robot.
        
        This function updates the status text for display.
        """
        try:
            self.status_text = msg.data
        except Exception as e:
            self.get_logger().error(f'Error processing status message: {e}')
    
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
            twist.linear.x = self.current_linear_speed  # Forward
        elif event.key == 'down':
            twist.linear.x = -self.current_linear_speed  # Backward
        elif event.key == 'left':
            twist.angular.z = self.current_angular_speed  # Turn left
        elif event.key == 'right':
            twist.angular.z = -self.current_angular_speed  # Turn right
        elif event.key == ' ':
            pass  # Stop (all values are 0 by default)
        elif event.key == 'escape':
            self.running = False
            plt.close(self.fig)
            return
        
        self.cmd_vel_pub.publish(twist)
        self.get_logger().debug(f'Sent command: linear.x={twist.linear.x}, angular.z={twist.angular.z}')
    
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
                
                # Update RGB camera view
                if self.enable_rgb_view and self.rgb_ax and self.rgb_image is not None:
                    self.rgb_ax.clear()
                    self.rgb_ax.imshow(cv2.cvtColor(self.rgb_image, cv2.COLOR_BGR2RGB))
                    self.rgb_ax.set_title('RGB Camera View')
                    self.rgb_ax.axis('off')
                
                # Update depth camera view
                if self.enable_depth_view and self.depth_ax and self.depth_image is not None:
                    self.depth_ax.clear()
                    self.depth_ax.imshow(cv2.cvtColor(self.depth_image, cv2.COLOR_BGR2RGB))
                    self.depth_ax.set_title('Depth Camera View')
                    self.depth_ax.axis('off')
                
                # Update LiDAR view
                if self.enable_lidar_view and self.lidar_ax and self.lidar_data is not None:
                    self.lidar_ax.clear()
                    angles = np.linspace(
                        self.lidar_data.angle_min,
                        self.lidar_data.angle_max,
                        len(self.lidar_data.ranges)
                    )
                    
                    # Filter out invalid readings
                    ranges = np.array(self.lidar_data.ranges)
                    valid_indices = np.isfinite(ranges) & (ranges > 0.0)
                    
                    if np.any(valid_indices):
                        # Plot valid points
                        self.lidar_ax.scatter(angles[valid_indices], ranges[valid_indices], s=2, c='blue')
                        
                        # Highlight obstacles
                        obstacle_indices = np.where((ranges < self.obstacle_threshold) & valid_indices)[0]
                        if len(obstacle_indices) > 0:
                            self.lidar_ax.scatter(
                                angles[obstacle_indices], 
                                ranges[obstacle_indices], 
                                s=10, c='red'
                            )
                        
                        self.lidar_ax.set_rmax(10)  # Set maximum range to 10 meters
                        self.lidar_ax.set_title('LiDAR View')
                        self.lidar_ax.grid(True)
                
                # Update status view
                if self.status_ax:
                    self.status_ax.clear()
                    self.status_ax.text(
                        0.5, 0.5, 
                        self.status_text + "\n\n" + 
                        f"Controls: Arrow keys to move, Space to stop, Esc to exit\n" +
                        f"Linear speed: {self.current_linear_speed:.2f} m/s, Angular speed: {self.current_angular_speed:.2f} rad/s" +
                        (f"\nWARNING: Obstacle detected {self.obstacle_direction}!" if self.obstacle_detected else ""),
                        horizontalalignment='center',
                        verticalalignment='center',
                        transform=self.status_ax.transAxes,
                        fontsize=10,
                        color='black' if not self.obstacle_detected else 'red'
                    )
                    self.status_ax.set_title('Status')
                    self.status_ax.axis('off')
                
                # Add key command help text to RGB or depth view if available
                if self.rgb_ax:
                    self.rgb_ax.text(
                        0.5, 0.02,
                        'Controls: Arrow keys to move, Space to stop, Esc to exit',
                        horizontalalignment='center',
                        verticalalignment='bottom',
                        transform=self.rgb_ax.transAxes,
                        color='white',
                        bbox=dict(facecolor='black', alpha=0.5)
                    )
                elif self.depth_ax:
                    self.depth_ax.text(
                        0.5, 0.02,
                        'Controls: Arrow keys to move, Space to stop, Esc to exit',
                        horizontalalignment='center',
                        verticalalignment='bottom',
                        transform=self.depth_ax.transAxes,
                        color='white',
                        bbox=dict(facecolor='black', alpha=0.5)
                    )
                
                # Add recording indicator if recording
                if self.recording and self.rgb_ax and self.rgb_image is not None:
                    self.rgb_ax.text(
                        0.05, 0.05,
                        'REC',
                        horizontalalignment='left',
                        verticalalignment='bottom',
                        transform=self.rgb_ax.transAxes,
                        color='red',
                        fontweight='bold',
                        bbox=dict(facecolor='black', alpha=0.5)
                    )
                
                # Update the figure
                self.fig.canvas.draw()
                self.fig.canvas.flush_events()
                
                # Sleep to reduce CPU usage
                time.sleep(0.05)
            
            except Exception as e:
                self.get_logger().error(f'Error in visualization loop: {e}')
                time.sleep(1)  # Sleep longer on error
    
    def destroy_node(self):
        """
        Clean up resources when the node is destroyed.
        """
        # Stop the robot
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        
        # Stop the visualization thread
        self.running = False
        if self.viz_thread.is_alive():
            self.viz_thread.join(timeout=1.0)
        
        # Close matplotlib figure
        plt.close(self.fig)
        
        super().destroy_node()

def main(args=None):
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='MentorPi PC Control Node')
    parser.add_argument('--robot-ip', type=str, help='IP address of the robot')
    parser.add_argument('--no-rgb', action='store_true', help='Disable RGB camera view')
    parser.add_argument('--depth', action='store_true', help='Enable depth camera view')
    parser.add_argument('--no-lidar', action='store_true', help='Disable LiDAR view')
    
    # Initialize ROS
    rclpy.init(args=args)
    
    # Get command line args
    cli_args = parser.parse_args()
    
    # Override ROS parameters with command line args if provided
    ros_args = []
    if cli_args.robot_ip:
        ros_args.extend(['--ros-args', '-p', f'robot_ip:={cli_args.robot_ip}'])
    if cli_args.no_rgb:
        ros_args.extend(['--ros-args', '-p', 'enable_rgb_view:=false'])
    if cli_args.depth:
        ros_args.extend(['--ros-args', '-p', 'enable_depth_view:=true'])
    if cli_args.no_lidar:
        ros_args.extend(['--ros-args', '-p', 'enable_lidar_view:=false'])
    
    # Create and initialize node
    node = PCControlNode()
    
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