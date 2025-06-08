from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='remote_control_pkg',
            executable='robot_control_node',
            name='robot_control',
            output='screen',
            parameters=[
                {'enable_camera': True},
                {'enable_lidar': True},
                {'camera_topic': '/depth_cam/rgb/image_raw'},
                {'lidar_topic': '/scan'},
                {'cmd_vel_topic': '/controller/cmd_vel'}
            ]
        )
    ])