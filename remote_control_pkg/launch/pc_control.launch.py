from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='remote_control_pkg',
            executable='pc_control_node',
            name='pc_control',
            output='screen',
            parameters=[
                {'robot_ip': '192.168.149.1'},  # Default IP when robot is in AP mode
                {'enable_camera_view': True},
                {'enable_lidar_view': True},
                {'camera_topic': '/robot_control/camera'},
                {'lidar_topic': '/robot_control/lidar'},
                {'cmd_vel_topic': '/robot_control/cmd_vel'}
            ]
        )
    ])