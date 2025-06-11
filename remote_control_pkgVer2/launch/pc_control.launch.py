from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generate launch description for the PC control node.
    
    This launch file starts the PC control node with configurable parameters.
    """
    # Declare launch arguments
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.149.1',
        description='IP address of the robot'
    )
    
    enable_rgb_view_arg = DeclareLaunchArgument(
        'enable_rgb_view',
        default_value='true',
        description='Enable RGB camera view'
    )
    
    enable_depth_view_arg = DeclareLaunchArgument(
        'enable_depth_view',
        default_value='false',
        description='Enable depth camera view'
    )
    
    enable_lidar_view_arg = DeclareLaunchArgument(
        'enable_lidar_view',
        default_value='true',
        description='Enable LiDAR view'
    )
    
    rgb_camera_topic_arg = DeclareLaunchArgument(
        'rgb_camera_topic',
        default_value='/robot_control/rgb_camera',
        description='RGB camera topic to subscribe to'
    )
    
    depth_camera_topic_arg = DeclareLaunchArgument(
        'depth_camera_topic',
        default_value='/robot_control/depth_camera',
        description='Depth camera topic to subscribe to'
    )
    
    lidar_topic_arg = DeclareLaunchArgument(
        'lidar_topic',
        default_value='/robot_control/lidar',
        description='LiDAR topic to subscribe to'
    )
    
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/robot_control/cmd_vel',
        description='Command velocity topic to publish to'
    )
    
    status_topic_arg = DeclareLaunchArgument(
        'status_topic',
        default_value='/robot_control/status',
        description='Status topic to subscribe to'
    )
    
    use_compressed_image_arg = DeclareLaunchArgument(
        'use_compressed_image',
        default_value='true',
        description='Use compressed image format'
    )
    
    linear_speed_arg = DeclareLaunchArgument(
        'linear_speed',
        default_value='0.2',
        description='Default linear speed (m/s)'
    )
    
    angular_speed_arg = DeclareLaunchArgument(
        'angular_speed',
        default_value='0.5',
        description='Default angular speed (rad/s)'
    )
    
    obstacle_threshold_arg = DeclareLaunchArgument(
        'obstacle_threshold',
        default_value='0.5',
        description='Obstacle detection threshold (m)'
    )
    
    # Create node
    pc_control_node = Node(
        package='remote_control_pkgVer2',
        executable='pc_control_node',
        name='pc_control',
        output='screen',
        parameters=[
            {
                'robot_ip': LaunchConfiguration('robot_ip'),
                'enable_rgb_view': LaunchConfiguration('enable_rgb_view'),
                'enable_depth_view': LaunchConfiguration('enable_depth_view'),
                'enable_lidar_view': LaunchConfiguration('enable_lidar_view'),
                'rgb_camera_topic': LaunchConfiguration('rgb_camera_topic'),
                'depth_camera_topic': LaunchConfiguration('depth_camera_topic'),
                'lidar_topic': LaunchConfiguration('lidar_topic'),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'status_topic': LaunchConfiguration('status_topic'),
                'use_compressed_image': LaunchConfiguration('use_compressed_image'),
                'linear_speed': LaunchConfiguration('linear_speed'),
                'angular_speed': LaunchConfiguration('angular_speed'),
                'obstacle_threshold': LaunchConfiguration('obstacle_threshold')
            }
        ]
    )
    
    # Return launch description
    return LaunchDescription([
        robot_ip_arg,
        enable_rgb_view_arg,
        enable_depth_view_arg,
        enable_lidar_view_arg,
        rgb_camera_topic_arg,
        depth_camera_topic_arg,
        lidar_topic_arg,
        cmd_vel_topic_arg,
        status_topic_arg,
        use_compressed_image_arg,
        linear_speed_arg,
        angular_speed_arg,
        obstacle_threshold_arg,
        pc_control_node
    ])