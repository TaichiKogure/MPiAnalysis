from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    """
    Generate launch description for the robot control node.
    
    This launch file starts the robot control node with configurable parameters.
    """
    # Declare launch arguments
    enable_camera_arg = DeclareLaunchArgument(
        'enable_camera',
        default_value='true',
        description='Enable camera processing'
    )
    
    enable_lidar_arg = DeclareLaunchArgument(
        'enable_lidar',
        default_value='true',
        description='Enable LiDAR processing'
    )
    
    enable_depth_camera_arg = DeclareLaunchArgument(
        'enable_depth_camera',
        default_value='false',
        description='Enable depth camera processing'
    )
    
    rgb_camera_topic_arg = DeclareLaunchArgument(
        'rgb_camera_topic',
        default_value='/ascamera/camera_publisher/rgb0/image',
        description='RGB camera topic to subscribe to'
    )
    
    depth_camera_topic_arg = DeclareLaunchArgument(
        'depth_camera_topic',
        default_value='/ascamera/camera_publisher/depth/image',
        description='Depth camera topic to subscribe to'
    )
    
    lidar_topic_arg = DeclareLaunchArgument(
        'lidar_topic',
        default_value='/scan',
        description='LiDAR topic to subscribe to'
    )
    
    cmd_vel_topic_arg = DeclareLaunchArgument(
        'cmd_vel_topic',
        default_value='/controller/cmd_vel',
        description='Command velocity topic to publish to'
    )
    
    camera_fps_arg = DeclareLaunchArgument(
        'camera_fps',
        default_value='10',
        description='Target FPS for camera republishing'
    )
    
    lidar_fps_arg = DeclareLaunchArgument(
        'lidar_fps',
        default_value='5',
        description='Target FPS for LiDAR republishing'
    )
    
    camera_width_arg = DeclareLaunchArgument(
        'camera_width',
        default_value='320',
        description='Resized width for bandwidth reduction'
    )
    
    camera_height_arg = DeclareLaunchArgument(
        'camera_height',
        default_value='240',
        description='Resized height for bandwidth reduction'
    )
    
    use_compressed_image_arg = DeclareLaunchArgument(
        'use_compressed_image',
        default_value='true',
        description='Use compressed image format'
    )
    
    # Create node
    robot_control_node = Node(
        package='remote_control_pkgVer2',
        executable='robot_control_node',
        name='robot_control',
        output='screen',
        parameters=[
            {
                'enable_camera': LaunchConfiguration('enable_camera'),
                'enable_lidar': LaunchConfiguration('enable_lidar'),
                'enable_depth_camera': LaunchConfiguration('enable_depth_camera'),
                'rgb_camera_topic': LaunchConfiguration('rgb_camera_topic'),
                'depth_camera_topic': LaunchConfiguration('depth_camera_topic'),
                'lidar_topic': LaunchConfiguration('lidar_topic'),
                'cmd_vel_topic': LaunchConfiguration('cmd_vel_topic'),
                'camera_fps': LaunchConfiguration('camera_fps'),
                'lidar_fps': LaunchConfiguration('lidar_fps'),
                'camera_width': LaunchConfiguration('camera_width'),
                'camera_height': LaunchConfiguration('camera_height'),
                'use_compressed_image': LaunchConfiguration('use_compressed_image')
            }
        ]
    )
    
    # Return launch description
    return LaunchDescription([
        enable_camera_arg,
        enable_lidar_arg,
        enable_depth_camera_arg,
        rgb_camera_topic_arg,
        depth_camera_topic_arg,
        lidar_topic_arg,
        cmd_vel_topic_arg,
        camera_fps_arg,
        lidar_fps_arg,
        camera_width_arg,
        camera_height_arg,
        use_compressed_image_arg,
        robot_control_node
    ])