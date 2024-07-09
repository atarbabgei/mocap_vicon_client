from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    # Declare the launch arguments
    declare_server_arg = DeclareLaunchArgument(
        'server', default_value='192.168.0.100',
        description='Server address for the Vicon system'
    )
    
    declare_buffer_size_arg = DeclareLaunchArgument(
        'buffer_size', default_value='256',
        description='Buffer size for the Vicon client'
    )
    
    declare_topic_namespace_arg = DeclareLaunchArgument(
        'namespace', default_value='mocap',
        description='Namespace for the Vicon topics'
    )

    # Use the launch arguments
    server = LaunchConfiguration('server')
    buffer_size = LaunchConfiguration('buffer_size')
    topic_namespace = LaunchConfiguration('namespace')

    return LaunchDescription([
        declare_server_arg,
        declare_buffer_size_arg,
        declare_topic_namespace_arg,
        Node(
            package='mocap_vicon_client', 
            executable='vicon_client', 
            output='screen',
            parameters=[{
                'server': server, 
                'buffer_size': buffer_size, 
                'namespace': topic_namespace
            }]
        )
    ])
