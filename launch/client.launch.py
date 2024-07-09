from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    hostname = '192.168.0.100'
    buffer_size = 256
    topic_namespace = 'vicon'

    return LaunchDescription([Node(
            package='mocap_vicon_client', executable='vicon_client', output='screen',
            parameters=[{'hostname': hostname, 'buffer_size': buffer_size, 'namespace': topic_namespace}]
        )])