from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Parent of the optional anchor transform. Fixed on purpose rather than exposed as a launch
# argument: a TF transform always needs a parent and a child, so making `parent_frame` exist
# in TF requires *some* second name, but that name is an implementation detail -- it sits at
# the same place as parent_frame (identity). Exposing it as a knob only invites confusion
# about which of the two frames is "the" mocap frame. It is parent_frame.
ANCHOR_PARENT_FRAME = 'world'

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

    declare_parent_frame_arg = DeclareLaunchArgument(
        'parent_frame', default_value='map',
        description='World-fixed frame the Vicon poses are expressed in. Becomes the TF parent '
                    'of every <subject>_link and the PoseStamped header.frame_id'
    )

    declare_publish_parent_tf_arg = DeclareLaunchArgument(
        'publish_parent_tf', default_value='false',
        description="Anchor parent_frame at the origin on /tf_static, so it exists in TF even "
                    "with no subject tracked. Publishes an identity '"
                    + ANCHOR_PARENT_FRAME + "' -> parent_frame transform"
    )

    # Use the launch arguments
    server = LaunchConfiguration('server')
    buffer_size = LaunchConfiguration('buffer_size')
    topic_namespace = LaunchConfiguration('namespace')
    parent_frame = LaunchConfiguration('parent_frame')
    publish_parent_tf = LaunchConfiguration('publish_parent_tf')

    return LaunchDescription([
        declare_server_arg,
        declare_buffer_size_arg,
        declare_topic_namespace_arg,
        declare_parent_frame_arg,
        declare_publish_parent_tf_arg,
        Node(
            package='mocap_vicon_client',
            executable='vicon_client',
            output='screen',
            parameters=[{
                'server': server,
                'buffer_size': buffer_size,
                'namespace': topic_namespace,
                'parent_frame': parent_frame
            }]
        ),
        # Optional: make parent_frame exist at the origin even when nothing is tracked.
        # The vicon_client only broadcasts parent_frame -> <subject>_link, so with no subject
        # visible /tf is empty and RViz reports "Fixed Frame [map] does not exist".
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='mocap_parent_frame_anchor',
            output='screen',
            condition=IfCondition(publish_parent_tf),
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', ANCHOR_PARENT_FRAME,
                '--child-frame-id', parent_frame
            ]
        )
    ])
