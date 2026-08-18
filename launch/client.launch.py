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
                    'of every <subject>_link (e.g. map -> robot_link) and the PoseStamped '
                    'header.frame_id. Override it if "map" is already taken in your system'
    )

    declare_publish_parent_tf_arg = DeclareLaunchArgument(
        'publish_parent_tf', default_value='false',
        description="Anchor parent_frame at the origin on /tf_static so it exists in TF even "
                    "with no subject tracked. parent_frame is the root of the tree, so until "
                    "something is tracked /tf is empty and RViz reports 'Fixed Frame [map] does "
                    "not exist'. This publishes an identity '"
                    + ANCHOR_PARENT_FRAME + "' -> parent_frame transform to fill that in"
    )

    declare_publish_velocity_arg = DeclareLaunchArgument(
        'publish_velocity', default_value='false',
        description='Publish a derived linear velocity as geometry_msgs/TwistStamped on '
                    '<namespace>/<subject>/twist, in the parent_frame. Off by default: it is '
                    'differentiated from the pose stream, so it is only as good as the link'
    )

    declare_velocity_window_arg = DeclareLaunchArgument(
        'velocity_window', default_value='5',
        description='Samples in the least-squares slope fit used for velocity. Trades noise '
                    'against lag: 2 is a plain backward difference, larger is smoother but '
                    'delayed by (N-1)/2 Vicon frames. Measured on a 200 Hz system: 2 -> ~21 mm/s '
                    'and no lag, 5 -> ~7 mm/s and +10 ms, 20 -> ~1 mm/s and +48 ms'
    )

    declare_velocity_max_gap_arg = DeclareLaunchArgument(
        'velocity_max_gap_frames', default_value='3',
        description='Discard the velocity history when more than this many Vicon frames go '
                    'missing, so the fit never spans a dropout'
    )

    # Use the launch arguments
    server = LaunchConfiguration('server')
    buffer_size = LaunchConfiguration('buffer_size')
    topic_namespace = LaunchConfiguration('namespace')
    parent_frame = LaunchConfiguration('parent_frame')
    publish_parent_tf = LaunchConfiguration('publish_parent_tf')
    publish_velocity = LaunchConfiguration('publish_velocity')
    velocity_window = LaunchConfiguration('velocity_window')
    velocity_max_gap_frames = LaunchConfiguration('velocity_max_gap_frames')

    return LaunchDescription([
        declare_server_arg,
        declare_buffer_size_arg,
        declare_topic_namespace_arg,
        declare_parent_frame_arg,
        declare_publish_parent_tf_arg,
        declare_publish_velocity_arg,
        declare_velocity_window_arg,
        declare_velocity_max_gap_arg,
        Node(
            package='mocap_vicon_client',
            executable='vicon_client',
            output='screen',
            parameters=[{
                'server': server,
                'buffer_size': buffer_size,
                'namespace': topic_namespace,
                'parent_frame': parent_frame,
                'publish_velocity': publish_velocity,
                'velocity_window': velocity_window,
                'velocity_max_gap_frames': velocity_max_gap_frames
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
