
# mocap_vicon_client

This ROS2 package is heavily derived from [ros2-vicon-receiver](https://github.com/OPT4SMART/ros2-vicon-receiver) but is now working on Ubuntu 22.04 and ROS2 Humble.

This package integrates Vicon motion capture data using the Vicon DataStream SDK.

## Installation

### Ensure ROS2 is Installed

Make sure you have ROS2 installed on your system. This package is tested on Ubuntu 22.04 and ROS2 Humble. You can follow the installation guide [here](https://docs.ros.org/en/humble/Installation.html).


### Install the Vicon DataStream SDK

To install the Vicon DataStream SDK, run the following command:

```bash
wget -qO- https://github.com/atarbabgei/miscellaneous/raw/main/mocap_vicon/scripts/install_vicon_datastream_sdk.sh | sudo bash
```

### Install Dependencies

Ensure that the necessary dependencies are installed. 

```bash
sudo apt update
sudo apt install libboost-all-dev python3-colcon-common-extensions cmake
```

### Create a New ROS2 Workspace

Create a new directory for your ROS2 workspace and navigate into it:

```bash
mkdir -p ~/mocap_ros2_ws/src
cd ~/mocap_ros2_ws
```

### Clone the Necessary Repositories

Clone the \`mocap_msgs\` and \`mocap_vicon_client\` repositories into the \`src\` directory:

```bash
cd src
git clone https://github.com/atarbabgei/mocap_vicon_client.git
```

### Build the Packages

Navigate to the root of your workspace and build the packages using \`colcon\`:

```bash
cd ~/mocap_ros2_ws
colcon build --packages-select mocap_vicon_client
```

### Source the Workspace

Before running the nodes, make sure to source your ROS2 workspace:

```bash
source ~/mocap_ros2_ws/install/setup.bash
```
## Example Usage

### Launching the Node

You can launch the node using the provided launch file. The parameters such as server address, buffer size, and namespace can be set from the command line.

Here's an example command to launch the node with the Vicon server at a specific IP (e.g. 192.168.0.100):

```bash
ros2 launch mocap_vicon_client client.launch.py server:=192.168.0.100
```

### Launch arguments

| Argument            | Default         | Description                                                      |
|---------------------|-----------------|------------------------------------------------------------------|
| `server`            | `192.168.0.100` | Vicon server address                                             |
| `buffer_size`       | `256`           | DataStream client buffer size                                    |
| `namespace`         | `mocap`         | Topic namespace, e.g. `/mocap/FlapperDrone`                      |
| `parent_frame`      | `map`           | World-fixed frame; TF parent and `PoseStamped` `header.frame_id` |
| `publish_parent_tf` | `false`         | Anchor `parent_frame` at the origin on `/tf_static`              |

### TF

For every tracked subject the node broadcasts `parent_frame` → `<subject>_link`, e.g.

```
map ──► FlapperDrone_link
```

`parent_frame` defaults to `map`, following [REP-105](https://www.ros.org/reps/rep-0105.html) and
matching the ENU world frame PX4 expects for external vision. Rename it if it collides with an
existing `map` in your system:

```bash
ros2 launch mocap_vicon_client client.launch.py parent_frame:=mocap_init
```

`parent_frame` is the root of the tree, so it has no transform of its own — with no subject
tracked, `/tf` is empty and RViz reports *"Fixed Frame [map] does not exist"*. That resolves as
soon as a subject is visible to Vicon. If you want the frame to exist regardless, set
`publish_parent_tf:=true`:

```bash
ros2 launch mocap_vicon_client client.launch.py publish_parent_tf:=true
```

This starts a `tf2_ros static_transform_publisher` alongside the client that puts `parent_frame`
at the origin on `/tf_static`:

```
world ──identity──► map ──live pose──► FlapperDrone_link
     (/tf_static)       (/tf)
```

The `world` above it is only there because a TF transform structurally needs both a parent and a
child — there is no way to say "this frame exists at the origin" with one name. It sits at the
same place as `parent_frame` (identity), so the two are one origin under two labels, and
`parent_frame` remains the frame your data is in. That is why `world` is fixed in the launch file
rather than exposed as an argument. If you need a different anchor name, or a real non-identity
offset (e.g. arena origin vs. Vicon origin), run your own `static_transform_publisher` instead.

## Acknowledgements

Special thanks to the contributors and maintainers of the Vicon DataStream SDK and the following projects:

- [OPT4SMART/ros2-vicon-receiver](https://github.com/OPT4SMART/ros2-vicon-receiver)
- [ethz-asl/vicon_bridge](https://github.com/ethz-asl/vicon_bridge)
- [aheuillet/Vicon-ROS2](https://github.com/aheuillet/Vicon-ROS2)
