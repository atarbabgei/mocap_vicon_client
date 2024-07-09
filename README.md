
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
git clone https://github.com/atarbabgei/mocap_msgs.git
git clone https://github.com/atarbabgei/mocap_vicon_client.git
```

### Build the Packages

Navigate to the root of your workspace and build the packages using \`colcon\`:

```bash
cd ~/ros2_ws
colcon build --packages-select mocap_vicon_client mocap_msgs
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

## Acknowledgements

Special thanks to the contributors and maintainers of the Vicon DataStream SDK and the following projects:

- [OPT4SMART/ros2-vicon-receiver](https://github.com/OPT4SMART/ros2-vicon-receiver)
- [ethz-asl/vicon_bridge](https://github.com/ethz-asl/vicon_bridge)
- [aheuillet/Vicon-ROS2](https://github.com/aheuillet/Vicon-ROS2)
