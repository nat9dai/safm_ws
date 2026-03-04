# SAFM Workspace

## Working with ros1_bridge
### Install some dependencies:
```bash
sudo apt-get install ros-foxy-mavros-msgs -y
sudo apt-get install ros-noetic-mavros ros-noetic-mavros-extras -y
```

### Build px4_msgs in ROS1 workspace:
```bash
cd ~
git clone https://github.com/nat9dai/ros1_msgs_ws.git
cd ros1_msgs_ws
git submodule update --init --recursive
source /opt/ros/noetic/setup.bash
colcon build --packages-select px4_msgs
```

### Build ros1_bridge:
```bash
cd safm_ws
git submodule update --init --recursive

source /opt/ros/foxy/setup.bash
colcon build --packages-select px4_msgs

source /opt/ros/noetic/setup.bash && source ~/ros1_msgs_ws/devel/setup.bash && source /opt/ros/foxy/setup.bash && source install/local_setup.bash
colcon build --packages-select ros1_bridge --cmake-force-configure
colcon build --packages-select conversion_pkg
```
### To run with ros1_bridge:
First terminal:
```bash
source /opt/ros/noetic/setup.bash
roscore # or launch a ROS1 launch
```
Second terminal:
```bash
source /opt/ros/noetic/setup.bash && source ~/ros1_msgs_ws/devel/setup.bash && source /opt/ros/foxy/setup.bash && source ~/safm_ws/install/local_setup.bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```