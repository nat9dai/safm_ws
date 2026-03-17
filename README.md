# SAFM Workspace

## Install some dependencies:
```bash
# For prepare.sh
sudo apt install socat

# Install Micro XRCE-DDS Agent
git clone -b v2.4.3 https://github.com/eProsima/Micro-XRCE-DDS-Agent.git
cd Micro-XRCE-DDS-Agent
mkdir build
cd build
cmake ..
make -j8 # adjust this according to CPU's core count
sudo make install
sudo ldconfig /usr/local/lib/

# Install ROS2 Foxy

# Install ROS1 Noetic
``` 

## Working with ros1_bridge

### Build px4_msgs in ROS1 workspace (skip this step if you don't want to "bridge-all-topics"):
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
./scripts/ros1_bridge.sh 1 # -> bridge all topics (frequencies drop a lot)
    or
./scripts/ros1_bridge.sh 2 # -> bridge specified topics (params/bridge_topics.yaml)
```