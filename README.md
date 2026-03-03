# SAFM Workspace

```bash
cd safm_ws
git submodule update --init --recursive
source /opt/ros/foxy/setup.bash
colcon build --packages-select px4_msgs mavros_msgs
source install/local_setup.bash
colcon build --packages-select conversion_pkg ros1_bridge
```

## To run with ros1_bridge:
First terminal:
```bash
source /opt/ros/noetic/setup.bash
```

Second terminal:
```bash
source /opt/ros/noetic/setup.bash
source /opt/ros/foxy/setup.bash
source ~/safm_ws/install/local_setup.bash
ros2 run ros1_bridge dynamic_bridge --bridge-all-topics
```