# SAFM Workspace

```bash
cd safm_ws
git submodule update --init --recursive
source /opt/ros/foxy/setup.bash
colcon build --packages-select px4_msgs mavros_msgs
source install/local_setup.bash
colcon build --packages-select conversion_pkg ros1_bridge
```