# Usage

Updated to work with ROS 2 and `colcon`.

```bash
# On the robot:
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/ual-arm-ros-pkg/summit-arm-ual.git

cd ~/ros2_ws
colcon build 
```

**TO-DO**: Port basic launch files and joystick control (or just use standard ROS 2 teleop)

