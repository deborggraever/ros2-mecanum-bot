# ros2-mecanum-bot
ROS2 Mecanum wheel robot

## Getting started

#### Prerequisites
This project is build and tested on Ubuntu 22.04 LTS with ROS 2 Humble LTS.  
For ROS 2 Foxy on Ubuntu 20.04 LTS see: [Branch: Foxy](https://github.com/deborggraever/ros2-mecanum-bot/tree/foxy)
* [ROS install instructions](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)
* [Colcon install instructions](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Colcon-Tutorial.html)

#### Setup workspace
```
mkdir -p ~/workspaces/ros2-mecanum-bot/src
cd ~/workspaces/ros2-mecanum-bot/src
git clone git clone git@github.com:deborggraever/ros2-mecanum-bot.git .
```

#### Install dependencies
```
cd ~/workspaces/ros2-mecanum-bot
rosdep update
rosdep install --from-paths src --ignore-src --rosdistro humble -r -y
```

#### Build and run
```
cd ~/workspaces/ros2-mecanum-bot
colcon build --symlink-install
source install/setup.bash
ros2 launch mecanumbot_bringup mecanumbot_hardware.py
```

#### Visualize the robot

```
cd ~/workspaces/ros2-mecanum-bot
source install/setup.bash
ros2 launch mecanumbot_bringup rviz2.py
```
