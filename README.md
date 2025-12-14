# TP3_ROS


### Compilation

```bash
# 1. cloner le repository
cd ~
git clone https://github.com/Sworkyx/TP3_ROS.git

# 2. compiler 
cd ~/TP3_ROS/myturtlebot-ws
colcon build 
source install/setup.bash
```

### Lancement

```bash
# T1 : gazebo
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# T2 : bridge
ros2 run ros_gz_bridge parameter_bridge /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist

# T3 : noeud (exemple : teleop)
ros2 run my_teleop teleop
```

