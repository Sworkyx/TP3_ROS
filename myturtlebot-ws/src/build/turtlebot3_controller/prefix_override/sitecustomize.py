import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/eheh1000/Bureau/ROB-POLYTECH/rob4/ROS/tp3-ros2/TP3_ROS/myturtlebot-ws/src/install/turtlebot3_controller'
