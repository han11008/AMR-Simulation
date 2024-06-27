export ROS_DISCOVERY_SERVER=$( hostname -I | grep -o "192.168.[0-9.]\+" ):11811

ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=motor_vel

