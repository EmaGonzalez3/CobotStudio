#!/bin/bash

source /opt/ros/humble/setup.bash
source ~/colcon_ws/install/setup.bash


echo "🧹 Cerrando RViz..."
pkill rviz2

echo "🛑 Matando launch..."
# Asume que el launch se ejecutó como: ros2 launch mi_robot_launch main.launch.py
pkill -f "ros2 launch mycobot_320 move.launch.py"

echo "🛑 Matando script de robtargets..."
# Asume que el script de ternas se ejecutó como: ros2 run mi_robot_tf publish_robtargets
pkill -f verTernas.py


sleep 1

echo "🚀 Relanzando launch limpio..."
gnome-terminal -- bash -c "ros2 launch mycobot_320 move.launch.py; exec bash"

