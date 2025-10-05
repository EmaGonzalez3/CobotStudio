import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import roboticstoolbox as rtb
import numpy as np

q = np.array([0.5, -0.5, 0.5, -0.5, 0.5, 0.0])

rclpy.init()
ros2node = Node('test_move')
joint_pub = ros2node.create_publisher(JointState, '/joint_states', 10)  
msg = JointState()
joint_names = ["joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3", "joint5_to_joint4", "joint6_to_joint5", "joint6output_to_joint6"]
msg.name = joint_names
msg.header.stamp = ros2node.get_clock().now().to_msg()
msg.position = q.tolist()

joint_pub.publish(msg)
ros2node.get_logger().info(f'Publicado: {msg.position}')
#self.get_logger().info(f'Publicado: {msg.position}')