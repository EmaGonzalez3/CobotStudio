#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory
from sensor_msgs.msg import JointState
from pymycobot.mycobot import MyCobot
import time
from std_msgs.msg import Header

class TrajectoryBridge(Node):
    def __init__(self):
        super().__init__('trajectory_bridge')

        # Configura el puerto serial según tu robot
        self.mc = MyCobot('/dev/ttyUSB0', 115200)
        time.sleep(2)  # Espera a que inicie

        # Nombres de articulaciones
        self.joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6']

        # Suscribirse a trayectorias
        self.subscription = self.create_subscription(
            JointTrajectory,
            '/arm_controller/joint_trajectory',
            self.trajectory_callback,
            10
        )

        # Publisher para RViz
        self.joint_pub = self.create_publisher(JointState, '/joint_states', 10)

    def trajectory_callback(self, msg):
        for point in msg.points:
            angles_rad = point.positions
            angles_deg = [a * 180.0 / 3.141592 for a in angles_rad]

            self.get_logger().info(f'Mandando ángulos: {angles_deg}')
            self.mc.send_angles(angles_deg, 50)  # 50 es velocidad arbitraria

            # Publicar joint_states
            joint_msg = JointState()
            joint_msg.header = Header()
            joint_msg.header.stamp = self.get_clock().now().to_msg()
            joint_msg.name = self.joint_names
            joint_msg.position = angles_rad
            self.joint_pub.publish(joint_msg)

            # Esperar a que termine el movimiento (simple)
            time.sleep(point.time_from_start.sec + point.time_from_start.nanosec / 1e9)

def main(args=None):
    rclpy.init(args=args)
    node = TrajectoryBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()