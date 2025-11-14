#!/usr/bin/env python3
"""
check_collision_humble.py
Consulta a MoveIt2 (ROS2 Humble) si una configuración articular está en colisión.

Ejecutar:
    python3 check_collision_humble.py
"""

import rclpy
from rclpy.node import Node
import numpy as np

from sensor_msgs.msg import JointState
from std_msgs.msg import Header
from moveit_msgs.msg import RobotState
from moveit_msgs.srv import GetStateValidity

# ========= CONFIGURAR TUS JOINTS ACÁ =========
JOINT_NAMES = [
    'joint2_to_joint1',
    'joint3_to_joint2',
    'joint4_to_joint3',
    'joint5_to_joint4',
    'joint6_to_joint5',
    'joint6output_to_joint6'
]

# POSITIONS = [-1.0, 1.5, -1.7, 1.0, 2.3, 0.8, 0.0]
POSITIONS = np.deg2rad([-10.0, 15.0, -17.0, 10.0, 23.0, 30.0, 0.0]).tolist()

GROUP_NAME = "arm"
SERVICE_NAME = "/check_state_validity"
# =============================================

class CollisionChecker(Node):
    def __init__(self):
        super().__init__("collision_checker_humble")

        self.cli = self.create_client(GetStateValidity, SERVICE_NAME)
        if not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"Servicio {SERVICE_NAME} no disponible. ¿move_group levantado?")
            raise RuntimeError("Servicio no disponible.")

        self.get_logger().info("Servicio detectado. Chequeando configuración...")
        self.check()

    def make_robot_state(self):
        js = JointState()
        js.name = JOINT_NAMES
        js.position = POSITIONS
        js.header = Header()
        js.header.stamp = self.get_clock().now().to_msg()

        rs = RobotState()
        rs.joint_state = js
        return rs

    def check(self):
        req = GetStateValidity.Request()
        req.group_name = GROUP_NAME
        req.robot_state = self.make_robot_state()
        # Nada de return_contacts aquí (no existe en Humble)

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if not future.done() or future.result() is None:
            self.get_logger().error("Sin respuesta del servicio.")
            return

        resp = future.result()

        if resp.valid:
            self.get_logger().info("✅ Configuración SIN colisión.")
        else:
            self.get_logger().warn("❌ Configuración EN COLISIÓN.")

        rclpy.shutdown()


def main():
    rclpy.init()
    try:
        CollisionChecker()
    except RuntimeError:
        pass


if __name__ == "__main__":
    main()
