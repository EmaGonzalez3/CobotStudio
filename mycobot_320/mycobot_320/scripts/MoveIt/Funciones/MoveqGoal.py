#!/usr/bin/env python3
"""
set_moveit_goal_state.py

- Hardcodeá JOINT_NAMES y POSITIONS abajo.
- Ejecutar: python3 set_moveit_goal_state.py
- Requiere:
  * move_group corriendo (MoveIt).
  * RViz MotionPlanning plugin con "Allow External Comm." activado
    para que el publish a /rviz/moveit/update_goal_state funcione.
"""

import rclpy
from rclpy.node import Node
import time
import numpy as np

from std_msgs.msg import Empty
from sensor_msgs.msg import JointState
from std_msgs.msg import Header

# Mensajes/servicios de MoveIt
from moveit_msgs.msg import PlanningScene, RobotState
from moveit_msgs.srv import ApplyPlanningScene

JOINT_NAMES = [
    'joint2_to_joint1',
    'joint3_to_joint2',
    'joint4_to_joint3',
    'joint5_to_joint4',
    'joint6_to_joint5',
    'joint6output_to_joint6',
    'gripper_controller',
]
POSITIONS = np.deg2rad([-10.0, 15.0, -17.0, 10.0, 23.0, 30.0, 0.0]).tolist()  # en radianes, mismo orden que JOINT_NAMES

APPLY_SCENE_SVC = "/apply_planning_scene"                 # servicio que expone move_group
UPDATE_GOAL_TOPIC = "/rviz/moveit/update_goal_state"     # tópico para disparar update (Empty)
# =================================================================

class ApplyPlanningSceneClient(Node):
    def __init__(self):
        super().__init__("apply_planning_scene_client")

        # Cliente del servicio apply_planning_scene
        self.cli = self.create_client(ApplyPlanningScene, APPLY_SCENE_SVC)
        if not self.cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"Servicio '{APPLY_SCENE_SVC}' no disponible (¿move_group levantado?). Abortando.")
            raise RuntimeError("apply_planning_scene service not available")

        # Publisher para forzar el update del Goal en RViz (si Allow External Comm. ON)
        self.pub_update = self.create_publisher(Empty, UPDATE_GOAL_TOPIC, 10)

    def make_request(self, joint_names, positions):
        # Construyo PlanningScene request
        req = ApplyPlanningScene.Request()
        scene = PlanningScene()
        rs = RobotState()
        js = JointState()

        js.header = Header()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = joint_names
        js.position = positions

        rs.joint_state = js
        scene.robot_state = rs

        # Algunos despliegues esperan diffs; intentamos marcar is_diff si el campo existe
        # (dependiendo de la versión del message puede/no existir; si no existe, pasamos)
        try:
            scene.is_diff = True
        except Exception:
            # campo no presente: ignore
            pass

        req.scene = scene
        return req

    def call_apply(self, joint_names, positions, timeout=5.0):
        req = self.make_request(joint_names, positions)
        fut = self.cli.call_async(req)
        self.get_logger().info("Llamando a /apply_planning_scene ...")
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if fut.done() and fut.result() is not None:
            res = fut.result()
            # ApplyPlanningScene.srv devuelve 'success' boolean
            try:
                ok = bool(res.success)
            except Exception:
                ok = False
            self.get_logger().info(f"apply_planning_scene returned success={ok}")
            return ok
        else:
            self.get_logger().error("No hubo respuesta del servicio /apply_planning_scene o timeout.")
            return False

    def trigger_rviz_update_goal(self):
        # Publicar varias veces por robustez
        e = Empty()
        for i in range(3):
            self.pub_update.publish(e)
            self.get_logger().info(f"Publicado Empty en '{UPDATE_GOAL_TOPIC}' ({i+1}/3)")
            # sleep corto para dar tiempo a RViz/plugin
            time.sleep(0.07)


def main():
    rclpy.init()
    try:
        client = ApplyPlanningSceneClient()
    except RuntimeError:
        rclpy.shutdown()
        return

    # 1) Aplicar estado en MoveIt (planning_scene.current)
    ok = client.call_apply(JOINT_NAMES, POSITIONS, timeout=6.0)
    if not ok:
        client.get_logger().warning("apply_planning_scene falló. Verificar nombres de joints y que move_group esté corriendo.")
    else:
        # 2) Trigger para que el plugin copie current -> goal (necesita Allow External Comm.)
        #    El orden importa: primero actualizamos planning_scene, luego pedimos update_goal_state.
        time.sleep(0.06)
        client.trigger_rviz_update_goal()

    # limpieza
    client.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
