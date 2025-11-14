#!/usr/bin/env python3
"""
extract_waypoints.py

Planifica una configuración articular con pymoveit2, extrae los waypoints articulares
de la trayectoria planificada y los guarda en:
  - waypoints.py   -> variable 'WAYPOINTS' como lista de listas (Python)
  - waypoints.csv  -> CSV, cada fila = un waypoint (posiciones separadas por coma)

Uso:
  python3 extract_waypoints.py
Reemplazá joint_names y target_positions según tu robot.
"""
import rclpy
from rclpy.node import Node
from threading import Thread
import time
import json
import csv
import os

# IMPORTA pymoveit2 (asegurate que esté en tu PYTHONPATH / entorno)
from pymoveit2 import MoveIt2

OUTPUT_PY = "waypoints.py"
OUTPUT_CSV = "waypoints.csv"

def save_as_py(waypoints, filename=OUTPUT_PY, var_name="WAYPOINTS"):
    with open(filename, "w", encoding="utf-8") as f:
        f.write(f"# Autogenerado: lista de waypoints (cada waypoint es lista de joint positions)\n")
        f.write(f"{var_name} = \\\n")
        f.write(repr(waypoints))
        f.write("\n")
    print(f"Guardado {len(waypoints)} waypoints en {filename}")

def save_as_csv(waypoints, filename=OUTPUT_CSV):
    with open(filename, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        for wp in waypoints:
            writer.writerow(wp)
    print(f"Guardado {len(waypoints)} waypoints en {filename} (CSV)")

def extract_joint_traj_points(trajectory):
    """
    Acepta diferentes tipos de retorno:
      - moveit_msgs/RobotTrajectory (tiene atributo joint_trajectory)
      - trajectory_msgs/JointTrajectory (tiene .points)
    Devuelve una lista de listas: [[q1_1, q1_2, ...], [q2_1, q2_2, ...], ...]
    """
    jt = None
    # robot_trajectory message (MoveIt suele devolver esto en algunos casos)
    if hasattr(trajectory, "joint_trajectory"):
        jt = trajectory.joint_trajectory
    # JointTrajectory directamente
    elif hasattr(trajectory, "points") and hasattr(trajectory, "joint_names"):
        jt = trajectory
    else:
        # intentar acceder por llave (por si es un dict-like)
        try:
            if "joint_trajectory" in trajectory:
                jt = trajectory["joint_trajectory"]
        except Exception:
            pass

    if jt is None:
        raise RuntimeError("No se pudo extraer JointTrajectory del objeto 'trajectory' retornado.")

    waypoints = []
    for p in jt.points:
        # 'positions' es una secuencia numérica
        waypoints.append([float(x) for x in p.positions])
    return jt.joint_names if hasattr(jt, "joint_names") else None, waypoints

def main():
    # --- AJUSTAR ESTO según tu robot ---
    joint_names = [
        "joint2_to_joint1",
        "joint3_to_joint2",
        "joint4_to_joint3",
        "joint5_to_joint4",
        "joint6_to_joint5",
        "joint6output_to_joint6",
        # si tu grupo tiene gripper u otras articulaciones, agregalas
    ]
    # target (objetivo) --- reemplazá por la configuración objetivo que quieras planificar
    target_positions = [1.5, -0.7, 0.7, 0.0, 1.2, 0.0]  # debe coincidir longitud con joint_names
    group_name = "arm"
    base_link = "base"
    ee_link = "link6"
    # ------------------------------------

    if len(joint_names) != len(target_positions):
        raise SystemExit("ERROR: joint_names y target_positions no tienen la misma longitud.")

    rclpy.init()
    node = Node("extract_waypoints_node")

    # Crear MoveIt2
    moveit2 = MoveIt2(
        node=node,
        joint_names=joint_names,
        base_link_name=base_link,
        end_effector_name=ee_link,
        group_name=group_name,
    )

    # Spin en background (necesario para que MoveIt2 inicialice internals / servicios)
    exec_thread = Thread(target=rclpy.spin, args=(node,), daemon=True)
    exec_thread.start()
    # dejar un instante para que el cliente/service se registren
    time.sleep(1.0)

    node.get_logger().info(f"Planificando hacia {target_positions} (sin ejecutar)...")

    # Usar la función de plan (pymoveit2 expone plan(...) que no ejecuta)
    try:
        # plan puede aceptar joint_positions=... o joint_goal=..., según versión; aquí usamos joint_positions
        trajectory = moveit2.plan(joint_positions=target_positions)
    except Exception as e:
        node.get_logger().error(f"Error llamando moveit2.plan(): {e}")
        rclpy.shutdown()
        exec_thread.join(timeout=0.1)
        return

    if trajectory is None:
        node.get_logger().error("Planificación fallida: se devolvió None (no hay trayectoria).")
        rclpy.shutdown()
        exec_thread.join(timeout=0.1)
        return

    # Extraer waypoints
    try:
        maybe_joint_names, waypoints = extract_joint_traj_points(trajectory)
    except Exception as e:
        node.get_logger().error(f"No pude extraer JointTrajectory: {e}")
        rclpy.shutdown()
        exec_thread.join(timeout=0.1)
        return

    # Si el objeto devuelve joint_names internos, los mostramos
    if maybe_joint_names:
        node.get_logger().info(f"Joint names from trajectory: {maybe_joint_names}")

    # Guardar archivos en el directorio actual
    save_as_py(waypoints, filename=os.path.abspath(OUTPUT_PY))
    save_as_csv(waypoints, filename=os.path.abspath(OUTPUT_CSV))

    # Cerrar
    rclpy.shutdown()
    exec_thread.join(timeout=0.1)
    print("Hecho.")

if __name__ == "__main__":
    main()
