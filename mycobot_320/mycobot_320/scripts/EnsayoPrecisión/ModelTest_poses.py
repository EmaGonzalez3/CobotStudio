from CobotStudio_rev4 import RobTarget, BaseRobotController, SimManager, MyCobotController
from spatialmath import SE3
import numpy as np
from DHRobotGT import myCobot320
import time
import random

joint_limits = {
    "joint_1": (-168.0, 168.0),
    "joint_2": (-135.0, 135.0),
    "joint_3": (-150.0, 150.0),
    "joint_4": (-145.0, 145.0),
    "joint_5": (-165.0, 165.0),
    "joint_6": (-180.0, 180.0),
}

def generar_q_respetando_limites(limits_mult=0.7):
    """Genera un vector de 6 valores dentro de los límites articulares del robot."""
    q = [
        round(random.uniform(lim_inf * limits_mult, lim_sup * limits_mult), 2)
        for lim_inf, lim_sup in joint_limits.values()
    ]
    return q

def q_alcanzables(nombre_archivo: str, cant=10):
    """ Genera una lista de vectores q alcanzables según lo visualizado en RViz."""
    nombre_archivo = nombre_archivo

    contador = 0

    rviz = SimManager()
    rviz.GripperState(0)
    time.sleep(1)
    while contador < cant:
        q_random = generar_q_respetando_limites()
        print(q_random)

        rviz.VerQ(q_random)
        confirmacion = input("Es alcanzable? (1): ")
        if confirmacion.lower() == '1':
            with open(nombre_archivo, "a") as f:
                f.write(f"q_{contador} = {q_random}\n")
            contador += 1

    rviz.shutdown()

def visualizar_qs(nombre_archivo: str, start = 0):
    """ Visualiza en RViz los vectores q almacenados en el archivo."""
    nombre_archivo = nombre_archivo

    rviz = SimManager()
    rviz.GripperState(0)
    time.sleep(1)

    with open(nombre_archivo, "r") as f:
        lines = f.readlines()

    q_list = []
    for line in lines:
        if line.startswith("q_"):
            q_str = line.split('=')[1].strip()
            q = eval(q_str)
            q_list.append(q)

    for i, q in enumerate(q_list[start:], start=start):
        print(f"q_{i} = {q}")
        rviz.MoveJAngles(q)
        input("Presiona Enter para continuar...")

    rviz.shutdown()

visualizar_qs("ModelTest_q5.py", 0)
# q_alcanzables("ModelTest_q5.py", 2)