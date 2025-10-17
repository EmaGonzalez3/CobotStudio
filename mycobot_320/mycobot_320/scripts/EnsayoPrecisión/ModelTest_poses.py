from scripts.CobotStudio_rev4 import RobTarget, BaseRobotController, SimManager, MyCobotController
from spatialmath import SE3
from scripts.DHRobotGT import myCobot320
import time
import random
import os
import importlib

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

def q_alcanzables(nombre_base_archivo: str, cant=10):
    """ Genera una lista de vectores q alcanzables según lo visualizado en RViz."""

    directorio_script = os.path.dirname(os.path.abspath(__file__))
    ruta_completa_archivo = os.path.join(directorio_script, 'Poses', nombre_base_archivo)

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
            with open(ruta_completa_archivo, "a") as f:
                f.write(f"q_{contador} = {q_random}\n")
            contador += 1

    rviz.shutdown()

def visualizar_qs(nombre_base_modulo: str, start = 0):
    """ Visualiza en RViz los vectores q almacenados en el archivo."""
    nombre_completo_modulo = f"scripts.EnsayoPrecisión.Poses.{nombre_base_modulo}"
    try:
        print(f"Intentando importar: {nombre_completo_modulo}") 
        modulo_poses = importlib.import_module(nombre_completo_modulo)
    except ImportError:
        print(f"Error: No se pudo encontrar el módulo '{nombre_completo_modulo}'.")
        print("Asegúrate de que el nombre del archivo es correcto y que Poses/__init__.py existe.")
        return

    # Extrae todas las variables que empiezan con 'q_' del módulo importado
    q_list = [getattr(modulo_poses, var) for var in dir(modulo_poses) if var.startswith('q_')]

    rviz = SimManager()
    rviz.GripperState(0)
    time.sleep(1)

    for i, q in enumerate(q_list[start:], start=start):
        print(f"q_{i} = {q}")
        rviz.MoveJAngles(q)
        input("Presiona Enter para continuar...")

    rviz.shutdown()

visualizar_qs("ModelTest_q5", 0)
# q_alcanzables("ModelTest_q5.py", 2)