import time
import numpy as np
from pymycobot import MyCobotSocket
import random
import os

script_dir = os.path.dirname(os.path.abspath(__file__))
filename = os.path.join(script_dir, "QvsPose30-04.txt")

# Inicialización del robot
mc = MyCobotSocket("10.42.0.1", 9000)

# Límites articulares (en grados, extraídos de tu modelo DH)
joint_limits_deg = [
    (-170, 170),
    (-120, 120),
    (-148, 148),
    (-120, 135),
    (-168, 168),
    (-180, 180)
]

def generar_q_respetando_limites():
    """Genera un vector de 6 valores dentro de los límites articulares del robot."""
    return [round(random.uniform(lim[0]*0.7, lim[1]*0.7), 2) for lim in joint_limits_deg]

# Archivo de salida
qs_prueba = [[20, 20, 30, 15, 0, 0], [-10, -25, 10, 0, 0, 0]]
with open(filename, "a") as f:
    for i in range(1, 6):
        if i == 1:
            q_random = generar_q_respetando_limites()
        mc.send_angles(q_random, 30)
        # mc.send_angles(qs_prueba[i-1], 30)
        # mc.send_angles([0, 0, 0, 0, 0, 0], 30)
        time.sleep(3)

        while not mc.is_in_position(q_random, 0):
            mc.resume()
            mc.pause()
        time.sleep(7)

        q_real = mc.get_angles()

        # Si corro estas 2 líneas introduzco el error del control!
        # mc.send_angles(q_real, 30)
        # time.sleep(2)

        pose = mc.get_coords()

        print(f"{i}\t{[round(q,2) for q in q_real]}\t{[round(p,2) for p in pose]}\n")
        # f.write(f"{i}\t{[round(q,2) for q in q_real]}\t{[round(p,2) for p in pose]}\n")
        f.write(f"{i}; {[round(q,2) for q in q_real]}; {[round(p,2) for p in pose]}; {[round(ang,2) for ang in q_random]}\n")
        mc.send_angles([0, 0, 0, 0, 0, 0], 40)
        print(f"Iteración {i} completada")
        time.sleep(5)
print(f"\nDatos guardados en {filename}")