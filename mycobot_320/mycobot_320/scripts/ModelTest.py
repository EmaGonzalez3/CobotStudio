from CobotStudio_rev4 import (
    RobTarget,
    MyCobotController,
    pose_to_matrix
)
from spatialmath import SE3
import numpy as np
from DHRobotGT import myCobot320
import time
import csv


cobot_tb = myCobot320(rotar_base=True, metros=False)
# pinza = SE3(-1.71, 106.91, 28.33) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)
pinza = SE3(0, 123, 27.5) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)
pinza_aux = pinza*SE3(0,0,25)



def testCoords (q_list, filename = "test_coords_log.csv"):
    """ Test del compatibilidad entre el modelo virtual y real para las coordenadas del robot."""
    cob = MyCobotController()

    print(f"Iniciando ensayo de correspondencia (versión consistente)...")

    fieldnames = [
        'punto_n',
        'q_esp', 'q_alc', 'err_q',
        'pose_esp', 'pose_alc', 'err_pose',
        'tb_check', 'error_rot_fro'
    ]

    with open(filename, 'w', newline='') as csvfile:
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames, delimiter=';')
        writer.writeheader()

        for i, q in enumerate(q_list):
            print(f'\nPosición {i+1} de {len(q_list)}: {q}')

            # Calculamos con la toolbox la pose correspondiente al vector q
            q_rad = np.deg2rad(q)
            print(f'Conf calculada\n{cobot_tb.calc_conf(q_rad)}')
            pose_esperada_SE3 = cobot_tb.fkine(q_rad)
            pose_esperada_rpy = matrix_to_pose(pose_esperada_SE3)

            # Armamos el robtarget según la pose calculada y lo enviamos al robot
            robt = RobTarget(pose_esperada_SE3, cobot_tb.calc_conf(q_rad).tolist())
            cob.MoveJ(robt, 30, SE3(), SE3())
            time.sleep(5)

            # Vemos las coordenadas alcanzadas por el robot
            pose_alcanzada = cob.mc.get_coords()
            pose_alcanzada_SE3 = pose_to_matrix(pose_alcanzada)
            # También le pedimos a qué ángulos llegó cada eje
            q_alcanzado = cob.mc.get_angles()

            # Chequeamos que la toolbox corresponda con la pose que reporta el robot (sin considerar error de ángulos)
            pose_alcanzada_tbSE3 = cobot_tb.fkine(np.deg2rad(q_alcanzado))
            pose_alcanzada_tbrpy = matrix_to_pose(pose_alcanzada_tbSE3)

            # Vemos la diferencia entre la pose que queríamos alcanzar y la que se alcanzó realmente
            q_error = np.array(q) - np.array(q_alcanzado)
            pose_error = np.array(pose_esperada_rpy) - np.array(pose_alcanzada)
            tb_check = np.array(pose_alcanzada[:3]) - np.array(pose_alcanzada_tbrpy[:3])
            rot_error = np.linalg.norm(pose_alcanzada_SE3.R - pose_esperada_SE3.R, 'fro')

            # --- Creación del diccionario de datos para la fila CSV ---
            row_data = {
                'punto_n': i + 1,
                'q_esp': format_vector(q),
                'q_alc': format_vector(q_alcanzado),
                'err_q': format_vector(q_error),
                'pose_esp': format_vector(pose_esperada_rpy),
                'pose_alc': format_vector(pose_alcanzada),
                'err_pose': format_vector(pose_error),
                'tb_check': format_vector(tb_check),
                'error_rot_fro': f"{rot_error:.6f}"
            }
            writer.writerow(row_data)
            print(f"-> Datos del punto {i+1} guardados.")

        print(f'\nEnsayo completado. Todos los datos han sido guardados en "{filename}"')

def matrix_to_pose(T_matrix):
    """Convierte una matriz homogénea SE3 en lista [x, y, z, rx, ry, rz] en grados."""
    T = SE3(T_matrix)
    x, y, z = T.t
    # Extraer rotación como RPY (en radianes), orden ZYX
    rx, ry, rz = T.rpy(order='zyx', unit='rad')

    # Convertimos a grados
    rx, ry, rz = np.rad2deg([rx, ry, rz])

    return [x, y, z, rx, ry, rz]

def format_vector(vec, precision=4):

    """Convierte un vector/lista de números en un string formateado y legible."""
    return " ".join([f"{v:.{precision}f}" for v in vec])

def get_qs(nombre_archivo: str):
    with open(nombre_archivo, "r") as f:
        lines = f.readlines()

    q_list = []
    for line in lines:
        if line.startswith("q_"):
            q_str = line.split('=')[1].strip()
            q = eval(q_str)
            q_list.append(q)
    
    return q_list

q_test = [[-52.82, -39.81, -71.63, 6.59, -3.25, 101.6], [-51.41, -62.49, -96.59, 137.28, 6.85, 14.5]]

# testCoords(q_test, filename="test_coords_log_v3.csv")

print(f'q_test = {q_test}')
print(f'q_leidos = {get_qs("ModelTest_q0.py")}')