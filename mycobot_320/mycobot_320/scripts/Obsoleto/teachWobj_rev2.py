from CobotStudio_rev4 import (
    RobTarget,
    MyCobotController,
    ROS_OK,
    pose_to_matrix,
    TCP_4puntos,
    TCP_4puntos_extendido,
    joystick_adjust,
    teach_wobj
)
from spatialmath import SE3, UnitQuaternion
import numpy as np
from DHRobotGT import myCobot320
import time
from pymycobot import MyCobotSocket
import datetime
import os

if ROS_OK:
    from CobotStudio_rev4 import SimManager, checkQ, checkPose

cobot_tb = myCobot320(rotar_base=True, metros=False)
# cob = MyCobotController()
# pinza = SE3(-1.71381642, 106.90735789, 28.32702833) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)
pinza = SE3(0, 123, 27.5) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)

def enseñarCobot(pinza):
    """Enseñar el wobj en el cobot. Devuelve el wobj como SE3."""
    q_vals = cob.grabar_poses(6, ajuste = True)
    wobj = teach_wobj(q_vals, pinza)
    print(f'Wobj calculado\n{wobj}')
    return wobj, q_vals

def sendWobj(wobj, pinza, conf = [1, 1, 1]):
    """Enviar el cobot real al wobj calculado"""
    robt0 = RobTarget(SE3(wobj), conf)
    print(robt0.pose)
    cob.MoveJ(robt0, 30, pinza, SE3())

def cheqWobj(wobj, pinza, conf = [1, 1, 1]):
    ''' Envía el wobj a RViz desde la pose home.'''
    rviz = SimManager()
    robt0 = RobTarget(SE3(wobj), conf)
    print(robt0.find_valid_configs(pinza, SE3()))
    # rviz.VerPose(robt0, SE3(), tool = pinza)
    # checkQ(np.zeros(6), pinza, True)
    rviz.MoveJ(robt0, speed = 30, tool = pinza, wobj = SE3())
    rviz.shutdown()

def ortonormarlizar_wobj(wobj_calc):
    ''' Corrige la rotación del wobj por determinante y ortogonalidad.'''
    R = wobj_calc[:3, :3]
    t = wobj_calc[:3, 3]

    # Proyección a SO(3) vía SVD (descomposición polar)
    U, _, Vt = np.linalg.svd(R)
    R_orth = U @ Vt
    if np.linalg.det(R_orth) < 0:        # asegurar rotación propia (det=+1)
        U[:, -1] *= -1
        R_orth = U @ Vt

    # print("det:", np.linalg.det(R_orth)) # Opcional: chequear

    # Re-armar el SE3 corregido
    wobj_fixed = np.eye(4)
    # R_orth[2,2] += 1e-14 # Con esta leve modificación ya se rompe
    wobj_fixed[:3, :3] = R_orth
    # print(f'wobj_fixed=\n{wobj_fixed}')
    wobj_fixed[:3, 3]  = t


    # Si usás SpatialMath/RTB:
    wobj_SE3 = SE3(wobj_fixed)

    print(f"Wobj corregido:\n{wobj_SE3}")
    return wobj_SE3

def rot_error(wobj_calc):
    ''' Calcula el error de ortogonalidad y determinante de la R del wobj.'''
    R = wobj_calc[:3, :3]
    I = np.eye(3)
    err_orth = np.linalg.norm(R.T @ R - I, 'fro')
    err_det = abs(np.linalg.det(R) - 1)
    print(f"Error ortogonalidad: {err_orth:.2e}, Error determinante: {err_det:.2e}")
    return err_orth, err_det

def save_terna(filename, name, se3_obj):
    """Guarda un SE3 en un archivo .npz con el nombre dado."""
    try:
        data = np.load(filename, allow_pickle=True)
        store = dict(data)   # convertir a dict editable
    except FileNotFoundError:
        store = {}
    
    store[name] = se3_obj.A   # guardamos la matriz homogénea (4x4)
    np.savez(filename, **store)
    print(f"[OK] Guardado {name} en {filename}")

def load_targets(filename, terna = None):
    """
    Carga SE3 guardados en un archivo .npz.
    - Si name=None: devuelve todos como dict.
    - Si name="algo": devuelve solo ese SE3.
    """
    data = np.load(filename, allow_pickle=True)
    if terna is None:
        return {key: SE3(data[key]) for key in data.files}
    else:
        key = terna if terna in data.files else None
        if key is None:
            raise KeyError(f"{terna} no encontrado en {filename}")
        return SE3(data[key])

def save_terna_txt(filename, name, se3_obj):
    """Guarda una terna SE3 en texto plano (editable)."""
    with open(filename, "a") as f:
        f.write(f"{name} =\n")
        np.savetxt(f, se3_obj.A, fmt="%.18e")
        f.write("\n")

def load_terna_txt(filename, name=None):
    """
    Carga ternas SE3 desde un archivo .txt.
    - Si name=None: devuelve todas como dict {nombre: SE3}
    - Si name="algo": devuelve solo ese SE3
    """
    ternas = {}
    with open(filename, "r") as f:
        lines = [line.strip() for line in f.readlines() if line.strip()]

    i = 0
    while i < len(lines):
        if "=" in lines[i]:
            key = lines[i].replace("=", "").strip()
            block = lines[i+1:i+5]
            mat = np.array([[float(x) for x in row.split()] for row in block])
            ternas[key] = SE3(mat)
            i += 5
        else:
            i += 1

    if name is None:
        return ternas
    else:
        if name not in ternas:
            raise KeyError(f"{name} no encontrado en {filename}")
        return ternas[name]

def save_terna_quat(filename, name, se3_obj):
    """Guarda una terna SE3 como posición + cuaternión en texto plano."""
    t = se3_obj.t
    q = UnitQuaternion(se3_obj.R)  # (qx, qy, qz, qw)
    data = np.hstack([t, q])
    with open(filename, "a") as f:
        f.write(f"{name} =\n")
        np.savetxt(f, data.reshape(1, -1), fmt="%.18e")
        f.write("\n")

def save_qvals(filename, name, q):
    """Guarda la lista con los q enseñados en texto plano."""
    with open(filename, "a") as f:
        f.write(f"{name} = [\n")
        for row in q:
            f.write("    " + str([round(val, 5) for val in row]) + ",\n")
        f.write("]\n\n")

def load_terna_quat(filename, name=None):
    """
    Carga ternas SE3 guardadas como [x y z qx qy qz qw].
    - Si name=None: devuelve todas como dict {nombre: SE3}
    - Si name="algo": devuelve solo ese SE3
    """
    ternas = {}
    with open(filename, "r") as f:
        lines = [line.strip() for line in f.readlines() if line.strip()]

    i = 0
    while i < len(lines):
        if "=" in lines[i]:
            key = lines[i].replace("=", "").strip()
            vals = [float(x) for x in lines[i+1].split()]
            # print(f'Vals leidos = {vals}')
            if len(vals) != 7:
                raise ValueError(f"Formato inválido en {key}: {vals}")
            t = vals[:3]
            q = vals[3:]
            ternas[key] = SE3.Rt(UnitQuaternion(q).R, t)
            i += 2
        else:
            i += 1

    if name is None:
        return ternas
    else:
        if name not in ternas:
            raise KeyError(f"{name} no encontrado en {filename}")
        return ternas[name]

def save_terna_se3(filename: str, name: str, se3_obj: SE3):
    """
    Guarda un objeto SE3 en un archivo .py como una variable.
    Añade la cabecera con los imports necesarios solo si el archivo es nuevo.

    Args:
        filename: Nombre del archivo (ej. 'mis_ternas.py').
        name: Nombre de la variable a guardar (ej. 'wobj1').
        se3_obj: El objeto SE3 a guardar.
    """

    header = "from spatialmath import SE3\nfrom numpy import array\n\n"
    needs_header = True

    # Comprobar si el archivo existe y tiene contenido para decidir si se necesita la cabecera.
    if os.path.exists(filename):
        try:
            with open(filename, 'r') as f:
                first_line = f.readline()
                # Si la primera línea ya es un import, asumimos que la cabecera existe.
                if first_line.strip().startswith('from spatialmath'):
                    needs_header = False
        except IOError:
            # El archivo podría existir pero no ser legible, en cuyo caso es mejor reescribir.
            needs_header = True

    # Abrir en modo 'a' (append) para añadir al final.
    # Si el archivo no existe, se crea automáticamente.
    with open(filename, "a") as f:
        if needs_header:
            # Si el archivo es nuevo, está vacío o no tiene los imports, los escribimos.
            f.write(header)
        
        timestamp = datetime.datetime.now().isoformat()
        f.write(f"# Guardado: {timestamp}\n") # Guardamos la fecha y hora actual
        f.write(f"{name} = {repr(se3_obj)}\n\n")

    print(f"[OK] Guardado '{name}' en '{filename}'")

def save_qvals_py(filename, name, q):
    """
    Guarda una lista de variables articulares como variable Python en un archivo .py
    """
    with open(filename, "a") as f:
        f.write(f"{name} = [\n")
        for row in q:
            f.write(f"    {list(map(float, row))},\n")
        f.write("]\n\n")


"""Cobot real"""
# wobj_enseñado, q_enseñados = enseñarCobot(pinza)
# save_terna_se3("Workobjects_v5.py", "wobj0", wobj_enseñado)
# save_qvals_py("Workobjects_qv5.py", "q_0", q_enseñados)

# from Workobjects_v5 import wobj0
# robt = RobTarget(wobj0, [1, -1, -1])
# cheqWobj(robt.pose, pinza, [1, -1, -1])
pinza_aux = pinza*SE3(0,0,25)

q_pedido = [[-59.58, -48.95, -105.82, 117.24, 23.46, 41.3],
            [-34.98, -102.65, 5.53, 119.26, -31.37, -2.81],
            [-39.28, -79.1, -64.77, 128.14, 50.18, 53.7],
            [-44.82, -109.24, -12.65, 125.68, 42.62, -51.94],
            [-71.1, -119.79, 56.6, 21.62, 132.09, -61.78],
            [-69.69, -110.21, 32.43, 21.18, 6.5, 97.38],
            ]
q_cobot = [[-59.15, -49.65, -106.25, 116.89, 23.46, 41.48],
           [-34.62, -103.97, 4.57, 119.61, -31.28, -2.63],
           [-38.84, -79.98, -65.39, 127.79, 50.27, 54.14],
           [-43.94, -110.12, -13.35, 125.24, 43.06, -52.11],
           [-70.75, -121.11, 55.19, 21.0, 131.3, -61.43],
           [-69.34, -111.7, 30.84, 20.65, 7.11, 97.03]
           ]

print(f'Pose según toolbox\n{cobot_tb.fkine(np.radians(q_cobot[4]))}')
# sendWobj(robt.pose, pinza_aux, [1, -1, -1])
# cob.MoveJ(robt.offset(15, -25, 10), 30, pinza_aux, SE3()) #workobjects_v3 - wobj2 OK

# cob.MoveJAngles(np.array(wobj2_q[2]), 30, 'deg')


# cob.MoveJAngles(np.array(wobj3_q[2]), 30, 'deg')

# Backup
# save_terna_quat("Workobjects_cobotv3.txt", "wobj4", wobj_enseñado)
# save_qvals("Workobjects_qv3.txt", "wobj4", q_enseñados)
# workobjects = load_terna_quat("Workobjects_cobotv3.txt")
# wobj_recuperado = workobjects["wobj4"] # Se cargan todos y se pasa como dict
# print(f'Wobj recuperado\n{wobj_recuperado}')
