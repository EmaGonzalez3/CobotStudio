from CobotStudio_rev2 import (
    RobTarget,
    SimManager,
    MyCobotController,
    checkQ,
    checkPose,
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

cobot_tb = myCobot320(rotar_base=True, metros=False)
# pinza = SE3(-1.71381642, 106.90735789, 28.32702833) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)
pinza = SE3(0, 123, 27.5) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)
cob = MyCobotController()

def enseñarCobot(pinza):
    """Enseñar el wobj en el cobot. Devuelve el wobj como SE3."""
    q_vals = cob.grabar_poses(6, ajuste = True)
    wobj = teach_wobj(q_vals, pinza)
    print(f'Wobj calculado\n{wobj}')
    return wobj, q_vals


"""----- Wobjs printeados -----"""
# wobj_viejo = np.array([
#     [0.9801, 0.1709, -0.1007, -29.4],
#     [0.1704, -0.9853,   -0.01321, -299.1],
#     [-0.1015, -0.00421, -0.9948, 61.65],
#     [0, 0, 0, 1]
#     ])

wobj_calc = np.array([
    [0.9964, -0.08171, -0.02141, -89.29],
    [-0.08258, -0.9956, -0.04369, -293],
    [-0.01774,  0.0453,  -0.9988,   89.26],
    [0, 0, 0, 1]
    ])

work_prueba = np.array([
    [ 9.96426161e-01, -8.17107051e-02, -2.14071382e-02, -8.92900000e+01],
    [-8.25837237e-02, -9.95625982e-01, -4.36901806e-02, -2.93000000e+02],
    [-1.77435475e-02,  4.53019201e-02, -9.98815750e-01,  8.92600000e+01],
    [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]
 ])

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
    rviz.MoveJ(SE3(), robt0, pinza, 'w', 'rt')
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


# cheqWobj(ortonormarlizar_wobj(wobj_calc), pinza)

# Poses grabadas con el cobot real
q1 = [72.77, 36.65, 75.76, -14.32, -5.09, -91.75]
q2 = [52.82, 31.37, 77.08, -7.11, -11.25, -94.04]
q3 = [49.04, 60.64, 27.07, 23.55, 5.36, -129.9]
q_grabadas = [q1, q2, q3]
save_qvals("q_prueba2.txt", "q_0", q_grabadas)
# wobj_enseñado = teach_wobj(q_grabadas, pinza)
# print(wobj_enseñado)
# save_terna_quat("Workobjects_quat.txt", "wobj1", wobj_enseñado)
# workobjects = load_terna_quat("Workobjects_quat.txt")
# wobj_recuperado = load_terna_quat("Workobjects_quat.txt", "wobj1") # Se carga solo uno
# wobj_recuperado = workobjects["wobj1"] # Se cargan todos y se pasa como dict
# print(wobj_recuperado)

# print(wobj_recuperado)
# cheqWobj(wobj_recuperado, pinza)

"""Cobot real"""
wobj_enseñado, q_enseñados = enseñarCobot(pinza)

# # Guardar y cargar como txt
# save_terna_quat("Workobjects_cobotv3.txt", "wobj4", wobj_enseñado)
# save_qvals("Workobjects_qv3.txt", "wobj4", q_enseñados)
# workobjects = load_terna_quat("Workobjects_cobotv3.txt")
# wobj_recuperado = workobjects["wobj4"] # Se cargan todos y se pasa como dict
print(f'Wobj recuperado\n{wobj_recuperado}')
robt = RobTarget(wobj_recuperado, [1, -1, -1])
cheqWobj(robt.pose, pinza, [1, -1, -1])
pinza_aux = pinza*SE3(0,0,25)
# sendWobj(robt.pose, pinza_aux, [1, -1, -1])
# cob.MoveJ(robt.offset(15, -25, 10), 30, pinza_aux, SE3()) #workobjects_v3 - wobj2 OK
wobj2_q = [
    [-22.5, -57.91, -81.38, 89.64, -0.08, 47.37],
    [-43.94, -50.71, -106.87, 110.03, -1.75, 39.63],
    [-68.73, -82.17, -19.51, 49.83, 4.3, 52.47],
]
# cob.MoveJAngles(np.array(wobj2_q[2]), 30, 'deg')

wobj3_q = [
    [-24.08, -81.91, -37.35, 92.9, 8.52, 19.95],
    [-43.24, -66.44, -81.91, 111.35, -10.28, 26.89],
    [-66.0, -102.21, 9.58, 67.32, 11.68, 29.0],
]

# cob.MoveJAngles(np.array(wobj3_q[2]), 30, 'deg')
