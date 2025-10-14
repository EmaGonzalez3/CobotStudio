from operator import index
from CobotStudio_rev4 import (
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
import datetime
import os

cobot_tb = myCobot320(rotar_base=True, metros=False)
pinza0 = SE3(-1.71381642, 106.90735789, 28.32702833) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)
pinza = SE3(0, 123, 27.5) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)
# cob = MyCobotController()

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
    rviz.MoveJ(robt0, 30, pinza, SE3())
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

def save_terna_se3(filename, name, se3_obj):
    # t = se3_obj.t.flatten().tolist()
    # R = se3_obj.R.tolist()
    # with open(filename, "a") as f:
    #     f.write("from spatialmath import SE3\n\n")
    #     f.write(f"# Guardado {datetime.datetime.now().isoformat()}\n")
    #     f.write(
    #         f"{name} = SE3({R}, {t})\n\n"
    #     )
    """
    Guarda una terna SE3 como variable Python directamente en formato SE3,
    asegurando alta precisión en los números.
    """
    # Formatear cada elemento de la matriz y el vector con alta precisión
    R_str = "[\n" + ",\n".join(["        [" + ", ".join([f"{val:.18e}" for val in row]) + "]" for row in se3_obj.R]) + "\n    ]"
    t_str = "[" + ", ".join([f"{val:.18e}" for val in se3_obj.t.flatten()]) + "]"

    with open(filename, "a") as f:
        # Asegurar que la importación solo se escriba una vez si el archivo es nuevo
        f.seek(0, 2) # Ir al final del archivo
        if f.tell() == 0: # Si el archivo está vacío
            f.write("from spatialmath import SE3\n")
            f.write("import numpy as np\n\n")

        f.write(f"# Guardado {datetime.datetime.now().isoformat()}\n")
        # Escribir la matriz de rotación y el vector de traslación como arrays de numpy
        # para facilitar la lectura y mantener el formato.
        f.write(f"{name}_R = np.array({R_str})\n")
        f.write(f"{name}_t = np.array({t_str})\n")
        f.write(f"{name} = SE3.Rt({name}_R, {name}_t)\n\n")

def save_qvals_py(filename, name, q):
    """
    Guarda una lista de variables articulares como variable Python en un archivo .py
    """
    with open(filename, "a") as f:
        f.write(f"{name} = [\n")
        for row in q:
            f.write(f"    {list(map(float, row))},\n")
        f.write("]\n\n")
# cheqWobj(ortonormarlizar_wobj(wobj_calc), pinza)
def verQs(q_list, pinza):
    rviz = SimManager()
    for idx, q in enumerate(q_list, start=1):
        if idx <= 3:
            rviz.MostrarTerna(cobot_tb.fkine(q) * pinza, f'x{idx}')
        else:
            rviz.MostrarTerna(cobot_tb.fkine(q) * pinza, f'y{idx-3}')
        time.sleep(1)
        rviz.VerQ(q, pinza)
        time.sleep(1)
    rviz.shutdown()
# Poses grabadas con el cobot real
q1 = [72.77, 36.65, 75.76, -14.32, -5.09, -91.75]
q2 = [52.82, 31.37, 77.08, -7.11, -11.25, -94.04]
q3 = [49.04, 60.64, 27.07, 23.55, 5.36, -129.9]
q_grabadas = [q1, q2, q3]
save_qvals_py("q_prueba2.txt", "q_0", q_grabadas)
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
# wobj_enseñado, q_enseñados = enseñarCobot(pinza)
# save_terna_se3("Workobjects_v4.py", "wobj0", wobj_enseñado)
# save_qvals_py("Workobjects_qv4.py", "q_0", q_enseñados)
# save_terna_quat("Workobjects_cobotv3.txt", "wobj4", wobj_enseñado)
# save_qvals("Workobjects_qv3.txt", "wobj4", q_enseñados)
# workobjects = load_terna_quat("Workobjects_cobotv3.txt")
# wobj_recuperado = workobjects["wobj4"] # Se cargan todos y se pasa como dict
# print(f'Wobj recuperado\n{wobj_recuperado}')
# robt = RobTarget(wobj_recuperado, [1, -1, -1])

# sendWobj(robt.pose, pinza_aux, [1, -1, -1])
# cob.MoveJ(robt.offset(15, -25, 10), 30, pinza_aux, SE3()) #workobjects_v3 - wobj2 OK
wobj2_q = [
    [-22.5, -57.91, -81.38, 89.64, -0.08, 47.37],
    [-43.94, -50.71, -106.87, 110.03, -1.75, 39.63],
    [-68.73, -82.17, -19.51, 49.83, 4.3, 52.47],
]

wobj3_q = [
    [-24.08, -81.91, -37.35, 92.9, 8.52, 19.95],
    [-43.24, -66.44, -81.91, 111.35, -10.28, 26.89],
    [-66.0, -102.21, 9.58, 67.32, 11.68, 29.0],
]

q_0 = [
    [-40.86, -118.03, 26.71, 65.3, -27.86, 20.03],
    [-27.24, -52.82, -89.56, 79.1, -7.29, 50.88],
    [-1.84, -68.02, -46.14, -7.82, -35.77, 85.69],
    [-52.82, -39.81, -71.63, 6.59, -3.25, 101.6],
    [-65.12, -100.89, 35.77, -4.13, 17.31, 86.74],
    [-54.66, -75.41, -37.44, 33.39, -17.75, 104.76],
]

q_1 = [
    [-40.42, -69.78, -81.29, 138.16, -0.79, 6.24],
    [-47.63, -69.08, -54.14, 90.61, 82.35, 7.64],
    [-33.83, -126.56, 27.42, 116.27, 63.36, 25.83],
    [-55.63, -106.87, 4.39, 137.81, 10.28, -11.6],
    [-79.89, -101.95, 12.3, 59.23, 48.33, 49.3],
    [-72.77, -110.39, 14.5, 58.62, 17.31, 83.93],
]

q_2 = [
    [-58.27, -10.28, -113.73, 30.93, 18.36, 102.21],
    [-61.17, -17.92, -109.51, 38.4, 47.28, 108.01],
    [-35.94, -122.6, 61.43, 23.55, 44.03, 55.1],
    [-66.26, -50.36, -86.57, 88.33, 18.63, 54.93],
    [-72.42, -119.79, 34.54, 65.47, 23.37, 55.28],
    [-61.69, -84.55, -29.79, 44.91, -19.07, 105.99],
]

q_3 = [
    [-59.58, -48.95, -105.82, 117.24, 23.46, 41.3],
    [-34.98, -102.65, 5.53, 119.26, -31.37, -2.81],
    [-39.28, -79.1, -64.77, 128.14, 50.18, 53.7],
    [-44.82, -109.24, -12.65, 125.68, 42.62, -51.94],
    [-71.1, -119.79, 56.6, 21.62, 132.09, -61.78],
    [-69.69, -110.21, 32.43, 21.18, 6.5, 97.38]
]

q_4 = [
    [-58.0, -38.67, -87.71, 49.48, 14.15, 61.87],
    [-18.36, -52.82, -91.75, 31.28, -26.19, 63.8],
    [-2.63, -105.55, -15.29, 74.26, -48.69, 58.18],
    [-66.0, -76.72, -15.29, 21.0, 15.38, 47.72],
    [-87.45, -115.75, 31.11, 52.47, 55.45, 68.64],
    [-80.59, -112.14, 29.0, 29.88, 35.59, 108.45]
]

q_5 = [
    [-48.42, -34.27, -94.04, 43.24, 4.48, 68.29],
    [-24.16, -124.98, 36.91, 81.82, -49.83, 24.6],
    [-8.78, -99.66, -21.62, 109.95, -33.92, 42.89],
    [-53.96, -133.24, 54.31, 79.98, -21.79, 25.57],
    [-58.09, -85.95, -32.6, 74.0, -21.0, 61.17],
    [-84.46, -91.23, -25.75, 69.16, 44.64, 107.22],
]# cob.MoveJAngles(np.array(wobj3_q[2]), 30, 'deg')
script_dir = os.path.dirname(os.path.abspath(__file__))
output_file = os.path.join(script_dir, 'work_prueba.py')
q_11 = [
    [-54.4, -11.86, -92.63, -12.65, 16.69, 117.59],
    [-11.68, -21.7, -113.37, -11.77, -26.01, 99.49],
    [-21.62, -129.63, 86.3, -10.63, 12.12, 58.0],
    [-76.11, -77.25, -4.3, -6.32, 20.03, 58.0],
    [-82.17, -125.77, 61.61, 29.44, 28.21, 57.04],
    [-80.77, -79.89, -20.56, 44.47, 20.3, 89.03],
]

q_12 = [
    [-54.84, -55.1, -101.51, 122.51, 13.0, 20.12],
    [-35.85, -57.83, -85.69, 103.09, 19.86, 18.45],
    [-25.66, -61.78, -73.47, 100.01, 21.79, 42.01],
    [-67.58, -110.74, 21.09, 76.11, 12.12, 4.3],
    [-84.11, -117.5, 35.85, 64.51, 39.37, 39.99],
    [-88.33, -98.61, 11.77, 32.34, 37.88, 95.18],
]

q_13 = [
    [-47.28, -134.2, 59.23, 70.13, -20.47, -5.53],
    [-24.25, -105.55, -11.51, 93.25, -57.3, 19.24],
    [-14.67, -70.04, -61.25, 94.39, 3.16, 28.65],
    [-81.21, -62.13, -84.11, 126.91, 49.04, 28.3],
    [-83.93, -83.84, -38.49, 107.66, 47.02, 39.11],
    [-80.59, -81.65, -22.41, 48.6, 23.99, 94.65],
]

q_14 = [
    [-47.37, -57.12, -103.09, 120.32, 3.69, 21.53],
    [-23.64, -118.65, 17.49, 77.16, -52.47, 15.99],
    [-17.13, -60.29, -66.26, 75.14, 9.22, 47.37],
    [-58.53, -122.69, 38.14, 63.98, -22.76, 15.55],
    [-73.91, -93.42, -19.33, 111.44, 31.9, 15.99],
    [-85.95, -89.56, -14.58, 61.69, 36.29, 79.89],
]

q_15 = [
    [-53.34, -52.2, -100.81, 108.1, 7.82, 31.11],
    [-39.46, -46.23, -114.52, 122.43, 26.36, 50.88],
    [-12.04, -83.32, -38.67, 98.34, -8.87, 24.08],
    [-58.35, -129.19, 50.88, 61.43, -22.58, 24.25],
    [-79.8, -88.06, -22.93, 98.43, 46.66, 24.16],
    [-78.39, -83.67, -32.25, 72.68, 20.12, 85.95],
]

q_16 = [
    [-52.29, -17.05, -124.8, 59.15, 2.98, 81.21],
    [-13.09, -20.91, -127.79, 29.79, -23.37, 87.62],
    [-30.23, -17.75, -96.15, 11.42, 32.25, 131.66],
    [-59.85, -13.44, -108.01, 14.76, -3.51, 111.18],
    [-79.62, -46.58, -62.75, 25.57, 24.34, 108.28],
    [-70.66, -84.19, -22.67, 40.86, -4.57, 107.92],
]

q_17 = [
    [-62.92, 4.92, -115.13, -0.26, 27.77, 134.2],
    [-32.34, -14.32, -78.04, -84.81, -4.92, -169.1],
    [-25.4, -76.99, -39.63, 89.12, 76.9, -8.7],
    [-90.96, -55.1, -79.89, 89.12, 54.14, 50.62],
    [-63.19, -94.74, -19.24, 92.63, -17.66, 40.51],
    [-78.57, -87.53, -19.33, 61.17, 19.16, 89.03],
]
# wobj_enseñado = teach_wobj(q_1, pinza0)
# save_terna_se3(output_file, 'wobj1', wobj_enseñado)


from Workobjects_v6 import wobj11, wobj12, wobj13, wobj14, wobj15, wobj16, wobj17
# print(wobj1)
robt = RobTarget(wobj15, [1, -1, -1])
pinza_aux = pinza*SE3(0,0,25)
verQs(np.deg2rad(q_15), pinza_aux)
cheqWobj(robt.pose, pinza_aux, [1, -1, -1])

def pose_to_matrix(pose):
    x, y, z, rx, ry, rz = pose

    # Pasamos a radianes
    rx, ry, rz = np.deg2rad([rx, ry, rz])

    # Crear objeto SE3 con rotación en RPY (Z-Y-X por defecto) y traslación
    T = SE3(x, y, z) * SE3.RPY([rx, ry, rz], order='zyx')
    T_matrix = T.A

    return T_matrix

# rviz = SimManager()
# print(SE3(pose_to_matrix([60.7, -300.6, 214.0, -94.66, -16.71, 133.3])))

q_5_pose1 = cobot_tb.fkine(np.deg2rad([[-48.42, -34.27, -94.04, 43.24, 4.48, 68.29]]))
q_5_pose2 = cobot_tb.fkine(np.deg2rad([[-24.16, -124.98, 36.91, 81.82, -49.83, 24.6]]))
q_5_pose3 = cobot_tb.fkine(np.deg2rad([[-8.78, -99.66, -21.62, 109.95, -33.92, 42.89]]))
robt_q5_1 = RobTarget(SE3(pose_to_matrix([60.7, -300.6, 214.0, -94.66, -16.71, 133.3])), [1, -1, 1])
robt_q5_2 = RobTarget(SE3(pose_to_matrix([120.7, -197.7, 200.4, -84.91, 20.5, 107.94])), [1, 1, -1])
robt_q5_3 = RobTarget(SE3(pose_to_matrix([194.0, -174.8, 189.2, -82.46, 33.21, 141.95])), [1, -1, -1])
# print(cobot_tb.calc_conf(np.array(np.deg2rad(q_5[3]))))
# print(robt_q5_3.find_valid_configs(SE3(), SE3()))
# cheqWobj(q_5_pose1, pinza_aux, [1, -1, -1])
# print(q_5_pose2)
# print(f'ikine: {cobot_tb.ikine(SE3(pose_to_matrix([60.7, -300.6, 214.0, -94.66, -16.71, 133.3])), [1, -1, -1])}')
# rviz.VerQ(np.zeros(6), SE3())
# rviz.MostrarTerna(q_5_pose1, 'q5_2')
# rviz.MoveJ(robt_q5_1, 30, pinza_aux, SE3(), 'wobj1', 'q5_1 fkine')
# rviz.VerPose(SE3(), robt_q5_3, SE3(), 'wobj1', 'q5_3 fkine')
# robt_traslacion = RobTarget(SE3(20, -280, 73)* SE3.Rx(np.pi), [1, -1, -1])
# rviz.VerPose(SE3(), robt_traslacion, pinza_aux, 'wobj1', 'traslacion')
# time.sleep(2)
# rviz.VerQ(np.deg2rad(q_5[2]), pinza_aux)
# time.sleep(2)
# rviz.shutdown()