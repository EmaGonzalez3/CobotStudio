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
cob = MyCobotController()
# pinza = SE3(-1.71381642, 106.90735789, 28.32702833) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)
# pinza_palp = SE3(0, 148, 27.5) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)
pinza = SE3(0, 123, 27.5) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)
# pinza = SE3(0, 138, 7) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)
# pinza = SE3(0, 123, 12) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)
# pinza = SE3(0, 138, -10) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)

def enseñarCobot(pinza):
    """Enseñar el wobj en el cobot. Devuelve el wobj como SE3."""
    q_vals, datos_cobot = cob.grabar_poses(6, ajuste = True)
    wobj = teach_wobj(q_vals, pinza)
    print(f'Wobj calculado\n{wobj}')
    return wobj, q_vals, datos_cobot

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


"""Cobot real"""
# wobj_name = wobj2
# q_name = q_2
# wobj_enseñado, q_enseñados, datos_robot = enseñarCobot(pinza)
# save_qvals_py("Workobjects_qv6.py", "q_17", q_enseñados)
# save_qvals_py("Workobjects_qcoordsv6.py", "q_17", datos_robot)
# save_terna_se3("Workobjects_v6.py", "wobj17", wobj_enseñado)

# wobj6 = teach_wobj(q_enseñados, pinza, 0)
from Workobjects_v6 import wobj15

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
]
cob.GripperState(0)
# print(wobj12)
robt = RobTarget(wobj15, [1, -1, -1])
# q5-q_2 
pinza_aux = pinza*SE3(0,0,25)

# print(f'Cobot coords\n{cob.mc.get_coords()}')
# print(f'Toolbox coords\n{robt.pose * pinza_aux.inv()}')
# sendWobj(robt.pose, pinza_aux, [1, -1, -1])
# cob.MoveJ(robt.offset(15, -25, 10), 30, pinza_aux, SE3()) #workobjects_v3 - wobj2 OK


# cob.MoveJAngles(np.array(wobj3_q[2]), 30, 'deg')

# Backup
# save_terna_quat("Workobjects_cobotv3.txt", "wobj4", wobj_enseñado)
# save_qvals("Workobjects_qv3.txt", "wobj4", q_enseñados)
# workobjects = load_terna_quat("Workobjects_cobotv3.txt")
# wobj_recuperado = workobjects["wobj4"] # Se cargan todos y se pasa como dict
# print(f'Wobj recuperado\n{wobj_recuperado}')

def registrar_datos():
    for i in range (6):
        cob.MoveJAngles(np.zeros(6), 40, 'deg')
        cob.MoveJAngles(np.array(q_3[i]), 30, 'deg')
        time.sleep(5)
        print(f'Ángulos pedidos\n{q_3[i]}')
        print(f'Cobot coords\n{cob.mc.get_angles()}')
        print(f'Pose según toolbox\n{cobot_tb.fkine(np.radians(q_3[i]))}')
        print(f'Cobot pose\n{cob.mc.get_coords()}')

# registrar_datos()
# time.sleep(3)
# cob.MoveJAngles(np.zeros(6), 40, 'deg')
# sendWobj(robt.pose, pinza_aux, [1, -1, -1])

# print(cob.mc.get_encoders())
# for i in range (6):
#     cob.mc.set_servo_calibration(i+1)
#     time.sleep(1)
# print(cob.mc.get_encoders())

# cob.MoveJAngles(np.zeros(6))

def corregir_offset(robt):
    # --- Corrección offset ---
    # Asumimos que 'wobj_enseñado' es el que tiene el offset
    # Mueve el robot real a esa pose. Terminará en la posición incorrecta.
    cob.MoveJ(robt, 30, pinza_aux, SE3())

    # Guarda los ángulos a los que llegó
    q_incorrecto = np.deg2rad(cob.mc.get_angles())

    # La función 'mover_callback' para el joystick
    def mover_robot(robt):
        cob.MoveJ(robt, 20, pinza_aux, SE3())

    time.sleep(1)

    print("\n--- Ajuste Manual ---")
    print("Usa el joystick para mover la punta a la esquina REAL.")
    # robt_corregido = joystick_adjust(q_incorrecto, mover_callback=mover_robot)

    robt_corregido = joystick_adjust(np.deg2rad(cob.mc.get_angles()), 
                                    mover_callback=lambda r: cob.MoveJ(r, 20, SE3(), SE3()))
    
    q_ajustado = cob.mc.get_angles()
    # Pose de la brida donde el robot cree que está bien
    T_brida_incorrecta = cobot_tb.fkine(q_incorrecto)

    # Pose de la brida donde el robot está físicamente bien
    T_brida_correcta = cobot_tb.fkine(np.deg2rad(q_ajustado))

    # T_brida_correcta = T_calibracion * T_brida_incorrecta
    # => T_calibracion = T_brida_correcta * T_brida_incorrecta.inv()

    T_calibracion = T_brida_correcta * T_brida_incorrecta.inv()

    print(f"\nTransformación de Calibración Calculada:\n{T_calibracion}")
    print(f"Offset en [x, y, z]: {T_calibracion.t.round(3)} mm")


# cob.MoveJAngles(np.array([135, 25, 30, 20, -30, 0]), 30, 'deg')
corregir_offset(robt)

"""
wobj15: Offset en [x, y, z]: [-9.199  1.534 28.009] mm
Si lo hago llegar desde donde quedó ajustado el offset cambia: Offset en [x, y, z]: [ 0.79  -3.483 21.856] mm
Desde MoveJAngles: Offset en [x, y, z]: [-9.911  0.588 29.286] mm

wobj16
Llegando a lo calculado desde 0: Offset en [x, y, z]: [-15.023  -2.646  27.512] mm
Llegando desde la esquina: Offset en [x, y, z]: [-1.48  -6.651 24.885] mm
Desde MoveJAngles: Offset en [x, y, z]: [-8.325  2.395 23.981] mm

wobj17
Desde 0: Offset en [x, y, z]: [12.354  4.259 29.53 ] mm
Desde la esquina: Offset en [x, y, z]: [12.48  -2.029 24.586] mm
Desde el MoveJAngles: Offset en [x, y, z]: [12.126 15.975 32.041] mm

"""