from CobotStudio_rev4 import (
    RobTarget,
    MyCobotController,
    ROS_OK,
    joystick_adjust,
    teach_wobj
)
from spatialmath import SE3
import numpy as np
from DHRobotGT import myCobot320
import time
import datetime

if ROS_OK:
    from CobotStudio_rev4 import SimManager, checkQ, checkPose

cobot_tb = myCobot320(rotar_base=True, metros=False)
# pinza = SE3(-1.71, 106.91, 28.33) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)
# pinza_palp = SE3(0, 148, 27.5) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)
pinza = SE3(0, 123, 27.5) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)
pinza_aux = pinza*SE3(0,0,25)

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
cob = MyCobotController()
## Nombre de los datos a grabar
iteracion_n = 20
wobj_name = f"wobj{iteracion_n}"
q_name = f"q_{iteracion_n}"
## Enseñanza del wobj y recolección de datos

# wobj_enseñado, q_enseñados, datos_robot = enseñarCobot(pinza)
# save_qvals_py("Workobjects_qv7.py", q_name, q_enseñados)
# save_qvals_py("Workobjects_qcoordsv7.py", q_name, datos_robot)
# save_terna_se3("Workobjects_v7.py", wobj_name, wobj_enseñado)

## Importar el wobj y q grabados
from importlib import import_module
mod = import_module("Workobjects_v7")
workobjects = {}
workobjects[wobj_name] = getattr(mod, wobj_name)
print(workobjects[wobj_name])

mod_q = import_module("Workobjects_qv7")
q_grabados = {}
q_grabados[q_name] = getattr(mod_q, q_name)
print(q_grabados[q_name])

## Chequear el wobj en el robot físico
# sendWobj(workobjects[wobj_name], pinza_aux, [1, -1, -1])

def registrar_datos(q_list):
    for i in range (6):
        cob.MoveJAngles(np.zeros(6), 40, 'deg')
        cob.MoveJAngles(np.array(q_list[i]), 30, 'deg')
        time.sleep(5)
        print(f'Ángulos pedidos\n{q_list[i]}')
        print(f'Cobot coords\n{cob.mc.get_angles()}')
        print(f'Pose según toolbox\n{cobot_tb.fkine(np.radians(q_list[i]))}')
        print(f'Cobot pose\n{cob.mc.get_coords()}')

def calibrar_servos():
    """Calibrar los servos del cobot real en su posición home."""
    cob.MoveJAngles(np.zeros(6), 20, 'deg') # Enviamos al cobot al home
    time.sleep(2)
    print(f'Lectura de los encoders:\n{cob.mc.get_encoders()}') # Vemos qué dicen los encoders
    for i in range (6):
        cob.mc.set_servo_calibration(i+1)
        time.sleep(2)
    print(f'Encoders calibrados:\n{cob.mc.get_encoders()}')

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

"""
cob.MoveJAngles(np.array([135, 25, 30, 20, -30, 0]), 30, 'deg')

wobj15 
Offset en [x, y, z]: [-9.199  1.534 28.009] mm
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