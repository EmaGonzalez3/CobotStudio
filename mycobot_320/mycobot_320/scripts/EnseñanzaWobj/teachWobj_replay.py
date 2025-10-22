from scripts.CobotStudio_rev4 import RobTarget, BaseRobotController, SimManager, MyCobotController, live_pose, teach_wobj
from spatialmath import SE3
import numpy as np
from scripts.DHRobotGT import myCobot320
import time
from importlib import import_module

# Importamos la función con la celda robótica
from scripts.EnseñanzaWobj.teachWobj_scene import setup_scene

cobot_tb = myCobot320(rotar_base=True, metros=False)


def get_robot(mode="sim", **kwargs) -> BaseRobotController:
    if mode == "sim":
        return SimManager()
    elif mode == "real":
        return MyCobotController(**kwargs)
    else:
        raise ValueError("Modo desconocido: usa 'sim' o 'real'")

def verQs(robot, q_list, pinza):
    "Reproduce en RViz los puntos de enseñanza grabados en el cobot."
    q_rads = np.deg2rad(q_list)
    for idx, q in enumerate(q_rads, start=1):
        if idx <= 3:
            robot.MostrarTerna(cobot_tb.fkine(q) * pinza, f'x{idx}')
        else:
            robot.MostrarTerna(cobot_tb.fkine(q) * pinza, f'y{idx-3}')
        robot.VerQ(q, pinza)
        time.sleep(2)

def cheqWobj(robot, wobj, pinza, conf = [1, 1, 1]):
    ''' Envía el wobj a RViz desde la pose home.'''
    robt0 = RobTarget(SE3(wobj), conf)
    print(robt0.find_valid_configs(pinza, SE3()))
    robot.VerPose(SE3(), robt0, tool = pinza, robt_name = 'WobjCalc')
    time.sleep(2)

def main(robot: BaseRobotController, devolver = True):
    iteracion_n = 26
    wobj_name = f"wobj{iteracion_n}"
    q_name = f"q_{iteracion_n}"

    mod = import_module("scripts.EnseñanzaWobj.Datos.Workobjects_v7")
    workobjects = {}
    workobjects[wobj_name] = getattr(mod, wobj_name)
    print(workobjects[wobj_name])

    mod_q = import_module("scripts.EnseñanzaWobj.Datos.Workobjects_qv7")
    q_grabados = {}
    q_grabados[q_name] = getattr(mod_q, q_name)
    print(q_grabados[q_name])
    
    pinza_palp = SE3(0, 148, 27.5) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)
    pinza = SE3(0, 123, 27.5) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)
    pinza_aux = pinza*SE3(0,0,25)

    robot.GripperState(0, 30) # Cerrar pinza
    time.sleep(2)
    verQs(robot, q_grabados[q_name], pinza_aux)
    q_vals = q_grabados[q_name]

    wobj_from_q = teach_wobj(q_vals, pinza, 25)

    cheqWobj(workobjects[wobj_name], pinza_palp, [1, -1, -1])

    time.sleep(2)
    cheqWobj(robot, wobj_from_q, pinza_palp, [1, -1, -1])


if __name__ == "__main__":
    robot_controller = get_robot("sim")
    # robot_controller = get_robot("real") # Cobot real

    # Si estamos en modo simulación, configuramos la escena.
    # Pasamos la instancia 'robot_controller' a la función importada.
    if isinstance(robot_controller, SimManager):
        setup_scene(robot_controller)

    try:
        main(robot_controller)
    except Exception as e:
        print(f"Ocurrió un error durante la ejecución: {e}")
    finally:
        # Apagamos el controlador de forma segura.
        if hasattr(robot_controller, 'shutdown'):
            robot_controller.shutdown()
        print("--- Secuencia finalizada ---")