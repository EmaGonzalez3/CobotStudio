from CobotStudio_rev3 import RobTarget, BaseRobotController, SimManager, MyCobotController, checkQ, checkPose, pose_to_matrix, joystick_adjust, teach_wobj
from spatialmath import SE3
import numpy as np
from DHRobotGT import myCobot320
import time
from pymycobot import MyCobotSocket
from object_manager_rev1 import CUBE, CYLINDER, MESH

def get_robot(mode="sim", **kwargs) -> BaseRobotController:
    if mode == "sim":
        return SimManager()
    elif mode == "real":
        return MyCobotController(**kwargs)
    else:
        raise ValueError("Modo desconocido: usa 'sim' o 'real'")

def main_0(robot: BaseRobotController):
    # Definir objetos, robtargets, etc.
    pinza = SE3(-1.71381642, 106.90735789, 28.32702833) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)
    robt = RobTarget(SE3(150, 150, 0)* SE3.Ry(-np.pi)*SE3.Rz(np.pi/2), [1, -1, 1])
    
    # Movimiento idéntico en sim o real
    robot.MoveJ(robt, 30, pinza, SE3())
    robot.GripperState(80)  # abrir
    
    # Otro movimiento
    # robot.MoveC(robt, speed=20, tool=SE3(), wobj=SE3())

def main(robot: BaseRobotController, devolver = True):
    cobot_tb = myCobot320(rotar_base=True, metros=False)
    altura_caja = 0.07
    # TCPs
    pinza = SE3(-1.71381642, 106.90735789, 28.32702833) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)
    pinza_caja = pinza * SE3(0, 0, 10)
    wobj = SE3() # wobj nulo por simplicidad
    home = RobTarget(cobot_tb.fkine(np.repeat(0.01, 6)) * pinza, [-1, 1, 1])
    offset_pick = 50 # Qué tan lejos de la caja frenamos
    offset_place = 30 # Qué tan lejos del 'pallet' frenamos
    for i in range (3):
        y_val = -200 + -60*i # Posición de las cajas en y
        # La primera caja está sobre la base, las otras 2 no. Corregimos.
        if i == 0:
            h_pick = 15
        else:
            h_pick = 0

        pick = RobTarget(SE3(100, y_val, h_pick)* SE3.Ry(-np.pi)*SE3.Rz(np.pi), [1, -1, -1])
        robot.GripperState(100, 30)
        # Acercamiento
        robot.MoveJ(pick.offset(0, 0, offset_pick), 50, pinza, wobj)
        robot.MoveJ(pick, 30, pinza, wobj)
        time.sleep(1)
        robot.GripperState(0, 30) # Cerrar pinza
        time.sleep(2) # Hay que darle tiempo a que cierre
        robot.MoveJ(pick.offset(0, 0, 30), 30, pinza_caja, wobj)
        
        # Calculamos la altura según la cantidad de cajas
        z_val = altura_caja*1000*i

        # Ajuste manual para la tercera caja
        if i == 2:
            z_val = 135
        
        place = RobTarget(SE3(150, 150, z_val)* SE3.Ry(-np.pi)*SE3.Rz(np.pi/2), [1, -1, 1])
        robot.MoveJ(place.offset(0, 0, offset_place), 50, pinza_caja, wobj)
        robot.MoveJ(place, 30, pinza_caja, wobj)
        time.sleep(1)
        print('Soltamos!')
        robot.GripperState(100, 30)
        time.sleep(2) # Le damos tiempo a que termine de abrir
        
        # Salimos para no tirar la pila de cajas
        robot.MoveJ(place.offset(0, 0, 50), 30, pinza, wobj)

    time.sleep(1)
    
    # Vamos cerca de donde estaban las cajas para esquivar la pila
    robot.MoveJ(pick.offset(0, 120, 180), 30, pinza, wobj)

    # Volvemos a Home
    # cobot.MoveJ(home, 30, pinza, wobj)
    if devolver == False:
        return
    for i in reversed(range(3)):
        # Armamos de nuevo los targets para este índice
        y_val = -200 + -60*i
        h_pick = 15 if i == 0 else 0

        pick = RobTarget(SE3(100, y_val, h_pick)* SE3.Ry(-np.pi)*SE3.Rz(np.pi), [1, -1, -1])

        z_val = altura_caja*1000*i
        if i == 2:
            z_val = 135

        place = RobTarget(SE3(150, 150, z_val)* SE3.Ry(-np.pi)*SE3.Rz(np.pi/2), [1, -1, 1])

        # --- Proceso inverso: ahora el "pick" es la pila ---
        robot.MoveJ(place.offset(0, 0, offset_pick), 30, pinza, wobj)
        robot.MoveJ(place, 30, pinza, wobj)
        robot.GripperState(1, 30) # Cerrar pinza
        time.sleep(2) # Hay que darle tiempo a que cierre
        robot.MoveJ(place.offset(0, 0, 30), 30, pinza_caja, wobj)

        # --- Y el "place" es la posición original ---
        robot.MoveJ(pick.offset(0, 0, offset_place), 30, pinza_caja, wobj)
        robot.MoveJ(pick, 30, pinza, wobj)
        robot.GripperState(0, 30) # Abrir pinza
        time.sleep(2) # Dejamos que termine de abrir
        robot.MoveJ(pick.offset(0, 0, 50), 30, pinza, wobj)
    
    # Vamos cerca de donde estaban las cajas para esquivar la pila
    # robot.MoveJ(wobj, pick.offset(0, 120, 180), pinza)

    # Volvemos a Home
    robot.MoveJ(home, 30, pinza, wobj)


if __name__ == "__main__":
    sim = get_robot("sim")
    # real = get_robot("real", host="10.42.0.1", port=9000)
    main(sim)   # o main(sim)
