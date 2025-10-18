from CobotStudio_rev2 import RobTarget, SimManager, MyCobotController, checkQ, checkPose, pose_to_matrix, joystick_adjust, teach_wobj
from spatialmath import SE3
import numpy as np
from DHRobotGT import myCobot320
import time
from pymycobot import MyCobotSocket
from object_manager import CUBE, CYLINDER, MESH

cobot_tb = myCobot320(rotar_base=True, metros=False)

def prueba1():
    rviz = SimManager()
    altura_base = 0.025
    altura_caja = 0.05
    rviz.node_obj.add_object("robot_base", pose_init=(0.0, 0.0, -altura_base/2),
                    size=(0.8, 0.8, altura_base),
                    color=(1.0, 1.0, 1.0),
                    shape=CYLINDER, movable=False)
    rviz.node_obj.add_object("caja1", pose_init=(0.1, -0.2, altura_caja/2),
                    size=(0.04, 0.04, altura_caja),
                    color=(1.0, 0.0, 0.0),
                    shape=CUBE, movable=True)
    rviz.node_obj.add_object("caja2", pose_init=(0.1, -0.25, altura_caja/2),
                size=(0.04, 0.04, altura_caja),
                color=(0.0, 1.0, 0.0),
                shape=CUBE, movable=True)
    rviz.node_obj.add_object("caja3", pose_init=(0.1, -0.3, altura_caja/2),
            size=(0.04, 0.04, altura_caja),
            color=(0.0, 0.0, 1.0),
            shape=CUBE, movable=True)
    pinza = SE3(-1.71381642, 106.90735789, 28.32702833) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)
    wobj = SE3()
    home = RobTarget(cobot_tb.fkine(np.repeat(0.01, 6)) * pinza, [-1, 1, 1])
    pinza_caja = pinza * SE3(0, 0, 25)
    place = RobTarget(SE3(150, 150, 0)* SE3.Ry(-np.pi)*SE3.Rz(np.pi/2), [1, -1, 1])
    offset_pick = 50
    offset_place = 30
    for i in range (3):
        y_val = -200 + -50*i
        pick = RobTarget(SE3(100, y_val, 30)* SE3.Ry(-np.pi)*SE3.Rz(np.pi), [1, -1, -1])
        rviz.MoveJ(wobj, pick.offset(0, 0, offset_pick), pinza)
        rviz.MoveC(wobj, pick, pinza)
        rviz.mover_pinza(20, pinza)
        rviz.node_obj.attach(f'caja{i+1}')
        rviz.MoveC(wobj, pick.offset(0, 0, 30), pinza_caja)
        z_val = altura_caja*1000*i+20
        place = RobTarget(SE3(150, 150, z_val)* SE3.Ry(-np.pi)*SE3.Rz(np.pi/2), [1, -1, 1])
        rviz.MoveJ(wobj, place.offset(0, 0, offset_place), pinza_caja)
        rviz.MoveC(wobj, place, pinza_caja)
        rviz.mover_pinza(100, pinza)
        rviz.node_obj.detach(f'caja{i+1}')
        rviz.MoveC(wobj, place.offset(0, 0, 50), pinza)

    time.sleep(2)
    rviz.MoveJ(wobj, home, pinza)


    # rviz.VerPose(wobj, place.offset(0, 0, 30), pinza_caja)
    # rviz.testPose(pick_1, pinza, wobj)

    rviz.shutdown()
    
def al_cobot(devolver = False):
    cobot = MyCobotController()
    cobot.mc.set_gripper_mode(1)
    cobot.mc.set_fresh_mode(0)
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
        cobot.GripperState(0, 30)
        # Acercamiento
        cobot.MoveJ(pick.offset(0, 0, offset_pick), 50, pinza, wobj)
        cobot.MoveJ(pick, 30, pinza, wobj)
        time.sleep(1)
        cobot.GripperState(1, 30) # Cerrar pinza
        print(cobot.mc.read_next_error())
        time.sleep(2) # Hay que darle tiempo a que cierre
        cobot.MoveJ(pick.offset(0, 0, 30), 30, pinza_caja, wobj)
        
        # Calculamos la altura según la cantidad de cajas
        z_val = altura_caja*1000*i

        # Ajuste manual para la tercera caja
        if i == 2:
            z_val = 135
        
        place = RobTarget(SE3(150, 150, z_val)* SE3.Ry(-np.pi)*SE3.Rz(np.pi/2), [1, -1, 1])
        cobot.MoveJ(place.offset(0, 0, offset_place), 50, pinza_caja, wobj)
        cobot.MoveJ(place, 30, pinza_caja, wobj)
        time.sleep(1)
        print('Soltamos!')
        print(cobot.mc.read_next_error())
        cobot.GripperState(0, 30)
        time.sleep(2) # Le damos tiempo a que termine de abrir
        
        # Salimos para no tirar la pila de cajas
        cobot.MoveJ(place.offset(0, 0, 50), 30, pinza, wobj)

    time.sleep(1)
    
    # Vamos cerca de donde estaban las cajas para esquivar la pila
    cobot.MoveJ(pick.offset(0, 120, 180), 30, pinza, wobj)

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
        cobot.MoveJ(place.offset(0, 0, offset_pick), 30, pinza, wobj)
        cobot.MoveJ(place, 30, pinza, wobj)
        cobot.GripperState(1, 30) # Cerrar pinza
        time.sleep(2) # Hay que darle tiempo a que cierre
        cobot.MoveJ(place.offset(0, 0, 30), 30, pinza_caja, wobj)

        # --- Y el "place" es la posición original ---
        cobot.MoveJ(pick.offset(0, 0, offset_place), 30, pinza_caja, wobj)
        cobot.MoveJ(pick, 30, pinza, wobj)
        cobot.GripperState(0, 30) # Abrir pinza
        time.sleep(2) # Dejamos que termine de abrir
        cobot.MoveJ(pick.offset(0, 0, 50), 30, pinza, wobj)
    
    # Vamos cerca de donde estaban las cajas para esquivar la pila
    # cobot.MoveJ(wobj, pick.offset(0, 120, 180), pinza)

    # Volvemos a Home
    cobot.MoveJ(home, 30, pinza, wobj)

def prueba2():
    rviz = SimManager()
    altura_base = 0.025 # Base de aluminio del cobot
    altura_caja = 0.07 # Altura de los bloques de madera a apilar
    brida_base = 0.02

    # Agregamos los objetos según sus dimensiones
    rviz.node_obj.add_object("robot_base", pose_init=(0.0, 0.0, -altura_base/2 - brida_base),
                    size=(0.51, 0.51, altura_base),
                    color=(1.0, 1.0, 1.0, 0.9),
                    shape=CYLINDER, movable=False)
    rviz.node_obj.add_object("caja1", pose_init=(0.1, -0.2, altura_caja/2- brida_base),
                    size=(0.04, 0.04, altura_caja),
                    color=(1.0, 0.0, 0.0, 0.9),
                    shape=CUBE, movable=True)
    rviz.node_obj.add_object("caja2", pose_init=(0.1, -0.265, altura_caja/2 - 0.015- brida_base),
                size=(0.04, 0.04, altura_caja),
                color=(0.0, 1.0, 0.0, 0.9),
                shape=CUBE, movable=True)
    rviz.node_obj.add_object("caja3", pose_init=(0.1, -0.33, altura_caja/2 - 0.015- brida_base),
            size=(0.04, 0.04, altura_caja),
            color=(0.0, 0.0, 1.0, 0.9),
            shape=CUBE, movable=True)
    
    # TCPs
    pinza = SE3(-1.71381642, 106.90735789, 28.32702833) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)
    # pinza = SE3(-1, 123, 28.32702833) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)
    pinza_caja = pinza * SE3(0, 0, 10)
    wobj = SE3() # wobj nulo por simplicidad
    home = RobTarget(cobot_tb.fkine(np.repeat(0.01, 6)) * pinza, [-1, 1, 1])
    offset_pick = 50 # Qué tan lejos de la caja frenamos
    offset_place = 30 # Qué tan lejos del 'pallet' frenamos
    for i in range (3):
        y_val = -200 + -65*i # Posición de las cajas en y
        # La primera caja está sobre la base, las otras 2 no. Corregimos:
        if i == 0:
            h_pick = 15
        else:
            h_pick = 0

        pick = RobTarget(SE3(100, y_val, h_pick)* SE3.Ry(-np.pi)*SE3.Rz(np.pi), [1, -1, -1])

        # Acercamiento
        rviz.MoveJ(wobj, pick.offset(0, 0, offset_pick), pinza)
        rviz.MoveC(wobj, pick, pinza)
        rviz.mover_pinza(20, pinza)
        rviz.node_obj.attach(f'caja{i+1}')
        rviz.MoveC(wobj, pick.offset(0, 0, 30), pinza_caja)
        
        # Calculamos la altura según la cantidad de cajas
        z_val = altura_caja*1000*i

        # Ajuste manual para la tercera caja
        # if i == 2:
        #     z_val = 135
        
        place = RobTarget(SE3(150, 150, z_val)* SE3.Ry(-np.pi)*SE3.Rz(np.pi/2), [1, -1, 1])
        rviz.MoveJ(wobj, place.offset(0, 0, offset_place), pinza_caja)
        rviz.MoveC(wobj, place, pinza_caja)
        rviz.mover_pinza(100, pinza)
        rviz.node_obj.detach(f'caja{i+1}')
        rviz.MoveC(wobj, place.offset(0, 0, 50), pinza)
    
    # Vamos cerca de donde estaban las cajas para esquivar la pila
    rviz.MoveJ(wobj, pick.offset(0, 120, 180), pinza)

    # Volvemos a Home
    # rviz.MoveJ(wobj, home, pinza)

    for i in reversed(range(3)):
        # Armamos de nuevo los targets para este índice
        y_val = -200 + -65*i
        h_pick = 15 if i == 0 else 0

        pick = RobTarget(SE3(100, y_val, h_pick)* SE3.Ry(-np.pi)*SE3.Rz(np.pi), [1, -1, -1])

        z_val = altura_caja*1000*i
        # if i == 2:
        #     z_val = 135

        place = RobTarget(SE3(150, 150, z_val)* SE3.Ry(-np.pi)*SE3.Rz(np.pi/2), [1, -1, 1])

        # --- Proceso inverso: ahora el "pick" es la pila ---
        rviz.MoveJ(wobj, place.offset(0, 0, offset_pick), pinza)
        rviz.MoveC(wobj, place, pinza_caja)
        rviz.mover_pinza(20, pinza_caja)  # agarrar
        rviz.node_obj.attach(f'caja{i+1}')
        rviz.MoveC(wobj, place.offset(0, 0, 30), pinza_caja)

        # --- Y el "place" es la posición original ---
        rviz.MoveJ(wobj, pick.offset(0, 0, offset_place), pinza_caja)
        rviz.MoveC(wobj, pick, pinza)
        rviz.mover_pinza(100, pinza)  # soltar
        rviz.node_obj.detach(f'caja{i+1}')
        rviz.MoveC(wobj, pick.offset(0, 0, 50), pinza)
    
    # Vamos cerca de donde estaban las cajas para esquivar la pila
    # rviz.MoveJ(wobj, pick.offset(0, 120, 180), pinza)

    # Volvemos a Home
    rviz.MoveJ(wobj, home, pinza)
    # rviz.VerQ(np.zeros(6), pinza)
    # time.sleep(1)
    # rviz.mover_pinza(25, pinza)

    rviz.shutdown()

# prueba1()
# al_cobot(devolver = True)
prueba2()


# cobot = MyCobotController()
# place = RobTarget(SE3(150, 150, 140)* SE3.Ry(-np.pi)*SE3.Rz(np.pi/2), [1, -1, 1])
# pinza = SE3(-1.71381642, 106.90735789, 28.32702833) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)
# wobj = SE3()
# pick = RobTarget(SE3(100, -320, 0)* SE3.Ry(-np.pi)*SE3.Rz(np.pi), [1, -1, -1])
# cobot.MoveJ(pick.offset(0, 120, 180), 30, pinza, wobj)

# pick = RobTarget(SE3(100, -200, 15)* SE3.Ry(-np.pi)*SE3.Rz(np.pi), [1, -1, -1])
# place = RobTarget(SE3(150, 150, 140)* SE3.Ry(-np.pi)*SE3.Rz(np.pi/2), [1, -1, 1])
# cobot.MoveJ(place.offset(0, 0, 30), 30, pinza, wobj)
# cobot.MoveJ(pick.offset(0, 0, 180), 30, pinza, wobj)

# cobot.MoveJ(place.offset(0, 0, 30), 30, pinza, wobj)

