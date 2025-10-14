from CobotStudio_rev2 import RobTarget, SimManager, MyCobotController, checkQ, checkPose, pose_to_matrix, joystick_adjust, teach_wobj
from spatialmath import SE3
import numpy as np
from DHRobotGT import myCobot320
import time
from pymycobot import MyCobotSocket
from object_manager import CUBE, SPHERE, MESH

cobot_tb = myCobot320(rotar_base=True, metros=False)
rviz = SimManager()

def pickPlace_v0(rviz):
    # Caja roja
    rviz.node_obj.add_object("caja1", pose_init=(0.0, -0.22, 0.03),
                    size=(0.06, 0.05, 0.06),
                    color=(1.0, 0.0, 0.0),
                    shape=CUBE)
    # # Esfera azul
    # rviz.node_obj.add_object("bola", pose_init=(0.2, 0.1, 0.05),
    #                   size=(0.05, 0.05, 0.05),
    #                   color=(0.0, 0.0, 1.0),
    #                   shape=SPHERE)
    # Mesa fija
    rviz.node_obj.add_object("mesa", pose_init=(0.0, 0.0, -0.02),
                    size=(1.0, 1.0, 0.04),
                    color=(0.5, 0.3, 0.1),
                    movable=False)

    pick = RobTarget(SE3(0, -220, 25)* SE3.Ry(-np.pi), [-1, 1, 1])
    # place = RobTarget(SE3(200, 0, 25)* SE3.Ry(-np.pi), [-1, 1, 1]) # No alcanzable
    place = RobTarget(SE3(200, -50, 25)* SE3.Ry(-np.pi), [-1, 1, 1]) # Llega el cobot real
    pinza = SE3(-2.84011157, 115.08057356,  22.28604936) * SE3.Rx(-np.pi/2)
    home = RobTarget(cobot_tb.fkine(np.repeat(0.01, 6)) * pinza, [-1, 1, 1])
    wobj = SE3()

    rviz.MoveJ(wobj, pick.offset(0, 0, 80), pinza, 'wobjMoveJ', 'offset_pick')
    pick = RobTarget(SE3(0, -220, 25)* SE3.Ry(-np.pi) * SE3.Rx(-20, 'deg'), [-1, 1, 1])
    rviz.MoveC(wobj, pick, pinza, 'wobjMoveC', 'pick')
    rviz.mover_pinza(10, pinza)
    rviz.node_obj.attach('caja1')
    rviz.MoveJ(wobj, home, pinza)

    rviz.MoveJ(wobj, place.offset(0, 0, 50), pinza, robt_name='offset_place')
    rviz.MoveC(wobj, place, pinza, robt_name='place')
    rviz.mover_pinza(100, pinza)
    rviz.node_obj.detach('caja1')
    rviz.MoveC(wobj, place.offset(0, 0, 30), pinza, robt_name='offset_place')
    rviz.MoveJ(wobj, home, pinza)

def movTCP(rviz):
    ''' Movimiento alrededor del TCP'''
    rviz.node_obj.add_object("caja1", pose_init=(0.0, -0.22, 0.03),
                    size=(0.06, 0.05, 0.06),
                    color=(1.0, 0.0, 0.0),
                    shape=CUBE)
    pose_pick_manual = RobTarget(SE3(0, -220, 25)* SE3.Ry(-np.pi), [-1, 1, 1]) 
    wobj = SE3()
    pinza = SE3(-1.71381642, 106.90735789, 28.32702833) * SE3.Rx(-np.pi/2)
    angle = -5
    while angle >= -20:
        rviz.MoveJ(wobj, pose_pick_manual.relTool(0, 0, 0, angle, angle, angle), pinza)
        if angle == -5:
            rviz.node_obj.attach('caja1')

        if angle == -15:
            rviz.node_obj.detach('caja1')  # el objeto se mueve con la pinza
        angle -= 5

def wobjCalc(rviz):
    rviz.node_obj.add_object("mesa_wobj", pose_init=(0.2, -0.32, 0.05),
                    size=(0.3, 0.3, 0.1),
                    color=(0.0, 1.0, 0.0),
                    shape=CUBE, movable=False)
    
    rviz.node_obj.add_object(name="pieza1",
    pose_init=(205e-3, -170e-3, 125e-3),
    size=(1.0e-3, 1.0e-3, 1.0e-3),  # escala en metros
    color=(1.0, 0.0, 0.0, 0.5),
    shape=MESH,
    movable=True,
    mesh="file:///home/ema/Downloads/Palpador_pinza4.stl",
    rot_euler=(np.pi/2 + 0*np.pi/15, 0, 0))
    
    # pinza = SE3(-1.71381642, 106.90735789, 28.32702833) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)
    pinza = SE3(0, 123, 27.5) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)
    pinza_aux = SE3(0, 148, 27.5) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)
    # rviz.VerQ(np.zeros(6), tool = pinza)
    # x1 = RobTarget(SE3(205, -170, 100)* SE3.Ry(-np.pi)* SE3.Rz(np.pi)*SE3.Rx(np.pi/15), [1, -1, 1])
    x1 = RobTarget(SE3(205, -170, 100)* SE3.Ry(-np.pi)* SE3.Rz(np.pi), [1, -1, -1])
    x2 = x1.relTool(-100, 0, 0, 10, 10, 10)
    y1 = RobTarget(SE3(50, -250, 100)* SE3.Ry(-np.pi)* SE3.Rz(np.pi)*SE3.Ry(np.pi/18)*SE3.Rx(np.pi/18), [1, -1, -1])
    q_wobj = []
    poses = [x1, x2, y1]
    for pose in poses:
        rviz.VerPose(SE3(), pose, pinza_aux, 'wobj', 'robtarget')
        # time.sleep(1)
        rviz.VerPose(SE3(), pose, pinza_aux, 'wobj', 'robtarget', )
        # rviz.MoveJ(SE3(), pose, pinza_aux, 'wobj', 'pinza_aux')
        time.sleep(3)
        q_wobj.append(rviz.q_current[:-1])
        if pose == x1:
            rviz.mover_pinza(20, pinza_aux)
            time.sleep(1)
            rviz.node_obj.attach('pieza1')
            time.sleep(3)
            # break
    rviz.VerPose(SE3(), y1, pinza_aux, 'wobj', 'robtarget')
    
    wobj_calculado = teach_wobj(np.rad2deg(q_wobj), pinza)
    robt0 = RobTarget(SE3(wobj_calculado))
    # pinza_aux = pinza_aux*SE3(0,0,25)
    time.sleep(1)
    rviz.VerPose(SE3(), robt0, pinza, robt_name='pinza_aux')
    time.sleep(1)
    rviz.VerPose(SE3(), robt0, pinza, robt_name='pinza_aux')
    # # rviz.testPose(robt0, pinza, SE3())

# movTCP(rviz)
# pickPlace_v0(rviz)
wobjCalc(rviz)
time.sleep(1)
rviz.shutdown()