from Cobot_sdk import *
import random

def run(robot: BaseRobotController, speed = 30, **kwargs):

    robot.load_scene("NDLM_scene")
    
    # TCP
    pinza = SE3(-1.71381642, 106.90735789, 28.32702833) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)
    wobj_ref = SE3.Ry(-np.pi)*SE3.Rz(np.pi/2 +np.pi/6.6)*SE3(7, 282, 0)
    dar = RobTarget(SE3(280, -135.721, 200)* SE3.Ry(np.pi/2)*SE3.Rz(np.pi), [-1, 1, 1])

    def no_llavero():
        """
        Cierre prematuro de la pinza, no da llavero.
        """
        robot.GripperState(0, 30) # Cierre de la pinza
        time.sleep(3)
        nop = RobTarget(SE3(215, 0.5, 300)*SE3.Ry(-np.pi)*SE3.Rz(np.pi/2), [1, 1, 1]) # 111 | 1-11 | -11-1 | -1-1-1!!
        robot.MoveJ(nop, 50, pinza, SE3())
        robot.MoveJ(nop.relTool(-30, 0, -5, 0, -10, 0), 30, pinza, SE3())
        robot.MoveJ(nop.relTool(30, 0, -5, 0, 10, 0), 30, pinza, SE3())
        robot.MoveJAngles(np.zeros(6), speed)
        robot.GripperState(100, 30)
        
    def amague(pose_pick):
        """
        Cierre parcial de la pinza en la posición del llavero.
        """
        robot.GripperState(20, 30) # Cierre de la pinza
        time.sleep(3)
        robot.MoveJ(pose_pick.offset(0, 0, -40), speed, pinza, wobj_ref)
        nop = RobTarget(SE3(215, 0.5, 300)*SE3.Ry(-np.pi)*SE3.Rz(np.pi/2), [1, 1, 1]) # 111 | 1-11 | -11-1 | -1-1-1!!
        robot.MoveJ(nop, 50, pinza, SE3())
        robot.MoveJ(nop.relTool(-30, 0, -5, 0, -10, 0), 30, pinza, SE3())
        robot.MoveJ(nop.relTool(30, 0, -5, 0, 10, 0), 30, pinza, SE3())
        robot.MoveJAngles(np.zeros(6), speed)
        robot.GripperState(100, 30)

    def dar_llavero(llavero, speed = 30):
        if llavero == 1:
            pick = RobTarget(SE3(0, 10, -40), [-1, 1, 1])

        elif llavero == 2:
            # pick = RobTarget(SE3(115, 10, -40)*SE3.Rz(-0.06), [-1, 1, 1])
            pick = RobTarget(SE3(115, 10, -40), [-1, 1, 1])
            
        elif llavero == 3:
            pick = RobTarget(SE3(-105, 10, -40), [-1, 1, 1])

        robot.GripperState(100, 30) # Abrir la pinza
        # Acercamiento al llavero
        robot.MoveJ(pick.offset(0, 0, -40), speed, pinza, wobj_ref)

        dado = random.randint(1, 6) # Define si da o no llavero
        dado = 1
        if dado == 2: # No da llavero y cierra la pinza antes
            no_llavero()
            return

        # Posición de pick
        robot.MoveC(pick, speed, pinza, wobj_ref)

        if dado == 4: # No da llavero por no cerrar del todo la pinza
            amague(pick)
            return
        
        robot.GripperState(0, 40) # Cierre de la pinza
        time.sleep(3) # Tiempo para que cierre
        # Sacar llavero de la base
        robot.MoveC(pick.offset(0, 0, -40), 30, pinza, wobj_ref)
        # Dar el llavero
        robot.MoveJ(dar, speed, pinza)
        robot.MoveJ(dar.relTool(0, 0, 50), 30, pinza)
        time.sleep(2)
        robot.GripperState(100, 30) # Abrir pinza
        time.sleep(5) # Tiempo para agarrar el llavero
        # Volver al home
        robot.MoveJAngles(np.zeros(6), speed)


    def saludar():
        saludo1 = RobTarget(SE3(215, 0.5, 425)* SE3.Ry(np.pi/2)*SE3.Rz(np.pi/2), [-1, 1, 1])
        saludo2 = RobTarget(SE3(250, 0.5, 350)* SE3.Ry(np.pi/2)*SE3.Rz(np.pi/2)*SE3.Rx(np.pi/4), [-1, 1, 1])
        robot.MoveJ(saludo1, 50, pinza, SE3())
        robot.MoveJ(saludo2, 40, pinza, SE3())
        robot.MoveJ(saludo1, 30, pinza, SE3())

    def recorrer():
        punto1 = RobTarget(SE3(324, -30, 90)* SE3.Ry(-np.pi)*SE3.Rz(np.pi/2 +np.pi/7.5), [-1, 1, 1])
        punto2 = RobTarget(SE3(223, -225, 90)* SE3.Ry(-np.pi)*SE3.Rz(np.pi/2 +np.pi/7.5), [-1, 1, 1])
        punto3 = RobTarget(SE3(270, -130, 90)* SE3.Ry(-np.pi)*SE3.Rz(np.pi/2 +np.pi/7.5), [-1, 1, 1])
        # robot.testPose(punto1, pinza, SE3())
        robot.MoveJ(punto1, 50, pinza, SE3())
        robot.MoveJ(punto2, 30, pinza, SE3())
        robot.MoveJ(punto3, 30, pinza, SE3())

    def testPoses():
        """
        Chequeo de las posiciones de los porta llaveros.
        """
        llav = int(input("Seleccione llavero 1, 2 o 3:"))
        dar_llavero(llav)

    wobj_ref = SE3.Ry(-np.pi)*SE3.Rz(np.pi/2 +np.pi/6.6)*SE3(7, 282, -0)
    pick = RobTarget(SE3(115, 10, -40)*SE3.Rz(-0.06), [-1, 1, 1])
    pinza = SE3(-1.71381642, 106.90735789, 28.32702833) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)
    # robot.GripperState(0)
    # time.sleep(2)
    # robot.GripperState(100)
    # robot.MostrarTerna(pick.pose, 'pick_llavero', wobj_ref)
    # robot.VerPose(wobj_ref, pick, pinza)
    # robot.MoveJ()
    # home = np.zeros(6)
    # robot.VerQ(home)
    # time.sleep(2)
    # saludar()
    # recorrer()
    # random_num = random.randint(1, 3)
    dar_llavero(2)
    # testPoses()

    # nop = RobTarget(SE3(215, 0.5, 300)*SE3.Ry(-np.pi)*SE3.Rz(np.pi/2), [1, 1, 1])
    # robot.VerPose(SE3(), nop, pinza)
    # robot.GripperState(100)


    # time.sleep(2)

    # robot.VerQ(np.zeros(6), pinza, brida = True)
    # print('Salimos de VerQ')

    # time.sleep(1)
    # robot.GripperState(0)
    # time.sleep(1)
    

    # time.sleep(2)

    
    # time.sleep(2)
    # robot.GripperState(0)
    # time.sleep(1)
    # # aaa = robot.traj_moveit('traj1', tool = pinza)
    # # aaa = robot.traj_moveit('traj21', 'sin_pinza', tool = pinza)
    # aaa = robot.traj_moveit('traj_2911', 'sin_inspect', tool = pinza)
    # robot.MostrarTerna(aaa.pose, 'terna_moveit')



    # q_60 = [
    # [-52.82, -33.92, -49.21, -81.56, -13.09, 162.5],
    # [-41.48, -51.15, -91.58, 89.82, 1.58, 53.17],
    # [-30.67, -63.36, -60.38, 73.38, 9.31, 57.65],
    # [-62.13, -52.55, -86.57, 71.71, -10.19, 67.41],
    # [-70.13, -60.82, -57.12, 51.06, 0.26, 80.59],
    # [-72.86, -84.55, -8.7, 18.89, -2.63, 108.1],
    # ]

    # q_71 = [
    # [41.04, 61.17, 52.64, -26.45, 99.31, -119.26],
    # [40.86, 59.32, 74.26, -34.01, 95.53, -124.36],
    # [42.71, 85.42, 63.45, -48.86, 94.65, -132.71],
    # [40.42, 29.79, 24.69, 47.81, 59.67, -126.91],
    # [52.47, 19.77, 36.47, 63.19, 76.2, -119.35],
    # [71.8, 23.99, 28.03, 55.72, 62.05, -126.38],
    # ]
    # pinza = SE3(0, 123, 27.5) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)

    # # print(f'Pinza: {pinza}')
    # # robot.teach_and_save_wobj('workobject_83', 'wobj00', pinza, save_q = True, q_test = q_71)

    # a_ver = robot.load_wobj('workobject_01', 'wobj4', q_teach = q_60, tool = pinza)
    
    # # q_cargado = robot.load_data("Workobjects", "workobject_83", "wobj00")
    # # print(f'q_cargado =\n{q_cargado}')
    # # print(f'A ver si lo cargó bien:\n{a_ver}')
    # from scripts.DHRobotGT import myCobot320
    # cobot_tb = myCobot320(rotar_base=True, metros=False)
    # q_conf = np.array([-0.15,  0.76,  1.78,  2.47,  0.77, -2.12])
    # pose = cobot_tb.fkine(q_conf)
    # robt = RobTarget(pose, [1, 1, 1])
    # robot.testPose(robt, SE3(), SE3())


    # wobj = SE3(100, 100, 200)*SE3.Rx(np.pi)
    # dar = RobTarget(SE3(100, 0, 50)* SE3.Ry(np.pi/2)*SE3.Rz(np.pi), [-1, 1, 1])
    # robot.MostrarTerna(wobj, 'wobj')
    # robot.VerPose(dar, SE3(), wobj, 'wobj', 'robt')
    # robot.VerPose(dar.offset(0, 0, 50, 0, 0, 0), SE3(), wobj, 'wobj', 'offset')
    # robot.MostrarTerna(dar.pose, 'robt', wobj)
    # robot.MostrarTerna(dar.offset(0, 0, 20, 0, 0, 90).pose, 'offset', wobj)
    # robot.MoveJAngles(np.ones(6))
    # robot.MoveJAngles(np.zeros(6))
    # robot.VerQ(np.zeros(6), pinza, True)

    time.sleep(5)