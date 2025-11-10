from scripts.CobotStudio_rev4 import RobTarget, BaseRobotController, MyCobotController
from spatialmath import SE3
import numpy as np
from scripts.DHRobotGT import myCobot320
import time
import random


try:
    from scripts.CobotStudio_rev4 import SimManager
    from scripts.NDLM.NDLM_scene import setup_scene
    SIM_AVAILABLE = True
    print("Modo de simulación disponible: ROS2 detectado.")
except ImportError:
    SimManager = None  # Definimos SimManager como None para evitar NameError
    SIM_AVAILABLE = False
    print("Advertencia: Modo de simulación no disponible (no se encontraron librerías de ROS2).")

# Importamos la función con la celda robótica

def get_robot(mode="sim", **kwargs) -> BaseRobotController:
    if mode == "sim":
        if not SIM_AVAILABLE:
            raise RuntimeError("No se puede crear un robot en modo 'sim' porque las librerías de ROS2 no están disponibles.")
        return SimManager()
    elif mode == "real":
        return MyCobotController(**kwargs)
    else:
        raise ValueError("Modo desconocido: usa 'sim' o 'real'")


def main(robot: BaseRobotController, spd = 30):
    cobot_tb = myCobot320(rotar_base=True, metros=False)
    altura_caja = 0.07
    # TCPs
    pinza = SE3(-1.71381642, 106.90735789, 28.32702833) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)
    wobj = SE3() # wobj nulo por simplicidad
    home = RobTarget(cobot_tb.fkine(np.repeat(0.01, 6)) * pinza, [-1, 1, 1])
    offset_pick = 50 # Qué tan lejos de la caja frenamos
    offset_place = 30 # Qué tan lejos del 'pallet' frenamos
    dar = RobTarget(SE3(280, -135.721, 200)* SE3.Ry(np.pi/2)*SE3.Rz(np.pi), [-1, 1, 1])

    def no_llavero():
        robot.GripperState(0, 30) # Cierre de la pinza
        time.sleep(3)
        nop = RobTarget(SE3(215, 0.5, 300)*SE3.Ry(-np.pi)*SE3.Rz(np.pi/2), [1, 1, 1]) # 111 | 1-11 | -11-1 | -1-1-1!!
        # robot.testPose(nop, pinza, SE3())
        robot.MoveJ(nop, 50, pinza, SE3())
        robot.MoveJ(nop.relTool(-30, 0, -5, 0, -10, 0), 30, pinza, SE3())
        robot.MoveJ(nop.relTool(30, 0, -5, 0, 10, 0), 30, pinza, SE3())
        robot.MoveJAngles(np.zeros(6), spd)
        robot.GripperState(100, 30)

    def dar_llavero(llavero, offset_pick = 50, offset_place = 30):
        if llavero == 1:
            pick = RobTarget(SE3(270, -130, 40)* SE3.Ry(-np.pi)*SE3.Rz(np.pi/2 +np.pi/7.5), [-1, 1, 1])

        elif llavero == 2:
            pick = RobTarget(SE3(324, -30, 40)* SE3.Ry(-np.pi)*SE3.Rz(np.pi/2 +np.pi/7.5), [-1, 1, 1])

        elif llavero == 3:
            pick = RobTarget(SE3(223, -225, 40)* SE3.Ry(-np.pi)*SE3.Rz(np.pi/2 +np.pi/7.5), [-1, 1, 1])

        def ole():
            robot.GripperState(20, 30) # Cierre de la pinza
            time.sleep(3)
            robot.MoveC(pick.offset(0, 0, 40), 30, pinza, wobj)
            nop = RobTarget(SE3(215, 0.5, 300)*SE3.Ry(-np.pi)*SE3.Rz(np.pi/2), [1, 1, 1]) # 111 | 1-11 | -11-1 | -1-1-1!!
            # robot.testPose(nop, pinza, SE3())
            robot.MoveJ(nop, 50, pinza, SE3())
            robot.MoveJ(nop.relTool(-30, 0, -5, 0, -10, 0), 30, pinza, SE3())
            robot.MoveJ(nop.relTool(30, 0, -5, 0, 10, 0), 30, pinza, SE3())
            robot.MoveJAngles(np.zeros(6), spd)
            robot.GripperState(100, 30)

            
        # Acercamiento al llavero
        robot.MoveJ(pick.offset(0, 0, 40), spd, pinza, wobj)

        dado = random.randint(1, 6)
        dado = 2
        # if dado == 2 or dado == 4:
        #     no_llavero()
        #     return


        # Posición de pick
        robot.MoveC(pick, 30, pinza, wobj)
        # robot.MostrarTerna(pick.pose, f'Llavero {llavero}')
        if dado == 2 or dado == 4:
            ole()
            return
        robot.GripperState(0, 30) # Cierre de la pinza
        time.sleep(3)
        # Sacar llavero de la base
        robot.MoveC(pick.offset(0, 0, 40), 30, pinza, wobj)
        # Dar el llavero
        robot.MoveJ(dar, spd, pinza, wobj)
        robot.MoveJ(dar.relTool(0, 0, 50), 30, pinza, wobj)
        time.sleep(2)
        robot.GripperState(100, 30) # Abrir pinza
        time.sleep(5)
        # Volver al home
        robot.MoveJAngles(np.zeros(6), spd)


    def saludar():
        saludo1 = RobTarget(SE3(215, 0.5, 425)* SE3.Ry(np.pi/2)*SE3.Rz(np.pi/2), [-1, 1, 1])
        saludo2 = RobTarget(SE3(250, 0.5, 350)* SE3.Ry(np.pi/2)*SE3.Rz(np.pi/2)*SE3.Rx(np.pi/4), [-1, 1, 1])
        # robot.testPose(saludo2, pinza, SE3())
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

    
    saludar()
    recorrer()
    random_num = random.randint(1, 3)
    # for i in range(3):
    dar_llavero(random_num)


def pos_llaveros(robot: BaseRobotController, devolver = True):
    cobot_tb = myCobot320(rotar_base=True, metros=False)
    def pose_to_matrix(pose):
        """Convierte pose dada como lista [x, y, z, rx, ry, rz] en una matriz homogénea SE3."""
        x, y, z, rx, ry, rz = pose
        rx, ry, rz = np.deg2rad([rx, ry, rz])
        T_se3 = SE3(x, y, z) * SE3.RPY([rx, ry, rz], order='zyx')
        return T_se3
    
    q_50 = [
    [13.97, -20.91, -89.12, -23.9, -1.31, 131.39, 260.8, -94.0, 194.0, -89.04, -2.55, -165.15],
    [19.68, -82.96, -60.82, 127.7, -32.69, 25.31, 234.1, -69.0, 194.6, -81.21, 11.54, 169.79],
    [-7.55, -103.09, -11.95, 108.54, -23.11, 15.29, 203.8, -177.3, 189.8, -87.41, 9.29, 149.87],
    [-19.33, -50.27, -62.92, 25.48, 16.96, 75.41, 242.1, -245.4, 197.5, -107.31, -11.66, 164.96],
    [-10.28, -37.79, -49.39, -66.7, 3.07, 159.52, 209.8, -194.7, 199.6, -91.35, 5.59, 166.82],
    [-11.51, -53.87, -19.24, -82.26, 14.32, 151.78, 213.3, -198.8, 195.1, -95.93, -4.25, 155.85],
]

    q_51 = [
    [17.4, -17.84, -88.5, -62.66, -95.62, 166.2, 252.3, -7.2, 187.7, -78.69, -14.61, -69.75],
    [0.52, -53.78, -26.27, -92.02, -143.34, 178.94, 279.2, -33.6, 185.6, -85.24, -7.39, -36.47],
    [0.43, -66.18, -12.48, -103.18, -156.53, 168.92, 264.3, -26.6, 156.2, -90.74, -9.38, -22.89],
    [-0.08, -64.16, -11.07, -104.76, -149.94, 172.26, 270.2, -32.5, 168.3, -90.0, -7.73, -30.14],
    [-2.02, -79.1, 10.72, -102.04, -153.98, -177.53, 287.0, -40.0, 154.7, -85.78, -6.14, -28.17],
    [29.79, -74.88, 7.73, -114.87, -17.92, 178.94, 298.7, -3.0, 160.0, -90.62, -2.97, -132.25],
]

    q_izq = [
    [5.53, -32.16, -99.93, -0.96, 6.59, 132.71, 238.9, -131.4, 137.3, -94.81, -0.53, -178.92],
    [6.32, -19.68, -135.08, 25.57, 8.52, 152.92, 180.0, -134.5, 124.8, -97.18, 23.25, 178.06],
    [10.54, -30.23, -97.82, -27.5, -4.13, 155.56, 230.7, -113.7, 132.0, -88.29, -0.05, -165.69],
    [-5.88, -54.66, -56.6, -15.38, 22.76, 114.16, 266.2, -177.4, 131.3, -108.65, -13.98, 164.71],
    [-6.85, -51.15, -53.7, -36.73, 31.2, 141.06, 234.4, -174.0, 132.2, -108.83, -4.53, 149.3],
    [6.41, -52.73, -49.83, -66.79, 16.25, 168.92, 239.6, -125.6, 132.7, -92.96, -0.85, 170.46],
    ]

    q_der = [
    [22.67, -36.82, -94.39, -4.83, 1.49, 131.13, 277.1, -51.3, 133.3, -91.04, -4.93, -158.31],
    [31.11, -50.97, -83.67, -10.01, -13.44, 114.69, 299.3, 2.5, 105.8, -81.02, -30.4, -142.41],
    [5.44, -62.4, -55.98, 9.4, 24.34, 91.31, 318.9, -118.7, 122.9, -114.16, -17.78, -175.12],
    [30.67, -77.78, -41.74, -25.57, -11.51, 92.63, 337.2, 22.2, 72.8, -79.18, -52.52, -148.46],
    [16.87, -51.5, -24.52, -68.29, 17.57, 108.45, 294.0, -68.8, 198.2, -102.65, -36.47, -169.95],
    [19.95, -132.01, 76.46, -49.74, 5.18, 78.39, 324.5, -46.0, 120.6, -95.6, -26.84, -158.88],
    ]

    altura_caja = 0.07
    # TCPs
    pinza = SE3(-1.71381642, 106.90735789, 28.32702833) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)
    pinza_aux = pinza*SE3(0,0,25)
    wobj = SE3() # wobj nulo por simplicidad
    home = RobTarget(cobot_tb.fkine(np.repeat(0.01, 6)) * pinza, [-1, 1, 1])
    offset_pick = 50 # Qué tan lejos de la caja frenamos
    offset_place = 30 # Qué tan lejos del 'pallet' frenamos
    llavero1 = RobTarget(SE3(228, -135.721, 20)* SE3.Ry(-np.pi)*SE3.Rz(np.pi+np.pi/15), [-1, 1, 1])
    dar = RobTarget(SE3(280, -135.721, 200)* SE3.Ry(np.pi/2)*SE3.Rz(np.pi), [-1, 1, 1])
    # robot.testPose(dar, pinza, wobj) #218.0e-3, -175.721e-3, - brida_base
    # robot.VerPose(wobj, llavero1, pinza, wobj_name='wobj1', robt_name='robt23')
    # robot.MostrarTerna(dar.pose, 'terna_dar')
    for i, muestra in enumerate(q_izq, 1):
        pose = pose_to_matrix(muestra[6:])
        q = muestra[:6]
        robot.MostrarTerna(pose*pinza_aux, f'izq_{i+6}')
        time.sleep(2)
        robot.VerQ(np.deg2rad(q), pinza_aux, brida=True)
        time.sleep(10)
    time.sleep(2)


if __name__ == "__main__":

    DEBUG_MODE_OVERRIDE = "sim" # "sim" o "real" para debug. None para correr el menu interactivo.

    selected_mode = None

    if DEBUG_MODE_OVERRIDE in ["sim", "real"]:
        selected_mode = DEBUG_MODE_OVERRIDE
        print(f"Modo seleccionado por variable de depuración: '{selected_mode}'")
    else:
        while selected_mode not in ["sim", "real"]:
            print("\nSelecciona el modo de ejecución:")
            print("1. Simulación (sim)")
            print("2. Robot Real (real)")
            choice = input("Ingresa tu opción (1 o 2): ")
            if choice == '1':
                selected_mode = "sim"
            elif choice == '2':
                selected_mode = "real"
            else:
                print("Opción no válida. Intente de nuevo.")

    robot_controller = None
    try:
        if selected_mode == "sim" and not SIM_AVAILABLE:
            raise RuntimeError("El modo 'sim' no está disponible porque las librerías de ROS2 no se encontraron.")
        
        robot_controller = get_robot(selected_mode)

    except (RuntimeError, ValueError) as e:
        print(f"Error al inicializar el robot: {e}")
        exit()

    # Si estamos en modo simulación, configuramos la escena
    if selected_mode == "sim" and isinstance(robot_controller, SimManager):
        print("Configurando la escena de simulación...")
        setup_scene(robot_controller)

    try:
        main(robot_controller)
    except Exception as e:
        print(f"Ocurrió un error durante la ejecución: {e}")
    finally:
        if robot_controller and hasattr(robot_controller, 'shutdown'):
            robot_controller.shutdown()
        print("--- Secuencia finalizada ---")