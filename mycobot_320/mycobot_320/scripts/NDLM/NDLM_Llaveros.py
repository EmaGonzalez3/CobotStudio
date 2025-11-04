from scripts.CobotStudio_rev4 import RobTarget, BaseRobotController, MyCobotController, live_pose
from spatialmath import SE3
import numpy as np
from scripts.DHRobotGT import myCobot320
import time


try:
    from scripts.CobotStudio_rev4 import SimManager
    SIM_AVAILABLE = True
    print("Modo de simulación disponible: ROS2 detectado.")
except ImportError:
    SimManager = None  # Definimos SimManager como None para evitar NameError
    SIM_AVAILABLE = False
    print("Advertencia: Modo de simulación no disponible (no se encontraron librerías de ROS2).")

# Importamos la función con la celda robótica
from scripts.NDLM.NDLM_scene import setup_scene

def get_robot(mode="sim", **kwargs) -> BaseRobotController:
    if mode == "sim":
        if not SIM_AVAILABLE:
            raise RuntimeError("No se puede crear un robot en modo 'sim' porque las librerías de ROS2 no están disponibles.")
        return SimManager()
    elif mode == "real":
        return MyCobotController(**kwargs)
    else:
        raise ValueError("Modo desconocido: usa 'sim' o 'real'")


def main(robot: BaseRobotController, devolver = True):
    cobot_tb = myCobot320(rotar_base=True, metros=False)
    altura_caja = 0.07
    # TCPs
    pinza = SE3(-1.71381642, 106.90735789, 28.32702833) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)
    wobj = SE3() # wobj nulo por simplicidad
    home = RobTarget(cobot_tb.fkine(np.repeat(0.01, 6)) * pinza, [-1, 1, 1])
    offset_pick = 50 # Qué tan lejos de la caja frenamos
    offset_place = 30 # Qué tan lejos del 'pallet' frenamos
    dar = RobTarget(SE3(280, -135.721, 200)* SE3.Ry(np.pi/2)*SE3.Rz(np.pi), [-1, 1, 1])

    def dar_llavero(llavero, offset_pick = 50, offset_place = 30):
        if llavero == 1:
            pick = RobTarget(SE3(270, -130, 30)* SE3.Ry(-np.pi)*SE3.Rz(np.pi/2 +np.pi/7.5), [-1, 1, 1])

        elif llavero == 2:
            pick = RobTarget(SE3(313, -21, 30)* SE3.Ry(-np.pi)*SE3.Rz(np.pi/2 +np.pi/7.5), [-1, 1, 1])

        elif llavero == 3:
            pick = RobTarget(SE3(223, -225, 30)* SE3.Ry(-np.pi)*SE3.Rz(np.pi/2 +np.pi/7.5), [-1, 1, 1])
            
        # Acercamiento al llavero
        robot.MoveJ(pick.offset(0, 0, 30), 30, pinza, wobj)
        # Posición de pick
        robot.MoveC(pick, 30, pinza, wobj)
        robot.GripperState(0, 30) # Cierre de la pinza
        time.sleep(3)
        # Sacar llavero de la base
        robot.MoveC(pick.offset(0, 0, 40), 30, pinza, wobj)
        # Dar el llavero
        robot.MoveJ(dar, 30, pinza, wobj)
        robot.MoveJ(dar.relTool(0, 0, 50), 30, pinza, wobj)
        time.sleep(2)
        robot.GripperState(100, 30) # Abrir pinza
        time.sleep(5)
        # Volver al home
        robot.MoveJAngles(np.zeros(6), 30)

    for i in range(3):
        dar_llavero(i+1)


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
    q_2 = []
    poses2 = []

    for q in q_50:
        q_2.append(q[:6])
        # poses2.append(q[6:])

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
    # for i, muestra in enumerate(q_50, 1):
    #     pose = pose_to_matrix(muestra[6:])
    #     q = muestra[:6]
    #     robot.MostrarTerna(pose*pinza_aux, f'punto_{i+6}')
    #     time.sleep(2)
    #     robot.VerQ(np.deg2rad(q), pinza_aux, brida=True)
    #     time.sleep(10)
    # time.sleep(2)


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