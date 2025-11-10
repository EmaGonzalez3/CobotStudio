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
    # cobot_tb = myCobot320(rotar_base=True, metros=False)
    
    # TCP
    pinza = SE3(-1.71381642, 106.90735789, 28.32702833) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)
    wobj = SE3() # wobj nulo por simplicidad
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
        robot.MoveJAngles(np.zeros(6), spd)
        robot.GripperState(100, 30)
        
    def amague(pose_pick):
        """
        Cierre parcial de la pinza en la posición del llavero.
        """
        robot.GripperState(20, 30) # Cierre de la pinza
        time.sleep(3)
        robot.MoveJ(pose_pick.offset(0, 0, 40), spd, pinza, wobj)
        nop = RobTarget(SE3(215, 0.5, 300)*SE3.Ry(-np.pi)*SE3.Rz(np.pi/2), [1, 1, 1]) # 111 | 1-11 | -11-1 | -1-1-1!!
        robot.MoveJ(nop, 50, pinza, SE3())
        robot.MoveJ(nop.relTool(-30, 0, -5, 0, -10, 0), 30, pinza, SE3())
        robot.MoveJ(nop.relTool(30, 0, -5, 0, 10, 0), 30, pinza, SE3())
        robot.MoveJAngles(np.zeros(6), spd)
        robot.GripperState(100, 30)

    def dar_llavero(llavero, spd = 30):
        if llavero == 1:
            pick = RobTarget(SE3(260, -125, 40)* SE3.Ry(-np.pi)*SE3.Rz(np.pi/2 +np.pi/7.5), [-1, 1, 1])

        elif llavero == 2:
            pick = RobTarget(SE3(315, -25, 40)* SE3.Ry(-np.pi)*SE3.Rz(np.pi/2 +np.pi/7.5), [-1, 1, 1])

        elif llavero == 3:
            pick = RobTarget(SE3(208, -221, 40)* SE3.Ry(-np.pi)*SE3.Rz(np.pi/2 +np.pi/7.5), [-1, 1, 1])

        robot.GripperState(100, 30) # Abrir la pinza
        # Acercamiento al llavero
        robot.MoveJ(pick.offset(0, 0, 40), spd, pinza, wobj)

        dado = random.randint(1, 6) # Define si da o no llavero
        # dado = 1
        if dado == 2: # No da llavero y cierra la pinza antes
            no_llavero()
            return

        # Posición de pick
        robot.MoveJ(pick, spd, pinza, wobj)

        if dado == 4: # No da llavero por no cerrar del todo la pinza
            amague(pick)
            return
        
        robot.GripperState(0, 40) # Cierre de la pinza
        time.sleep(3) # Tiempo para que cierre
        # Sacar llavero de la base
        robot.MoveJ(pick.offset(0, 0, 40), 30, pinza, wobj)
        # Dar el llavero
        robot.MoveJ(dar, spd, pinza, wobj)
        robot.MoveJ(dar.relTool(0, 0, 50), 30, pinza, wobj)
        time.sleep(2)
        robot.GripperState(100, 30) # Abrir pinza
        time.sleep(5) # Tiempo para agarrar el llavero
        # Volver al home
        robot.MoveJAngles(np.zeros(6), spd)


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


    saludar()
    recorrer()
    random_num = random.randint(1, 3)
    dar_llavero(random_num)

    # testPoses()


if __name__ == "__main__":

    DEBUG_MODE_OVERRIDE = "real" # "sim" o "real" para debug. None para correr el menu interactivo.

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