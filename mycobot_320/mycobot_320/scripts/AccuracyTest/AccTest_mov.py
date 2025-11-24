from scripts.CobotStudio_rev4 import RobTarget, BaseRobotController, MyCobotController
from spatialmath import SE3
import numpy as np
from scripts.DHRobotGT import myCobot320
import time
import datetime

# Importación condicional para el modo de simulación
try:
    from scripts.CobotStudio_rev4 import SimManager
    SIM_AVAILABLE = True
    print("Modo de simulación disponible: ROS2 detectado.")
except ImportError:
    SimManager = None  # Definimos SimManager como None para evitar NameError
    SIM_AVAILABLE = False
    print("Advertencia: Modo de simulación no disponible (no se encontraron librerías de ROS2).")

# Importamos la función con la celda robótica
from scripts.AccuracyTest.AccTest_Scene import setup_scene

log_point_counter = 1

def get_robot(mode="sim", **kwargs) -> BaseRobotController:
    if mode == "sim":
        if not SIM_AVAILABLE:
            raise RuntimeError("No se puede crear un robot en modo 'sim' porque las librerías de ROS2 no están disponibles.")
        return SimManager()
    elif mode == "real":
        return MyCobotController(**kwargs)
    else:
        raise ValueError("Modo desconocido: usa 'sim' o 'real'")

def matrix_to_pose(T_matrix):
    """Convierte una matriz homogénea SE3 en lista [x, y, z, rx, ry, rz] en grados."""
    T = SE3(T_matrix)
    x, y, z = T.t
    # Extraer rotación como RPY (en radianes), orden ZYX
    rx, ry, rz = T.rpy(order='zyx', unit='rad')

    # Convertimos a grados
    rx, ry, rz = np.rad2deg([rx, ry, rz])

    return [x, y, z, rx, ry, rz]

def main(robot: BaseRobotController):
    def data_reg(robt, tool, filename, sleep=1):
        global log_point_counter
        time.sleep(sleep)

        try:
            lectura_robot = robot.mc.get_angles_coords()
            q_robot = lectura_robot[:6]
            coords_robot = lectura_robot[6:] # Asumo que las coordenadas son los últimos elementos
        except Exception as e:
            print(f"No se pudo acceder al controlador del robot: {e}. Devolviendo q de ROS2:")
            q_robot = robot.q_current[:6]
            coords_robot = [] # Dejar vacío o con un valor por defecto si no se pueden obtener
            print(f'q_ROS\n{q_robot}')

        pose_pedida = matrix_to_pose(robt.pose * tool.inv())
        print(f"En xyzrpy mandamos al cobot a\n{pose_pedida}")

        q_tb = cobot_tb.ikine(robt.pose * tool.inv(), robt.config)[0]
        print(f'Según la tb se llegaba con los ángulos\n{q_tb}')

        pose_alc_tb = matrix_to_pose(cobot_tb.fkine(q_robot))
        print(f'Los ángulos devueltos por el robot según la tb van a\n{pose_alc_tb}')

        try:
            with open(filename, 'a') as f:
                timestamp = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
                f.write(f"# --- Punto {log_point_counter} - {timestamp} ---\n")

                f.write(f"q_robot = {list(q_robot)}\n")
                f.write(f"coords_robot = {list(coords_robot)}\n")
                f.write(f"pose_pedida = {list(pose_pedida)}\n")
                f.write(f"q_tb = {list(q_tb)}\n")
                f.write(f"pose_alc_tb = {list(pose_alc_tb)}\n")
                f.write("\n") # Añadir una línea en blanco para separar los puntos

            print(f"Datos del punto {log_point_counter} guardados en '{filename}'")
            log_point_counter += 1

        except Exception as e:
            print(f"Error al guardar los datos en el archivo: {e}")

        time.sleep(sleep)

    cobot_tb = myCobot320(rotar_base=True, metros=False)
    # TCPs
    # pinza = SE3(-1.71381642, 106.90735789, 28.32702833) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)
    pinza = SE3(0, 123, 27.5) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)
    l_marker = 35
    # d_offset = 15
    pinza_marker = pinza * SE3(0, 0, l_marker)
    wobj = SE3() # wobj nulo por simplicidad
    home = RobTarget(cobot_tb.fkine(np.repeat(0, 6)), [1, 1, 1])

    def ensayo1 (d_offset = 15, iteracion = 0):
        file_log = f'ensayo1_v{iteracion}.txt'
        punto_centro = RobTarget(SE3(0, -260, 50)* SE3.Ry(-np.pi)*SE3.Rz(np.pi), [-1, 1, 1]) #[-1, 1, 1], [1, -1, -1]
        # robot.testPose(punto_centro, pinza_marker, wobj)
        punto1 = RobTarget(SE3(40, -300, 50)* SE3.Ry(-np.pi)*SE3.Rz(np.pi), [-1, 1, 1])
        punto2 = RobTarget(SE3(-40, -300, 50)* SE3.Ry(-np.pi)*SE3.Rz(np.pi), [-1, 1, 1])
        punto3 = RobTarget(SE3(40, -220, 50)* SE3.Ry(-np.pi)*SE3.Rz(np.pi), [-1, 1, 1])
        punto4 = RobTarget(SE3(-40, -220, 50)* SE3.Ry(-np.pi)*SE3.Rz(np.pi), [-1, 1, 1])
        puntos = [punto_centro, punto1, punto2, punto3, punto4]

        robot.MoveJ(punto_centro.offset(0, 0, d_offset), 30, pinza_marker, robt_name='Centro')
        print(punto_centro.offset(0, 0, d_offset).pose)
        robot.GripperState(100, 30)
        time.sleep(7)
        robot.GripperState(0, 30)
        time.sleep(3)
        for idx, punto in enumerate(puntos[1:], 1):
            robot.MoveJ(punto.offset(0, 0, d_offset), 30, pinza_marker, wobj)
            robot.MoveC(punto, 30, pinza_marker, wobj, robt_name=f'punto{idx}')
            print(f'Llegamos al punto {idx}')
            data_reg(punto, pinza_marker, file_log)
            robot.MoveC(punto.offset(0, 0, d_offset), 30, pinza_marker, wobj)
            robot.MoveJ(punto_centro.offset(0, 0, d_offset), 30, pinza_marker, wobj, robt_name='Centro')
            data_reg(punto_centro, pinza_marker, file_log)
        
        robot.MoveJ(punto_centro.offset(0, 0, 50), 30, pinza_marker, wobj)
        print(f'Volvemos al centro y terminamos.')
        robot.GripperState(100, 30)
        time.sleep(7)
    
        # Volvemos a Home
        robot.MoveJAngles(np.zeros(6), 30)
        # robot.MoveJ(home, 30, SE3(), wobj)
 
    def ensayo2 (d_offset = 15, iteracion = 0):
        file_log = f'ensayo2_v{iteracion}.txt'
        punto_centro = RobTarget(SE3(100, 200, 35)* SE3.Ry(-np.pi)*SE3.Rz(np.pi/2), [-1, 1, 1]) #(-1, 1, 1) [1, -1, 1]
        punto1 = RobTarget(SE3(70, 230, 35)* SE3.Ry(-np.pi)*SE3.Rz(np.pi/2), [-1, 1, 1])
        punto2 = RobTarget(SE3(130, 230, 35)* SE3.Ry(-np.pi)*SE3.Rz(np.pi/2), [-1, 1, 1])
        punto3 = RobTarget(SE3(70, 170, 35)* SE3.Ry(-np.pi)*SE3.Rz(np.pi/2), [-1, 1, 1])
        punto4 = RobTarget(SE3(130, 170, 35)* SE3.Ry(-np.pi)*SE3.Rz(np.pi/2), [-1, 1, 1])
        puntos = [punto_centro, punto1, punto2, punto3, punto4]

        robot.MoveJ(punto_centro.offset(0, 0, d_offset), 30, pinza_marker, robt_name='Centro')
        print(punto_centro.offset(0, 0, d_offset).pose)
        robot.GripperState(100, 30)
        time.sleep(7)
        robot.GripperState(0, 30)
        time.sleep(3)

        for idx, punto in enumerate(puntos[1:], 1):
            # robot.testPose(punto, pinza_marker, wobj)
            robot.MoveJ(punto.offset(0, 0, d_offset), 30, pinza_marker, wobj)
            robot.MoveC(punto, 30, pinza_marker, wobj, robt_name=f'punto{idx}')
            print(f'Llegamos al punto {idx}')
            data_reg(punto, pinza_marker, file_log)
            robot.MoveC(punto.offset(0, 0, d_offset), 30, pinza_marker, wobj)
            robot.MoveJ(punto_centro.offset(0, 0, d_offset), 30, pinza_marker, wobj, robt_name='Centro')
            data_reg(punto_centro, pinza_marker, file_log)
        
        robot.MoveJ(punto_centro.offset(0, 0, 20), 30, pinza_marker, wobj)
        print(f'Volvemos al centro y terminamos.')
        robot.GripperState(100, 30)
        time.sleep(7)
    
        # Volvemos a Home
        robot.MoveJAngles(np.zeros(6), 30)
        # robot.MoveJ(home, 30, SE3(), wobj)

    def ensayo3(iteracion = 0):
        file_log = f'ensayo3_v{iteracion}.txt'
        robot.GripperState(100, 30)
        time.sleep(7)
        robot.GripperState(0, 30)
        time.sleep(3)

        punto5 = RobTarget(SE3(150, -250, 50)* SE3.Ry(-np.pi)*SE3.Rz(np.pi), [-1, 1, 1])
        punto6 = RobTarget(SE3(100, -250, 50)* SE3.Ry(-np.pi)*SE3.Rz(np.pi), [-1, 1, 1])
        punto7 = RobTarget(SE3(100, -235, 50)* SE3.Ry(-np.pi)*SE3.Rz(np.pi), [-1, 1, 1])
        punto8 = RobTarget(SE3(150, -235, 50)* SE3.Ry(-np.pi)*SE3.Rz(np.pi), [-1, 1, 1])

        puntos = [punto5, punto6, punto7, punto8]

        # robot.MoveJ(punto5.offset(0, 0, 15), 30, pinza_marker, wobj)
        for punto in puntos:
            if punto == punto6 or punto == punto8:
                robot.MoveC(punto, 30, pinza_marker, wobj)
            else: 
                robot.MoveJ(punto.offset(0, 0, 15), 30, pinza_marker, wobj)
                time.sleep(2)
                robot.MoveJ(punto, 30, pinza_marker, wobj)
            data_reg(punto, pinza_marker, file_log)

        robot.MoveJ(punto8.offset(0, 0, 15), 30, pinza_marker, wobj)
        time.sleep(3)
        robot.GripperState(100, 30)
        time.sleep(2)
        robot.MoveJAngles(np.zeros(6), 30)
    
    def ensayo4(iteracion = 0):
        file_log = f'ensayo4_v{iteracion}.txt'
        robot.GripperState(100, 30)
        time.sleep(7)
        robot.GripperState(0, 30)
        time.sleep(3)

        punto1 = RobTarget(SE3(150, -150, 50)* SE3.Ry(-np.pi)*SE3.Rz(np.pi/2), [-1, 1, -1]) #[-1, 1, 1], [1, -1, -1]
        punto2 = RobTarget(SE3(150, -230, 50)* SE3.Ry(-np.pi)*SE3.Rz(np.pi/2), [-1, 1, -1]) 
        punto3 = RobTarget(SE3(135, -230, 50)* SE3.Ry(-np.pi)*SE3.Rz(np.pi/2), [-1, 1, -1]) 
        punto4 = RobTarget(SE3(135, -150, 50)* SE3.Ry(-np.pi)*SE3.Rz(np.pi/2), [-1, 1, -1])

        puntos = [punto1, punto2, punto3, punto4]
        for punto in puntos:
            if punto == punto2 or punto == punto4:
                robot.MoveC(punto, 15, pinza_marker, wobj)
            else:
                robot.MoveJ(punto.offset(0, 0, 5), 30, pinza_marker, wobj)
                time.sleep(2)
                robot.MoveJ(punto, 30, pinza_marker, wobj)
            data_reg(punto, pinza_marker, file_log)

        robot.MoveJ(punto4.offset(0, 0, 15), 30, pinza_marker, wobj)
        time.sleep(3)
        robot.GripperState(100, 30)
        time.sleep(2)
        robot.MoveJAngles(np.zeros(6), 30)

    ensayo4()

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
        setup_scene(robot_controller, 3)

    try:
        main(robot_controller)
    except Exception as e:
        print(f"Ocurrió un error durante la ejecución: {e}")
    finally:
        if robot_controller and hasattr(robot_controller, 'shutdown'):
            robot_controller.shutdown()
        print("--- Secuencia finalizada ---")