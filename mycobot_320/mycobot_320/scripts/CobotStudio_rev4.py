import numpy as np
from scipy.spatial.transform import Rotation as R
from scripts.DHRobotGT import myCobot320, IKineError
import time
import datetime
import threading
import itertools
from pymycobot import MyCobotSocket, MyCobot320
from scipy.spatial.transform import Rotation as R
from abc import ABC, abstractmethod
from spatialmath import SE3
from pynput import keyboard
from pathlib import Path
import importlib.util
import inspect
import sys

# Imports de ROS2
try:
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import TransformBroadcaster
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    from builtin_interfaces.msg import Duration
    from rclpy.executors import MultiThreadedExecutor
    from scripts.object_manager_rev1 import ObjectManager
    from control_msgs.action import FollowJointTrajectory
    ROS_OK = True
    print('>>> Librerías de ROS2 importadas correctamente. <<<')
except ImportError:
    ROS_OK = False
    print('>>> No se pudieron importar las librerías de ROS2. Funciones de visualización deshabilitadas. <<<')
    class Node: pass

# Si se importó correctamente lo referido a ROS2, buscamos MoveIt
if ROS_OK:
    try:
        from moveit_msgs.msg import DisplayRobotState, RobotState
        from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
        from sensor_msgs.msg import JointState
        MoveIt_OK = True
    except ImportError:
        MoveIt_OK = False

cobot_tb = myCobot320(rotar_base=True, metros=False)

joint_names = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3', 'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6', 'gripper_controller']

joint_limits = {
    "joint_1": (-168.0, 168.0),
    "joint_2": (-135.0, 135.0),
    "joint_3": (-150.0, 150.0),
    "joint_4": (-145.0, 145.0),
    "joint_5": (-165.0, 165.0),
    "joint_6": (-180.0, 180.0),
}

class RobTarget:
    def __init__(self, pose, config=None):
        """
        Args:
            pose: Matriz 4x4 (tipo numpy o SE3) que representa la pose cartesiana en mm.
            config: Lista con configuración articular, por ejemplo [1, -1, 1].
            ROS2: Si es True, convierte las posiciones de mm a m.
        """
        # Convertir SE3 a matriz si es necesario
        if isinstance(pose, SE3):
            self.pose = pose
        elif hasattr(pose, "shape") and pose.shape == (4, 4):
            self.pose = SE3(pose)
        else:
            raise ValueError("La pose debe ser una matriz 4x4 o un objeto SE3.")

        if config is not None:
            if isinstance(config, list) and all(isinstance(x, int) for x in config): # No hace falta forzar a que sea lista... calc_conf devuelve un np.array
                self.config = config
            else:
                raise ValueError("La configuración debe ser una lista como [1, -1, 1].")
        else:
            self.config = [1, 1, 1]  # Configuración por defecto
        
        self.valid_configs = []

    def find_valid_configs(self, tool, wobj):
        """
        Analiza todas las configuraciones posibles para un RobTarget con una determinada tool y wobj.
        Devuelve aquellas que tienen solución analítica, sin verificar colisiones o límites articulares.
        """
        T_global = wobj * self.pose * tool.inv()
        # Todas las combinaciones posibles de [±1, ±1, ±1]
        configs = list(itertools.product([1, -1], repeat=3))
        
        for conf in configs:
            try:
                solution = cobot_tb.ikine(T_global, conf)[0]
                if len(solution) > 0:
                    self.valid_configs.append(conf)
            except IKineError:
                # Si ocurre un error, simplemente continuamos
                continue
    
        if self.valid_configs:
            print("Configuraciones válidas:", ", ".join([str(c) for c in self.valid_configs]))
            return self.valid_configs
        else:
            print("Ninguna configuración resuelve la IK para esta pose.")
        
    def offset(self, dx=0, dy=0, dz=0, rx=0, ry=0, rz=0):
        """
        Aplica una rototraslación al RobTarget respecto al wobj.
        
        Args:
            dx, dy, dz: Desplazamientos en mm respecto al wobj.
            rx, ry, rz: Rotaciones en grados respecto al wobj (orden 'zyx').
            wobj: Objeto SE3 que representa el marco del wobj (default: identidad).

        Returns:
            Nuevo RobTarget desplazado y rotado.
        """
        # Vector de desplazamiento en el marco del wobj
        T_offset = SE3(dx, dy, dz) * SE3.RPY(rx, ry, rz, order='zyx', unit='deg')
        Robt_T = T_offset * self.pose.copy()

        return RobTarget(Robt_T, config=self.config.copy())
    
    def relTool(self, dx=0, dy=0, dz=0, rx=0, ry=0, rz=0):
        """
        Aplica una rototraslación al RobTarget respecto a sí mismo.
        
        Args:
            dx, dy, dz: Desplazamientos en mm respecto a la terna del RobTarget.
            rx, ry, rz: Rotaciones en grados respecto a la terna del RobTarget (orden 'zyx').
            wobj: Objeto SE3 que representa el marco del wobj (default: identidad).

        Returns:
            Nuevo RobTarget desplazado y rotado.
        """

        T_robt = SE3(dx, dy, dz) * SE3.RPY(rx, ry, rz, order='zyx', unit='deg')

        # Aplicar traslación y rotación respecto a la terna RobTarget
        Robt_T = self.pose * T_robt

        return RobTarget(Robt_T, config=self.config.copy())
    
    def __repr__(self):
        return f"RobTarget(pose=SE3({self.pose.t.tolist()}, rpy={self.pose.rpy(unit='deg').tolist()} deg), config={self.config})"

class BaseRobotController(ABC):
    def __init__(self, project_path):
        if project_path:
            self.project_root = Path(project_path).resolve()
        else:
            self.project_root = Path.cwd()
            
        print(f"[Init] Project Root: {self.project_root}")
    @abstractmethod
    def MoveJ(self, robt, speed:int = 30, tool=SE3(), wobj=SE3()):
        pass

    @abstractmethod
    def MoveC(self, robt, speed:int = 30, tool=SE3(), wobj=SE3()):
        pass

    @abstractmethod
    def GripperState(self, apertura: float, spd: int):
        pass

    def _resolve_project_path(self, subfolder: str, filename: str, create_dir: bool = False) -> Path:
        """
        Resuelve la ruta absoluta basándose en self.project_root.
        """
        target_folder = self.project_root / subfolder

        if create_dir:
            target_folder.mkdir(parents=True, exist_ok=True)
        if not filename.endswith(".py"):
            filename += ".py"
        return target_folder / filename
    
    def _load_project_variable(self, subfolder: str, filename: str, var_name: str, verbose: bool = False) -> any:       
        path = self._resolve_project_path(subfolder, filename, create_dir=False)
        
        if not path.exists():
             raise FileNotFoundError(f"Archivo no encontrado: {path}")
             
        spec = importlib.util.spec_from_file_location("mod_dyn", path)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        
        if not hasattr(module, var_name):
            raise AttributeError(f"Variable '{var_name}' no existe en {path.name}")
        
        val = getattr(module, var_name)

        # --- LOGGING ELEGANTE ---
        if verbose:
            # Intentamos mostrar la ruta relativa para que sea más legible
            try:
                display_path = path.relative_to(self.project_root)
            except ValueError:
                # Si por alguna razón el archivo está fuera del root, mostramos el nombre solo
                display_path = path.name
                
            # Log conciso: [Carga] Archivo -> Variable (Tipo)
            type_name = type(val).__name__
            info_extra = f" (Len: {len(val)})" if hasattr(val, '__len__') else ""
            
            print(f"[Load] {display_path} -> '{var_name}' <{type_name}>{info_extra}")
            
        return val

        
    def load_wobj(self, archivo: str, wobj_name: str, tool: SE3 = SE3(), auto_teach: bool = True, q_teach = None) -> SE3:
        """
        Intenta cargar un Wobj. Si falla y auto_teach es True, 
        ofrece al usuario enseñarlo en el momento.
        
        Args:
            archivo: Nombre del archivo .py (sin .py opcional).
            wobj_name: Nombre de la variable dentro del archivo.
            tool: Herramienta usada para enseñar (solo si falla la carga).
            auto_teach: Si True, habilita la pregunta interactiva ante error.
        """
        try:
            # Intentamos cargar usando tu lógica existente
            wobj = self._load_project_variable("Workobjects", archivo, wobj_name)
            
            # Validación de tipo
            if not isinstance(wobj, SE3):
                print(f"[Warn] '{wobj_name}' cargado, pero no parece ser un SE3.")
            else:
                print(f"[Info] Wobj '{wobj_name}' cargado correctamente.")
                
            return wobj

        except (FileNotFoundError, AttributeError) as e:
            # Si algo falló (no existe archivo o variable)
            print(f"\n[!] Error al cargar Wobj: {e}")
            
            if not auto_teach:
                raise e # Si desactivamos la ayuda, que falle normal.

            # Interacción con el usuario
            print(f"¿Deseas ejecutar la rutina de enseñanza para '{wobj_name}' ahora?")
            resp = input("Escribe 's' para enseñar, o cualquier tecla para cancelar: ").lower().strip()
            
            if resp == 's':
                print(f"--- Iniciando recuperación interactiva para {wobj_name} ---")
                new_wobj = self.teach_and_save_wobj(
                    filename=archivo, 
                    wobj_name=wobj_name, 
                    tool=tool, 
                    save_q=True,
                    q_test = q_teach
                )
                return new_wobj
            else:
                print("[!] Operación cancelada por el usuario.")
                raise e # Relanzamos el error original para detener el script

    def load_scene(self, scene_filename: str):
        """
        En modo Simulación: Carga y configura la escena desde un archivo .py.
        En modo Real: No hace nada (se ignora).
        """
        pass
    
    def load_data(self, subfolder: str, filename: str, var_name: str) -> any:
        """
        Carga una variable genérica desde un archivo del proyecto.
        Útil para cargar configuraciones articulares (Q), diccionarios, etc.
        """
        # Al llamar a _load desde aquí, restauramos la profundidad de pila esperada (3)
        return self._load_project_variable(subfolder, filename, var_name)
    
    def _save_aux_q_data(self, folder: str, base_filename: str, var_name: str, q_vals: list, extra_depth: int = 0):
        """Helper interno para guardar los valores articulares (Q) en un archivo separado."""
        name_q = base_filename.replace(".py", "") + "_q.py"
        path_q = self._resolve_project_path(folder, name_q) # Depth 4 pq está dentro de otro helper
        
        variable_name_q = f"{var_name}_q"
        mode = 'a' if path_q.exists() else 'w'
        
        try:
            with open(path_q, mode) as f:
                if mode == 'w':
                    f.write(f"# Config Articulares (Q) para {folder}\n\n")
                f.write(f"# Q Values para: {var_name}\n")
                f.write(f"{variable_name_q} = [\n")
                for row in q_vals:
                    clean_row = [float(x) for x in row]
                    f.write(f"    {clean_row},\n")
                f.write("]\n\n")
            print(f"[Guardado] Q Values -> {path_q.name}")
        except Exception as e:
            print(f"[Error] Falló guardado Q en {name_q}: {e}")

    def _save_tcp_definition(self, folder: str, filename: str, var_name: str, tcp_data: tuple, q_vals: list = None):
        """
        Guarda un TCP definido por vector de traslación + métricas.
        Genera un archivo .py que exporta un objeto SE3.Trans().
        """
        
        # Desempaquetado de tu tupla personalizada
        # p_tool_raw: calculado directo, p_tool_real: corregido con z_aux
        p_tool_raw, p_tool_real, residuals, ecm, rmse = tcp_data

        if not filename.endswith(".py"): filename += ".py"
        path_tcp = self._resolve_project_path(folder, filename, create_dir=True)
        mode = 'a' if path_tcp.exists() else 'w'

        try:
            with open(path_tcp, mode) as f:
                if mode == 'w':
                    f.write(f"# Archivo de definiciones TCP: {folder}\n")
                    f.write("from spatialmath import SE3\n")
                    f.write("import numpy as np\n\n")

                # Escribimos las métricas como comentarios para control de calidad
                f.write(f"# --- Registro: {var_name} ---\n")
                f.write(f"# Fecha: {datetime.datetime.now().isoformat()}\n")
                f.write(f"# RMSE (Error Medio Cuadrático): {rmse:.6f} mm\n")
                f.write(f"# ECM: {ecm:.6f}\n")
                if hasattr(residuals, 'tolist'): # Por si residuals es array o lista
                    f.write(f"# Residuales: {residuals.flatten().tolist()}\n")
                
                # Guardamos el vector crudo por si acaso
                raw_str = "[" + ", ".join([f"{val:.6f}" for val in p_tool_raw]) + "]"
                f.write(f"# Raw calc (sin corrección z_aux): {raw_str}\n")

                # --- LO IMPORTANTE: Guardar el TCP Real como SE3 ---
                # Usamos p_tool_real que es el que tiene la corrección de la pieza auxiliar
                t_str = "[" + ", ".join([f"{val:.6f}" for val in p_tool_real]) + "]"
                
                f.write(f"{var_name}_t = np.array({t_str})\n")
                # Creamos un SE3 de pura traslación (identidad en rotación)
                f.write(f"{var_name} = SE3.Trans({var_name}_t)\n\n")

            print(f"[Guardado] TCP {var_name} (RMSE: {rmse:.4f}) -> {path_tcp.name}")

        except Exception as e:
            print(f"[Error] Falló guardado TCP en {filename}: {e}")

        # Reutilizamos la lógica para guardar Q
        if q_vals:
            self._save_aux_q_data(folder, filename, var_name, q_vals)

    def _save_se3_definition(self, folder: str, filename: str, var_name: str, se3_obj: SE3, q_vals: list = None):
            path_se3 = self._resolve_project_path(folder, filename, create_dir=True)
            mode = 'a' if path_se3.exists() else 'w'
            
            try:
                with open(path_se3, mode) as f:
                    # Escribir cabecera solo si es archivo nuevo
                    if mode == 'w':
                        f.write(f"# Archivo de definiciones: {folder}\n")
                        f.write("from spatialmath import SE3\n")
                        f.write("import numpy as np\n\n")
                    
                    # Formateo de alta precisión (Tu lógica intacta)
                    R_str = "[\n" + ",\n".join(["    [" + ", ".join([f"{val:.18e}" for val in row]) + "]" for row in se3_obj.R]) + "\n  ]"
                    t_str = "[" + ", ".join([f"{val:.18e}" for val in se3_obj.t.flatten()]) + "]"
                    
                    f.write(f"# Guardado: {datetime.datetime.now().isoformat()}\n")
                    f.write(f"{var_name}_R = np.array({R_str})\n")
                    f.write(f"{var_name}_t = np.array({t_str})\n")
                    f.write(f"{var_name} = SE3.Rt({var_name}_R, {var_name}_t)\n\n")
                print(f"[Guardado] {folder}/{var_name} -> {path_se3.name}")
            except Exception as e:
                print(f"[Error] Falló guardado SE3: {e}")

            # Llamada al helper compartido
            if q_vals:
                self._save_aux_q_data(folder, filename, var_name, q_vals)

    def teach_and_save_wobj(self, filename: str, wobj_name: str, tool: SE3 = SE3(), save_q: bool = False, q_test: list = None) -> SE3:
        """
        Enseña un Wobj, lo guarda en 'Workobjects/filename.py'.
        Opcionalmente guarda los q leídos por el cobot en 'Workobjects/filename_q.py'.
        """
        # --- 1. ENSEÑANZA FÍSICA ---
        print("\n" + "="*60)
        print(f"  ENSEÑANZA DE WOBJ: {wobj_name}")
        print("="*60 + "\n")
        
        # q_vals es una lista de 6 listas (o arrays)
        q_vals, _ = self.grabar_poses(6, ajuste=True, q_list = q_test)
        
        print(f"\n[Calculando] Procesando geometría...")
        wobj_calculated = teach_wobj(q_vals, tool)
        print(f"[Resultado] Wobj:\n{wobj_calculated}")
        # 2. Delegar guardado
        qs_to_save = q_vals if save_q else None
        
        self._save_se3_definition(
            folder="Workobjects",
            filename=filename,
            var_name=wobj_name,
            se3_obj=wobj_calculated,
            q_vals=qs_to_save
            )

        return wobj_calculated

    def teach_and_save_TCP(self, filename: str, tcp_name: str, save_q: bool = False, q_test: list = None) -> SE3:
        """
        Enseña un TCP (4 puntos) y lo guarda en carpeta TCPs.
        """
        print("\n" + "="*60)
        print(f"  ENSEÑANZA DE TCP: {tcp_name}")
        print("="*60 + "\n")

        # 1. Obtener datos físicos (4 puntos para TCP pivot)
        q_vals, _ = self.grabar_poses(4, ajuste=True, q_list = q_test)

        print(f"\n[Calculando] Procesando geometría TCP...")
        # Asumo que tienes una función teach_tcp(q_vals)
        tcp_result_tuple = TCP_4puntos_extendido(q_vals)
        
        # Desempaquetamos solo lo necesario para el print y el return
        _, p_tool_real, _, _, rmse = tcp_result_tuple

        print(f"[Resultado] TCP (Traslación): {p_tool_real}")
        print(f"[Calidad]   RMSE: {rmse:.5f}")

        # 2. Delegar guardado al helper ESPECÍFICO para TCP
        qs_to_save = q_vals if save_q else None

        self._save_tcp_definition(
            folder="TCPs",
            filename=filename,
            var_name=tcp_name,
            tcp_data=tcp_result_tuple,
            q_vals=qs_to_save
        )

        # 3. Retornar un objeto SE3 útil para el programa en ejecución
        # Así puedes hacer: tool = sim.teach_and_save_TCP(...) -> sim.MoveJ(..., tool=tool)
        return SE3.Trans(p_tool_real)

    @abstractmethod
    def grabar_poses(self, cantidad, ajuste=False, q_list=None):
        """
        Cada hijo DEBE implementar cómo conseguir las poses.
        """
        pass

if ROS_OK:
    class joint_pub(Node):
        def __init__(self):
            super().__init__('joint_state_publisher')
            self.publisher = self.create_publisher(JointState, '/joint_states', 10)

            # Subscripción para leer estado actual
            self.create_subscription(
                JointState,
                '/joint_states',
                self.joint_state_callback,
                10
            )

            # Estado actual
            self.q_current = None
            self.q_current_time = None

        def joint_state_callback(self, msg):
            # print(f"[DEBUG] JointState recibido: {msg.position}")
            self.last_joint_msg = msg
            position_map = {name: pos for name, pos in zip(msg.name, msg.position)}
            ordered_positions = [position_map.get(name, 0.0) for name in joint_names]
            self.q_current = np.array(ordered_positions)
            # self.q_current = np.array(msg.position)
            self.q_current_time = msg.header.stamp
        
        def publish_trajectory(self, trajectory, joint_names, dt=1):
            """
            :param trajectory: Trayectoria a publicar
            :param joint_names: Nombre de las articulaciones. Verificar que coincida con la definición del robot (.urdf).
            :param dt: Tiempo de espera entre la publicación de cada pose de la trayectoria.
            """
            for q in trajectory:
                msg = JointState()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.name = joint_names
                # msg.position = pad_for_urdf(q)
                msg.position = _normalize_q(q).tolist()
                self.publisher.publish(msg)
                # self.get_logger().info(f'q: {msg.position}')
                time.sleep(dt)

    class TFPublisher(Node):
        # NOTA: RViz mantiene listados todos los frames que escuchó al menos una vez.
        # Si se eliminan ternas de este script, puede que sigan apareciendo en la pestaña TF de RViz.
        # Esto es solo visual; el frame ya no se publica ni influye en la escena.
        # Para eliminar ternas obsoletas se debe reiniciar RViz. Esto se facilita con el script reset_scene.

        def __init__(self):
            super().__init__('multi_tf_publisher')
            self.br = TransformBroadcaster(self)
            self.timer = self.create_timer(0.1, self.publish_all_transforms)
            
            # Diccionario para almacenar las ternas: {frame_name: SE3}
            self.transforms = {}

        def add_wobj(self, transform: SE3, name: str):
            """Agrega un workobject. Se asume referido a la terna base."""
            wobj_m = SE3(np.array(transform))
            wobj_m.t = wobj_m.t / 1000.0
            self.transforms[name] = (wobj_m, 'base')

        def add_robt(self, transform: SE3, name: str, reference_frame: str):
            """Agrega un robtarget referido a otra terna: puede ser 'base' o un wobj, por ejemplo."""

            # self.get_logger().info(f"--- TFPublisher.add_robt() fue llamada con name = '{name}' ---")
            robt_m = SE3(np.array(transform))
            robt_m.t = robt_m.t / 1000.0
            self.transforms[name] = (robt_m, reference_frame)

        def publish_all_transforms(self):
            now = self.get_clock().now().to_msg()

            for child, (se3, parent) in self.transforms.items():
                t = TransformStamped()
                t.header.stamp = now
                t.header.frame_id = parent
                t.child_frame_id = child

                pos = se3.t
                rot = se3.R
                quat = R.from_matrix(rot).as_quat()

                t.transform.translation.x = pos[0]
                t.transform.translation.y = pos[1]
                t.transform.translation.z = pos[2]
                t.transform.rotation.x = quat[0]
                t.transform.rotation.y = quat[1]
                t.transform.rotation.z = quat[2]
                t.transform.rotation.w = quat[3]

                self.br.sendTransform(t)
        
        # Puede ser útil a futuro. Provoca un salto en la terna que se puede solucionar congelándola en la posición actual y 
        # cambiándole el parent a 'base' para que no se intente actualizar con el movimiento de la brida.
        def remove_transform(self, name: str): 
            """
            Elimina una terna específica del diccionario para que deje de publicarse.
            """
            if name in self.transforms:
                del self.transforms[name]
                self.get_logger().info(f"Terna '{name}' eliminada de la publicación.")
            else:
                self.get_logger().warn(f"Se intentó eliminar la terna '{name}', pero no existía.")

    class RobotStateVisualizer(Node):
        """
        Un nodo simple cuya única responsabilidad es publicar estados articulares
        en el topic /display_robot_state para que MoveIt/RViz los muestren.
        """
        def __init__(self, joint_names_ordered):
            super().__init__('robot_state_visualizer')
            self.publisher = self.create_publisher(DisplayRobotState, '/display_robot_state', 10)
            self.ARM_JOINT_NAMES = joint_names_ordered
            self.get_logger().info("Visualizador de estado del robot listo.")

        def publish_pose(self, joint_positions):
            """ Publica una pose articular para que el goal state de MotionPlanning (robot naranja) salte a ella. """
            if len(joint_positions) != len(self.ARM_JOINT_NAMES):
                self.get_logger().error(f"Error: Se esperaban {len(self.ARM_JOINT_NAMES)} posiciones, pero se recibieron {len(joint_positions)}.")
                return

            # Crear el mensaje JointState
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.name = self.ARM_JOINT_NAMES
            joint_state_msg.position = [float(q) for q in joint_positions]

            # Colocarlo en un mensaje RobotState
            robot_state_msg = RobotState()
            robot_state_msg.joint_state = joint_state_msg

            # Armado del mensaje final DisplayRobotState
            display_robot_state_msg = DisplayRobotState()
            display_robot_state_msg.state = robot_state_msg

            # Publicación
            self.publisher.publish(display_robot_state_msg)
            self.get_logger().info(f"Publicando estado de visualización para el robot naranja.")

    class SimManager(BaseRobotController):
        def __init__(self, project_path=None):
            super().__init__(project_path=project_path)
            # Inicializamos rclpy solo una vez
            rclpy.init()
            self.node_tf = TFPublisher()
            self.node_joint = joint_pub()
            self.node_obj = ObjectManager()

            self.ARM_JOINT_NAMES = joint_names[:6]
            self.ARM_JOINT_NAMES_FULL = joint_names

            from scripts.MoveItAdapter import MoveItAdapter
            self.moveit_adapter = MoveItAdapter(project_path=self.project_root)
            self.moveit_adapter.simmanager_ref = self

            # Publicamos q = 0 inicial
            msg = JointState()
            msg.name = joint_names
            msg.position = [0.0] * len(joint_names)
            self.node_joint.publisher.publish(msg)

            # Executor multithread para los nodos
            self.executor = MultiThreadedExecutor()
            self.executor.add_node(self.node_tf)
            self.executor.add_node(self.node_joint)
            self.executor.add_node(self.node_obj)
            self.executor.add_node(self.moveit_adapter)

            # Arrancamos el executor en un hilo aparte
            self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
            self.executor_thread.start()

            self.ARM_JOINT_NAMES = [
            'joint2_to_joint1', 
            'joint3_to_joint2', 
            'joint4_to_joint3', 
            'joint5_to_joint4', 
            'joint6_to_joint5', 
            'joint6output_to_joint6'
        ]

            while self.node_joint.q_current is None:
                print(">>> Esperando joint_states...")
                time.sleep(0.1)

            self.q_current = self.node_joint.q_current

            print(">>> SimManager iniciado")

        def MoveJ(self, robtarget, speed: int = 30, tool: SE3 = SE3(), wobj: SE3 = SE3(), wobj_name='wobj1', robt_name='robtarget'):
            self.node_tf.add_wobj(wobj, wobj_name)
            self.node_tf.add_robt(robtarget.pose, robt_name, wobj_name)
            self.node_tf.add_robt(tool, 'tool', 'link6')


            # La cinemática inversa devuelve 6 ejes, sin la pinza
            pose_calc = wobj * robtarget.pose * tool.inv()
            q_goal_arm = cobot_tb.ikine(pose_calc, robtarget.config)[0] # Devuelve 6 ejes

            # 3. Planificación
            # Usamos _normalize_q para garantizar que q_current sea array y tomar solo los 6 primeros
            q_start_arm = self._normalize_q(self.q_current)[:6]
            
            # Generar puntos clave (Start -> End -> End)
            traj_keypoints = np.vstack([q_start_arm, q_goal_arm, q_goal_arm])
            cobot_tb.genTrJoint(traj_keypoints, np.zeros(traj_keypoints.shape[0]))

            # Subsampling de la trayectoria (solo brazo)
            traj_arm_subsampled = cobot_tb.q_ref[::15]

            # 4. Ejecución (Delegada al helper maestro)
            self._execute_arm_trajectory(traj_arm_subsampled, robt_name, wobj_name)
            # q_brazo = cobot_tb.ikine(pose_calc, robtarget.config)[0]

            # q_start = self.node_joint.q_current

            # # Respetar la pinza actual
            # gripper_val = self.q_current[6] if self.q_current is not None else 0.0
            # q_goal = np.concatenate([np.array(q_brazo), [gripper_val]])
            # q_start = to_array(self.q_current)[:7]   # <--- forzar 7
            # q_end = to_array(q_goal)[:7] 

            # traj_q = np.vstack([q_start, q_end, q_end])

            # traj_arm = traj_q[:, :6]
            # cobot_tb.genTrJoint(traj_arm, np.zeros(traj_arm.shape[0]))

            # qtraj_a = cobot_tb.q_ref[::15]
            # q_limit = check_joint_limits(np.rad2deg(qtraj_a), joint_limits)
            # if q_limit:
            #     if len(q_limit) == 1:
            #         print(f"⚠️  Valor fuera de límite para el eje {q_limit[0]}")
            #     else:
            #         ejes = ", ".join(map(str, q_limit))
            #         print(f"⚠️  Se sobrepasan límites en los ejes: {ejes}")

            # qtraj_a_full = [np.concatenate([q, [gripper_val]]) for q in qtraj_a]

            # self._send_move(qtraj_a_full, robt_name, wobj_name)

        def MoveC(self, robtarget, speed: int = 30, tool: SE3 = SE3(), wobj: SE3 = SE3(), wobj_name='wobj1', robt_name='robtarget'):
            self.node_tf.add_wobj(wobj, wobj_name)
            self.node_tf.add_robt(robtarget.pose, robt_name, wobj_name)
            # self.node_tf.add_robt(tool, 'TCP', 'link6')
            self.node_tf.add_robt(tool, 'tool', 'link6')
            print(f'DEBUG post ternas')
            pose_goal = wobj * robtarget.pose * tool.inv()
            q_start_arm = self._normalize_q(self.q_current)[:6]
            pose_start = cobot_tb.fkine(q_start_arm)

            print(f'DEBUG Pre generación')
            # Generación Cartesiana
            cobot_tb.genTrCart([pose_start, pose_goal, pose_goal], 0*np.ones(3), conf=robtarget.config)
            traj_arm_subsampled = cobot_tb.q_ref[::10]
            print(f'DEBUG Post generación')

            self._execute_arm_trajectory(traj_arm_subsampled, robt_name, wobj_name)
            # pose_start = cobot_tb.fkine(self.q_current.copy()[:6])
            # q_gripper = self.q_current[6] if self.q_current is not None else 0.0
            # # print(f'Pose start:\n{pose_start}')
            # # print(f'Pose goal:\n{pose_goal}')

            # cobot_tb.genTrCart([pose_start, pose_goal, pose_goal], 0*np.ones(3), conf = robtarget.config)
            # qtraj_a = cobot_tb.q_ref[::10]

            # q_limit = check_joint_limits(np.rad2deg(qtraj_a), joint_limits)
            # if q_limit:
            #     if len(q_limit) == 1:
            #         print(f"⚠️  Valor fuera de límite para el eje {q_limit[0]}")
            #     else:
            #         ejes = ", ".join(map(str, q_limit))
            #         print(f"⚠️  Se sobrepasan límites en los ejes: {ejes}")
            # qtraj_a_full = [np.concatenate([q, [q_gripper]]) for q in qtraj_a]

            # self._send_move(qtraj_a_full, robt_name, wobj_name)
            # print(f'Llegamos a:\n{cobot_tb.fkine(self.q_current.copy()[:6])}')
        
        def MoveJAngles(self, q, spd = 30, unit = 'deg'):
            """
            Envía al robot al vector de variables articulares pedido.

            Parameters
            ----------
                q : Array(1,6)
                    Vector de variables articulares
                spd : int (1 - 100)
                    Velocidad 
                unit : str
                    Unidad de las variables articulares. Puede ser 'rad' o 'deg'.
            """
            if unit == 'deg':
                q_brazo = np.deg2rad(q).tolist()
            elif unit == 'rad':
                q_brazo = q

            q_start_arm = self._normalize_q(self.q_current)[:6]
        
            traj_keypoints = np.vstack([q_start_arm, q_brazo, q_brazo])
            cobot_tb.genTrJoint(traj_keypoints, np.zeros(traj_keypoints.shape[0]))
            
            traj_arm_subsampled = cobot_tb.q_ref[::10]

            self._execute_arm_trajectory(traj_arm_subsampled, "MoveJAngles", "JointSpace")
            # # Respetar la pinza actual
            # gripper_val = self.q_current[6] if self.q_current is not None else 0.0
            # q_goal = np.concatenate([np.array(q_brazo), [gripper_val]])
            # q_start = to_array(self.q_current)[:7]
            # q_end = to_array(q_goal)[:7]

            # traj_q = np.vstack([q_start, q_end, q_end])

            # traj_arm = traj_q[:, :6]
            # cobot_tb.genTrJoint(traj_arm, np.zeros(traj_arm.shape[0]))

            # qtraj_a = cobot_tb.q_ref[::10]
            # q_limit = check_joint_limits(np.rad2deg(qtraj_a), joint_limits)
            # if q_limit:
            #     if len(q_limit) == 1:
            #         print(f"⚠️  Valor fuera de límite para el eje {q_limit[0]}")
            #     else:
            #         ejes = ", ".join(map(str, q_limit))
            #         print(f"⚠️  Se sobrepasan límites en los ejes: {ejes}")

            # qtraj_a_full = [np.concatenate([q, [gripper_val]]) for q in qtraj_a]

            # self._send_move(qtraj_a_full)
        
        def MostrarTerna(self, terna, nombre='terna1', ref = SE3()):
            """
            Muestra en RViz una terna dada por un objeto SE3 definida respecto a `ref`.
            """
            self.node_tf.add_wobj(ref*terna, nombre)

        def VerPose(self, robtarget, tool: SE3 | None = SE3(), wobj: SE3 = SE3(), wobj_name='wobj1', robt_name='robt1'):
            # gripper_val = self.q_current[6] if self.q_current is not None else 0.0
            pose = wobj * robtarget.pose * tool.inv()
            config = robtarget.config
            try:
                q_sol = cobot_tb.ikine(pose, config)[0]
            except IKineError as e:
                print("Error en el problema inverso:", e)
                return
            
            # q_full = np.concatenate([q, [gripper_val]])

            self.node_tf.add_wobj(wobj, wobj_name)
            self.node_tf.add_robt(robtarget.pose, robt_name, wobj_name)
            q_arm_target = np.array([q_sol]) 
        
            self._execute_arm_trajectory(q_arm_target, robt_name, wobj_name)
            time.sleep(0.1)

            # self._send_move([q_full], robt_name, wobj_name)
        
        def VerQ(self, q, tool: SE3 | None = SE3(), brida = False):
            # gripper_val = self.q_current[6] if self.q_current is not None else 0.0
            # q_full = np.concatenate([q, [gripper_val]])

            q_input = _normalize_q(q)
            q_arm = q_input[:6]

            if brida: self.node_tf.add_robt(cobot_tb.fkine(q), 'brida', 'base')
            self.node_tf.add_robt(tool, 'tool', 'link6')
            time.sleep(0.1)

            q_arm_target = np.array([q_arm])
            self._execute_arm_trajectory(q_arm_target, "VerQ", "JointSpace")
            # self._send_move([q_full])

        def _send_move(self, qtraj_a, robt_name = 'robt', wobj_name = 'wobj', dt=0.1):
            print(f">>> Move: {robt_name} @ {wobj_name}")

            self.node_joint.publish_trajectory(qtraj_a, joint_names, dt)

            if len(qtraj_a) > 0:
                self.q_current = qtraj_a[-1].copy()

            # print(f'Trayectoria finalizada. q_current =\n{self.q_current}')
        
        def get_current_q(self, prefer_gripper=False, timeout=2.0, info=False, tool = 1):
            """Devuelve q_current ordenado como np.array o None si timeout. Útil cuando se mueve al robot con MotionPlanning."""
            t0 = time.time()
            target_len = len(self.ARM_JOINT_NAMES) + (1 if prefer_gripper else 0)  # si ARM_JOINT_NAMES=6
            while time.time() - t0 < timeout:
                # preferimos el mensaje crudo (tiene los names)
                last_msg = getattr(self.node_joint, "last_joint_msg", None)
                if last_msg is not None and hasattr(last_msg, "name"):
                    position_map = {n: p for n, p in zip(last_msg.name, last_msg.position)}
                    if prefer_gripper and hasattr(self, "JOINT_NAMES_FULL"):
                        names = self.JOINT_NAMES_FULL
                    else:
                        names = self.ARM_JOINT_NAMES
                    ordered = [position_map.get(n, 0.0) for n in names]

                    if info:
                        robt = robt_from_q(ordered[:6], tool)
                        print(f'robt = {robt}')
                        return np.array(list(ordered)[:len(self.ARM_JOINT_NAMES)]), robt
                    return np.array(ordered)
                
                q = getattr(self.node_joint, "q_current", None)
                if q is not None:
                    if prefer_gripper:
                        # devolver hasta 7
                        return np.array(list(q)[:target_len]) 
                    else:                      
                        return np.array(list(q)[:len(self.ARM_JOINT_NAMES)])
                time.sleep(0.05)
            # timeout
            self.node_joint.get_logger().warning("get_current_q: timeout esperando joint_states")
            return None

        def GripperState(self, apertura: float, spd: int = 30):
            """
            Mueve la pinza a una apertura específica.
            
            Args:
                apertura: Apertura porcentual de la pinza.
            """
            # self.node_tf.add_robt(tool, 'tool', 'link6')
            if not (0 <= apertura <= 100):
                raise ValueError("La apertura debe estar entre 0 y 100%.")
            
            # Copiar el estado actual para modificar solo gripper_controller
            # q_actual = to_array(self.q_current)

            # Mapear apertura [0,100] a rango [-0.7, 0.3]
            # q_actual[6] = apertura / 100 - 0.7
            val_gripper = apertura / 100 - 0.7
            # msg = JointState()
            # msg.name = joint_names
            # self._send_move([q_actual])

            q_full = self._normalize_q(self.q_current)
            q_full[6] = val_gripper # Índice 6

            # Enviar como movimiento de un solo punto
            self._send_move([q_full], "Gripper", "Tool")
        
        def testPose(self, robt, tool, wobj):
            confs = robt.find_valid_configs(tool, wobj)
            last_conf = None

            while True:
                print("\n--- Menú de configuraciones ---")
                for i, conf in enumerate(confs, start = 1):
                    marker = " *" if conf == last_conf else ""
                    print(f"{i}: {conf}{marker}")

                print("\nSeleccione un número para ver la configuración.")
                print("Presione 'q' para salir.\n")

                user_in = input("Opción: ")

                if user_in.lower() == 'q':   # salir del menú
                    if last_conf:
                        print(f"Saliendo del menú. La última configuración seleccionada fue: {last_conf}")
                    else:
                        print("Saliendo del menú. No se seleccionó ninguna configuración.")
                    break

                try:
                    idx = int(user_in)
                    if 1 <= idx <= len(confs):
                        last_conf = confs[idx - 1]
                        print(f"Configuración seleccionada: {last_conf}")

                        robt_sel = RobTarget(robt.pose, list(last_conf))
                        self.VerPose(wobj, robt_sel, tool)

                    else:
                        print("Índice fuera de rango, intente de nuevo.")
                except ValueError:
                    print("Entrada inválida, use un número o 'q' para salir.")

        def explore_ik_configs(self, robt, tool, wobj, ver_todo=True):
            """
            Recorre todas las configuraciones IK de robt.find_valid_configs(),
            evalúa colisiones y muestra un menú interactivo.

            modo:
                - "filtrar": solo configuraciones sin colisión
                - "ver_todo": todas (marcando colisión / sin colisión)
            """
            configs = robt.find_valid_configs(tool, wobj)
            if not configs:
                print("No se encontraron configuraciones IK.")
                return

            print(f"Se encontraron {len(configs)} configuraciones IK.")

            T_global = wobj * robt.pose * tool.inv()
            results = []   # (conf, q_sol_list, collision_free:bool)

            # obtener valor actual del gripper si existe
            cur_q = None
            try:
                cur_q = self.get_current_q(timeout=0.5)
            except Exception:
                cur_q = None
            # Si no existe se asume nulo
            gripper_default = float(cur_q[6]) if (cur_q is not None and len(cur_q) > 6) else 0.0

            # Recorremos las configuraciones encontradas analíticamente:
            for conf in configs:
                # Obtenemos q para cada conf
                try:
                    ik_res = cobot_tb.ikine(T_global, conf)  # devuelve (q, status)
                    q_sol = ik_res[0] if isinstance(ik_res, (list, tuple)) else ik_res
                    q_sol = np.asarray(q_sol).ravel().astype(float).tolist()
                except Exception as e:
                    # No debería entrar nunca, las configuraciones recorridas son sólo las válidas
                    print(f"IK fallo para conf {conf}: {e}")
                    continue

                # Asegurar longitud: q_sol no incluye el gripper
                q_for_check = q_sol + [gripper_default]

                # Chequear colisión con MoveItAdapter
                try:
                    if hasattr(self.moveit_adapter, "check_collision"):
                        collision_free = bool(self.moveit_adapter.check_collision(q_for_check))
                    else:
                        # fallback: usar método mínimo que devuelva True/False
                        collision_free = True
                        self.get_logger().warn("moveit_adapter.check_collision no disponible: asumiendo libre")
                except Exception as e:
                    print(f"Error al chequear colisión para {conf}: {e}")
                    collision_free = False

                results.append((conf, q_for_check, collision_free))

                # actualizar robot naranja si corresponde
                if ver_todo or collision_free:
                    # llamar apply_goal_state con listas en el formato esperado
                    ok = self.moveit_adapter.apply_goal_state(q_for_check)
                    if not ok:
                        print(f"Warning: apply_goal_state devolvió False para conf {conf}")
                    else:
                        self.moveit_adapter.trigger_rviz_update_goal()
                    print(f"Configuración {conf} → {'✅ Libre' if collision_free else '❌ En colisión'}")

            # Filtrar según ver_todo
            display_list = results if ver_todo else [r for r in results if r[2]]

            if not display_list:
                print("No hay configuraciones para mostrar (filtrado eliminó todo).")
                return

            last_conf = None
            while True:
                print("\n--- Menú de configuraciones ---")
                for i, (conf, q, ok) in enumerate(display_list, start=1):
                    mark = " *" if conf == last_conf else ""
                    estado = "Libre" if ok else "En colisión"
                    print(f"{i}: {conf}  ({estado}){mark}")

                print("\nSeleccione un número para visualizar la configuración o 'q' para salir.\n")
                user_in = input("Opción: ").strip()
                if user_in.lower() == 'q':
                    print("Saliendo del menú.")
                    break

                # validación más robusta: aceptar solo enteros dentro del rango
                if not user_in:
                    print("Entrada vacía. Ingrese un número o 'q'.")
                    continue
                try:
                    idx = int(user_in)
                except ValueError:
                    print(f"Entrada inválida (no es número): {repr(user_in)}")
                    continue

                if 1 <= idx <= len(display_list):
                    conf, q_sel, ok = display_list[idx - 1]
                    last_conf = conf
                    print(f"Configuración seleccionada: {conf} → {'✅ Libre' if ok else '❌ En colisión'}")
                    # mostrar en RViz
                    ok_apply = self.moveit_adapter.apply_goal_state(q_sel)
                    if not ok_apply:
                        print("apply_goal_state falló (ver logs).")
                else:
                    print("Índice fuera de rango. Intente nuevamente.")

        def OcultarTerna(self, terna = 'tool'):
            """Limpia específicamente la terna 'tool' de la visualización."""
            self.node_tf.remove_transform(terna)

        def publish_goal_tf(self, q, tool, frame_name='moveit_goal', reference_frame='base'):
            """
            Publica el robtarget (pose del TCP) correspondiente a q como un frame TF llamado frame_name.
            q: lista/np.array (6 o 7 elementos). Si tiene 7, se usa solo 6 primeros para FK.
            """
            # Tomar los primeros 6 q si hay 7 (gripper)
            q6 = list(q)[:6]

            # Calculá FK con el robot model (cobot_tb) -> SE3
            T = cobot_tb.fkine(q6) * tool 
            self.node_tf.add_robt(T, frame_name, reference_frame)

        def traj_moveit(self, archivo: str, traj_name: str = "TRAJ", tool = SE3()):
            """
            Ejecuta una trayectoria guardada en un archivo .py.
            
            Args:
                archivo (str): Nombre del archivo .py (ej: 'movimientos_llavero')
                traj_name (str): Nombre de la variable dentro del archivo (ej: 'PICK_UP')
                tool (SE3): Herramienta a usar.
            """
            # Recuperar la lista de puntos
            trajectory_points =  self._load_project_variable("Trayectorias", archivo, traj_name, verbose=True)
            first_len = len(trajectory_points[0])
            if first_len not in (6, 7):
                raise ValueError(f"Dimensión inesperada en el primer punto: {first_len} (se espera 6 o 7)")

            out = []
            for i, pt in enumerate(trajectory_points):
                # Chequeamos que todos los puntos tengan la misma dimensión, ya sea 6 o 7.
                if len(pt) != first_len:
                    raise ValueError(f"Punto {i} tiene longitud {len(pt)} incompatible con el primer punto.")

                arr6 = np.asarray(pt)
                # convertir a lista de floats puros (send_angles suele esperar lista de python floats)
                out.append([float(x) for x in arr6])
            
            if first_len == 7:
                # Si los puntos tienen dimensión 7 entonces también se mueve el gripper.
                self._send_move(trajectory_points, f'{traj_name}', 'Moveit')
            else:
                # Si los puntos son de dimensión 6, respetamos la pinza actual.
                self._execute_arm_trajectory(np.asarray(trajectory_points), f'{traj_name}', 'Moveit')

            # armamos el RobTarget para devolverlo
            last_q = trajectory_points[-1][:6]
            last_robt = robt_from_q(last_q, tool)

            return last_robt
        
        def load_scene(self, scene_filename: str):
            """
            Busca el archivo de escena en la misma carpeta que la rutina,
            importa 'setup_scene' y la ejecuta.
            """
            print(f"[Sim] Intentando cargar escena: {scene_filename}...")
            
            try:
                # 1. Usamos el helper que ya creamos.
                # subfolder="" significa "la misma carpeta donde está la rutina".
                # Buscamos la función "setup_scene" dentro de ese archivo.
                setup_func = self._load_project_variable(
                    subfolder="", 
                    filename=scene_filename, 
                    var_name="setup_scene"
                )
                
                # 2. Ejecutamos la función pasando este robot
                setup_func(self)
                print(f"[Sim] Escena '{scene_filename}' cargada correctamente.")
                
            except FileNotFoundError:
                print(f"[Sim] Advertencia: No se encontró el archivo de escena '{scene_filename}'.")
            except AttributeError:
                print(f"[Sim] Advertencia: El archivo '{scene_filename}' no tiene una función 'setup_scene(robot)'.")
            except Exception as e:
                print(f"[Sim] Error cargando escena: {e}")

        def _check_limits(self, trajectory_arm: np.ndarray):
            """Verifica límites solo para el brazo y notifica."""
            # Asumiendo que check_joint_limits y joint_limits existen en tu contexto global o self
            q_limit = check_joint_limits(np.rad2deg(trajectory_arm), joint_limits)
            if q_limit:
                if len(q_limit) == 1:
                    print(f"⚠️  Valor fuera de límite para el eje {q_limit[0]}")
                else:
                    ejes = ", ".join(map(str, q_limit))
                    print(f"⚠️  Se sobrepasan límites en los ejes: {ejes}")

        def _execute_arm_trajectory(self, trajectory_arm_6dof: np.ndarray, robt_name: str, wobj_name: str):
            """
            HELPER MAESTRO:
            1. Recibe trayectoria solo del brazo (N, 6).
            2. Verifica límites.
            3. Le pega el valor actual del gripper a toda la trayectoria (N, 7).
            4. Envía a ejecutar.
            """
            # 1. Validar límites
            self._check_limits(trajectory_arm_6dof)

            # 2. Obtener valor actual del gripper (índice 6)
            # Si q_current no está inicializado, asumimos 0.0 (o GRIPPER_OPEN)
            gripper_val = self.q_current[6] if self.q_current is not None else 0.0

            # 3. Vectorización: Crear columna de gripper y unir
            # (N, 6) -> (N, 7) de forma eficiente con NumPy
            filas = trajectory_arm_6dof.shape[0]
            columna_gripper = np.full((filas, 1), gripper_val)
            
            trajectory_full = np.hstack([trajectory_arm_6dof, columna_gripper])

            # 4. Enviar
            self._send_move(trajectory_full, robt_name, wobj_name)

        def _normalize_q(self, q) -> np.ndarray:
            """
            Asegura que el vector 'q' tenga siempre 7 elementos (float).
            Recorta si sobran, rellena con ceros si faltan.
            """
            q = np.array(q, dtype=float).flatten()
            if q.size == 7:
                return q
            elif q.size > 7:
                return q[:7]
            else:
                # Rellenar con ceros al final
                return np.pad(q, (0, 7 - q.size), 'constant')

        def grabar_poses(self, cantidad, ajuste=False, q_list = None):
            """
            Se debe pasar una lista en grados para lograr compatibilidad con métodos matemáticos. Si los datos obtenidos provienen de la función `grabar_poses` del cobot, ya se encuentran en grados.
            """
            print("Modo SIM: Usando lista de vectores articulares...")
            if len (q_list) != cantidad:
                raise ValueError("La cantidad de vectores articulares no coincide con la requerida por el método.")
            return q_list, 0

        def shutdown(self):
            print(">>> Apagando SimManager...")
            self.executor.shutdown()

            if self.executor_thread.is_alive():
                self.executor_thread.join(timeout=1.0)
                self.node_tf.destroy_node()
                self.node_joint.destroy_node()
                self.node_obj.destroy_node()
                rclpy.shutdown()
                print(">>> SimManager finalizado.")

else:
    class SimManager(BaseRobotController):
        def __init__(self, *args, **kwargs):
            # Si alguien intenta usar esto en la Raspi, le avisamos amablemente
            raise RuntimeError(
                "\nERROR CRÍTICO: Intentaste iniciar 'SimManager' pero ROS2 no está instalado.\n"
                "En este dispositivo (Raspi) solo puedes usar el modo 'real'."
            )
            
        # Métodos vacíos solo para satisfacer a BaseRobotController si fuera estricto
        def MoveJ(self, *args, **kwargs): pass
        def MoveC(self, *args, **kwargs): pass
        def GripperState(self, *args, **kwargs): pass

class MyCobotController(BaseRobotController):
    def __init__(self, mode = 'raspi', rotar_base: bool = True, project_path=None):
        super().__init__(project_path=project_path)
        # Conexión con el robot físico
        if mode == 'raspi':
            self.mc = MyCobot320('/dev/ttyAMA0', 115200)
        elif mode == 'TCP/IP':
             host = "10.42.0.1"
             port = 9000
             self.mc = MyCobotSocket(host, port)
        else:
            raise ValueError("Modo de conexión no válido. Intente con 'raspi' o 'TCP/IP'.")

        # Modelo DH de la toolbox para la cinemática y la IK
        self.cobot_tb = myCobot320(rotar_base=rotar_base)

        self._server_ref = None

        # Activamos la pinza
        # self.mc.set_gripper_mode(0)

    def set_server_observer(self, server):
        self._server_ref = server

    def MoveJ(self, robtarget, speed: int = 30, tool: SE3 = SE3(), wobj: SE3 = SE3()):
        # Pose global = wobj * robtarget * inv(tool)
        pose_calc = wobj * robtarget.pose * tool.inv()

        # Usamos la cinemática inversa de la toolbox para tener control de la config.
        q_pose = self.cobot_tb.ikine(pose_calc, robtarget.config)[0]


        # coords = self.mc.get_coords()
        # angles = self.mc.get_angles()
        # print(f'El cobot estaba en\n{coords}')
        # print(f'Ángulos del cobot\n{angles}')
        # Enviar al robot (la API espera grados)
        self.mc.sync_send_angles(np.degrees(q_pose).tolist(), speed)
        # print("Ya salimos de MoveJ")

    def MoveC(self, robtarget, speed: int = 30, tool: SE3 = SE3(), wobj: SE3 = SE3()):
        pose_calc = wobj * robtarget.pose * tool.inv()
        coords = self.mc.get_coords()
        print(f'El cobot estaba en\n{coords}')
        # angles = self.mc.get_angles()
        # print(f'Ángulos del cobot\n{angles}')

        # Convertir a formato [x, y, z, rx, ry, rz] en grados.
        pose = list(pose_calc.t) + list(pose_calc.rpy(order='zyx', unit='deg'))
        pose_v2 = matrix_to_pose(pose_calc)

        print(f'Le pedimos al cobot\n{pose}')
        print(f'Con la función matrix_to_pose:\n{pose_v2}')
        self.mc.sync_send_coords(pose, speed, 1)
        print(f'Terminó llegando a:\n{self.mc.get_coords()}')

    def MoveCTesting(self, robtarget, speed: int = 30, tool: SE3 = SE3(), wobj: SE3 = SE3(), skip: int = 10):
        pose_goal = wobj * robtarget.pose * tool.inv() # Pose objetivo
        # Preguntamos al cobot su pose actual y convertimos a SE3
        q_actual = self.mc.get_angles() # Obtenemos los ángulos de los encoders
        pose_start = cobot_tb.fkine(np.deg2rad(q_actual)) # Usamos la toolbox para encontrar la pose actual de la brida
        # print(f'Pose start:\n{pose_start}')
        # print(f'Pose goal:\n{pose_goal}')

        cobot_tb.genTrCart([pose_start, pose_goal, pose_goal], 0*np.ones(3), conf = robtarget.config)
        qtraj_a = cobot_tb.q_ref[::skip]
        q_traj_deg = np.rad2deg(qtraj_a)
        print(f'Se generó una trayectoria con {len(q_traj_deg)} puntos.')

        q_limit = check_joint_limits(q_traj_deg, joint_limits)
        if q_limit:
            if len(q_limit) == 1:
                print(f"⚠️  Valor fuera de límite para el eje {q_limit[0]}")
            else:
                ejes = ", ".join(map(str, q_limit))
                print(f"⚠️  Se sobrepasan límites en los ejes: {ejes}.")

        freq_q = 20 # Hz 
        for q in q_traj_deg:
            self.mc.send_angles(q.tolist(), 100)
            time.sleep(1/freq_q)
    
    def MoveJAngles(self, q, spd = 30, unit = 'rad'):
        """
        Envía al robot al vector de variables articulares pedido.

        Parameters
        ----------
            q : Array(1,6)
                Vector de variables articulares
            spd : int (1 - 100)
                Velocidad 
            unit : str
                Unidad de las variables articulares. Puede ser 'rad' o 'deg'.
        """
        if unit == 'rad':
            self.mc.sync_send_angles(np.degrees(q).tolist(), spd)
        elif unit == 'deg':
            self.mc.sync_send_angles(q.tolist(), spd)

    def levantar_traj(self, archivo: str, traj_name: str = "TRAJ", tool = SE3()):
        """
        Levanta una trayectoria generada con MoveIt y la envía al cobot.
        """
        # try:
        #     # Obtenemos el objeto módulo del script principal
        #     main_module = sys.modules['__main__']
            
        #     # Obtenemos su ruta absoluta y su carpeta contenedora (la carpeta del Proyecto)
        #     # Ejemplo: si main es scripts/Proyecto/rutina.py, esto da .../scripts/Proyecto
        #     project_folder = Path(main_module.__file__).resolve().parent
            
        # except (AttributeError, KeyError):
        #     # Fallback por si estás probando desde una consola interactiva
        #     print("Advertencia: No se pudo detectar el script principal, usando directorio actual.")
        #     project_folder = Path.cwd()

        # # 2. Apuntamos a la carpeta Trayectorias dentro de ESE proyecto
        # tray_folder = project_folder / "Trayectorias"

        # # Debug (opcional, para que veas que funciona)
        # # print(f"Buscando trayectorias en: {tray_folder}")

        # # normalizar nombre
        # if not nombre.endswith(".py"):
        #     nombre += ".py"

        # path_traj = tray_folder / nombre

        # if not path_traj.exists():
        #     raise FileNotFoundError(
        #     f"No se encontró el archivo '{nombre}' en la carpeta del proyecto:\n"
        #     f"{tray_folder}\n"
        #     f"Asegurate de haber creado la carpeta 'Trayectorias' dentro de tu carpeta de Proyecto."
        # )

        # # importar módulo desde archivo
        # spec = importlib.util.spec_from_file_location("mod_traj", path_traj)
        # module = importlib.util.module_from_spec(spec)
        # spec.loader.exec_module(module)  # type: ignore

        # if not hasattr(module, "TRAJ"):
        #     raise AttributeError(f"El archivo {nombre} no contiene la variable TRAJ")

        # traj_moveit = module.TRAJ
        traj_moveit =  self._load_project_variable("Trayectorias", archivo, traj_name)
        first_len = len(traj_moveit[0])
        if first_len not in (6, 7):
            raise ValueError(f"Dimensión inesperada en el primer punto: {first_len} (se espera 6 o 7)")

        trim_last = (first_len == 7)

        out = []

        for i, pt in enumerate(traj_moveit):
            if len(pt) < (7 if trim_last else 6):
                raise ValueError(f"Punto {i} tiene longitud {len(pt)} incompatible con el primer punto.")

            # tomamos solo las 6 primeras entradas (si trim_last True/False, ambas hacen [:6])
            arr6 = np.asarray(pt)[:6]
            deg = np.rad2deg(arr6)              # ndarray con floats
            # convertir a lista de floats puros (send_angles suele esperar lista de python floats)
            out.append([float(x) for x in deg])

        freq_q = 20 # Hz 
        for q in out:
            self.mc.send_angles(q, 100)
            time.sleep(1/freq_q)

        # armamos el RobTarget para devolverlo
        last_q = traj_moveit[-1][:6]
        last_robt = robt_from_q(last_q, tool)

        return last_robt

    def GripperState(self, apertura: float, spd: int = 30):
        """
        Modifica el estado de la pinza.
        
        Args:
            apertura: grado de apertura de la pinza. < 50 cierre total, > 50 apertura total.
            spd: velocidad.
        """
        if apertura < 50:
            mov = 1  # cerrar
        else:
            mov = 0  # abrir
        
        self.mc.set_gripper_mode(0)
        self.mc.set_gripper_state(mov, spd)

        if self._server_ref is not None:
            self._server_ref.set_gripper_state(apertura)

    # def teach_and_save_TCP(self, filename: str, tcp_name: str, save_q: bool = False) -> SE3:
    
        """
        Permite recolectar o corregir poses manuales para calibrar el TCP.
        Si 'poses' se pasa, usa esa lista y solo graba los índices indicados en 'indices_a_grabar'.
        Si no, graba las 4 poses como siempre.
        """
        """Antes de grabar las posiciones colocamos la pieza auxiliar y cerramos la pinza.
        self.GripperState(0)
        time.sleep(5)
        self.GripperState(1)

        if poses is None:
            poses = [None] * 4

        if indices_a_grabar is None:
            indices_a_grabar = list(range(4))

        for n in indices_a_grabar:
            print(f"\nPreparando para liberar los motores. Posición {n+1} de 4.")
            print("Tenés 5 segundos para acercarte al robot y sujetarlo.")
            for i in range(5, 0, -1):
                print(f"Tiempo restante: {i} segundos...", end='\r')
                time.sleep(1)
            print("\nLiberando motores...")

            for eje in range(1, 7):
                self.mc.release_servo(eje)
                print(f"Motor {eje} liberado.")

            print("Tenés 20 segundos para ajustar la herramienta en la posición y orientación deseada.")
            for i in range(20, 0, -1):
                print(f"Tiempo restante antes de activar motores: {i} segundos...", end='\r')
                time.sleep(1)

            if ajuste:
                print("\nActivando todos los motores para fijar la posición...")
                self.mc.focus_all_servos()
                time.sleep(2)
                q_inicial = self.mc.get_angles()
                pose_ajustada = joystick_adjust(np.deg2rad(q_inicial), 
                                                mover_callback=lambda r: self.MoveJ(r, 20, SE3(), SE3()))
                q_ajustado = self.mc.get_angles()
                poses[n] = q_ajustado
            
            else:
                mediciones = []
                print("\nTomando 10 mediciones antes de activar motores...")
                for i in range(10):
                    pose = self.mc.get_angles()
                    # pose = generar_q_random()
                    mediciones.append(pose)
                    print(f"Medición {i+1}/10: {pose}")
                    time.sleep(0.1)

                print("\nActivando todos los motores para fijar la posición...")
                self.mc.focus_all_servos()
                print("Motores activados. El robot está firme.")

                time.sleep(2)  # Espera breve para asegurar que los motores estén activos

                # Tomar 5 mediciones después de activar motores
                print("Tomando 5 mediciones después de activar motores...")
                for i in range(5):
                    pose = self.mc.get_angles()
                    # pose = generar_q_random()
                    mediciones.append(pose)
                    print(f"Medición {i+1}/5: {pose}")
                    time.sleep(0.1)

                # Calcular el promedio de las 15 mediciones
                mediciones_np = np.array(mediciones)
                pose_promedio = np.mean(mediciones_np, axis=0).tolist()
                print(f"Pose {n+1} guardada (promedio de 15 mediciones): {pose_promedio}")


                poses[n] = pose_promedio

        print("\nPoses actuales:")
        for i, p in enumerate(poses):
            print(f"Pose {i+1}: {p}")

        print("\nRecolecta finalizada. Ya podés calcular el TCP con TCP_4puntos(poses).")
        return poses"""
        
    def grabar_poses(self, cant_poses : int, ajuste = False, q_wobj=None):
    
        """
        Permite recolectar o corregir poses manuales para grabar un wobj o TCP.
        Si `poses` se pasa, usa esa lista y solo graba los índices indicados en `indices_a_grabar`.
        Si no, graba las `cant_poses` poses indicadas.
        """
        # Antes de grabar las posiciones colocamos la pieza auxiliar y cerramos la pinza.
        self.GripperState(100)
        time.sleep(5)
        self.GripperState(0)

        poses = [None] * cant_poses
        datos_robot = [None] * cant_poses

        indices_a_grabar = list(range(cant_poses))

        for n in indices_a_grabar:
            print(f"\nPreparando para liberar los motores. Posición {n+1} de {cant_poses}.")
            print("Tenés 5 segundos para acercarte al robot y sujetarlo.")
            for i in range(5, 0, -1):
                print(f"Tiempo restante: {i} segundos...", end='\r')
                time.sleep(1)
            print("\nLiberando motores...")

            for eje in range(1, 7):
                self.mc.release_servo(eje)
                print(f"Motor {eje} liberado.")

            if n == 0:
                tiempo = 20
            else: 
                tiempo = 10

            print(f"Tenés {tiempo} segundos para ajustar la herramienta en la posición y orientación deseada.")
            for i in range(tiempo, 0, -1):
                print(f"Tiempo restante antes de activar motores: {i} segundos...", end='\r')
                time.sleep(1)

            if ajuste:
                print("\nActivando todos los motores para fijar la posición...")
                self.mc.focus_all_servos()
                time.sleep(2)
                q_inicial = self.mc.get_angles()
                pose_ajustada = joystick_adjust(np.deg2rad(q_inicial), 
                                                mover_callback=lambda r: self.MoveJ(r, 20, SE3(), SE3()))
                time.sleep(1)
                q_ajustado = self.mc.get_angles()
                dato_cobot = self.mc.get_angles_coords()
                print(f'Pose {n+1} guardada: {q_ajustado}')
                poses[n] = q_ajustado
                datos_robot[n] = dato_cobot
            
            else:
                mediciones = []
                print("\nTomando 5 mediciones antes de activar motores...")
                for i in range(5):
                    pose = self.mc.get_angles()
                    mediciones.append(pose)
                    print(f"Medición {i+1}/5: {pose}")
                    time.sleep(0.1)

                print("\nActivando todos los motores para fijar la posición...")
                self.mc.focus_all_servos()
                print("Motores activados. El robot está firme.")

                time.sleep(1)  # Espera breve para asegurar que los motores estén activos

                # Tomar 5 mediciones después de activar motores
                print("Tomando 5 mediciones después de activar motores...")
                for i in range(5):
                    pose = self.mc.get_angles()
                    mediciones.append(pose)
                    print(f"Medición {i+1}/5: {pose}")
                    time.sleep(0.1)

                # Calcular el promedio de las mediciones
                mediciones_np = np.array(mediciones)
                pose_promedio = np.mean(mediciones_np, axis=0).tolist()
                print(f"Pose {n+1} guardada (promedio de 10 mediciones): {pose_promedio}")

                poses[n] = pose_promedio

        print("\nPoses grabadas:")
        for i, p in enumerate(poses):
            print(f"Pose {i+1}: {p}")

        print("\nCoords grabadas:")
        for i, d in enumerate(datos_robot):
            print(f"Q-coords {i+1}: {d}")

        return poses, datos_robot
     
def TCP_4puntos(q_pose, z_aux = 25):
    # Convert poses to transformation matrices
    T_list = []
    for q in q_pose:
        T_se3 = cobot_tb.fkine(np.radians(q))
        T = T_se3.A
        T_list.append(T)

    # Set up equations: (T_i - T_1) @ tcp = T_1[:3, 3] - T_i[:3, 3]
    A = []
    b = []
    T_ref = T_list[0]
    for T in T_list[1:]:
        A.append(T[:3, :3] - T_ref[:3, :3])
        b.append(T_ref[:3, 3] - T[:3, 3])
    A = np.vstack(A)
    b = np.hstack(b)

    # Solve for tcp_offset in end effector frame
    tcp_offset, residuals, _, _ = np.linalg.lstsq(A, b, rcond=None)
    # Calcular ECM (error cuadrático medio)
    if residuals.size > 0:
        N = A.shape[0]  # cantidad de ecuaciones
        ecm = residuals[0] / N
    else:
        ecm = 0.0  # Si el sistema es compatible, no hay residuos
    rmse = np.sqrt(ecm)

    # Corrección por pieza auxiliar
    # correccion = np.array([0, 0, -abs(z_aux)])
    correccion = np.array([0, -abs(z_aux), 0])
    tcp_offset_real = tcp_offset + correccion

    return tcp_offset, tcp_offset_real, residuals, ecm, rmse  # [x, y, z] in end effector frame

def pose_to_matrix(pose):
        """Convierte pose dada como lista [x, y, z, rx, ry, rz] en una matriz homogénea SE3."""
        x, y, z, rx, ry, rz = pose
        rx, ry, rz = np.deg2rad([rx, ry, rz])
        T_se3 = SE3(x, y, z) * SE3.RPY([rx, ry, rz], order='zyx')
        return T_se3

def matrix_to_pose(T_matrix):
    """Convierte una matriz homogénea SE3 en lista [x, y, z, rx, ry, rz] en grados."""
    T = SE3(T_matrix)
    x, y, z = T.t
    # Extraer rotación como RPY (en radianes), orden ZYX
    rx, ry, rz = T.rpy(order='zyx', unit='rad')

    # Convertimos a grados
    rx, ry, rz = np.rad2deg([rx, ry, rz])

    return [x, y, z, rx, ry, rz]

def TCP_4puntos_extendido(q_pose, z_aux = 25):
    """
    Método extendido para calibrar TCP:
    Devuelve el vector x donde:
      x[:3] = traslación TCP (brida a herramienta)
      x[3:] = traslación base al punto ensayado (con signo invertido)
    """
    
    T_list = []
    for q in q_pose:
        T_se3 = cobot_tb.fkine(np.radians(q))
        T = T_se3.A
        T_list.append(T)

    A_blocks = []
    b_blocks = []
    for T in T_list:
        R = T[:3, :3]
        t = -1*T[:3, 3].reshape((3, 1))
        A_block = np.hstack([R, np.eye(3)])
        A_blocks.append(A_block)
        b_blocks.append(t)

    A = np.vstack(A_blocks)   # 12x6
    b = np.vstack(b_blocks).flatten()  # 12x1

    # Resolver el sistema sobredeterminado
    x, residuals, _, _ = np.linalg.lstsq(A, b, rcond=None)
    if residuals.size > 0:
        N = A.shape[0]
        ecm = residuals[0] / N
    else:
        ecm = 0.0
    rmse = np.sqrt(ecm)

    x = x.flatten()
    p_tool = x[:3]
    # Corrección por pieza auxiliar
    correccion = np.array([0, -abs(z_aux), 0])
    p_tool_real = p_tool[:3] + correccion

    return p_tool, p_tool_real, residuals, ecm, rmse
    # print(f"Offset TCP (despeje Pablo): {p_tool[:3]}")
    # print(f"Offset TCP real (con corrección): {tcp_offset_real}")
    # # print(f"Traslación base al punto ensayado (con signo invertido): {x[3:]}")
    # print(f"ECM: {ecm:.4f}")
    # print(f"RMSE: {rmse:.4f}")

def fit_line_pca(points):
    """Ajuste de recta por PCA (TLS). Devuelve centroid, direction(normalizado), residuals."""
    centroid = np.mean(points, axis=0)
    X = points - centroid
    U, S, Vt = np.linalg.svd(X, full_matrices=False)
    direction = Vt[0, :]  # principal component (Vt filas = componentes)
    direction = direction / np.linalg.norm(direction)
    # Distancias perpendiculares (residuales)
    proj = X @ direction[:, None] @ direction[None, :]
    perp = X - proj
    dists = np.linalg.norm(perp, axis=1)
    return centroid, direction, dists

def teach_wobj(q_poses, tool: SE3, z_aux = 25):
    """
    Enseña un workobject a partir de 6 puntos medidos manualmente con el robot.
    El origen se define como la intersección de X (que colineal con P1 Y P2) e Y (perpendicular a X que pasa por P3).
    Devuelve un objeto SE3 con la transformación del workobject respecto a la base.
    """
    cant_puntos = len(q_poses)
    puntos = np.zeros((cant_puntos, 3))

    for i in range (cant_puntos):
        puntos[i] = ((cobot_tb.fkine(np.radians(q_poses[i])) )* tool* SE3(0, 0, abs(z_aux))).t
    print(f'[DEBUG] Puntos:\n{puntos}')

    puntos_x = puntos[:3]
    puntos_y = puntos[3:]

    centroid_x, x_axis, dists_x = fit_line_pca(puntos_x)
    centroid_y, y_axis_pca, dists_y = fit_line_pca(puntos_y)

    # Eje Y: desde la proyección de p3 sobre el eje x a p3
    y_axis_raw = y_axis_pca - np.dot(y_axis_pca, x_axis) * x_axis
    if np.linalg.norm(y_axis_raw) < 1e-9:
        raise ValueError("Los puntos para el eje Y son casi colineales con X.")
    y_axis = y_axis_raw / np.linalg.norm(y_axis_raw)

    # origen: proyectar centroid_y sobre linea X (como hiciste)
    origen = centroid_x + np.dot(centroid_y - centroid_x, x_axis) * x_axis

    # z axis
    z_axis = np.cross(x_axis, y_axis)
    z_axis /= np.linalg.norm(z_axis)

    # Matriz de rotación
    R = np.column_stack((x_axis, y_axis, z_axis))
    # Ortonormalización por SVD
    U, _, Vt = np.linalg.svd(R)
    R_orth = U @ Vt
    if np.linalg.det(R_orth) < 0:
        U[:, -1] *= -1
        R_orth = U @ Vt

    # Armar SE3 limpio
    wobj = SE3.Rt(R_orth, origen)

    # métricas de calidad
    stats = {
        "resid_x_mean": float(np.mean(dists_x)),
        "resid_x_std": float(np.std(dists_x)),
        "resid_y_mean": float(np.mean(dists_y)),
        "resid_y_std": float(np.std(dists_y)),
    }

    print(f"Wobj enseñado. resid X mean/std: {stats['resid_x_mean']:.3f}/{stats['resid_x_std']:.3f} mm; "
          f"resid Y mean/std: {stats['resid_y_mean']:.3f}/{stats['resid_y_std']:.3f} mm")

    def rot_error(R):
        I = np.eye(3)
        err_orth = np.linalg.norm(R.T @ R - I, 'fro')
        err_det = abs(np.linalg.det(R) - 1)
        return err_orth, err_det

    err_orth, err_det = rot_error(R)
    print(f"Error ortogonalidad: {err_orth:.2e}, Error determinante: {err_det:.2e}")

    return wobj

def joystick_adjust(q, mover_callback, step_fino=5, step_grueso=10):
    """
    Ajuste interactivo de un robtarget con teclado.
    - mover_callback: función que mueve el robot (ej: SimManager.MoveC)
    - Flechas = XY
    - PgUp/PgDn = Z
    - Shift = paso grueso
    - Esc = salir
    """
    print(f'Pose recibida = {q}')
    pose = cobot_tb.fkine(q).copy()

    print(f'Pose calculada con fkine =\n {pose}')
    robt = RobTarget(pose, cobot_tb.calc_conf(q).tolist())
    print(f"RobTarget inicial: \n{robt.pose} | Configuración: {robt.config}")
    paso = step_fino

    print("\n--- Modo joystick ---")
    print("Flechas = mover XY | PgUp/PgDn = mover Z | Shift = paso grueso | Esc = salir")

    def on_press(key):
        nonlocal robt, paso
        try:
            if key == keyboard.Key.shift:
                paso = step_grueso
            elif key == keyboard.Key.up:
                robt = robt.offset(dy=paso)
            elif key == keyboard.Key.down:
                robt = robt.offset(dy=-paso)
            elif key == keyboard.Key.left:
                robt = robt.offset(dx=-paso)
            elif key == keyboard.Key.right:
                robt = robt.offset(dx=paso)
            elif key == keyboard.Key.page_up:
                robt = robt.offset(dz=paso)
            elif key == keyboard.Key.page_down:
                robt = robt.offset(dz=-paso)
            elif key == keyboard.Key.esc:
                return False
            # print(f"RobTarget ajustado: \n{robt.pose} | Configuración: {robt.config}")
            mover_callback(robt)
        except IKineError as e:
            print(f'Error con el movimiento del robot: {e}')
        except Exception as e:
            print(f'Error inesperado: {e}')


            # print(f"RobTarget enviado: \n{robt.pose}")

    def on_release(key):
        nonlocal paso
        if key == keyboard.Key.shift:
            paso = step_fino

    with keyboard.Listener(on_press=on_press, on_release=on_release) as listener:
        listener.join()

    return robt

def to_array(q) -> np.ndarray:
    """Convierte listas a np.ndarray y asegura 7 DOF (6 brazo + pinza)."""
    q = np.array(q, dtype=float)
    if q.shape[0] < 7:
        q = np.concatenate([q, np.zeros(7 - q.shape[0])])
    elif q.shape[0] > 7:
        q = q[:7]   # descartar extras (relleno del URDF)
    return q

def pad_for_urdf(q: np.ndarray, total: int = 7) -> list:

    """
    Adapta el q interno (7 valores) a lo que espera el URDF en /joint_states.
    Rellena con ceros hasta 'total'.
    """
    q = to_array(q)
    return q.tolist() + [0.0]*(total - len(q))

def check_joint_limits(q_traj, joint_limits):
    """
    Chequea que cada valor de q esté dentro de los límites.

    Args:
        q: lista/array con los valores articulares (en el mismo orden que joint_limits)
        joint_limits: dict con min/max por articulación

    Returns:
        list de índices (o nombres) de ejes con valores fuera de límite
    """
    flag_lim = []
    # for i, (name, (q_min, q_max)) in enumerate(joint_limits.items(), start=1):
    #     if not (q_min <= q[i-1] <= q_max):
    #         flag_lim.append(i)  # o `name` si preferís
    for j, (name, (q_min, q_max)) in enumerate(joint_limits.items()):
        # columna j de la trayectoria
        q_vals = q_traj[:, j]
        if np.any(q_vals < q_min) or np.any(q_vals > q_max):
            flag_lim.append(j+1)  # eje en base 1
    return flag_lim

def robt_from_q(q, tool):
    """
    Construye un robtarget a partir de un vector de variables articulares y una herramienta.
    """
    pose_SE3 = cobot_tb.fkine(q) * tool
    config = cobot_tb.calc_conf(np.array(q))
    robt = RobTarget(pose_SE3, config)
    return robt

def _normalize_q(q) -> np.ndarray:
    """
    Asegura que el vector 'q' tenga siempre 7 elementos (float).
    Recorta si sobran, rellena con ceros si faltan.
    """
    q = np.array(q, dtype=float).flatten()
    if q.size == 7:
        return q
    elif q.size > 7:
        return q[:7]
    else:
        # Rellenar con ceros al final
        return np.pad(q, (0, 7 - q.size), 'constant')
    
"""def _save_se3_definition(self, folder: str, filename: str, var_name: str, se3_obj: SE3, q_vals: list = None):
        
        Helper genérico para guardar definiciones SE3 (Wobj o TCP) y sus Q asociados en archivos .py.
             
        # Limpieza del nombre de archivo (asegurar extensión .py)
        if not filename.endswith(".py"):
            filename += ".py"

        # --- 1. GUARDADO DEL SE3 ---
        path_se3 = self._resolve_project_path(folder, filename, create_dir=True)
        
        mode_se3 = 'a' if path_se3.exists() else 'w'
        
        try:
            with open(path_se3, mode_se3) as f:
                # Escribir cabecera solo si es archivo nuevo
                if mode_se3 == 'w':
                    f.write(f"# Archivo de definiciones: {folder}\n")
                    f.write("from spatialmath import SE3\n")
                    f.write("import numpy as np\n\n")
                
                # Formateo de alta precisión (Tu lógica intacta)
                R_str = "[\n" + ",\n".join(["    [" + ", ".join([f"{val:.18e}" for val in row]) + "]" for row in se3_obj.R]) + "\n  ]"
                t_str = "[" + ", ".join([f"{val:.18e}" for val in se3_obj.t.flatten()]) + "]"
                
                f.write(f"# Guardado: {datetime.datetime.now().isoformat()}\n")
                f.write(f"{var_name}_R = np.array({R_str})\n")
                f.write(f"{var_name}_t = np.array({t_str})\n")
                f.write(f"{var_name} = SE3.Rt({var_name}_R, {var_name}_t)\n\n")
            
            print(f"[Guardado] {folder}/{var_name} -> {path_se3.name}")

        except Exception as e:
            print(f"[Error] Falló guardado SE3 en {filename}: {e}")

        # --- 2. GUARDADO DE Q (Opcional) ---
        if q_vals:
            name_q = filename.replace(".py", "") + "_q.py"
            path_q = self._resolve_project_path(folder, name_q)
            
            variable_name_q = f"{var_name}_q"
            mode_q = 'a' if path_q.exists() else 'w'
            
            try:
                with open(path_q, mode_q) as f:
                    if mode_q == 'w':
                        f.write(f"# Config Articulares (Q) para {folder}\n\n")

                    f.write(f"# Q Values para: {var_name}\n")
                    f.write(f"{variable_name_q} = [\n")
                    for row in q_vals:
                        # Limpieza a lista de floats estándar
                        clean_row = [float(x) for x in row]
                        f.write(f"    {clean_row},\n")
                    f.write("]\n\n")
                
                print(f"[Guardado] Q Values -> {path_q.name} (Var: {variable_name_q})")
                
            except Exception as e:
                print(f"[Error] Falló guardado Q en {name_q}: {e}")"""

