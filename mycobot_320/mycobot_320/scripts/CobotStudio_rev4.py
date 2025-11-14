import numpy as np
from scipy.spatial.transform import Rotation as R
from scripts.DHRobotGT import myCobot320, IKineError
import time
import datetime
import threading
import itertools
from pymycobot import MyCobotSocket, MyCobot320Socket, MyCobot320
from scipy.spatial.transform import Rotation as R
from abc import ABC, abstractmethod
from spatialmath import SE3
from pynput import keyboard
from typing import List, Optional
import json
import os

try:
    from geometry_msgs.msg import TransformStamped
    from tf2_ros import TransformBroadcaster
    import rclpy
    from rclpy.node import Node
    from rclpy.action import ActionClient
    # --------------- MoveIt! --------------------- #
    from moveit_msgs.msg import DisplayRobotState, RobotState
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from std_msgs.msg import Empty, Header
    from sensor_msgs.msg import JointState
    from moveit_msgs.msg import DisplayTrajectory, RobotState
    from moveit_msgs.srv import ApplyPlanningScene
    # --------------- MoveIt! --------------------- #
    from builtin_interfaces.msg import Duration
    from rclpy.executors import MultiThreadedExecutor
    from scripts.object_manager_rev1 import ObjectManager
    from control_msgs.action import FollowJointTrajectory
    ROS_OK = True
    print('>>> Librerías de ROS2 importadas correctamente. <<<')
except ImportError:
    ROS_OK = False
    print('>>> No se pudieron importar las librerías de ROS2. Funciones de visualización deshabilitadas. <<<')

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
        
    def get_SE3(self):
        """
        Devuelve la pose como objeto SE3.
        """
        return self.pose

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
    @abstractmethod
    def MoveJ(self, robt, speed:int = 30, tool=SE3(), wobj=SE3()):
        pass

    @abstractmethod
    def MoveC(self, robt, speed:int = 30, tool=SE3(), wobj=SE3()):
        pass

    @abstractmethod
    def GripperState(self, apertura: float, spd: int):
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
                msg.position = pad_for_urdf(q)
                self.publisher.publish(msg)
                # self.get_logger().info(f'q: {msg.position}')
                time.sleep(dt)

        def publish_pose(self, q, joint_names):
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = joint_names
            msg.position = pad_for_urdf(q)
            self.publisher.publish(msg)
            # self.get_logger().info(f'q: {msg.position}')
            self.q_current = np.array(msg.position)
            self.q_current_time = msg.header.stamp
            # print(f'Se envía un q_current de {q}')
            # print(f'q al final de publish_pose: {self.q_current}')

        def publish_q(self, q, joint_names):
            msg = JointState()
            msg.name = joint_names
            msg.position = pad_for_urdf(q)
            print(q)
            msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher.publish(msg)
            self.get_logger().info(f'q: {msg.position}')
            self.q_current = msg.position
            # print(f'el q_current queda como {self.q_current}')

    class TFPublisher(Node):
        # NOTA: RViz mantiene listados todos los frames que escuchó al menos una vez.
        # Si se eliminan ternas de este script, puede que sigan apareciendo en la pestaña TF de RViz.
        # Esto es solo visual; el frame ya no se publica ni influye en la escena.
        # Para eliminar ternas obsoletas se debe reiniciar RViz. Esto se facilita con el ejecutable Reset Scene

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
            """Agrega un robtarget referido a otra terna: puede ser la base o un wobj, por ejemplo."""
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
            # En la clase TFPublisher
        
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
            """ Publica una pose articular para que el 'robot naranja' salte a ella. """
            if len(joint_positions) != len(self.ARM_JOINT_NAMES):
                self.get_logger().error(f"Error: Se esperaban {len(self.ARM_JOINT_NAMES)} posiciones, pero se recibieron {len(joint_positions)}.")
                return

            # 1. Crear el mensaje JointState
            joint_state_msg = JointState()
            joint_state_msg.header.stamp = self.get_clock().now().to_msg()
            joint_state_msg.name = self.ARM_JOINT_NAMES
            joint_state_msg.position = [float(q) for q in joint_positions]

            # 2. Embeberlo en un mensaje RobotState
            robot_state_msg = RobotState()
            robot_state_msg.joint_state = joint_state_msg

            # 3. Embeberlo en el mensaje final DisplayRobotState
            display_robot_state_msg = DisplayRobotState()
            display_robot_state_msg.state = robot_state_msg

            # 4. Publicar
            self.publisher.publish(display_robot_state_msg)
            self.get_logger().info(f"Publicando estado de visualización para el robot naranja.")

    class JointActionClient(Node):
        def __init__(self, joint_names_ordered=joint_names):
            super().__init__('joint_action_client')
            
            # --- CAMBIO FUNDAMENTAL ---
            # Creamos un cliente de acción, no un publicador
            self._action_client = ActionClient(
                self,
                FollowJointTrajectory,
                '/arm_controller/follow_joint_trajectory' # El nombre de la acción que encontramos
            )
            # --------------------------

            # La subscripción a /joint_states sigue siendo útil para leer el estado inicial
            self.create_subscription(
                JointState,
                '/joint_states',
                self.joint_state_callback,
                10
            )

            self.q_current = None
            self.ARM_JOINT_NAMES = joint_names_ordered

        def joint_state_callback(self, msg):
            # Esta lógica ya está validada y es correcta. Reordena el estado.
            if self.q_current is not None: # Solo lo hacemos una vez para evitar spam
                return
                
            position_map = {name: pos for name, pos in zip(msg.name, msg.position)}
            ordered_positions = [position_map.get(name, 0.0) for name in self.ARM_JOINT_NAMES]
            self.q_current = np.array(ordered_positions)
            self.get_logger().info(f"Estado inicial de articulaciones recibido y ordenado: {self.q_current}")

        def send_trajectory(self, trajectory_points, joint_names, dt=0.1):
            self.get_logger().info("Esperando al servidor de acción...")
            self._action_client.wait_for_server()

            # --- CONSTRUCCIÓN DEL MENSAJE DE OBJETIVO (GOAL) ---
            goal_msg = FollowJointTrajectory.Goal()
            
            # El mensaje de trayectoria ahora va *dentro* del mensaje de objetivo
            traj_msg = JointTrajectory()
            traj_msg.header.stamp = self.get_clock().now().to_msg()
            traj_msg.joint_names = joint_names

            total_time = 0.0
            for q_point in trajectory_points:
                point = JointTrajectoryPoint()
                point.positions = q_point
                total_time += dt
                point.time_from_start = Duration(sec=int(total_time), nanosec=int((total_time % 1) * 1e9))
                traj_msg.points.append(point)

            goal_msg.trajectory = traj_msg
            # ----------------------------------------------------

            self.get_logger().info('Enviando objetivo de trayectoria al servidor de acción...')

            # Enviamos el objetivo de forma asíncrona
            self._send_goal_future = self._action_client.send_goal_async(
                goal_msg,
                feedback_callback=self.feedback_callback)

            self._send_goal_future.add_done_callback(self.goal_response_callback)

        def goal_response_callback(self, future):
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().info('El objetivo fue rechazado :(')
                return

            self.get_logger().info('El objetivo fue aceptado :)')
            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(self.get_result_callback)

        def get_result_callback(self, future):
            result = future.result().result
            self.get_logger().info(f'Resultado: {result.error_code}') # ERROR_CODE 0 es éxito

        def feedback_callback(self, feedback_msg):
            feedback = feedback_msg.feedback
            # Puedes imprimir el feedback si quieres ver el progreso en tiempo real
            # self.get_logger().info(f'Feedback recibido: {feedback.actual.positions}')

    class SimManager(BaseRobotController):
        def __init__(self):
            # Inicializamos rclpy solo una vez
            rclpy.init()
            self.node_tf = TFPublisher()
            self.node_joint = joint_pub()
            # self.node_joint = JointCommandPublisher()
            self.node_obj = ObjectManager()

            self.ARM_JOINT_NAMES = joint_names[:6]
            self.ARM_JOINT_NAMES_FULL = joint_names
            # self.node_joint = JointActionClient(self.ARM_JOINT_NAMES)
            self.node_visualizer = RobotStateVisualizer(self.ARM_JOINT_NAMES)
            from scripts.MoveItAdapter import MoveItAdapter

            self.moveit_adapter = MoveItAdapter()
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
            # self.executor.add_node(self.node_visualizer)

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

        def ShowPose(self, q):
            """
            'Teletransporta' el robot de planificación (naranja) a la pose articular 'q'.
            Ideal para comprobaciones visuales y de colisión sin mover el robot real.

            :param q: Una lista o array de 6 posiciones articulares.
            """
            print(f">>> Visualizando pose: {q}")
            self.node_visualizer.publish_pose(q)

        def MoveJ(self, robtarget, speed: int = 30, tool: SE3 = SE3(), wobj: SE3 = SE3(), wobj_name='wobj1', robt_name='robtarget'):
            self.node_tf.add_wobj(wobj, wobj_name)
            self.node_tf.add_robt(robtarget.pose, robt_name, wobj_name)
            # self.node_tf.add_robt(tool, 'TCP', 'link6')
            self.node_tf.add_robt(tool, 'tool', 'link6')


            # La cinemática inversa devuelve 6 ejes, sin la pinza
            pose_calc = wobj * robtarget.pose * tool.inv()
            q_brazo = cobot_tb.ikine(pose_calc, robtarget.config)[0]

            q_start = self.node_joint.q_current

            # Respetar la pinza actual
            gripper_val = self.q_current[6] if self.q_current is not None else 0.0
            q_goal = np.concatenate([np.array(q_brazo), [gripper_val]])
            q_start = to_array(self.q_current)[:7]   # <--- forzar 7
            q_end = to_array(q_goal)[:7] 

            traj_q = np.vstack([q_start, q_end, q_end])
            # traj_q = np.vstack([q_start, q_brazo, q_brazo])

            traj_arm = traj_q[:, :6]
            cobot_tb.genTrJoint(traj_arm, np.zeros(traj_arm.shape[0]))

            qtraj_a = cobot_tb.q_ref[::15]
            q_limit = check_joint_limits(np.rad2deg(qtraj_a), joint_limits)
            if q_limit:
                if len(q_limit) == 1:
                    print(f"⚠️  Valor fuera de límite para el eje {q_limit[0]}")
                else:
                    ejes = ", ".join(map(str, q_limit))
                    print(f"⚠️  Se sobrepasan límites en los ejes: {ejes}")

            qtraj_a_full = [np.concatenate([q, [gripper_val]]) for q in qtraj_a]

            # self._send_move(qtraj_a.tolist(), robt_name, wobj_name, dt=0.1)
            self._send_move(qtraj_a_full, robt_name, wobj_name)

        def MoveC(self, robtarget, speed: int = 30, tool: SE3 = SE3(), wobj: SE3 = SE3(), wobj_name='wobj1', robt_name='robtarget'):
            self.node_tf.add_wobj(wobj, wobj_name)
            self.node_tf.add_robt(robtarget.pose, robt_name, wobj_name)
            # self.node_tf.add_robt(tool, 'TCP', 'link6')
            # pinza = SE3(-1.71381642, 106.90735789, 28.32702833) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)
            self.node_tf.add_robt(tool, 'tool', 'link6')

            pose_goal = wobj * robtarget.pose * tool.inv()
            q_gripper = self.q_current[6] if self.q_current is not None else 0.0
            pose_start = cobot_tb.fkine(self.q_current.copy()[:6])
            # q_end = np.array(pose_goal)
            # traj_q = np.vstack([pose_start, pose_goal, pose_goal])
            # print(f'Pose start:\n{pose_start}')
            # print(f'Pose goal:\n{pose_goal}')

            cobot_tb.genTrCart([pose_start, pose_goal, pose_goal], 0*np.ones(3), conf = robtarget.config)
            # cobot_tb.genTrCart([pose_a, pose_b, pose_b], 0*np.ones(3), conf=config_ros)
            qtraj_a = cobot_tb.q_ref[::10]

            q_limit = check_joint_limits(np.rad2deg(qtraj_a), joint_limits)
            if q_limit:
                if len(q_limit) == 1:
                    print(f"⚠️  Valor fuera de límite para el eje {q_limit[0]}")
                else:
                    ejes = ", ".join(map(str, q_limit))
                    print(f"⚠️  Se sobrepasan límites en los ejes: {ejes}")
            qtraj_a_full = [np.concatenate([q, [q_gripper]]) for q in qtraj_a]

            self._send_move(qtraj_a_full, robt_name, wobj_name)
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


            # Respetar la pinza actual
            gripper_val = self.q_current[6] if self.q_current is not None else 0.0
            q_goal = np.concatenate([np.array(q_brazo), [gripper_val]])
            q_start = to_array(self.q_current)[:7]
            q_end = to_array(q_goal)[:7]

            traj_q = np.vstack([q_start, q_end, q_end])

            traj_arm = traj_q[:, :6]
            cobot_tb.genTrJoint(traj_arm, np.zeros(traj_arm.shape[0]))

            qtraj_a = cobot_tb.q_ref[::10]
            q_limit = check_joint_limits(np.rad2deg(qtraj_a), joint_limits)
            if q_limit:
                if len(q_limit) == 1:
                    print(f"⚠️  Valor fuera de límite para el eje {q_limit[0]}")
                else:
                    ejes = ", ".join(map(str, q_limit))
                    print(f"⚠️  Se sobrepasan límites en los ejes: {ejes}")

            qtraj_a_full = [np.concatenate([q, [gripper_val]]) for q in qtraj_a]

            self._send_move(qtraj_a_full)
        
        def MostrarTerna(self, terna, nombre='terna1'):
            self.node_tf.add_wobj(terna, nombre)

        def VerPose(self, wobj, robtarget, tool: SE3 | None = SE3(), wobj_name='wobj1', robt_name='robt1'):
            gripper_val = self.q_current[6] if self.q_current is not None else 0.0
            pose = wobj * robtarget.pose * tool.inv()
            config = robtarget.config
            try:
                q = cobot_tb.ikine(pose, config)[0]
            except IKineError as e:
                print("Error en el problema inverso:", e)
                return
            
            q_full = np.concatenate([q, [gripper_val]])

            self.node_tf.add_wobj(wobj, wobj_name)
            self.node_tf.add_robt(robtarget.pose, robt_name, wobj_name)

            time.sleep(0.1)

            self._send_pose(q_full, robt_name, wobj_name)
        
        def VerQ(self, q, tool: SE3 | None = SE3(), brida = False):
            if brida: self.node_tf.add_robt(cobot_tb.fkine(q), 'brida', 'base')
            time.sleep(1)
            self.node_tf.add_robt(tool, 'tool', 'link6')
            time.sleep(1)

            gripper_val = self.q_current[6] if self.q_current is not None else 0.0
            q_full = np.concatenate([q, [gripper_val]])

            # msg = JointState()
            # msg.name = joint_names
            self._send_pose(q_full)

        def _send_pose(self, q_pos, robt_name = 'robt', wobj_name = 'wobj'):

            def send_pose():
                self.node_joint.publish_pose(q_pos, joint_names)

            self.node_joint.publish_pose(q_pos, joint_names)

            # Procesar callbacks pendientes para actualizar q_current
            rclpy.spin_once(self.node_joint, timeout_sec=0.01)

            # Asegurar que el estado local refleje la posición enviada
            self.q_current = pad_for_urdf(q_pos)
            # print(f'El q que sale de _send_pose es {self.q_current}')

        def _send_move(self, qtraj_a, robt_name = 'robt', wobj_name = 'wobj', dt=0.1):
            print(f">>> Move: {robt_name} @ {wobj_name}")

            # def send_trajectory():
            #     self.node_joint.publish_trajectory(qtraj_a, joint_names, dt=0.1)

            # pub_thread = threading.Thread(target=send_trajectory)
            # pub_thread.start()
            # pub_thread.join()
            self.node_joint.publish_trajectory(qtraj_a, joint_names, dt)
            # self.node_joint.send_trajectory(qtraj_a, joint_names, dt)
            # self.q_current = qtraj_a[-1].copy()

            # for q in qtraj_a:
            #     # Publicar cada punto de la trayectoria
            #     self.node_joint.publish_pose(q, joint_names)

            #     # Procesar callbacks pendientes para mantener q_current actualizado
            #     rclpy.spin_once(self.node_joint, timeout_sec=0.001)

            #     # Actualizar q_current local
            #     self.q_current = pad_for_urdf(q)

            #     # Espera para simular tiempo de trayectoria
            #     time.sleep(dt)

            if qtraj_a:
                self.q_current = qtraj_a[-1].copy()

            # print(f'Trayectoria enviada. Objetivo final =\n{self.q_current}')

            # print(f'Trayectoria finalizada. q_current =\n{self.q_current}')
        
        def get_current_q(self, prefer_gripper=False, timeout=2.0):
            """Devuelve q_current ordenado como np.array o None si timeout."""
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
                    return np.array(ordered)
                
                q = getattr(self.node_joint, "q_current", None)
                if q is not None:
                    if prefer_gripper:
                        # devolver hasta 7
                        return np.array(list(q)[:target_len])
                    else:
                        return np.array(list(q)[:len(self.ARM_JOINT_NAMES)])
                # if q is not None and len(q) >= len(self.ARM_JOINT_NAMES):
                #     return np.array(q)[:len(self.ARM_JOINT_NAMES)]
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
            # print(f'Posición antes de mover_pinza: {self.q_current}')
            if not (0 <= apertura <= 100):
                raise ValueError("La apertura debe estar entre 0 y 100%.")
            
            # Copiar el estado actual para modificar solo gripper_controller
            q_actual = to_array(self.q_current)

            # Mapear apertura [0,100] a rango [-0.7, 0.3]
            q_actual[6] = apertura / 100 - 0.7
            # print(f'q_actual = {q_actual}')

            msg = JointState()
            msg.name = joint_names
            self._send_pose(q_actual)
        
        def testPose(self, robt, tool, wobj):
            confs = robt.find_valid_configs(tool, wobj)
            last_conf = None

            while True:
                print("\n--- Menú de configuraciones ---")
                for i, conf in enumerate(confs, start = 1):
                    # print(f"{i}: {conf}")
                    marker = " *" if conf == last_conf else ""
                    print(f"{i}: {conf}{marker}")

                print("\nSeleccione un número para ver la configuración.")
                print("Presione 'q' para salir.\n")

                user_in = input("Opción: ")

                if user_in.lower() == 'q':   # salir del menú
                    # print("Saliendo del menú.")
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
                # for conf in confs:
                #     print(list(conf))
        
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
            T = cobot_tb.fkine(q6) * tool  # asumo que devuelve SpatialMath SE3 (unit: normalmente metros)
            # Detectar unidades: si la traslación es muy grande asumimos que está en mm y convertimos.
            # trans = np.array(T.t).astype(float).flatten()
            # if np.linalg.norm(trans) > 10.0:  # si >10 asumo que son mm (heurística)
            #     trans = trans / 1000.0
            #     T = SE3(trans, T.r)  # construyo nuevo SE3 con rotación igual y traslación corregida

            # Ahora publicamos en TFPublisher (que espera metros)
            self.node_tf.add_robt(T, frame_name, reference_frame)

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

class MyCobotController(BaseRobotController):
    def __init__(self, mode = 'raspi', rotar_base: bool = True):

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

        # Activamos la pinza
        # self.mc.set_gripper_mode(0)

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
            self.mc.send_angles(q, 100)
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

    def GripperState(self, apertura: float, spd: int = 30):
        """
        Modifica el estado de la pinza.
        
        Args:
            mov: 0 - Abrir, 1 - Cerrar, 10 - Soltar
            spd: velocidad.
        """
        if apertura < 50:
            mov = 1  # cerrar
        else:
            mov = 0  # abrir
        # if mov not in (0, 1, 10):
        #     raise ValueError("El parámetro 'mov' debe ser 0 (abrir), 1 (cerrar) o 10 (soltar).")
        
        self.mc.set_gripper_mode(0)
        self.mc.set_gripper_state(mov, spd)

    def recolectar_puntos_TCP(self, poses=None, indices_a_grabar=None, ajuste = False):
    
        """
        Permite recolectar o corregir poses manuales para calibrar el TCP.
        Si 'poses' se pasa, usa esa lista y solo graba los índices indicados en 'indices_a_grabar'.
        Si no, graba las 4 poses como siempre.
        """
        # Antes de grabar las posiciones colocamos la pieza auxiliar y cerramos la pinza.
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
        return poses
    
    def grabar_poses(self, cant_poses : int,  poses=None, indices_a_grabar=None, ajuste = False):
    
        """
        Permite recolectar o corregir poses manuales para grabar un wobj o TCP.
        Si `poses` se pasa, usa esa lista y solo graba los índices indicados en `indices_a_grabar`.
        Si no, graba las `cant_poses` poses indicadas.
        """
        # Antes de grabar las posiciones colocamos la pieza auxiliar y cerramos la pinza.
        self.GripperState(100)
        time.sleep(5)
        self.GripperState(0)

        if poses is None:
            poses = [None] * cant_poses
            datos_robot = [None] * cant_poses

        if indices_a_grabar is None:
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

    p_tool = x[:3]
    # Corrección por pieza auxiliar
    correccion = np.array([0, -abs(z_aux), 0])
    p_tool_real = p_tool[:3] + correccion

    # print(f"Offset TCP (despeje Pablo): {p_tool[:3]}")
    # print(f"Offset TCP real (con corrección): {tcp_offset_real}")
    # # print(f"Traslación base al punto ensayado (con signo invertido): {x[3:]}")
    # print(f"ECM: {ecm:.4f}")
    # print(f"RMSE: {rmse:.4f}")

    return p_tool, p_tool_real, residuals, ecm, rmse

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

def save_terna_se3(filename: str, name: str, se3_obj: SE3):
    """
    Guarda una terna SE3 como variable Python directamente en formato SE3,
    asegurando alta precisión en los números.

    Args:
        filename (str): Ruta del archivo donde se guardará la terna. Ej: `Ternas.py`.
        name (str): Nombre de la variable que se guardará.
        se3_obj (SE3): Objeto SE3 que representa la transformación a guardar.
    """
    # Formatear cada elemento de la matriz y el vector con alta precisión
    R_str = "[\n" + ",\n".join(["        [" + ", ".join([f"{val:.18e}" for val in row]) + "]" for row in se3_obj.R]) + "\n    ]"
    t_str = "[" + ", ".join([f"{val:.18e}" for val in se3_obj.t.flatten()]) + "]"

    with open(filename, "a") as f:
        # Asegurar que la importación solo se escriba una vez si el archivo es nuevo
        f.seek(0, 2) # Ir al final del archivo
        if f.tell() == 0: # Si el archivo está vacío
            f.write("from spatialmath import SE3\n")
            f.write("import numpy as np\n\n")

        f.write(f"# Guardado {datetime.datetime.now().isoformat()}\n")
        # Escribir la matriz de rotación y el vector de traslación como arrays de numpy
        # para facilitar la lectura y mantener el formato.
        f.write(f"{name}_R = np.array({R_str})\n")
        f.write(f"{name}_t = np.array({t_str})\n")
        f.write(f"{name} = SE3.Rt({name}_R, {name}_t)\n\n")

if ROS_OK:
    def live_pose(sleep = 0.3, max_stable=10, tol=0.1):
        cobot = MyCobotController()
        rviz = SimManager()
        stable_count = 0
        prev_q = None
        try:
            while True:
                q = cobot.mc.get_angles()
                time.sleep(0.3)
                rviz.VerQ(np.radians(q))

                if prev_q is not None:
                    dif = np.max(np.abs(np.array(q) - np.array(prev_q)))
                    if dif < tol:  # prácticamente no cambió
                        stable_count += 1
                    else:
                        stable_count = 0
                prev_q = q

                if stable_count >= max_stable or not cobot.mc.is_moving():
                    print("Robot detenido, cerrando live pose")
                    break
                time.sleep(sleep)
        except KeyboardInterrupt:
            print("Live pose detenido por el usuario")
