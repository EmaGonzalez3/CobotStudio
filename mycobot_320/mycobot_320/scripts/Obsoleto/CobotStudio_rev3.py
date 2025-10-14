from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import rclpy
from rclpy.node import Node
from spatialmath import SE3
import numpy as np
from scipy.spatial.transform import Rotation as R
from DHRobotGT import myCobot320, IKineError
from sensor_msgs.msg import JointState
import time
from rclpy.executors import MultiThreadedExecutor
import threading
import itertools
from pymycobot import MyCobotSocket, MyCobot320Socket
from scipy.spatial.transform import Rotation as R
from pynput import keyboard
# from objetos import ObjectManager
from object_manager_rev1 import ObjectManager
from abc import ABC, abstractmethod
from spatialmath import SE3

cobot_tb = myCobot320(rotar_base=True, metros=False)
joint_names2 = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3', 'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6','gripper_controller','gripper_base_to_gripper_left2','gripper_left3_to_gripper_left1','gripper_base_to_gripper_right3','gripper_base_to_gripper_right2','gripper_right3_to_gripper_right1']
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
            if isinstance(config, list) and all(isinstance(x, int) for x in config):
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
        self.q_current = np.array(msg.position)
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
        print(f'el q_current queda como {self.q_current}')

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
        # if isinstance(transform, SE3):
        #     robt_m = transform
        # else:
        #     robt_m = SE3(np.array(transform))
        # robt_m.t = robt_m.t / 1000.0
        # self.transforms[name] = (robt_m, reference_frame)
        # print(f'Lo que se pasa como transform\n{transform}')
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

def prueba_sim1():
    # Iniciamos nodos y el executor
    rclpy.init()
    node = TFPublisher()
    node2 = joint_pub()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.add_node(node2)

    # Definimos el workobject
    wobj1 = SE3(100, 100, 100)

    # Definimos los robtargets
    rob_clase1 = RobTarget(SE3(50, -120, 220))
    rob_clase2 = RobTarget(SE3(100, 100, 370))

    # Publicamos las ternas
    node.add_wobj(wobj1, 'wobj1')
    node.add_robt(rob_clase1.pose, 'robtarget1', 'wobj1')  # Definido respecto a wobj1
    node.add_robt(rob_clase2.pose, 'robtarget2', 'base')   # Robt respecto a la base

    # Calculamos las q inicial y final
    q1 = np.array([cobot_tb.ikine(wobj1*rob_clase1.pose)[0]])
    q2 = np.array([cobot_tb.ikine(rob_clase2.pose)[0]])

    # Generamos la trayectoria joint
    cobot_tb.genTrJoint(np.array([q1[0], q2[0], q2[0]]), 0*np.ones(3))
    qtraj_a = cobot_tb.q_ref[::10]

    time.sleep(3.0)  # Esperar a que RViz muestre las TFs primero

    # Ejecutar el trajectory publisher en un hilo separado
    def send_trajectory():
        node2.publish_trajectory(qtraj_a, joint_names, dt=0.1)
        # Para liberar la terminal después de unos segundos. Esto hace que las ternas desaparezcan también.
        # time.sleep(3)
        # executor.shutdown()

    threading.Thread(target=send_trajectory, daemon=True).start()

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        node2.destroy_node()
        executor.shutdown()
        rclpy.shutdown()

def checkPose(robt, wobj = SE3(), tool = SE3(), ternas = False):
    rclpy.init()
    node_joint = joint_pub()
    if ternas:
        node_TF = TFPublisher()
        executor = MultiThreadedExecutor()
        executor.add_node(node_joint)
        executor.add_node(node_TF)
        node_TF.add_wobj(wobj, 'wobj')
        # print(f'robt\n{robt.pose* tool.inv()}')
        node_TF.add_robt(robt.pose * tool.inv(), 'robt', 'wobj')
        pinza = SE3(0, 120, 50) * SE3.Rx(-np.pi/2)
        node_TF.add_robt(tool, 'tool', 'robt')

        # print(f'Tool en checkPose:\n{tool}')

        def send_pose():
            node_joint.publish_pose(robt, wobj, tool, cobot_tb, joint_names)

        threading.Thread(target=send_pose, daemon=True).start()

        # Ejecutar el executor en un hilo aparte
        def spin_executor():
            executor.spin()

        spin_thread = threading.Thread(target=spin_executor)
        spin_thread.start()

        # Esperar 2 segundos y luego cerrar todo
        time.sleep(2)
        node_joint.destroy_node()
        node_TF.destroy_node()
        executor.shutdown()
        rclpy.shutdown()
        spin_thread.join()

    #     try:
    #         executor.spin()
    #         time.sleep(2)
    #         node_joint.destroy_node()
    #         node_TF.destroy_node()
    #         executor.shutdown()
    #         rclpy.shutdown()
    #     except KeyboardInterrupt:
    #         pass
    #     finally:
    #         node_joint.destroy_node()
    #         node_TF.destroy_node()
    #         executor.shutdown()
    #         rclpy.shutdown()
    else:
        node_joint.publish_pose(robt, wobj, tool, cobot_tb, joint_names)
    #     # rclpy.shutdown()

def checkQ(q, tool = SE3(), ternas= False):
    rclpy.init()
    node_joint = joint_pub()
    # node_joint.publish_q(q, joint_names)

    if ternas:
        node_TF = TFPublisher()
        executor = MultiThreadedExecutor()
        executor.add_node(node_joint)
        executor.add_node(node_TF)
        node_TF.add_robt(cobot_tb.fkine(q), 'brida', 'base')
        # pinza = SE3(0, 120, 50) * SE3.Rx(-np.pi/2)
        pinza = SE3(-1.71381642, 106.90735789, 28.32702833) * SE3.Rx(-np.pi/2)
        node_TF.add_robt(tool, 'tool', 'brida')

        def send_pose():
            q_wgripper = np.concatenate([q, np.zeros(6)])  # Agregar ceros para el gripper
            node_joint.publish_q(q_wgripper, joint_names)

        threading.Thread(target=send_pose, daemon=True).start()

        try:
            executor.spin()
        except KeyboardInterrupt:
            pass
        finally:
            node_joint.destroy_node()
            node_TF.destroy_node()
            executor.shutdown()
            rclpy.shutdown()
    else:
        node_joint.publish_q(q, joint_names)

class BaseRobotController(ABC):
    @abstractmethod
    def MoveJ(self, robt, speed:int = 30, tool:SE3|None=None, wobj:SE3|None=None):
        pass

    @abstractmethod
    def MoveC(self, robt, speed:int = 30, tool:SE3|None=None, wobj:SE3|None=None):
        pass

    @abstractmethod
    def GripperState(self, apertura: float, spd: int):
        pass

class SimManager(BaseRobotController):
    def __init__(self):
        # Inicializamos rclpy solo una vez
        rclpy.init()
        self.node_tf = TFPublisher()
        self.node_joint = joint_pub()
        self.node_obj = ObjectManager()

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

        # Arrancamos el executor en un hilo aparte
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()

        # # Estado actual
        # if q_init is not None:
        #     self.q_current = np.array(q_init)
        # else:
        #     self.q_current = np.zeros(cobot_tb.n)  # robot en home (o ceros)

        while self.node_joint.q_current is None:
            print(">>> Esperando joint_states...")
            time.sleep(0.1)

        self.q_current = self.node_joint.q_current

        print(">>> SimManager iniciado")

    def MoveJ(self, robtarget, speed: int = 30, tool: SE3 = SE3(), wobj: SE3 = SE3(), wobj_name='wobj1', robt_name='robtarget'):
        self.node_tf.add_wobj(wobj, wobj_name)
        self.node_tf.add_robt(robtarget.pose, robt_name, wobj_name)
        # self.node_tf.add_robt(tool, 'TCP', 'link6')
        # pinza = SE3(-1.71381642, 106.90735789, 28.32702833) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)
        self.node_tf.add_robt(tool, 'tool', 'link6')

        # print(f'[DEBUG] Q recibido antes del MoveJ a 0: {self.q_current}')

        # La cinemática inversa devuelve 6 ejes, sin la pinza
        q_brazo = cobot_tb.ikine(wobj * robtarget.pose * tool.inv(), robtarget.config)[0]

        # Respetar la pinza actual
        gripper_val = self.q_current[6] if self.q_current is not None else 0.0
        q_goal = np.concatenate([np.array(q_brazo), [gripper_val]])
        q_start = to_array(self.q_current)[:7]   # <--- forzar 7
        q_end = to_array(q_goal)[:7] 

        traj_q = np.vstack([q_start, q_end, q_end])

        traj_arm = traj_q[:, :6]
        cobot_tb.genTrJoint(traj_arm, np.zeros(traj_arm.shape[0]))

        qtraj_a = cobot_tb.q_ref[::10]
        # print(f'q_traj_a =\n{qtraj_a}')
        q_limit = check_joint_limits(np.rad2deg(qtraj_a), joint_limits)
        if q_limit:
            if len(q_limit) == 1:
                print(f"⚠️  Valor fuera de límite para el eje {q_limit[0]}")
            else:
                ejes = ", ".join(map(str, q_limit))
                print(f"⚠️  Se sobrepasan límites en los ejes: {ejes}")

        qtraj_a_full = [np.concatenate([q, [gripper_val]]) for q in qtraj_a]

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
    
    def MostrarTerna(self, terna, nombre='terna1'):
        self.node_tf.add_wobj(terna, nombre)

    def VerPose(self, wobj, robtarget, tool: SE3 | None = SE3(), wobj_name='wobj1', robt_name='robtarget'):
        self.node_tf.add_wobj(wobj, wobj_name)
        self.node_tf.add_robt(robtarget.pose, robt_name, wobj_name)

        gripper_val = self.q_current[6] if self.q_current is not None else 0.0

        msg = JointState()
        msg.name = joint_names
        pose = wobj * robtarget.pose * tool.inv()
        config = robtarget.config
        try:
            q = cobot_tb.ikine(pose, config)[0]
        except IKineError as e:
            print("Error en el problema inverso:", e)
            return
        # print(f'El q que sale de ikine es tipo {type(q)} y vale {q}')
        q_full = np.concatenate([q, [gripper_val]])
        # q_pos = q.tolist() + [0.0]*6
        # msg.position = q
        # self.q_current = q.tolist() + [0.0]*6

        self._send_pose(q_full, robt_name, wobj_name)
    
    def VerQ(self, q, tool: SE3 | None = SE3()):
        self.node_tf.add_robt(cobot_tb.fkine(q), 'brida', 'base')
        self.node_tf.add_robt(tool, 'tool', 'brida')

        gripper_val = self.q_current[6] if self.q_current is not None else 0.0
        q_full = np.concatenate([q, [gripper_val]])

        # msg = JointState()
        # msg.name = joint_names
        self._send_pose(q_full)

    def _send_pose(self, q_pos, robt_name = 'robt', wobj_name = 'wobj'):

        def send_pose():
            self.node_joint.publish_pose(q_pos, joint_names)

        # pub_thread = threading.Thread(target=send_pose)
        # pub_thread.start()
        # pub_thread.join()
        # self.q_current = pad_for_urdf(q_pos)
        # print(f'El q que sale de _send_pose es {self.q_current}')

        self.node_joint.publish_pose(q_pos, joint_names)

        # Procesar callbacks pendientes para actualizar q_current
        rclpy.spin_once(self.node_joint, timeout_sec=0.01)

        # Asegurar que el estado local refleje la posición enviada
        self.q_current = pad_for_urdf(q_pos)
        # print(f'El q que sale de _send_pose es {self.q_current}')

    def _send_move(self, qtraj_a, robt_name, wobj_name, dt=0.1):
        print(f">>> Move: {robt_name} @ {wobj_name}")

        # def send_trajectory():
        #     self.node_joint.publish_trajectory(qtraj_a, joint_names, dt=0.1)

        # pub_thread = threading.Thread(target=send_trajectory)
        # pub_thread.start()
        # pub_thread.join()

        # self.q_current = qtraj_a[-1].copy()

        for q in qtraj_a:
            # Publicar cada punto de la trayectoria
            self.node_joint.publish_pose(q, joint_names)

            # Procesar callbacks pendientes para mantener q_current actualizado
            rclpy.spin_once(self.node_joint, timeout_sec=0.001)

            # Actualizar q_current local
            self.q_current = pad_for_urdf(q)

            # Espera para simular tiempo de trayectoria
            time.sleep(dt)

        # print(f'Trayectoria finalizada. q_current =\n{self.q_current}')

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
    def __init__(self, host: str = "10.42.0.1", port: int = 9000, rotar_base: bool = True):

        # Conexión con el robot físico
        self.mc = MyCobotSocket(host, port)

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
        # angles = self.mc.get_angles()
        print(f'El cobot estaba en\n{coords}')
        # print(f'Ángulos del cobot\n{angles}')
        # Convertir a formato [x, y, z, rx, ry, rz] en grados.
        pose = list(pose_calc.t) + list(pose_calc.rpy(order='zyx', unit='deg'))
        print(f'Le pedimos al cobot\n{pose}')
        self.mc.sync_send_coords(pose, speed, 1)
        print(f'Terminó llegando a{self.mc.get_coords()}')
    
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
            self.mc.send_angles(np.degrees(q).tolist(), spd)
        elif unit == 'deg':
            self.mc.send_angles(q.tolist(), spd)

    def GripperState(self, apertura: float, spd: int = 30):
        """
        Modifica el estado de la pinza.
        
        Args:
            mov: 0 - Abrir, 1 - Cerrar, 10 - Soltar
            spd: velocidad.
        """
        if apertura < 50:
            mov = 0  # cerrar
        else:
            mov = 1  # abrir
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
        self.GripperState(0)
        time.sleep(5)
        self.GripperState(1)

        if poses is None:
            poses = [None] * cant_poses

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
                tiempo = 15

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
                q_ajustado = self.mc.get_angles()
                print(f'Pose {n+1} guardada: {q_ajustado}')
                poses[n] = q_ajustado
            
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

        return poses
    
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
        x, y, z, rx, ry, rz = pose
        rx, ry, rz = np.deg2rad([rx, ry, rz])
        T_se3 = SE3(x, y, z) * SE3.RPY([rx, ry, rz], order='zyx')
        return T_se3

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

def live_pose(sleep = 0.3, max_stable=10, tol=0.1):
    cobot = MyCobotController()
    rviz = SimManager()
    stable_count = 0
    prev_q = None
    try:
        while True:
            q = cobot.mc.get_angles()
            time.sleep(0.3)
            print(f'Angulos leídos:{q}')
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