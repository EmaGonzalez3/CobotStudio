from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import rclpy
from rclpy.node import Node
from spatialmath import SE3
import numpy as np
from scipy.spatial.transform import Rotation as R
from DHRobotGT import myCobot320
from sensor_msgs.msg import JointState
import time
from rclpy.executors import MultiThreadedExecutor
import threading
import itertools

cobot = myCobot320(rotar_base=True, metros=False)
robot_model = cobot
joint_names = ["joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3", "joint5_to_joint4", "joint6_to_joint5", "joint6output_to_joint6"]


class RobTarget:
    def __init__(self, pose, config=None, ROS2=False):
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
        
        # if ROS2:
        #     self.pose.t = self.pose.t / 1000.0

        if config is not None:
            if isinstance(config, list) and all(isinstance(x, int) for x in config):
                self.config = config
            else:
                raise ValueError("La configuración debe ser una lista como [1, -1, 1].")
        else:
            self.config = [1, 1, 1]  # Configuración por defecto
        
        self.valid_configs = []

    def find_valid_configs(self, robot_model, tool, wobj):
        T_global = wobj * self.pose * tool.inv()
        # Todas las combinaciones posibles de [±1, ±1, ±1]
        configs = list(itertools.product([1, -1], repeat=3))
        
        valid_solutions = []

        for conf in configs:
            solution = robot_model.ikine(T_global, conf)[0]
            if len(solution) > 0:
                self.valid_configs.append(conf)
    
        if self.valid_configs:
            print("Configuraciones válidas:", ", ".join([str(c) for c in self.valid_configs]))
        else:
            print("⚠️ Ninguna configuración resuelve la IK para esta pose.")
        
    def get_SE3(self):
        """
        Devuelve la pose como objeto SE3.
        """
        return self.pose

    def offset(self, dx=0, dy=0, dz=0, wobj=SE3()):
        """
        Desplaza el RobTarget en el marco del wobj, sin cambiar orientación ni configuración.
        
        Args:
            dx, dy, dz: Desplazamientos en mm respecto al wobj.
            wobj: Objeto SE3 que representa el marco del wobj (default: identidad).

        Returns:
            Nuevo RobTarget desplazado.
        """
        # Vector de desplazamiento en el marco del wobj
        T_offset_local = SE3(dx, dy, dz)
        T_local_offset = T_offset_local * self.pose

        return RobTarget(T_local_offset, config=self.config.copy())
    
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

    def joint_state_callback(self, msg):
        # print(f"[DEBUG] JointState recibido: {msg.position}")
        self.q_current = np.array(msg.position)
    
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
            msg.position = q.tolist()
            self.publisher.publish(msg)
            self.get_logger().info(f'q: {msg.position}')
            time.sleep(dt)

    def publish_pose(self, robt, wobj, robot_model, joint_names):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = joint_names
        pose = wobj * robt.pose # Después agregamos la inversa de la tool
        config = robt.config
        q = cobot.ikine(pose, config)[0]
        msg.position = q.tolist()
        self.publisher.publish(msg)
        self.get_logger().info(f'q: {msg.position}')

    def publish_q(self, q, joint_names):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = joint_names
        msg.position = q.tolist()
        print(q)
        self.publisher.publish(msg)
        self.get_logger().info(f'q: {msg.position}')


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

def jTrajROS(robt1, robt2, tool, wobj1, wobj2, step = 10):
    # La pose se va a tener que calcular como el robtarget desde el wobj para una tool dada.
    # Calcular pose global: wobj * robtarget * inv(tool)
    pose_calc1 = wobj1 * robt1.pose * tool.inv()
    pose_calc2 = wobj2 * robt2.pose * tool.inv()

    q1 = np.array([cobot.ikine(pose_calc1)[0]])
    q2 = np.array([cobot.ikine(pose_calc2)[0]])

    # Convertir a formato [x, y, z, rx, ry, rz] en grados. En realidad no haría falta:
    # vamos a usar ikine para controlar la conf.
    # pose = list(pose_calc.t) + list(pose_calc.rpy(unit='deg'))

    # Usamos la cinemática inversa de la toolbox para tener control de la config.
    # Acá va a haber que agregar un try porque puede llegar a fallar la resolución de ikine.
    robot_model.genTrJoint(np.array([q1[0], q2[0], q2[0]]), 0*np.ones(3))
    q_traj = robot_model.qref[::step]
    return q_traj
    # mc.send_angles(np.degrees(q_pose).tolist(), spd)

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
    q1 = np.array([cobot.ikine(wobj1*rob_clase1.pose)[0]])
    q2 = np.array([cobot.ikine(rob_clase2.pose)[0]])

    # Generamos la trayectoria joint
    cobot.genTrJoint(np.array([q1[0], q2[0], q2[0]]), 0*np.ones(3))
    qtraj_a = cobot.q_ref[::10]

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

def checkPose(robt, wobj, ternas = False):
    rclpy.init()
    node_joint = joint_pub()
    if ternas:
        node_TF = TFPublisher()
        executor = MultiThreadedExecutor()
        executor.add_node(node_joint)
        executor.add_node(node_TF)
        node_TF.add_wobj(wobj, 'wobj')
        node_TF.add_robt(robt.pose, 'robt', 'wobj')

        def send_pose():
            node_joint.publish_pose(robt, wobj, cobot, joint_names)

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
        node_joint.publish_pose(robt, wobj, cobot, joint_names)

def checkQ(q):
    rclpy.init()
    node_joint = joint_pub()
    node_joint.publish_q(q, joint_names)

# if __name__ == '__main__':
#     prueba_sim1()


class SimManager:
    def __init__(self):
        # Inicializamos rclpy solo una vez
        rclpy.init()
        self.node_tf = TFPublisher()
        self.node_joint = joint_pub()

        # Publicamos q = 0 inicial
        msg = JointState()
        msg.name = joint_names
        msg.position = [0.0] * len(joint_names)
        self.node_joint.publisher.publish(msg)

        # Executor multithread para los nodos
        self.executor = MultiThreadedExecutor()
        self.executor.add_node(self.node_tf)
        self.executor.add_node(self.node_joint)

        # Arrancamos el executor en un hilo aparte
        self.executor_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.executor_thread.start()

        # # Estado actual
        # if q_init is not None:
        #     self.q_current = np.array(q_init)
        # else:
        #     self.q_current = np.zeros(cobot.n)  # robot en home (o ceros)

        while self.node_joint.q_current is None:
            print(">>> Esperando joint_states...")
            time.sleep(0.1)

        self.q_current = self.node_joint.q_current

        print(">>> SimManager iniciado")

    def move(self, wobj, robtarget, wobj_name='wobj1', robt_name='robtarget'):
        self.node_tf.add_wobj(wobj, wobj_name)
        self.node_tf.add_robt(robtarget.pose, robt_name, wobj_name)

        q_goal = cobot.ikine(wobj * robtarget.pose)[0]

        q_start = self.q_current.copy()
        q_end = np.array(q_goal)
        traj_q = np.vstack([q_start, q_end, q_end])
        cobot.genTrJoint(traj_q, 0 * np.ones(3))
        qtraj_a = cobot.q_ref[::10]

        print(f">>> Move: {robt_name} @ {wobj_name}")

        def send_trajectory():
            self.node_joint.publish_trajectory(qtraj_a, joint_names, dt=0.1)

        # threading.Thread(target=send_trajectory, daemon=True).start()

        pub_thread = threading.Thread(target=send_trajectory)
        pub_thread.start()

        # Esperar que termine
        pub_thread.join()

        time.sleep(3.0)

        # Actualizamos q_current
        self.q_current = q_end.copy()

    def move_joint(self, wobj, robtarget, wobj_name='wobj1', robt_name='robtarget'):
        self.node_tf.add_wobj(wobj, wobj_name)
        self.node_tf.add_robt(robtarget.pose, robt_name, wobj_name)

        q_goal = cobot.ikine(wobj * robtarget.pose, robtarget.config)[0]

        q_start = self.q_current.copy()
        q_end = np.array(q_goal)
        traj_q = np.vstack([q_start, q_end, q_end])

        cobot.genTrJoint(traj_q, 0 * np.ones(3))
        qtraj_a = cobot.q_ref[::10]

        self._send_move(qtraj_a, robt_name, wobj_name)

    def move_cartesian(self, wobj, robtarget, wobj_name='wobj1', robt_name='robtarget'):
        self.node_tf.add_wobj(wobj, wobj_name)
        self.node_tf.add_robt(robtarget.pose, robt_name, wobj_name)

        pose_goal = wobj * robtarget.pose
        print(self.q_current)
        pose_start = cobot.fkine(self.q_current)
        print(pose_start)
        print(pose_goal)
        # q_end = np.array(pose_goal)
        # traj_q = np.vstack([pose_start, pose_goal, pose_goal])

        cobot.genTrCart([pose_start, pose_goal, pose_goal], 0*np.ones(3), conf = robtarget.config)  # si tu función acepta otros parámetros, los podés poner acá
        # cobot.genTrCart([pose_a, pose_b, pose_b], 0*np.ones(3), conf=config_ros)
        qtraj_a = cobot.q_ref[::10]

        self._send_move(qtraj_a, robt_name, wobj_name)

    def _send_move(self, qtraj_a, robt_name, wobj_name):
        print(f">>> Move: {robt_name} @ {wobj_name}")

        def send_trajectory():
            self.node_joint.publish_trajectory(qtraj_a, joint_names, dt=0.1)

        pub_thread = threading.Thread(target=send_trajectory)
        pub_thread.start()
        pub_thread.join()

        time.sleep(3.0)  # Podés parametrizar este sleep si querés

        self.q_current = qtraj_a[-1].copy()

    def shutdown(self):
        print(">>> Apagando SimManager...")
        self.node_tf.destroy_node()
        self.node_joint.destroy_node()
        self.executor.shutdown()
        rclpy.shutdown()
        print(">>> SimManager finalizado.")

# checkQ(np.zeros(6))

# 1. Definir wobj y robtarget
wobj1 = SE3(100, 100, 100) #* SE3.Rx(np.pi)
rob_clase1 = RobTarget(SE3(50, -120, 220))
rob_clase2 = RobTarget(SE3(50, -120, 200))
wobj2 = SE3(200, 200, 200) * SE3.Rx(np.pi)
rob_nulo = RobTarget(SE3())
TA=SE3([
       [ -1,   0,   0,    103.6 ],
       [  0,  -1,   0,   -89.1  ],
       [  0,   0,   1,    331.4 ],
       [  0,   0,   0,   1      ]
       ])

# Crear robtarget
target = RobTarget(TA, [-1, -1, -1])

# 2. Crear simulador
sim = SimManager()

# 3. Hacer el movimiento
# sim.move_joint(wobj1, rob_clase1, wobj_name='wobj1', robt_name='pt1')
# sim.move_cartesian(wobj1, rob_clase1.offset(0, 20, 0, wobj1), wobj_name='wobj1', robt_name='pt2')

### NO CORRER EN EL COBOT ###
# sim.move_joint(wobj1, rob_nulo, wobj_name='wobj1', robt_name='')
# sim.move_cartesian(wobj1, rob_nulo.offset(20, 0, 0, wobj2), wobj_name='wobj1', robt_name='offset')
### NO CORRER EN EL COBOT ###

# sim.move_joint(rob_nulo.pose, target, wobj_name='wobj', robt_name='robt1')
# sim.move_cartesian(rob_nulo.pose, target.offset(20, 0, 0, rob_nulo.pose), wobj_name='wobj', robt_name='offset')

sim.move_joint(wobj2, rob_nulo, wobj_name='wobj2', robt_name='robt1')
sim.move_cartesian(wobj2, rob_nulo.offset(0, 20, 0, wobj2), wobj_name='wobj2', robt_name='offset')


# 4. Finalizar
sim.shutdown()

# checkPose(RobTarget(SE3(50, -120, 220)), SE3(100, 100, 100), True)
