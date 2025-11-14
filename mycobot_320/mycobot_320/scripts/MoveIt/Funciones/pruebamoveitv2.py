#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from threading import Thread, Event
from pymoveit2 import MoveIt2
from sensor_msgs.msg import JointState
from rclpy.qos import QoSProfile, ReliabilityPolicy

class MoveItController(Node):
    def __init__(self):
        super().__init__('moveit_controller')

        # Evento para señalizar cuando los joint_states estén listos
        self._joint_states_ready = Event()

        # --- Parámetros del Nodo ---
        self.declare_parameter("position", [0.25, 0.0, 0.25])
        self.declare_parameter("quat_xyzw", [1.0, 0.0, 0.0, 0.0])
        self.declare_parameter("cartesian", False)

        # --- Configuración de MoveIt2 ---
        callback_group = ReentrantCallbackGroup()
        joint_names = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3', 'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']
        
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=joint_names,
            base_link_name="base",
            end_effector_name="link6",
            group_name="arm",
            callback_group=callback_group
        )
        self.get_logger().info("Nodo MoveItController inicializado.")

    def _joint_states_callback(self, msg):
        """
        Callback que se activa una vez al recibir el primer mensaje de JointState.
        """
        if not self._joint_states_ready.is_set():
            self.get_logger().info("¡Recibido el primer mensaje de JointStates! El sistema está listo.")
            self._joint_states_ready.set() # Activa el evento

    def wait_for_joint_states(self, timeout_sec=5.0):
        """
        Espera hasta que se reciba el primer mensaje en /joint_states.
        """
        qos_profile = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        
        # Crea una suscripción temporal solo para la espera
        sub = self.create_subscription(
            JointState,
            '/joint_states',
            self._joint_states_callback,
            qos_profile)
        
        self.get_logger().info("Esperando el primer mensaje de /joint_states...")
        
        # Espera a que el evento se active, con un timeout
        ready = self._joint_states_ready.wait(timeout=timeout_sec)
        
        if not ready:
            self.get_logger().error(f"No se recibieron JointStates después de {timeout_sec} segundos.")
        
        # Destruye la suscripción temporal, ya no la necesitamos
        self.destroy_subscription(sub)
        return ready

    def move_to_pose_goal(self, position, quat_xyzw, cartesian=False):
        # ... (este método no cambia)
        self.get_logger().info(f"Moviendo a la pose: posición={position}, orientación={quat_xyzw}")
        self.moveit2.max_velocity = 0.5
        self.moveit2.max_acceleration = 0.5
        self.moveit2.move_to_pose(position=position, quat_xyzw=quat_xyzw, cartesian=cartesian)
        self.moveit2.wait_until_executed()
        self.get_logger().info("Movimiento completado.")

def main(args=None):
    rclpy.init(args=args)

    moveit_controller = MoveItController()

    executor = rclpy.executors.MultiThreadedExecutor(2)
    executor.add_node(moveit_controller)
    executor_thread = Thread(target=executor.spin, daemon=True, args=())
    executor_thread.start()

    # --- LÓGICA DE ESPERA ---
    # Llama al nuevo método para esperar a que todo esté listo
    # antes de enviar cualquier comando.
    if not moveit_controller.wait_for_joint_states():
        rclpy.shutdown()
        executor_thread.join()
        return # Sale del script si no se pudo inicializar

    # --- Ahora puedes usar los métodos de tu clase con seguridad ---
    position = moveit_controller.get_parameter("position").get_parameter_value().double_array_value
    quat_xyzw = moveit_controller.get_parameter("quat_xyzw").get_parameter_value().double_array_value
    cartesian = moveit_controller.get_parameter("cartesian").get_parameter_value().bool_value

    moveit_controller.move_to_pose_goal(list(position), list(quat_xyzw), cartesian)

    moveit_controller.get_logger().info("Enviando segundo objetivo...")
    moveit_controller.move_to_pose_goal(position=[0.0, 0.25, 0.3], quat_xyzw=[0.0, 0.0, 0.0, 1.0])

    rclpy.shutdown()
    executor_thread.join()

if __name__ == '__main__':
    main()