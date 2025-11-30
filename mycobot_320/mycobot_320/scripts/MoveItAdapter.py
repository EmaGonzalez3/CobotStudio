# moveit_adapter.py
#!/usr/bin/env python3
import time
import threading
from typing import List, Optional
import inspect
from pathlib import Path
import json
import os

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, Header
from sensor_msgs.msg import JointState
from moveit_msgs.msg import DisplayTrajectory, RobotState
from moveit_msgs.srv import ApplyPlanningScene, GetStateValidity
from moveit_msgs.srv import GetMotionPlan
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint, RobotState
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rclpy.action import ActionClient
from control_msgs.action import GripperCommand
from control_msgs.msg import GripperCommand as GripperCommandMsg

class MoveItAdapter(Node):
    """
    Nodo opcional que encapsula interacciones con MoveIt/RViz:
    - apply_planning_scene (para setear current state)
    - publish Empty a /rviz/moveit/update_goal_state (si Allow External Comm. ON)
    - subscribe a /display_planned_path para capturar el DisplayTrajectory publicado por move_group
    """

    def __init__(self,
                 node_name: str = "moveit_adapter",
                 apply_scene_srv_name: str = "/apply_planning_scene",
                 update_topic: str = "/rviz/moveit/update_goal_state",
                 display_traj_topic: str = "/display_planned_path",
                 project_path: str = None):
        super().__init__(node_name)

        if project_path:
            self.project_root = Path(project_path).resolve()
        else:
            self.project_root = Path.cwd()
        
        self.get_logger().info(f"MoveItAdapter Root: {self.project_root}")

        self.joint_names_full = [
        'joint2_to_joint1',
        'joint3_to_joint2',
        'joint4_to_joint3',
        'joint5_to_joint4',
        'joint6_to_joint5',
        'joint6output_to_joint6',
        'gripper_controller',
        ]
        self.apply_scene_srv_name = apply_scene_srv_name
        self.update_topic = update_topic
        self.display_traj_topic = display_traj_topic
        self.simmanager_ref = None

        # Crear el client si el servicio está disponible
        self._apply_cli = self.create_client(ApplyPlanningScene, self.apply_scene_srv_name)
        self._apply_available = self._apply_cli.wait_for_service(timeout_sec=1.0)


        self.plan_client = self.create_client(GetMotionPlan, "/plan_kinematic_path")
        self.plan_available = self.plan_client.wait_for_service(timeout_sec=1.0)
        if not self.plan_available:
            self.get_logger().warn("MoveItAdapter: servicio '/plan_kinematic_path' no disponible (MoveIt no levantado)")

        # Publisher para actualizar el Goal State en RViz con un mensaje vacío
        self._pub_update = self.create_publisher(Empty, self.update_topic, 10)

        # Subscripción a DisplayTrajectory para capturar trayectorias planeadas con MotionPlanning
        self._last_display_trajectory = None
        self._display_sub = self.create_subscription(
            DisplayTrajectory,
            self.display_traj_topic,
            self._display_cb,
            10
        )

        self.last_robot_trajectory = None   # RobotTrajectory completo
        self.last_q_trajectory = None       # lista de vectores q
        
        # small lock to protect shared data
        self._lock = threading.Lock()

        # Chequeamos que apply_planning_scene esté disponible
        if self._apply_available:
            self.get_logger().info(f"MoveItAdapter: apply_planning_scene service available at '{self.apply_scene_srv_name}'")
        else:
            self.get_logger().warning(f"MoveItAdapter: service '{self.apply_scene_srv_name}' NOT available (MoveIt may not be running)")

    def _display_cb(self, msg: DisplayTrajectory):
        with self._lock:
            # Mantener el último mensaje de DisplayTrajectory
            self._last_display_trajectory = msg

    def is_available(self) -> bool:
        """Devuelve True si al menos el cliente apply_planning_scene está disponible o si /display_planned_path publica."""
        return self._apply_available or (self._last_display_trajectory is not None)

    def apply_goal_state(self, positions: List[float], timeout=5.0, publish_tf:bool = False, tool = 1,  tf_frame_name: str = "moveit_goal", tf_reference: str = "base") -> bool:
        """
        Mueve el Goal State (robot naranja) de MoveIt a un vector de variables articulares dado.
        Llama a /apply_planning_scene para setear planning_scene.robot_state.
        Devuelve True si el servicio respondió success=True.
        """
        # apply_planning_scene tiene que estar disponible
        if not self._apply_available:
            self.get_logger().warning("apply_planning_scene service not available")
            return False
        
        if len(positions) == 7:
            used_joints = self.joint_names_full
        elif len(positions) == 6:
            used_joints = self.joint_names_full[:-1]
        else:
            self.get_logger().error(
            f"apply_goal_state: longitud inválida de posiciones ({len(positions)}). "
            f"Esperaba 6 o 7 elementos para {self.joint_names_full}"
        )
            raise ValueError("Invalid number of joint positions")
        
        # Construir request
        req = ApplyPlanningScene.Request()
        scene = req.scene
        rs = RobotState()
        js = JointState()
        js.header = Header()
        js.header.stamp = self.get_clock().now().to_msg()
        js.name = used_joints
        js.position = positions
        rs.joint_state = js
        scene.robot_state = rs

        # marcar como diff si existe
        try:
            scene.is_diff = True
        except Exception:
            pass

        fut = self._apply_cli.call_async(req)
        self.get_logger().info("Llamando a /apply_planning_scene ...")
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if fut.done() and fut.result() is not None:
            res = fut.result()
            try:
                ok = bool(res.success)
            except Exception:
                ok = False
            self.get_logger().info(f"apply_planning_scene returned success={ok}")
            self.trigger_rviz_update_goal()
            time.sleep(0.5)

            # Publicar la terna de la pose según la herramienta mediante TF.
            if publish_tf and getattr(self, 'simmanager_ref', None) is not None:
                try:
                    # SimManager maneja las ternas con TF y calcula FK con la toolbox
                    self.simmanager_ref.publish_goal_tf(positions, tool = tool, frame_name=tf_frame_name, reference_frame=tf_reference)
                except Exception as e:
                    self.get_logger().warn(f"publish_goal_tf fallo: {e}")
            return ok
        else:
            self.get_logger().error("No hubo respuesta de apply_planning_scene (timeout o error).")
            return False

    def trigger_rviz_update_goal(self, n_publish:int = 3, delay:float = 0.1):
        """
        Actualiza el Goal State del MotionPlanning a la posición actual (<current>) enviando un mensaje vacío. Requiere External Comm. activado en el plugin desde RViz.
        """
        e = Empty()
        for i in range(n_publish):
            self._pub_update.publish(e)
            # self.get_logger().info(f"Publicado Empty en '{self.update_topic}' ({i+1}/{n_publish})")
            time.sleep(delay)

    def last_planned_trajectory(self) -> Optional[DisplayTrajectory]:
        """
        Acceso al último DisplayTrajectory. Necesario para guardarla.
        """
        with self._lock:
            return self._last_display_trajectory

    def wait_for_planned_trajectory(self, timeout: float = 5.0) -> Optional[DisplayTrajectory]:
        """Espera hasta que llegue un DisplayTrajectory en /display_planned_path y lo retorna."""
        t0 = time.time()
        while time.time() - t0 < timeout:
            with self._lock:
                if self._last_display_trajectory is not None:
                    return self._last_display_trajectory
            time.sleep(0.05)
        return None

    def save_last_planned_trajectory(self, filename: str, variable_name: str = "TRAJ") -> bool:
        """
        Guarda la última trayectoria planificada en la carpeta 'Trayectorias' del proyecto actual.
        
        Args:
            filename (str): Nombre del archivo (ej: 'movimientos_llavero.py').
            variable_name (str): Nombre de la variable python (ej: 'TRAY_PICK').
        """
        msg = self.last_planned_trajectory()
        if msg is None or not msg.trajectory:
            self.get_logger().warning("No hay trayectoria planificada para guardar.")
            return False

        # Extraer datos
        jt = msg.trajectory[0].joint_trajectory
        joint_names = jt.joint_names
        q_list = [list(p.positions) for p in jt.points]

        # --- 1. DETECCIÓN DE CARPETA (Igual que en traj_moveit) ---
        # try:
        #     # Buscamos quién llamó a esta función (la rutina)
        #     # stack[1] suele ser el caller inmediato. 
        #     caller_frame = inspect.stack()[1]
        #     caller_filename = caller_frame.filename
        #     project_folder = Path(caller_filename).resolve().parent
        # except Exception:
        #     self.get_logger().warn("No se detectó caller, usando directorio actual.")
        #     project_folder = Path.cwd()

        # Configurar carpeta Trayectorias
        # tray_folder = project_folder / "Trayectorias"
        # tray_folder.mkdir(parents=True, exist_ok=True) # Crea la carpeta si no existe

        # --- 1. DETECCIÓN DE CARPETA (LIMPIA) ---
        # Usamos la raíz almacenada en el objeto
        tray_folder = self.project_root / "Trayectorias"
        tray_folder.mkdir(parents=True, exist_ok=True)

        # Asegurar extensión .py
        if not filename.endswith(".py"):
            filename += ".py"
            
        full_path = tray_folder / filename

        # --- 2. LÓGICA DE GUARDADO (APPEND vs WRITE) ---
        file_exists = full_path.exists()
        
        try:
            # Abrimos en modo 'a' (append) si existe, o 'w' (write) si es nuevo
            mode = 'a' if file_exists else 'w'
            
            with open(full_path, mode) as f:
                # Si es archivo nuevo, escribimos cabecera e imports
                if not file_exists:
                    f.write("# Archivo de Trayectorias generado automáticamente\n")
                    f.write(f"# Joint Names de referencia: {joint_names}\n\n")
                
                # Escribimos la variable separada por saltos de línea
                f.write(f"\n# Trayectoria: {variable_name} (Puntos: {len(q_list)})\n")
                f.write(f"{variable_name} = [\n")
                for q in q_list:
                    f.write(f"    {q},\n")
                f.write("]\n")

            self.get_logger().info(f"Guardado '{variable_name}' en: {full_path}")
            return True

        except Exception as e:
            self.get_logger().error(f"Error guardando trayectoria: {e}")
            return False
            
    def _get_q_start_from_sim(self, timeout=2.0):
        """
        Intentar obtener q_start desde simmanager_ref (SimManager) usando su método público get_current_q().
        """
        if self.simmanager_ref is None:
            self.get_logger().warn("_get_q_start_from_sim: no simmanager_ref registrado")
            return None
        try:
            # Preferible: que SimManager exponga get_current_q(timeout)
            q = self.simmanager_ref.get_current_q(timeout=timeout)
            if q is None:
                self.get_logger().warn("_get_q_start_from_sim: SimManager no devolvió q_current")
                return None
            return list(q)
        except Exception as e:
            self.get_logger().warn(f"_get_q_start_from_sim fallo: {e}")
            return None

    def plan_joint_trajectory(self, q_goal, q_start=None, group_name=None, timeout=5.0):
        """
        Planea una trayectoria con el plugin MotionPlanning de MoveIt. No la ejecuta por lo que la posición del robot no cambia.
        Devuelve (traj_msg, q_list) o (None, None) en caso de error.
        q_goal debe ser lista de len == len(joint_names) (sin gripper si el grupo no lo incluye).
        """
        if not self.plan_available:
            self.get_logger().warn("plan_joint_trajectory(): servicio /plan_kinematic_path no disponible")
            return None, None

        if q_start is None:
            q_start = None
            if hasattr(self, "simmanager_ref") and self.simmanager_ref is not None:
                # Pedimos q_current sin forzar pinza (devolverá lo que haya)
                q_start = self.simmanager_ref.get_current_q(timeout=1.0)
                # q_start puede ser None o array
                if q_start is not None:
                    q_start = list(q_start)

        # Decidimos joint_names a usar y group_name si no especificado
        joint_names_used, q_start_h, q_goal_h, decided_group = self._harmonize_joint_inputs(
            self.joint_names_full, q_start, q_goal
        )

        print(f'DEBUG: group name inicial {group_name}')

        if group_name is None:
            group_name = decided_group

        print(f'DEBUG: decided group {decided_group}')

        # Rechazar si aún falta q_start (cuando no había y SimManager no dio nada)
        if q_start_h is None:
            self.get_logger().warn("q_start no provisto y no se pudo obtener q_current. Aborting plan.")
            return None, None

        # Construir GetMotionPlan.Request
        req = GetMotionPlan.Request()
        mp = MotionPlanRequest()
        mp.group_name = group_name

        start_js = JointState()
        start_js.name = joint_names_used
        start_js.position = q_start_h
        mp.start_state = RobotState(joint_state=start_js)

        goal = Constraints()
        for name, q in zip(joint_names_used, q_goal_h):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = q
            jc.tolerance_above = 0.001
            jc.tolerance_below = 0.001
            jc.weight = 1.0
            goal.joint_constraints.append(jc)
        mp.goal_constraints.append(goal)
        req.motion_plan_request = mp

        # Llamar al servicio
        try:
            fut = self.plan_client.call_async(req)
            self.get_logger().info("Llamando a /plan_kinematic_path ...")
            rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        except Exception as e:
            self.get_logger().error(f"Excepción llamando a plan_kinematic_path: {e}")
            return None, None

        if not fut.done():
            self.get_logger().error("Timeout en /plan_kinematic_path")
            return None, None

        print(f'DEBUG: joint_names_used: {joint_names_used}')
        print(f'DEBUG: q_start_h: {q_start_h}')
        print(f'DEBUG: q_goal_h: {q_goal_h}')
        print(f'DEBUG: group_name: {group_name}')

        res = fut.result()
        if res is None or res.motion_plan_response.error_code.val != 1:
            val = res.motion_plan_response.error_code.val if res is not None else None
            print(f'res_motion_plan_response: {res.motion_plan_response}')
            self.get_logger().warn(f"Planificación falló. Error code={val}")
            return None, None

        traj = res.motion_plan_response.trajectory

        # Convertimos puntos de la trayectoria (waypoints) en lista de vectores q
        q_list = [list(pt.positions) for pt in traj.joint_trajectory.points]

        self.get_logger().info(f"Planificación OK. {len(q_list)} waypoints generados.")
        self.last_robot_trajectory = traj
        self.last_q_trajectory = q_list
        return traj, q_list
   
    def _build_joint_trajectory(self, joint_names: List[str], q_list: List[List[float]],
                            dt: float, q_start: List[float] = None, prepend_start: bool = True) -> JointTrajectory:
        """
        Construye un JointTrajectory a partir de q_list con pasos dt.
        Si prepend_start True añade un primer punto con time_from_start = 0.0 con q_start (o con el primer punto si q_start None).
        Garantiza time_from_start crecientes y que cada point.positions tenga la longitud correcta.
        """
        jt = JointTrajectory()
        jt.joint_names = list(joint_names)
        n = len(joint_names)

        points = []
        # helper: normaliza una lista q a la longitud n (trim o pad con último valor o 0.0)
        def _normalize_q(q):
            q = list(q)
            if len(q) > n:
                return q[:n]
            if len(q) < n:
                fill = q[-1] if q else 0.0
                return (q + [fill] * (n - len(q)))[:n]
            return q

        # punto inicial (time = 0)
        if prepend_start:
            if q_start is None:
                # si no hay q_start usamos el primer punto de q_list (normalizado)
                if not q_list:
                    raise ValueError("_build_joint_trajectory: q_list vacía y q_start None")
                p0_q = _normalize_q(q_list[0])
            else:
                p0_q = _normalize_q(q_start)
            p0 = JointTrajectoryPoint()
            p0.positions = p0_q
            p0.time_from_start = Duration(sec=0, nanosec=0)
            points.append(p0)

        # agregar todos los waypoints de q_list con tiempos dt, 2*dt, ...
        t = dt
        for q in q_list:
            qc = _normalize_q(q)
            p = JointTrajectoryPoint()
            p.positions = qc
            # set duration
            sec = int(t)
            nanosec = int((t - sec) * 1e9)
            p.time_from_start = Duration(sec=sec, nanosec=nanosec)
            points.append(p)
            t += dt

        jt.points = points
        return jt

    def _send_trajectory_action(self, jt: JointTrajectory,
                            controller_action_name: str = '/arm_controller/follow_joint_trajectory',
                            wait_for_result: bool = True,
                            action_timeout: float = 60.0,
                            gripper_action_name: str = '/grip_action_controller/gripper_cmd') -> bool:
        """
        Envía JointTrajectory al action server FollowJointTrajectory.
        Si jt contiene 'gripper_controller' lo separa: envía los joints del brazo al
        controller_action_name y la orden de pinza al gripper_action_name (GripperCommand).
        - Devuelve True solo si los goals aceptados/completados según wait_for_result.
        """

        # Validaciones
        if not jt.points:
            self.get_logger().error("_send_trajectory_action: jt sin puntos -> nothing to send")
            return False

        # Detectar si la pinza está en la lista de joints
        gripper_name = 'gripper_controller'
        has_gripper = gripper_name in jt.joint_names

        # Si no hay gripper, enviar JointTrajectory al arm controller
        if not has_gripper:
            if not hasattr(self, "_traj_action_client") or self._traj_action_client is None:
                self._traj_action_client = ActionClient(self, FollowJointTrajectory, controller_action_name)

            action_client = self._traj_action_client
            if not action_client.wait_for_server(timeout_sec=2.0):
                self.get_logger().error(f"Action server {controller_action_name} no disponible")
                return False

            goal_msg = FollowJointTrajectory.Goal()
            goal_msg.trajectory = jt

            send_goal_future = action_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=5.0)
            goal_handle = send_goal_future.result()
            if goal_handle is None or not goal_handle.accepted:
                self.get_logger().error("Goal rechazado por el action server (brazo)")
                return False
            self.get_logger().info("Goal brazo aceptado")

            if not wait_for_result:
                return True

            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, result_future, timeout_sec=action_timeout)
            res = result_future.result()
            if res is None:
                self.get_logger().error("No se obtuvo resultado de la acción (brazo)")
                return False
            self.get_logger().info(f"Action result (brazo) received: {res}")
            return True

        # Si hay gripper se separan las trayectorias en sus respectivos controladores
        # Trajectory para brazo (todos los joints excepto gripper)
        arm_joint_names = [jn for jn in jt.joint_names if jn != gripper_name]
        arm_jt = JointTrajectory()
        arm_jt.joint_names = arm_joint_names
        arm_points = []
        for p in jt.points:
            q = list(p.positions)
            # Mapear por indice: construir positions sin el elemento de la pinza.
            # Evita problemas con el orden de los joint_names
            pos_map = list(zip(jt.joint_names, p.positions))
            arm_pos = [val for (name, val) in pos_map if name != gripper_name]
            new_p = JointTrajectoryPoint()
            new_p.positions = arm_pos
            new_p.time_from_start = p.time_from_start
            arm_points.append(new_p)
        arm_jt.points = arm_points

        # Enviar la trayectoria del brazo al arm_controller
        # (reutilizamos cliente si existe)
        if not hasattr(self, "_traj_action_client") or self._traj_action_client is None:
            self._traj_action_client = ActionClient(self, FollowJointTrajectory, controller_action_name)
        arm_client = self._traj_action_client
        if not arm_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(f"Action server {controller_action_name} no disponible")
            return False

        arm_goal = FollowJointTrajectory.Goal()
        arm_goal.trajectory = arm_jt
        arm_send_fut = arm_client.send_goal_async(arm_goal)
        rclpy.spin_until_future_complete(self, arm_send_fut, timeout_sec=5.0)
        arm_goal_handle = arm_send_fut.result()
        if arm_goal_handle is None or not arm_goal_handle.accepted:
            self.get_logger().error("Goal rechazado por el action server (brazo)")
            return False
        self.get_logger().info("Goal brazo (sin gripper) aceptado")

        # Extraer target final de la pinza y enviarlo al gripper action (control_msgs/GripperCommand)
        # Tomamos el último point y su valor asociado a gripper_name
        last_point = jt.points[-1]
        pos_map = list(zip(jt.joint_names, last_point.positions))
        gripper_pos = None
        for name, val in pos_map:
            if name == gripper_name:
                gripper_pos = float(val)
                break

        gripper_result_ok = True
        gripper_goal_handle = None
        gripper_client = None
        if gripper_pos is not None:
            try:
                # crear action client para gripper (control_msgs/action/GripperCommand)
                gripper_client = ActionClient(self, GripperCommand, gripper_action_name)
                if gripper_client.wait_for_server(timeout_sec=2.0):
                    ggoal = GripperCommand.Goal()
                    gcmd = GripperCommandMsg()
                    gcmd.position = gripper_pos
                    gcmd.max_effort = 0.0  # 0 -> no limitar
                    ggoal.command = gcmd
                    gsend = gripper_client.send_goal_async(ggoal)
                    rclpy.spin_until_future_complete(self, gsend, timeout_sec=5.0)
                    gripper_goal_handle = gsend.result()
                    if gripper_goal_handle is None or not gripper_goal_handle.accepted:
                        self.get_logger().warn("Goal gripper rechazado o no aceptado")
                        gripper_result_ok = False
                    else:
                        self.get_logger().info("Goal gripper aceptado")
                else:
                    self.get_logger().warn(f"Gripper action server {gripper_action_name} NO disponible")
                    gripper_result_ok = False
            except Exception as e:
                self.get_logger().warn(f"Error enviando goal gripper: {e}")
                gripper_result_ok = False
        else:
            self.get_logger().warn("No se encontró valor final para la pinza en la trayectoria")

        # Esperar resultados. Primero del brazo
        arm_ok = True
        if wait_for_result:
            arm_result_fut = arm_goal_handle.get_result_async()
            rclpy.spin_until_future_complete(self, arm_result_fut, timeout_sec=action_timeout)
            try:
                arm_res = arm_result_fut.result()
                self.get_logger().info(f"Resultado acción brazo: {arm_res}")
            except Exception as e:
                self.get_logger().error(f"Error al obtener resultado brazo: {e}")
                arm_ok = False

            # Esperar resultado del gripper si se envió
            if gripper_client is not None and gripper_goal_handle is not None:
                try:
                    grip_res_fut = gripper_goal_handle.get_result_async()
                    rclpy.spin_until_future_complete(self, grip_res_fut, timeout_sec=5.0)
                    grip_res = grip_res_fut.result()
                    self.get_logger().info(f"Resultado acción gripper: {grip_res}")
                except Exception as e:
                    self.get_logger().warn(f"Error obteniendo resultado gripper: {e}")

        return arm_ok and gripper_result_ok

    def plan_and_execute(self, q_goal, q_start=None, execute=False,
                        controller_action_name: str = '/arm_controller/follow_joint_trajectory',
                        dt: float = 0.1, wait_for_result: bool = True,
                        update_rviz: bool = True, timeout: float = 5.0,
                        prepend_start_point: bool = True):
        """
        Planifica y opcionalmente ejecuta la trayectoria (usando ros2_control action).
        - Si execute=True, se construye la JointTrajectory (con prepend_start_point=True por defecto)
        y se envía al action server indicado.
        - Devuelve q_list (lista de waypoints) si ok, o None si falló.
        """
        _, q_list = self.plan_joint_trajectory(q_goal, q_start=q_start, timeout=timeout)
        if q_list is None:
            return None

        self.last_q_trajectory = q_list

        joint_names_used, q_start_h, q_goal_h, _ = self._harmonize_joint_inputs(self.joint_names_full, q_start, q_goal)

        if update_rviz:
            try:
                self.apply_goal_state(joint_names_used, q_goal_h)
            except Exception:
                self.get_logger().warn("apply_goal_state falló o no disponible.")
            try:
                self.trigger_rviz_update_goal()
            except Exception:
                self.get_logger().warn("trigger_rviz_update_goal falló o no disponible.")

        if execute:
            jt = self._build_joint_trajectory(joint_names_used, q_list, dt, q_start=q_start_h if prepend_start_point else None, prepend_start=prepend_start_point)
            print("jt.joint_names:", jt.joint_names)
            print("len(points):", len(jt.points))
            print("len(first.positions):", len(jt.points[0].positions))

            ok = self._send_trajectory_action(jt, controller_action_name=controller_action_name, wait_for_result=wait_for_result)
            return q_list if ok else None

        return q_list

    def _init_collision_service(self, timeout=3.0):
        """Inicializa el cliente de GetStateValidity si aún no existe."""
        if hasattr(self, "_collision_client"):
            return

        self._collision_client = self.create_client(GetStateValidity, "/check_state_validity")
        if not self._collision_client.wait_for_service(timeout_sec=timeout):
            self.get_logger().warn("Servicio /check_state_validity NO disponible. ¿move_group levantado?")
            self._collision_available = False
        else:
            self.get_logger().info("Servicio /check_state_validity disponible.")
            self._collision_available = True

    def check_collision(self, positions, group_name="arm", timeout=5.0):

        """
        Consulta a MoveIt2 si una configuración articular está en colisión.
        Devuelve:
            True  -> válido (sin colisión)
            False -> en colisión
            None  -> error o servicio no disponible
        """
        # Inicializar servicio on-demand
        self._init_collision_service()

        if not getattr(self, "_collision_available", False):
            self.get_logger().warn("check_collision(): servicio no disponible.")
            return None

        # Construir RobotState
        js = JointState()
        js.name = self.joint_names_full
        js.position = positions
        js.header = Header()
        js.header.stamp = self.get_clock().now().to_msg()

        rs = RobotState()
        rs.joint_state = js

        # Construir request
        req = GetStateValidity.Request()
        req.group_name = group_name
        req.robot_state = rs

        # Llamar servicio
        fut = self._collision_client.call_async(req)
        try:
            rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        except Exception as e:
            self.get_logger().error(f"Error en check_collision(): {e}")
            return None

        if not fut.done() or fut.result() is None:
            self.get_logger().error("check_collision() sin respuesta.")
            return None

        resp = fut.result()

        # True si NO hay colisión, False si hay
        if resp.valid:
            self.get_logger().info("Configuración válida (sin colisión).")
            return True
        else:
            self.get_logger().info("Configuración EN COLISIÓN.")
            return False
        

            # def plan_and_execute(self, joint_names, q_goal, q_start=None, execute=False, simmanager=None, dt=0.1, mode='controller', update_rviz=True, timeout=5.0):
    
    def _harmonize_joint_inputs(self, joint_names_full, q_start, q_goal, gripper_open_val=0.3):
        """
        Normaliza q_start, q_goal y decide joint_names y group_name.
        - joint_names_full: lista completa (7 names).
        - q_start, q_goal: listas o None.
        Retorna: (joint_names_used, q_start_h, q_goal_h, group_name)
        """
        n_full = len(joint_names_full)
        if n_full < 6:
            raise ValueError("joint_names_full debe tener al menos 6 elementos")

        # longitudes (0 si None)
        n_start = len(q_start) if q_start is not None else 0
        n_goal  = len(q_goal)  if q_goal  is not None else 0

        print(f'DEBUG n_start = {n_start}')
        print(f'DEBUG n_goal = {n_goal}')

        # caso: ambos 6 -> usar solo brazo
        if n_start <= 6 and n_goal <= 6:
            joint_names = joint_names_full[:6]
            group = "arm"
            desired_n = 6
        # caso: ambos 7 -> usar full
        elif n_start >= 7 and n_goal >= 7:
            joint_names = joint_names_full[:7]
            group = "arm_with_gripper"
            desired_n = 7
        else:
            # mezcla 6 y 7 -> usar 7 y padear el de 6 con gripper_open_val
            joint_names = joint_names_full[:7]
            group = "arm_with_gripper"
            desired_n = 7


        def norm(q):
            if q is None:
                return None
            qc = list(q)
            if len(qc) >= desired_n:
                return qc[:desired_n]
            # rellenar con gripper_open_val en la última posición si es necesario
            fill = gripper_open_val if desired_n == 7 else (qc[-1] if qc else 0.0)
            return (qc + [fill] * (desired_n - len(qc)))[:desired_n]

        print(f'DEBUG joint_names = {joint_names}')
        print(f'DEBUG group = {group}')
        print(f'DEBUG norm(q_start) = {norm(q_start)}')
        print(f'DEBUG norm(q_goal) = {norm(q_goal)}')
        return joint_names, norm(q_start), norm(q_goal), group