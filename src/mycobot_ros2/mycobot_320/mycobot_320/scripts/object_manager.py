import rclpy
from sensor_msgs.msg import JointState
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, TransformStamped
from builtin_interfaces.msg import Duration as MsgDuration
from tf2_ros import TransformListener, Buffer, TransformBroadcaster
from scipy.spatial.transform import Rotation as R
import numpy as np

CUBE = Marker.CUBE
SPHERE = Marker.SPHERE
CYLINDER = Marker.CYLINDER
MESH = Marker.MESH_RESOURCE

class ManagedObject:
    def __init__(self, name, shape, size, color, pose_init, movable=True, mesh=None,
                 marker_id=0, marker_ns='objects'):
        self.name = name
        self.shape = shape
        self.size = size
        self.color = color
        self.pose_fixed = pose_init  # Pose inicial o última pose al soltar
        self.attached = False
        self.movable = movable
        self.mesh = mesh
        self.rel_pose = None  # Pose relativa tool->obj cuando está agarrado

        self.marker = Marker()
        self.marker.ns = marker_ns
        # self.marker.ns = 'objects'
        # self.marker.id = hash(name) % 2**31  # id único
        self.marker.type = shape
        self.marker.action = Marker.ADD

        # Damos la posibilidad de usar RGB o RGBA (transparencia)
        if len(color) == 3:
            r, g, b = color
            a = 1.0
        elif len(color) == 4:
            r, g, b, a = color
        else:
            raise ValueError("El color debe ser una tupla/lista con 3 (RGB) o 4 (RGBA) valores")
    
        self.marker.scale.x, self.marker.scale.y, self.marker.scale.z = size
        self.marker.color.r, self.marker.color.g, self.marker.color.b, self.marker.color.a = (r, g, b, a)
        self.marker.lifetime = MsgDuration(sec=0)  # visible indefinidamente
        self.marker.id = marker_id

        if mesh and shape == Marker.MESH_RESOURCE:
            self.marker.mesh_resource = mesh
            self.marker.mesh_use_embedded_materials = True

class ObjectManager(Node):
    def __init__(self, base_frame='base', tool_frame='tool', freq=5.0):
        super().__init__("object_manager")
        self.base_frame = base_frame
        self.tool_frame = tool_frame
        self.publisher = self.create_publisher(MarkerArray, '/visualization_marker_array', 10)

        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.objects = {}  # dict: nombre -> ManagedObject

        self._next_id = 0  # contador para IDs únicos

        # Guardar último timestamp recibido de joint_states
        self.last_stamp = None

        # Suscripción a /joint_states
        self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

    def joint_state_callback(self, msg):
        # Guardamos el último timestamp
        self.last_stamp = msg.header.stamp
        # opcional: debug
        # print(f"Último stamp guardado: {self.last_stamp.sec}.{self.last_stamp.nanosec}")    
        self._update_markers()

    def publish_object_tf(self, obj_name):
        if obj_name not in self.objects:
            self.get_logger().warn(f"Objeto {obj_name} no existe")
            return

        obj = self.objects[obj_name]
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base'           # marco de referencia
        t.child_frame_id = f"{obj_name}_frame"

        # posición
        t.transform.translation.x = obj.pose_fixed.position.x
        t.transform.translation.y = obj.pose_fixed.position.y
        t.transform.translation.z = obj.pose_fixed.position.z

        # orientación
        t.transform.rotation = obj.pose_fixed.orientation

        self.tf_broadcaster.sendTransform(t)

    def add_object(self, name, pose_init, size, color=(0.5, 0.5, 0.5),
                   shape=Marker.CUBE, movable=True, mesh=None, rot_euler=(0.0, 0.0, 0.0)):
        """Agrega un objeto a la escena"""
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = pose_init
        # pose.orientation.w = 1.0  # sin rotación inicial
        quat = R.from_euler('xyz', rot_euler, degrees=False).as_quat()
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quat

        marker_id = self._alloc_id()
        obj = ManagedObject(name, shape, size, color, pose, movable, mesh,
                            marker_id=marker_id, marker_ns="objects")
        obj.marker.pose = pose
        obj.marker.mesh_use_embedded_materials = False # Para cambiar el color del STL

        # print(f"[DEBUG] base_frame={self.base_frame} (type={type(self.base_frame)})")
        obj.marker.header.frame_id = 'base'

        self.objects[name] = obj
        self._update_markers()
        self.get_logger().info(f"Objeto agregado: {name}(ns=objects, id={marker_id})")

    def attach(self, name):
        if name not in self.objects:
            self.get_logger().warn(f"Objeto {name} no existe")
            return
        obj = self.objects[name]
        if not obj.movable:
            self.get_logger().warn(f"Objeto {name} es fijo, no puede adherirse")
            return
        
        try:
            # Tool en coordenadas de base
            tf_bt = self.tf_buffer.lookup_transform(
                'base',
                'link6',#self.tool_frame
                rclpy.time.Time()
            )
            Tb_tool = tf_to_matrix(tf_bt.transform) # Pasamos a matriz

            # T_base_obj actual (el marker fijo en base antes de agarrar)
            Tb_obj = pose_to_matrix(obj.pose_fixed) # Objeto respecto a base

            # Guardamos offset relativo: objeto respecto a tool = base respecto a tool * objeto respecto a base
            obj.rel_pose = np.linalg.inv(Tb_tool) @ Tb_obj

            obj.attached = True
            self.get_logger().info(f"Objeto {name} adherido a {self.tool_frame}")
        except Exception as e:
            self.get_logger().warn(f"No pude calcular offset al adherir: {e}")

    def detach(self, name):
        if name not in self.objects:
            self.get_logger().warn(f"Objeto {name} no existe")
            return
        obj = self.objects[name]
        if not obj.attached:
            return

        try:
            # T_base_tool
            tf_bt = self.tf_buffer.lookup_transform(
                'base',
                'link6', #self.tool_frame
                rclpy.time.Time()
            )
            Tb_tool = tf_to_matrix(tf_bt.transform)

            # Pose real del objeto al soltar: T_base_obj = T_base_tool * T_tool_obj
            Tb_obj = Tb_tool @ obj.rel_pose if obj.rel_pose is not None else Tb_tool
            obj.pose_fixed = matrix_to_pose(Tb_obj)

        except Exception as e:
            self.get_logger().warn(f"No pude leer TF {'base'}<-{self.tool_frame}: {e}")

        obj.attached = False
        obj.rel_pose = None
        self.get_logger().info(f"Objeto {name} liberado en posición actual")

    def _update_markers(self):
        arr = MarkerArray()
        for obj in self.objects.values():
            obj.marker.header.stamp = rclpy.time.Time().to_msg()

            if obj.attached and obj.rel_pose is not None:
                try:
                    tf_bt = self.tf_buffer.lookup_transform(
                        'base',
                        'link6',#self.tool_frame
                        rclpy.time.Time()
                    )
                    Tb_tool = tf_to_matrix(tf_bt.transform)

                    # T_base_obj = T_base_tool * T_tool_obj
                    Tb_obj = Tb_tool @ obj.rel_pose

                    obj.marker.header.frame_id = 'base'
                    obj.marker.pose = matrix_to_pose(Tb_obj)

                except Exception as e:
                    self.get_logger().warn(f"No pude actualizar objeto {obj.name}: {e}")
            else:
                obj.marker.header.frame_id = 'base'
                obj.marker.pose = obj.pose_fixed

            arr.markers.append(obj.marker)
            self.publish_object_tf(obj.name)

        # for obj in self.objects.values():
        #     print(obj.name, obj.marker.ns, obj.marker.id)

        self.publisher.publish(arr)

    def _alloc_id(self):
        mid = self._next_id
        self._next_id += 1
        return mid

""" Helpers """

def tf_to_matrix(tf):  # Transform -> 4x4
    T = np.eye(4)
    t = [tf.translation.x, tf.translation.y, tf.translation.z]
    q = [tf.rotation.x, tf.rotation.y, tf.rotation.z, tf.rotation.w]
    T[:3, :3] = R.from_quat(q).as_matrix()
    T[:3,  3] = t
    return T

def pose_to_matrix(pose: Pose):  # Pose -> 4x4
    T = np.eye(4)
    T[:3, 3] = [pose.position.x, pose.position.y, pose.position.z]
    q = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]
    T[:3, :3] = R.from_quat(q).as_matrix()
    return T

def matrix_to_pose(T):  # 4x4 -> Pose
    pose = Pose()
    pose.position.x, pose.position.y, pose.position.z = T[:3, 3]
    q = R.from_matrix(T[:3, :3]).as_quat()
    pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = q
    return pose
