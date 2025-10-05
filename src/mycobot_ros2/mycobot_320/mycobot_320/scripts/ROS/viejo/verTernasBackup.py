from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import rclpy
from rclpy.node import Node
from spatialmath import SE3
import numpy as np

# class TFPublisher(Node):
#     def __init__(self):
#         super().__init__('robtarget_tf_publisher')
#         self.br = TransformBroadcaster(self)
#         self.timer = self.create_timer(0.1, self.publish_tf)

#         # Simulamos un robtarget en RViz
#         self.robtarget = SE3(500, 100, 200) * SE3.RPY([0, 0, 90], unit='deg')

#     def publish_tf(self):
#         t = TransformStamped()
#         t.header.stamp = self.get_clock().now().to_msg()
#         t.header.frame_id = 'base'
#         t.child_frame_id = 'robtarget1'

#         pos = self.robtarget.t
#         rot = self.robtarget.R

#         # Convert rotation matrix to quaternion
#         from scipy.spatial.transform import Rotation as R
#         quat = R.from_matrix(rot).as_quat()

#         t.transform.translation.x = pos[0] / 1000.0  # ROS usa metros
#         t.transform.translation.y = pos[1] / 1000.0
#         t.transform.translation.z = pos[2] / 1000.0
#         t.transform.rotation.x = quat[0]
#         t.transform.rotation.y = quat[1]
#         t.transform.rotation.z = quat[2]
#         t.transform.rotation.w = quat[3]

#         self.br.sendTransform(t)

# rclpy.init()
# node = TFPublisher()
# # rclpy.spin(node)
# rclpy.spin_once(node)  # Publica una vez y termina
# # node.destroy_node()
# # rclpy.shutdown()

from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import rclpy
from rclpy.node import Node
from spatialmath import SE3
import numpy as np
from scipy.spatial.transform import Rotation as R

class TFPublisher(Node):
    # NOTA: RViz mantiene listados todos los frames que escuchó al menos una vez.
    # Si se eliminan ternas de este script, puede que sigan apareciendo en la pestaña TF de RViz.
    # Esto es solo visual; el frame ya no se publica ni influye en la escena.

    def __init__(self):
        super().__init__('multi_tf_publisher')
        self.br = TransformBroadcaster(self)
        self.timer = self.create_timer(0.1, self.publish_all_transforms)
        
        # Diccionario para almacenar las ternas: {frame_name: SE3}
        self.transforms = {}

        # Agregamos un ejemplo
        # self.add_tf(SE3(0.5, 0.1, 0.2) * SE3.RPY([0, 0, 90], unit='deg'), 'robtarget1')

        # Ejemplo: wobj y robtarget
        self.add_wobj(SE3(0.5, 0.0, 0.2), 'wobj1')
        self.add_robt(SE3(0.1, 0.0, 0.0), 'robtarget1', 'wobj1')

    def add_tf(self, transform: SE3, name: str):
        """Agrega una nueva terna SE3 con un nombre único."""
        self.transforms[name] = transform

    def publish_all_transforms(self):
        now = self.get_clock().now().to_msg()

        for name, se3 in self.transforms.items():
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = 'base'
            t.child_frame_id = name

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

# Main
def main():
    rclpy.init()
    node = TFPublisher()

    # Agregar más ternas
    # node.add_tf(SE3(0.5, 0.2, 0.3) * SE3.RPY([0, 45, 0], unit='deg'), 'robtarget2')
    # node.add_tf(SE3(0.3, 0.5, 0.5), 'robtarget3')
    # node.add_tf(SE3(0.2, 0.2, 0.2), 'robtarget4')

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
