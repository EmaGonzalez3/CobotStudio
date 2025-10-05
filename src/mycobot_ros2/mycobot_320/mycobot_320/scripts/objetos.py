from visualization_msgs.msg import Marker
import rclpy
from rclpy.node import Node
from builtin_interfaces.msg import Duration as MsgDuration
from tf2_ros import Buffer, TransformListener
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Pose
from rclpy.duration import Duration

class ObjectManager(Node):
    def __init__(self):
        super().__init__('object_manager')
        self.publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        self.timer = self.create_timer(0.1, self.publish_marker)

        # Estado inicial: objeto en la mesa
        self.attached = True
        self.fixed_pose = None

        self.base_frame = 'base'
        self.tool_frame = 'tool'

        # TF listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

    def publish_marker(self):
        marker = Marker()
        # marker.header.frame_id = 'robtarget'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'mesa'
        marker.id = 0
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.scale.x = 0.1
        marker.scale.y = 0.1
        marker.scale.z = 0.025
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        # ¿Está agarrado o en la mesa?
        if self.attached:
            marker.header.frame_id = 'tool'   # se mueve con la pinza
            marker.pose.position.x = 0.0     # relativo al TCP
            marker.pose.position.y = 0.0
            marker.pose.position.z = 0.0
        else:
            if self.fixed_pose is not None:
                marker.header.frame_id = "base"
                marker.pose = self.fixed_pose
            else:
                marker.header.frame_id = "base"
                marker.pose.position.x = 0.3
                marker.pose.position.y = 0.0
                marker.pose.position.z = 0.05

        # marker.lifetime = MsgDuration(sec=0, nanosec=0)
        self.publisher.publish(marker)

    def attach(self):
        self.attached = True
        self.fixed_pose = None   # olvidamos la vieja pose fija

    def detach(self):
        # self.attached = False
        dur = Duration(seconds=0.2)
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.tool_frame,
                rclpy.time.Time(),                 # último disponible
                timeout=dur      # evita bloquear indefinidamente
            )
            self.fixed_pose = Pose()
            self.fixed_pose.position.x = tf.transform.translation.x
            self.fixed_pose.position.y = tf.transform.translation.y
            self.fixed_pose.position.z = tf.transform.translation.z
            self.fixed_pose.orientation = tf.transform.rotation

            self.attached = False
        except Exception as e:
            self.get_logger().warn(f"No pude leer TF {self.base_frame}<-{self.tool_frame}: {e}")
# def main():
#     rclpy.init()
#     node = ObjectManager()
#     # try:
#     #     rclpy.spin(node)
#     # except KeyboardInterrupt:
#     #     pass
#     # node.destroy_node()
#     # rclpy.shutdown()
#     rclpy.spin(node)

# if __name__ == '__main__':
#     main()