from geometry_msgs.msg import TransformStamped, PoseStamped
from tf2_ros import TransformBroadcaster
import rclpy
from rclpy.node import Node
from spatialmath import SE3
from scipy.spatial.transform import Rotation as R

class TFPublisher(Node):
    def __init__(self):
        super().__init__('robtarget_tf_publisher')
        self.br = TransformBroadcaster(self)
        self.robtarget = SE3(100, 0, 200) * SE3.RPY([0, 0, 90], unit='deg')  # valor inicial
        self.timer = self.create_timer(0.1, self.publish_tf)

        # Subscriptor al tópico 'robtarget_pose'
        self.sub = self.create_subscription(
            PoseStamped,
            'robtarget_pose',
            self.pose_callback,
            10
        )

    def pose_callback(self, msg: PoseStamped):
        # Convertir PoseStamped → SE3
        t = msg.pose.position
        q = msg.pose.orientation
        rot = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
        T = SE3(rot, [t.x * 1000, t.y * 1000, t.z * 1000])  # convertir a mm
        self.robtarget = T
        self.get_logger().info(f"RobTarget actualizado: {T.t.tolist()}")

    def publish_tf(self):
        if self.robtarget is None:
            return

        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base'
        t.child_frame_id = 'robtarget1'

        pos = self.robtarget.t
        rot = self.robtarget.R
        quat = R.from_matrix(rot).as_quat()

        t.transform.translation.x = pos[0] / 1000.0
        t.transform.translation.y = pos[1] / 1000.0
        t.transform.translation.z = pos[2] / 1000.0
        t.transform.rotation.x = quat[0]
        t.transform.rotation.y = quat[1]
        t.transform.rotation.z = quat[2]
        t.transform.rotation.w = quat[3]

        self.br.sendTransform(t)

def main():
    rclpy.init()
    node = TFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
