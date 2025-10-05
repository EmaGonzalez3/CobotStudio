import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import roboticstoolbox as rtb
import numpy as np

class myCobot320(rtb.DHRobot):
    def __init__(self):
        # Definición de los enlaces usando parámetros DH
        eje1 = rtb.RevoluteDH(alpha=-np.pi/2, a=0, d=173.9e-3, offset=0, qlim=[-170*np.pi/180, 170*np.pi/180])
        eje2 = rtb.RevoluteDH(alpha=0, a=135e-3, d=0, offset=-np.pi/2, qlim=[-120*np.pi/180, 120*np.pi/180])
        eje3 = rtb.RevoluteDH(alpha=0, a=120e-3, d=0, offset=0, qlim=[-148*np.pi/180, 148*np.pi/180]) 
        eje4 = rtb.RevoluteDH(alpha=np.pi/2, a=0, d=88.78e-3, offset=np.pi/2, qlim=[-120*np.pi/180, 135*np.pi/180])
        eje5 = rtb.RevoluteDH(alpha=-np.pi/2, a=0, d=95e-3, offset=0, qlim=[-169*np.pi/180, 169*np.pi/180])    
        eje6 = rtb.RevoluteDH(alpha=0, a=0, d=65.5e-3, offset=0, qlim=[-180*np.pi/180, 180*np.pi/180])

        # Crear la estructura del robot
        super().__init__([eje1, eje2, eje3, eje4, eje5, eje6], name='myCobot320')

class joint_pub(Node):
    def __init__(self):
        super().__init__('joint_state_publisher')
        self.publisher = self.create_publisher(JointState, '/joint_states', 10)
    
    def publish_trajectory(self, trajectory, joint_names, dt=1):
        """
        :param trajectory: Trayectoria a publicar
        :param joint_names: Nombre de las articulaciones. Verificar que coincida con la definición del robot (.urdf).
        :param dt: Tiempo de espera entre la publicación de cada pose de la trayectoria.
        """
        for q in trajectory.q:
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = joint_names
            msg.position = q.tolist()
            self.publisher.publish(msg)
            self.get_logger().info(f'q: {msg.position}')
            time.sleep(dt)

def main():
    rclpy.init()
    node = joint_pub()

    robot = myCobot320()

    # Generador de trajectoria joint de la toolbox
    q_start = np.zeros(robot.n)  
    q_end = np.array([-1.5, -1.5, 0.5, -1.5, 0.5, 1.0])
    trajectory = rtb.jtraj(q_start, q_end, 100)

    # Articulaciones (según .urdf)
    joint_names = ["joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3", "joint5_to_joint4", "joint6_to_joint5", "joint6output_to_joint6"]

    # Publicar la trayectoria
    node.publish_trajectory(trajectory, joint_names, dt=0.1)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()