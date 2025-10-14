import numpy as np
import roboticstoolbox as rtb
import spatialmath as sm
import matplotlib.pyplot as plt
from roboticstoolbox import DHRobot
from scipy.linalg import null_space
from roboticstoolbox.tools.trajectory import ctraj, jtraj
from DHRobotGT import DHRobotGT, myCobot320
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

cobot = myCobot320()

# Configuración dada por el codo
q0 = np.radians(np.array([[0, -30.0, 60.0, -45.0, 30, 0.0]])) #conf = [1, 1, 1]
pose0 = cobot.fkine(q0)
q0a = np.array([cobot.ikine(pose0, [1, -1, 1])[0]])

# Configuración dada por el hombro
q1 = np.radians(np.array([[45, 10, 45, 110.0, 60, 0.0]])) #conf = [1, 1, 1]
pose1 = cobot.fkine(q1)
q1a = np.array([cobot.ikine(pose1, [1, 1, 1])[0]])
# print(cobot.calc_conf(q1))
# print(q1a)

# Configuración dada por la muñeca
q2 = np.radians(np.array([[70, -40.0, 20.0, -145.0, 20, 0.0]])) #conf = [1, 1, 1]
pose2 = cobot.fkine(q2)
q2a = np.array([cobot.ikine(pose2, [1, 1, -1])[0]])
print(cobot.calc_conf(q2))
print(q2a)

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
        for q in trajectory:
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

    Trayectoria_codo = cobot.genTrJoint(np.array([q0[0], q0a[0], q0a[0]]), 0*np.ones(3))
    qtraj_codo = cobot.q_ref[::6]

    Trayectoria_hombro = cobot.genTrJoint(np.array([q1[0], q1a[0], q1a[0]]), 0*np.ones(3))
    qtraj_hombro = cobot.q_ref[::6]

    Trayectoria_muñeca = cobot.genTrJoint(np.array([q2[0], q2a[0], q2a[0]]), 0*np.ones(3))
    qtraj_muñeca = cobot.q_ref[::6]

    # Articulaciones (según .urdf)
    joint_names = ["joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3", "joint5_to_joint4", "joint6_to_joint5", "joint6output_to_joint6"]

    # Publicar la trayectoria
    node.publish_trajectory(qtraj_muñeca, joint_names, dt=0.1)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

