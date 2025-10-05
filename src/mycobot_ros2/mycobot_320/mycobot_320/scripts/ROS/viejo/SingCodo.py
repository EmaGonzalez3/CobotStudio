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

q_sing_c = np.array([0,  np.pi/2,  0,  0,  -np.pi/2, 0])
print("POSE en la singularidad del codo:\n",cobot.fkine(q_sing_c))
Jsing_c = cobot.jacob0(q_sing_c)

tol_dir = 1e-3
eje_nullJ_c = np.where(np.abs(null_space(Jsing_c)) < tol_dir, 0, np.sign(null_space(Jsing_c)))
eje_nullJt_c = np.where(np.abs(null_space(Jsing_c.T)) < tol_dir, 0, np.sign(null_space(Jsing_c.T)))

print(f" El valor singular mas pequeño para esta singularidad es: {cobot.manipulability(q_sing_c,method = 'minsingular')}")
print(f" Ejes que no producen velocidad en la brida: null(J): \n{eje_nullJ_c}")
print(f" Ejes en que el robot no se puede mover: null(J^T): \n{eje_nullJt_c}")

### Voy a generar una trayectoria cartesiana que pase cerca de la singularidad
q_arround_sing_c = q_sing_c + np.array([0,0,0.15,0,0,0])
conf_arround_sing = cobot.calc_conf(q_arround_sing_c)
print("POSE cercana a la singularidad del codo:\n",cobot.fkine(q_arround_sing_c))
# TA_c=sm.SE3([[0,-1,0,0.25],[-1,0,0,0.08878],[0,0,-1,0.09774],[0,0,0,1]])
# TB_c=sm.SE3([[0,-1,0,0.36],[-1,0,0,0.08878],[0,0,-1,0.09774],[0,0,0,1]])
TA_c=sm.SE3([[0,-0.9988,0.04998,-0.30],[-1,0,0,0.08878],[0,0.04998 ,-0.9988,0.09774],[0,0,0,1]])
TB_c=sm.SE3([[0,-0.9988,0.04998,-0.355],[-1,0,0,0.08878],[0,0.04998 ,-0.9988,0.09774],[0,0,0,1]])
qtraj_c = np.empty((0,cobot.n))

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

    robot = myCobot320()

    qtraj_c = np.empty((0,cobot.n))
    Trayectoria_c = cobot.genTrCart([TA_c, TB_c, TB_c], 0*np.ones(3))
    qtraj_c = cobot.q_ref[::6]

    # Articulaciones (según .urdf)
    joint_names = ["joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3", "joint5_to_joint4", "joint6_to_joint5", "joint6output_to_joint6"]

    # Publicar la trayectoria
    node.publish_trajectory(qtraj_c, joint_names, dt=0.1)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()