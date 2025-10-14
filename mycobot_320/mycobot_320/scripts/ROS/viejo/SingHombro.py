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

POSE_sing = sm.SE3([[1,0,0,0],[0,1,0,0.08878],[0,0,1,0.135],[0,0,0,1]])
# POSE_sing = cobot.fkine(qsing_hombro)
q_sing_h, _ = cobot.ikine(POSE_sing) 
print("POSE en la singularidad del hombro:\n",cobot.fkine(q_sing_h))
print("POSE en la singularidad del hombro:\n",POSE_sing)
Jsing_h = cobot.jacob0(q_sing_h)
print(f" El valor singular mas pequeño para esta singularidad es: {cobot.manipulability(q_sing_h,method = 'minsingular')}")
print(f" El nucleo de el Jacobiano es: {null_space(Jsing_h)}")
print(f" El nucleo de el Jacobiano transpuesto es {null_space(Jsing_h.T)}")

### Voy a generar una trayectoria cartesiana que pase cerca de la singularidad
TA_h=sm.SE3([[1,0,0,-0.100],[0,1,0,0.089],[0,0,1,0.335],[0,0,0,1]])
TB_h=sm.SE3([[1,0,0,0.100],[0,1,0,0.089],[0,0,1,0.335],[0,0,0,1]])
conf_psing = cobot.calc_conf(q_sing_h)
# TA_h=sm.SE3(np.array(POSE_sing))
# TB_h=sm.SE3(np.array(POSE_sing))
# TA_h.A[1,3] += 0.5
# TB_h.A[1,3] = TA_h.A[1,3]-0.1
# TA_h.A[0,3] += 0.1
# TB_h.A[0,3] = TA_h.A[0,3]-0.1
# TB_h.A[0,3] = -1*TA_h.A[0,3]
# qtraj_h = np.empty((0,cobot.n))
# Trayectoria_h = ctraj(TA_h,TB_h,100) # Genero una trayectoria cartesiana sobre el eje Y que pasa cerca de la singularidad
# for pose in Trayectoria_h:
# for q in cobot.q_ref:
    # q_ant,status = cobot.ikine(pose, conf_psing) 
    # if status==1:
    #   qtraj_h = np.vstack([qtraj_h,q])
# cobot.plot(qtraj_h, backend='pyplot', limits= np.array([-0.100, 0.300, -0.300, 0.300, 0.000, 0.400]),
            # jointaxes = False ,block=True, name=False)

# q_tray = np.array([cobot.ikine(Trayectoria_h[i])[0] for i in range(100)])

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

    qtraj_h = np.empty((0,cobot.n))
    Trayectoria_h = cobot.genTrCart([TA_h, TB_h, TB_h], 0*np.ones(3))
    qtraj_h = cobot.q_ref[::6]

    # Articulaciones (según .urdf)
    joint_names = ["joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3", "joint5_to_joint4", "joint6_to_joint5", "joint6output_to_joint6"]

    # Publicar la trayectoria
    node.publish_trajectory(qtraj_h, joint_names, dt=0.1)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

