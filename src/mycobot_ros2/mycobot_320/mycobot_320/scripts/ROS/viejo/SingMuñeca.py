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
from spatialmath import SE3

cobot = myCobot320()
cobot.base = SE3.Rz(np.pi)

q_sing_m = np.array([0,  0,  np.pi/2,  0,  0, 0])
# print("POSE en la singularidad de la muñeca:\n",cobot.fkine(q_sing_m))
# cobot.plot(q_sing_m, backend='pyplot', limits= np.array([-0.100, 0.300, -0.300, 0.300, 0.000, 0.400]), jointaxes = True, block=False, name=False)
Jsing_m = cobot.jacob0(q_sing_m)

tol_dir = 1e-3
eje_nullJ_m = np.where(np.abs(null_space(Jsing_m)) < tol_dir, 0, np.sign(null_space(Jsing_m)))
eje_nullJt_m = np.where(np.abs(null_space(Jsing_m.T)) < tol_dir, 0, np.sign(null_space(Jsing_m.T)))
# print(f" El valor singular mas pequeño para esta singularidad es: {cobot.manipulability(q_sing_m,method = 'minsingular')}")
# print(f" Ejes que no producen velocidad en la brida: null(J): \n{eje_nullJ_m}")
# print(f" Ejes en que el robot no se puede mover: null(J^T): \n{eje_nullJt_m}")

### Voy a generar una trayectoria cartesiana que pase cerca de la singularidad
q_arround_sing_m = q_sing_m + np.array([0,0,0,0,0.1,0])
conf_arround_sing = cobot.calc_conf(q_arround_sing_m)
# print("POSE cercana a la singularidad de la muñeca:\n",cobot.fkine(q_arround_sing_m))
TA_m=sm.SE3([[0,-1,0,-0.145],[0.09983, 0 ,0.995,0.124],[-0.995,0  ,0.09983,0.154],[0,0,0,1]])
TB_m=sm.SE3([[0,-1,0,-0.145],[0.09983, 0 ,0.995,-0.124],[-0.995,0  ,0.09983,0.154],[0,0,0,1]])
qtraj_m = np.empty((0,cobot.n))
R = np.array([[0,-1 ,0 ],[0.09983, 0 ,0.995 ],[-0.995,0  ,0.09983 ]])  #Matriz auxiliar para reemplazar, debido a limitacion de la funcion ctraj
# Trayectoria_m = ctraj(TA_m,TB_m,500) # Genero una trayectoria cartesiana sobre el eje Y que pasa cerca de la singularidad
# for pose in Trayectoria_m:
#     pose.A[:3, :3] = R 
#     q_ant,status = cobot.ikine(pose, conf_arround_sing) 
#     if status==1:
#       qtraj_m = np.vstack([qtraj_m,q_ant])

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
    # posicion = np.radians(np.array([[90, -10.0, -80.0, -110.0, 0, 0.0]]))
    posicion = np.radians(np.array([[90, 20.0, -80.0, -70.0, 4.0, 0.0]]))
    POSEsing = cobot.fkine(posicion)
    TA_m2 = sm.SE3(np.array(POSEsing.A))
    TA_m2.t[0] += -0.05
    # TA_m2.t[1] += -0.05
    TB_m2 = sm.SE3(np.array(POSEsing.A))
    TB_m2.t[0] += +0.05
    # TB_m2.t[1] += +0.05
    TA=SE3([
           [ 0.07 , -0.   ,  0.998,  134.12],
           [ 0.641, -0.766, -0.045,  127.59],
           [ 0.764,  0.643, -0.053,  296.19],
           [ 0.   ,  0.   ,  0.   ,  1.   ]
           ])
    
    TB=SE3([
           [ 0.07 , -0.   ,  0.998,  174.12],
           [ 0.641, -0.766, -0.045,  127.59],
           [ 0.764,  0.643, -0.053,  296.19],
           [ 0.   ,  0.   ,  0.   ,  1.    ]
           ])
    # Trayectoria_c = cobot.genTrCart([TA_m2, TB_m2, TB_m2], 0*np.ones(3), [1, -1, 1])
    Trayectoria_c = cobot.genTrCart([TA, TB, TB], 0*np.ones(3), [1, -1, 1])
    qtraj_c = cobot.q_ref[::3]

    # Articulaciones (según .urdf)
    joint_names = ["joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3", "joint5_to_joint4", "joint6_to_joint5", "joint6output_to_joint6"]
    
    # trayectoria = cobot.traj_nucleo(posicion[0], 200, 0.01, 2)

    # Publicar la trayectoria
    node.publish_trajectory(qtraj_c, joint_names, dt=0.1)
    # node.publish_trajectory(trayectoria, joint_names, dt=0.1)

    # Publicar una pose
    # posicion = np.array([cobot.ikine(TB_m2, [1, -1, 1])[0]])
    # posicion2 = np.array([cobot.ikine(TB_m2)[0]])
    print(posicion)
    # node.publish_trajectory(posicion, joint_names)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()