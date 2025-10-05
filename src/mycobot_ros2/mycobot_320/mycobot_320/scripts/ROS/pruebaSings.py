import numpy as np
import roboticstoolbox as rtb
import spatialmath as sm
from spatialmath import SE3
import matplotlib.pyplot as plt
from roboticstoolbox import DHRobot
from scipy.linalg import null_space
from roboticstoolbox.tools.trajectory import ctraj, jtraj
from DHRobotGT import DHRobotGT, myCobot320
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

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

def singHombro(cobot, config_ros):
    pose_a = SE3([
       [ -1,  0,  0,   103.6 ],
       [  0, -1,  0,   -89.1 ],
       [  0,  0,  1,   331.4 ],
       [  0,  0,  0,   1     ]
    ])
    
    pose_b = SE3([
       [ -1,  0,  0,  -103.6 ],
       [  0, -1,  0,   -89.1 ],
       [  0,  0,  1,   331.4 ],
       [  0,  0,  0,   1     ]
    ])
    
    cobot.genTrCart([pose_a, pose_b, pose_b], 0*np.ones(3), conf=config_ros)
    return cobot.q_ref[::4]

def singCodo(cobot, config_ros):
    pose_a = SE3([
       [  0,  -1,  0,   300  ],
       [ -1,   0,  0,  -90.2 ],
       [  0,   0,  -1,  86   ],
       [  0,   0,  0,   1    ]
       ])
    
    pose_b = SE3([
       [  0,  -1,  0,   348  ],
       [ -1,   0,  0,  -90.2 ],
       [  0,   0,  -1,  86   ],
       [  0,   0,  0,   1    ]
       ])
    cobot.genTrCart([pose_a, pose_b, pose_b], 0*np.ones(3), conf=config_ros)
    return cobot.q_ref[::4]

def singMuñeca(cobot, config_ros):
    pose_a = SE3([
           [ 0.07 , -0.   ,  0.998,  114.12],
           [ 0.641, -0.766, -0.045,  127.59],
           [ 0.764,  0.643, -0.053,  296.19],
           [ 0.   ,  0.   ,  0.   ,  1.   ]
           ])
    
    pose_b = SE3([
           [ 0.07 , -0.   ,  0.998,  194.12],
           [ 0.641, -0.766, -0.045,  127.59],
           [ 0.764,  0.643, -0.053,  296.19],
           [ 0.   ,  0.   ,  0.   ,  1.    ]
           ])
    cobot.genTrCart([pose_a, pose_b, pose_b], 0*np.ones(3), conf=config_ros)
    return cobot.q_ref[::4]

def main(trayectoria, config_ros):
    cobot = myCobot320(rotar_base=True)
    cobot_ros = myCobot320(rotar_base=False)

    trayectorias = {
        "singHombro": singHombro,
        "singCodo": singCodo,
        "singMuñeca": singMuñeca,
    }

    if trayectoria not in trayectorias:
        raise ValueError(f"Trayectoria inválida: {trayectoria}. Elegí entre {list(trayectorias.keys())}")
    # config_ros = [-1, -1, -1]

    rclpy.init()
    node = joint_pub()

    q_a = np.radians(np.array([[10, 20, 30, 40, 10, 20]]))

    
    qtraj = trayectorias[trayectoria](cobot, config_ros)

    # q_pose_a = cobot.ikine(pose_a)[0]
    # q_pose_a = np.array([cobot.ikine(pose_a, config_ros)[0]])
    qtraj_a = cobot.q_ref[::4]

    # Articulaciones (según .urdf)
    joint_names = ["joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3", "joint5_to_joint4", "joint6_to_joint5", "joint6output_to_joint6"]

    # Publicar la trayectoria
    # node.publish_trajectory(q_pose_a, joint_names, dt=0.1)
    node.publish_trajectory(qtraj_a, joint_names, dt=0.1)
    node.destroy_node()
    rclpy.shutdown()
    # print(np.degrees(q_pose_a).tolist())


if __name__ == '__main__':
    main("singCodo", [-1, 1, -1])

# Codo: [1, 1, 1] ; [1, -1, 1] ; [-1, 1, -1]
# Muñeca: [1, -1, 1]
# Hombro: [1, 1, 1] ; [-1, -1, -1] ; [-1, 1, 1] ; [1, -1, -1]

# rclpy.init()
# node = joint_pub()
# cobot = myCobot320(rotar_base=True)
# pose_a = SE3([
#        [ 1,  0,  0,   100 ],
#        [  0, 1,  0,   150 ],
#        [  0,  0,  1,   150 ],
#        [  0,  0,  0,   1     ]
#     ])
# q_pose_a = np.array([cobot.ikine(pose_a, [1, 1, 1])[0]])
# joint_names = ["joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3", "joint5_to_joint4", "joint6_to_joint5", "joint6output_to_joint6"]
# node.publish_trajectory(q_pose_a, joint_names, dt=0.1)
# node.destroy_node()
# rclpy.shutdown()