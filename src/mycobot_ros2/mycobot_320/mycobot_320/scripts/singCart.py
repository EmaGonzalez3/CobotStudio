from pymycobot import MyCobotSocket
import time
import numpy as np
from DHRobotGT import myCobot320
from spatialmath import SE3

mc = MyCobotSocket('10.42.0.1', 9000)
cobot = myCobot320(rotar_base=True)

# Antes de mover el robot chequeamos la conexión
if mc.is_controller_connected() != 1:
    print("Please connect the robot arm correctly for program writing")
    exit(0)

def matrix_to_pose(T_matrix):
    T = SE3(T_matrix)
    x, y, z = T.t       # Extraer traslación

    # Extraer rotación como RPY (en radianes), orden ZYX
    rx, ry, rz = T.rpy(order='zyx', unit='rad')
    rx, ry, rz = np.rad2deg([rx, ry, rz])   # Convertimos a grados
    return [x, y, z, rx, ry, rz]

def pose_to_matrix(pose):
    x, y, z, rx, ry, rz = pose

    # Pasamos a radianes
    rx, ry, rz = np.deg2rad([rx, ry, rz])

    # Crear objeto SE3 con rotación en RPY (Z-Y-X por defecto) y traslación
    T = SE3(x, y, z) * SE3.RPY([rx, ry, rz], order='zyx')
    T_matrix = T.A

    return T_matrix

def singHombro(spd = 20, config = [1, -1, -1]):
    """
    Envía una trayectoria al robot mediante pyMyCobot pasando cerca de la singularidad del hombro.
    """

    TA=SE3([
       [ -1,   0,   0,    103.6 ],
       [  0,  -1,   0,   -89.1  ],
       [  0,   0,   1,    331.4 ],
       [  0,   0,   0,   1      ]
       ])

    TB=SE3([
       [ -1,   0,   0,   -103.6  ],
       [  0,  -1,   0,   -89.1   ],
       [  0,   0,   1,    331.4  ],
       [  0,   0,   0,   1       ]
       ])
    
    q_TA = cobot.ikine(TA, config)[0]
    print(f'Ángulos pedidos:\n{np.degrees(q_TA).tolist()}')
    mc.send_angles(np.degrees(q_TA).tolist(), 30)

    coords_TA = matrix_to_pose(TA)
    coords_TB = matrix_to_pose(TB)

    # mc.send_coords(coords_TA, spd, 0)
    time.sleep(7)
    print(f'Ángulos de la pose inicial:\n{mc.get_angles()}')
    print(f'Pose inicial alcanzada:\n{mc.get_coords()}')
    mc.send_coords(coords_TB, spd, 1)

def singCodo(spd = 20, config = [1, -1, -1]):
    """
    Envía una trayectoria al robot mediante pyMyCobot pasando cerca de la singularidad del codo.
    """
    TA=SE3([
       [  0,  -1,  0,   300  ],
       [ -1,   0,  0,  -90.2 ],
       [  0,   0,  -1,  86   ],
       [  0,   0,  0,   1    ]
       ])
    
    TB=SE3([
       [  0,  -1,  0,   355  ],
       [ -1,   0,  0,  -90.2 ],
       [  0,   0,  -1,  86   ],
       [  0,   0,  0,   1    ]
       ])

    q_TA = cobot.ikine(TA, config)[0]
    print(f'Ángulos pedidos:\n{np.degrees(q_TA).tolist()}')
    mc.send_angles(np.degrees(q_TA).tolist(), 30)

    coords_TA = matrix_to_pose(TA)
    coords_TB = matrix_to_pose(TB)

    # mc.send_coords(coords_TA, spd, 0)
    time.sleep(7)
    print(f'Ángulos de la pose inicial:\n{mc.get_angles()}')
    print(f'Pose inicial alcanzada:\n{mc.get_coords()}')
    mc.send_coords(coords_TB, spd, 1)
    # coords_TA = matrix_to_pose(TA)
    # coords_TB = matrix_to_pose(TB)

    # mc.send_coords(coords_TA, spd, 0)
    # time.sleep(7)
    # mc.send_coords(coords_TB, spd, 1)

def singMuñeca(spd = 20, config = [1, -1, -1]):
    """
    Envía una trayectoria al robot mediante pyMyCobot pasando cerca de la singularidad de la muñeca.
    """
    TA=SE3([
           [ 0.07 , -0.   ,  0.998,  114.12],
           [ 0.641, -0.766, -0.045,  127.59],
           [ 0.764,  0.643, -0.053,  296.19],
           [ 0.   ,  0.   ,  0.   ,  1.   ]
           ])
    
    TB=SE3([
           [ 0.07 , -0.   ,  0.998,  194.12],
           [ 0.641, -0.766, -0.045,  127.59],
           [ 0.764,  0.643, -0.053,  296.19],
           [ 0.   ,  0.   ,  0.   ,  1.    ]
           ])
    
    q_TA = cobot.ikine(TA, config)[0]
    print(f'Ángulos pedidos:\n{np.degrees(q_TA).tolist()}')
    mc.send_angles(np.degrees(q_TA).tolist(), 30)

    coords_TA = matrix_to_pose(TA)
    coords_TB = matrix_to_pose(TB)

    # mc.send_coords(coords_TA, spd, 0)
    time.sleep(7)
    print(f'Ángulos de la pose inicial:\n{mc.get_angles()}')
    print(f'Pose inicial alcanzada:\n{mc.get_coords()}')
    mc.send_coords(coords_TB, spd, 1)
    # coords_TA = matrix_to_pose(TA)
    # coords_TB = matrix_to_pose(TB)

    # mc.send_coords(coords_TA, spd, 0)
    # time.sleep(7)
    # mc.send_coords(coords_TB, spd, 1)

# time.sleep(5)
singHombro(30, [1, -1, -1])

# singHombro(30, [-1, 1, 1]) # Se traba en el movimiento cartesiano
# Codo: [1, 1, 1] ; [1, -1, 1] ; [-1, 1, -1]
# Muñeca: [1, -1, 1]
# Hombro: [1, 1, 1] ; [-1, -1, -1] ; [-1, 1, 1] ; [1, -1, -1]