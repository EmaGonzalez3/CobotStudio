from CobotStudio_rev2 import RobTarget, SimManager, MyCobotController, checkQ, checkPose, pose_to_matrix, joystick_adjust
from spatialmath import SE3
import numpy as np
from DHRobotGT import myCobot320
import time
from pymycobot import MyCobotSocket

cobot_tb = myCobot320(rotar_base=True, metros=False)

# pose_pick_manual = RobTarget(SE3(0, -25, -220)* SE3.Rx(-np.pi/2) * SE3.Rz(-np.pi), [-1, 1, 1]) 
# wobj = SE3() * SE3.Rx(-np.pi/2)
pinza = SE3(-1.71381642, 106.90735789, 28.32702833) * SE3.Rx(-np.pi/2) #* SE3.Rz(np.pi)
# checkPose(pose_pick_manual, wobj, pinza, True)


pose_pick_manual = RobTarget(SE3(0, -220, 25) * SE3.Ry(-np.pi), [-1, 1, 1]) 
wobj = SE3()
# checkPose(pose_pick_manual.offset(0, 0, 10, 0, 0, 0), wobj, pinza, True)
# checkPose(pose_pick_manual, wobj, pinza, True)


''' Movimiento alrededor del TCP'''
# rviz = SimManager()
# angle = -5
# while angle >= -20:
#     rviz.MoveJ(wobj, pose_pick_manual.relTool(0, 0, 0, angle, angle, angle), pinza)
#     angle -= 5
#     if angle == -10:
#         rviz.node_obj.detach()  # el objeto se mueve con la pinza
# rviz.shutdown()

''' Prueba del cálculo del wobj '''
# p1_l = [200, 300, 100]
# p2_l = [120, 300, 100]
# p3_l = [25, 125, 150]

# tras1 = [0, -225, 100]
# tras2 = [0, -275, 100]
# tras3 = [150, -300, 100]
# Rot = SE3.Ry(np.pi).R 
# robt1 = RobTarget(SE3.Rt(Rot, tras1), [1, 1, 1])
# robt2 = RobTarget(SE3.Rt(Rot, tras2), [-1, 1, 1])
# robt3 = RobTarget(SE3.Rt(Rot, tras3), [1, 1, 1])
# checkPose(robt3, wobj, pinza, True)

# # print(f'q2_pose:\n{cobot_tb.ikine(robt2.pose * pinza.inv(), robt2.config)[0]}')
# print(f'Posición de la brida\n{robt3.pose * pinza.inv()}')

# # q1 = [-0.59039283, -2.09389856,  0.57725802,  1.51664054,  0.59039283,  0.]
# # q2 = [1.49474743,  0.64130669,  0.77592196,  1.72436401,  1.49474743, -3.14159265]
# # q3 = [0.81493714, -0.1661377, 1.79593746, 1.5117929, 0.81493714, -3.14159265]
# print(np.rad2deg(cobot_tb.ikine(robt3.pose * pinza.inv(), robt3.config)[0]).tolist())
# q1_d = [-48.18932016902527, -178.50341101132463, 127.35355836632716, 51.14985264499751, 48.18932016902527, 0.0]
# q2_d = [60.135102475540045, 16.34036025117441, 75.29025529474599, 88.36938445407961, 60.135102475540045, -180.0]
# q3_d = [-33.8270175012487, -119.97155021990243, 33.0744484991454, 86.89710172075705, 33.82701750124867, 0.0]
# # p1 = np.asarray(p1_l, dtype=float).reshape(-1)
# # p2 = np.asarray(p2_l, dtype=float).reshape(-1)
# # p3 = np.asarray(p3_l, dtype=float).reshape(-1)

# print(cobot_tb.fkine(np.radians(q1_d)))

# q_poses = [q1_d, q2_d, q3_d]
# puntos = np.zeros((3,3))
# for i in range (3):
#     puntos_sc = (cobot_tb.fkine(np.radians(q_poses[i])) * pinza * SE3(0, 0, 25)).t
#     # correccion = np.array([0, 0, -abs(25)])
#     puntos[i] = puntos_sc #+ correccion

# p1, p2, p3 = puntos

# print(f'p1: {p1}\np2: {p2}\np3: {p3}')

# # Eje X: de p2 a p1
# x_axis = p1 - p2
# x_axis /= np.linalg.norm(x_axis)

# # Origen de la terna
# origen = p2 + np.dot(p3 - p2, x_axis) * x_axis
# # Eje Y: desde la proyección de p3 sobre el eje x a p3
# v = p3 - origen
# if np.linalg.norm(v) < 1e-9:
#     raise ValueError("p3 cae sobre la recta X: no se puede definir Y.")
# y_axis = v / np.linalg.norm(v)

# # Eje Z: perpendicular a X e Y
# z_axis = np.cross(x_axis, y_axis)
# z_axis /= np.linalg.norm(z_axis)

# y_axis = np.cross(z_axis, x_axis); y_axis /= np.linalg.norm(y_axis)  # re-ortonormaliza

# # Matriz de rotación
# R = np.column_stack((x_axis, y_axis, z_axis))

# # Armar SE3
# wobj = SE3.Rt(R, origen)

# rviz = SimManager()
# rviz.MostrarTerna(SE3(p1), 'p1')
# rviz.MostrarTerna(SE3(p2), 'p2')
# rviz.MostrarTerna(SE3(p3), 'p3')
# rviz.MostrarTerna(SE3(wobj), 'origen')
# time.sleep(10)
# rviz.shutdown()


""" Movimiento del cobot con teclado """
# rviz = SimManager()
# rviz.MoveJ(SE3(), pose_pick_manual, pinza)
# q_final = rviz.q_current[:6]
# print(f'q_final: {q_final}')
# pose = cobot_tb.fkine(q_final)
# print(f'pose: {pose}')

# print(f'{cobot_tb.calc_conf(q_final)}')
# robt = RobTarget(pose, cobot_tb.calc_conf(q_final).tolist())
# print(robt)
# robt_final = joystick_adjust(
#     rviz.q_current[:6],
#     mover_callback=lambda r: rviz.MoveJ(SE3(), r, SE3())
#     # mover_callback = rviz.MoveJ,
# )
# rviz.shutdown()

""" Test configs para una pose """
rviz = SimManager()
rviz.testPose(pose_pick_manual, pinza, wobj)
rviz.shutdown()