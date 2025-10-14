from CobotStudio_rev1 import RobTarget, SimManager, MyCobotController, checkQ, checkPose, pose_to_matrix
from spatialmath import SE3
import numpy as np
from DHRobotGT import myCobot320
import time

cobot_tb = myCobot320(rotar_base=True, metros=False)

def pruebasPrevias():
    # 1. Definir wobj y robtarget
    wobj1 = SE3(100, 100, 100) #* SE3.Rx(np.pi)
    rob_clase1 = RobTarget(SE3(50, -120, 220))
    rob_clase2 = RobTarget(SE3(50, -120, 200))
    wobj2 = SE3(200, 200, 200) * SE3.Rx(np.pi)
    rob_nulo = RobTarget(SE3())
    TA=SE3([
        [ -1,   0,   0,    103.6 ],
        [  0,  -1,   0,   -89.1  ],
        [  0,   0,   1,    331.4 ],
        [  0,   0,   0,   1      ]
        ])

    # Crear robtarget
    target = RobTarget(TA, [-1, -1, -1])

    # 2. Crear simulador
    sim = SimManager()

    # 3. Hacer el movimiento
    sim.move_joint(wobj1, rob_clase1, wobj_name='wobj1', robt_name='pt1')
    sim.move_cartesian(wobj1, rob_clase1.offset(0, 20, 0, wobj1), wobj_name='wobj1', robt_name='pt2')

    ### NO CORRER EN EL COBOT ###
    # sim.move_joint(wobj1, rob_nulo, wobj_name='wobj1', robt_name='')
    # sim.move_cartesian(wobj1, rob_nulo.offset(20, 0, 0, wobj2), wobj_name='wobj1', robt_name='offset')
    ### NO CORRER EN EL COBOT ###

    # sim.move_joint(rob_nulo.pose, target, wobj_name='wobj', robt_name='robt1')
    # sim.move_cartesian(rob_nulo.pose, target.offset(20, 0, 0, rob_nulo.pose), wobj_name='wobj', robt_name='offset')

    # sim.move_joint(wobj2, rob_nulo, wobj_name='wobj2', robt_name='robt1')
    # sim.move_cartesian(wobj2, rob_nulo.offset(0, 20, 0, wobj2), wobj_name='wobj2', robt_name='offset')

    # 4. Finalizar
    # sim.shutdown()



pose_1 = [104.7, -153.7, 180, -91.56, -66.61, 1.86]
pose_2 = [-102.4, -154.7, 180, -90.35, 38.05, 0.7]
pose_3 = [3.8, -154.2, 180, -90, 12.48, -1.79]
pose_4 = [-5.6, -123.7, 180, -91.56, 26.16, 0.48]

q_pose_1 = np.radians([0, 15, -45, -20, 0, -15])
q_pose_2 = np.radians([0, -15, 45, 20, 0, 15])
q_pose_3 = np.radians([0, 0, 0, 0, 0, 0])
q_pose_4 = np.radians([54, 2, 12, -14, -108, 27])

pose_1se3 = pose_to_matrix(pose_1)
rob_pose1 = RobTarget(pose_1se3, [1, 1, 1])

pose_2se3 = pose_to_matrix(pose_2)
rob_pose2 = RobTarget(pose_2se3, [1, 1, 1])

pose_3se3 = pose_to_matrix(pose_3)
rob_pose3 = RobTarget(pose_3se3, [1, 1, 1])

pose_4se3 = pose_to_matrix(pose_4)
rob_pose4 = RobTarget(pose_4se3, [1, 1, 1])

# pinza = SE3(0, 120, 50) * SE3.Rx(-np.pi/2) # Suposición inicial
pinza_calc = SE3(-1.71381642, 106.90735789, 28.32702833) * SE3.Rx(-np.pi/2) # Pinza calculada

wobj1 = SE3(100, 100, 100) #* SE3.Rx(np.pi)
rob_clase1 = RobTarget(SE3(50, -120, 220))
# sim = SimManager()
# sim.MoveJ(wobj1, rob_clase1, tool=pinza_calc, wobj_name='wobj1', robt_name='pt1')
# sim.MoveC(wobj1, rob_clase1.offset(0, 20, 0, wobj1), tool=pinza_calc, wobj_name='wobj1', robt_name='pt2')
# sim.mover_pinza(20)
# time.sleep(2)
# sim.mover_pinza(100)
# sim.shutdown()

#  Ejemplo con el cobot
# cob = MyCobotController()
# tool = SE3()
# cob.MoveJ(rob_nulo, 30, tool, wobj2)
# cob.MoveC(rob_nulo.offset(0, 20, 0, wobj2), 30, tool, wobj2)
# cob.MoveJ(rob_clase1, 30, tool, wobj1)
# cob.MoveC(rob_clase1.offset(0, 20, 0, wobj1), 30, tool, wobj1)

# print(cobot_tb.fkine(np.zeros(6)))
# print(rob_pinza.find_valid_configs(pinza_calc, SE3()))


rob_pinza = RobTarget(SE3(100, -150, 170)* SE3.Ry(np.pi), [1, 1, -1]) 
rob_cheq = RobTarget(SE3([
       [ -1,   0,   0,    103.6 ],
       [  0,  -1,   0,   -89.1  ],
       [  0,   0,   1,    331.4 ],
       [  0,   0,   0,   1      ]
       ]))
# checkPose(rob_pose1, SE3(), SE3(), True) # Sin considerar la herramienta
# checkQ(q_pose_1, pinza_calc, True) # Pose alcanzable con la pinza
# rob_pose1.find_valid_configs(pinza, SE3())
# checkQ(cobot_tb.ikine(rob_clase1.pose)[0], SE3(), True) # Pose no lograble con y sin pinza
# checkPose(rob_pinza, SE3(), pinza_calc, True) # No lograble
# checkPose(rob_cheq, SE3(), pinza_calc, True) # No lograble
# checkQ(np.zeros(6), pinza_calc, True) # q = 0
