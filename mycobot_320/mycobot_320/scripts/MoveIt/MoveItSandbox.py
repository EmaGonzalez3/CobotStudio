from scripts.CobotStudio_rev4 import RobTarget, BaseRobotController, MyCobotController
from spatialmath import SE3
import numpy as np
from scripts.DHRobotGT import myCobot320
import time 
try:
    from scripts.CobotStudio_rev4 import SimManager
    SIM_AVAILABLE = True
    print("Modo de simulación disponible: ROS2 detectado.")
except ImportError:
    SimManager = None  # Definimos SimManager como None para evitar NameError
    SIM_AVAILABLE = False
    print("Advertencia: Modo de simulación no disponibple (no se encontraron librerías de ROS2).")


rviz = SimManager()
cobot_tb = myCobot320(rotar_base=True, metros=False)

POSITIONS = np.deg2rad([-25.0, 45.0, -37.0, 10.0, 23.0, 30.0, 0.0]).tolist()

q_goal = np.deg2rad([-85.0, 45.0, -37.0, 90.0, 23.0, 40.0, -20.0]).tolist()
q_start = np.deg2rad([-10.0, 0.0, 0.0, 0.0, 0.0, 0.0]).tolist()
# traj, q_list = rviz.moveit_adapter.plan_joint_trajectory(
#     q_goal=q_goal, q_start=q_start
# )
# rviz.moveit_adapter.save_last_planned_trajectory('traj1')
# print(f'traj =\n{traj}')
# print(f'q_list =\n{q_list}')


# rviz.moveit_adapter.plan_and_execute(q_goal, execute=True)
# rviz.moveit_adapter.save_last_planned_trajectory('traj1')


# q_test = np.deg2rad([-40, 15, -17, 45, 23, 30, -20]).tolist()
# q_start = np.deg2rad([0, 0, 0, 0, 0, 0, 10]).tolist()
# ok = rviz.moveit_adapter.check_collision(q_test)
# if ok:
#     print("Pose segura -> Planifico y ejecuto")
#     rviz.moveit_adapter.plan_and_execute(q_test, q_start, execute = True)
# else:
#     print("Pose NO segura -> no planifico.")


pinza = SE3(-1.71381642, 106.90735789, 28.32702833) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)
dar = RobTarget(SE3(280, -135.721, 200)* SE3.Ry(np.pi/2)*SE3.Rz(np.pi), [-1, 1, 1])
print(f'q_robt = {cobot_tb.ikine((dar.pose)*pinza.inv(), dar.config)[0]}')

rviz.moveit_adapter.apply_goal_state(cobot_tb.ikine((dar.pose)*pinza.inv(), dar.config)[0].tolist(), publish_tf=True, tool=pinza, tf_frame_name='Objetivo')
# rviz.moveit_adapter.plan_and_execute(rviz.get_current_q(), execute = True)
# print(f'q_current = {np.rad2deg(rviz.get_current_q())}')
# rviz.publish_goal_tf(rviz.get_current_q(), pinza, 'goal')
# rviz.moveit_adapter.apply_goal_state(q_test)
# rviz.explore_ik_configs(dar, pinza, SE3(), ver_todo=True)
# print(cobot_tb.fkine(np.array(rviz.get_current_q()))*pinza)

time.sleep(5)
rviz.shutdown()