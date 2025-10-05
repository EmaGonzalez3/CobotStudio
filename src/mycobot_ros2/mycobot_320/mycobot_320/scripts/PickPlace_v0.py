from CobotStudio_rev1 import RobTarget, SimManager, MyCobotController, checkQ, checkPose, pose_to_matrix
from spatialmath import SE3
import numpy as np
from DHRobotGT import myCobot320
import time
from pymycobot import MyCobotSocket


cobot_tb = myCobot320(rotar_base=True, metros=False)
cob = MyCobotController()

# 1 y 2 - Mover al robot mediante myBlocky para agilizar
mc = MyCobotSocket("10.42.0.1", 9000)

pose_pick_manual = RobTarget(SE3(0, -220, 25)* SE3.Ry(-np.pi), [-1, 1, 1]) 
# checkPose(pose_pick_manual, wobj, pinza, True)
wobj = SE3()
pinza = SE3(-1.71381642, 106.90735789, 28.32702833) * SE3.Rx(-np.pi/2)
cob.GripperState(0, 30)
cob.MoveJ(pose_pick_manual, 30, pinza, wobj)
# mc.set_gripper_mode(1)
time.sleep(10)
cob.GripperState(1, 30)
time.sleep(5)
cob.MoveJAngles(np.zeros(6), 30)
time.sleep(10)
cob.MoveJ(pose_pick_manual, 30, pinza, wobj)
time.sleep(10)
cob.GripperState(0)
time.sleep(3)
cob.MoveJAngles(np.zeros(6), 30)
