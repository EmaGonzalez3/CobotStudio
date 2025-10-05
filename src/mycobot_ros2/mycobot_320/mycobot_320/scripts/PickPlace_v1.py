from CobotStudio_rev2 import RobTarget, SimManager, MyCobotController, checkQ, checkPose, pose_to_matrix, joystick_adjust
from spatialmath import SE3
import numpy as np
from DHRobotGT import myCobot320
import time
from pymycobot import MyCobotSocket

cobot_tb = myCobot320(rotar_base=True, metros=False)
cobot = MyCobotController()
pick = RobTarget(SE3(0, -220, 25)* SE3.Ry(-np.pi), [-1, 1, 1])
place = RobTarget(SE3(200, -50, 25)* SE3.Ry(-np.pi), [-1, 1, 1])
pinza = SE3(-2.84011157, 115.08057356,  22.28604936) * SE3.Rx(-np.pi/2)
print(place.find_valid_configs(pinza, SE3()))
home = RobTarget(cobot_tb.fkine(np.repeat(0.01, 6)) * pinza, [-1, 1, 1])
wobj = SE3()
# rviz.VerPose(wobj, pick.offset(0, 0, 90), pinza)
# time.sleep(1)
# cobot.MoveJ(pick.offset(0, 0, 80), 30, pinza, wobj)
# pick = RobTarget(SE3(0, -220, 25)* SE3.Ry(-np.pi) * SE3.Rx(-20, 'deg'), [-1, 1, 1])

# cobot.MoveC(pick, 30, pinza, wobj)
# time.sleep(2)
# cobot.GripperState(1, 30)
# time.sleep(3)
# rviz.VerQ(np.zeros(6), pinza)
# rviz.VerPose(wobj, home, SE3())
# cobot.MoveJ(home, 30, pinza, wobj)
cobot.MoveJ(place.offset(0, 0, 20), 30, pinza, wobj)
cobot.MoveC(place, 30, pinza, wobj)
# time.sleep(10)
cobot.GripperState(0, 30)
cobot.MoveC(place.offset(0, 0, 50), 30, pinza, wobj)
cobot.MoveJ(home, 30, pinza, wobj)
