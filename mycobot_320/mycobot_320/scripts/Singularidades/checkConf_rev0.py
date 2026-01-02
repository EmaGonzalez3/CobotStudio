from Cobot_sdk import *

def run(robot: BaseRobotController, **kwargs):
    dar = RobTarget(SE3(340, -135.721, 200)* SE3.Ry(np.pi/2)*SE3.Rz(np.pi), [-1, 1, 1])
    pinza = SE3(-1.71381642, 106.90735789, 28.32702833) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)


    def moveit(robot:ROSManager):
        # robot.testPose(dar, pinza, SE3())
        robot.explore_ik_configs(dar, pinza)

    moveit(robot)