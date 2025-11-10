from scripts.CobotStudio_rev4 import RobTarget, MyCobotController
from spatialmath import SE3
import numpy as np
from scripts.DHRobotGT import myCobot320
import time

def pose_to_matrix(pose):
        """Convierte pose dada como lista [x, y, z, rx, ry, rz] en una matriz homogénea SE3."""
        x, y, z, rx, ry, rz = pose
        rx, ry, rz = np.deg2rad([rx, ry, rz])
        T_se3 = SE3(x, y, z) * SE3.RPY([rx, ry, rz], order='zyx')
        return T_se3

robot = MyCobotController()
cobot_tb = myCobot320(rotar_base=True, metros=False)

# Abrir la pinza
robot.GripperState(100, 30)
time.sleep(3)

# Grabar pose
teach = robot.grabar_poses(1, palpador = False)
# print(f'Teach\n{teach}')

# Calcular y mostrar q, pose y conf
q_teach = teach[0][0]
print(f'q_enseñado =\n{q_teach}')
pose_teach = pose_to_matrix(teach[1][0])
print(f'pose_enseñada =\n{pose_teach}')
conf = cobot_tb.calc_conf(np.deg2rad(q_teach)).tolist()
print(f'Configuración detectada: {conf}')

# Crear el robtarget enseñado
robt = RobTarget(pose_teach, conf)

# Volver a home
for i in range(5, 0, -1):
    print(f"Volviendo al home en {i} segundos...", end='\r')
    time.sleep(1)
robot.MoveJ(robt.offset(0, 0, 50), 30, SE3(), SE3()) # Antes de volver sube un poco
robot.MoveJAngles(np.zeros(6), 30)
time.sleep(2)

print("Yendo a la pose enseñada...")
# Ir a buscar la caja
robot.MoveJ(robt.offset(0, 0, 50), 30, SE3(), SE3()) # Posicionamiento
robot.MoveJ(robt, 30, SE3(), SE3())
robot.GripperState(0, 30) # Cierre de la pinza
time.sleep(3)

print("Levantando caja...")
# Levantar la caja y esperar
robot.MoveJ(robt.offset(0, 0, 50), 30, SE3(), SE3())
time.sleep(5)

print("Devolviendo caja...")
# Dejar la caja donde estaba
robot.MoveJ(robt, 30, SE3(), SE3())
robot.GripperState(100, 30)
time.sleep(2)

print("Volviendo a home...")
# Salir hacia arriba e ir a home
robot.MoveJ(robt.offset(0, 0, 50), 30, SE3(), SE3())
robot.MoveJAngles(np.zeros(6), 30)


