from pymycobot import MyCobot320
from pymycobot import MyCobotSocket
import time
import numpy as np
from DHRobotGT import myCobot320
import spatialmath as sm


mc = MyCobotSocket("10.42.0.1", 9000)

# Ángulos actuales de las articulaciones
# angles = mc.get_angles()
# print(angles)

# Antes de mover el robot chequeamos la conexión
if mc.is_controller_connected() != 1:
    print("Please connect the robot arm correctly for program writing")
    exit(0)

# Llevar el eje 1 a 40° al 20% de la velocidad (20 mm/s)
# mc.send_angle(1, 40, 20)

# Posición home (hay que ver si realmente llega)
# mc.send_angles([0, 0, 0, 0, 0, 0], 30)

# # Para las trayectorias hay que usar un sleep entre cada pose.
# # De esta manera nos aseguramos que el robot llegó antes de mandar la siguiente.
# mc.send_angles([0, 0, 0, 0, 0, 0], 30)
# time.sleep(2.5)
# mc.send_angle(1, 90, 30)

# # Para trabajar con coordenadas
# # La brida alcanza las coordenadas [-2.3, -153.2, 400.8] en un movimiento cartesiano,
# # con la orientación [-120.0, 0.7, 179.73] y una velocidad de 40 mm/s
# mc.send_coords([-2.3, -153.2, 400.8, -120.0, 0.7, 179.73], 40, 1)

# # Soltar los frenos
# mc.release_all_servos()

# # Ejemplo: chequeamos si el robot llegó a la pose destino.
# start = time.time()
# mc.send_angles([-1.49, 115, -145, 30, -33.42, 137.9], 40)
# while not mc.is_in_position([-1.49, 115, -145, 30, -33.42, 137.9], 0):
#     # Si no llegó se sigue moviendo
#     mc.resume()
#     # Después de un tiempo que se estuvo moviendo, lo frenamos
#     time.sleep(0.5)
#     mc.pause()
#     # Si ya pasó un tiempo determinado, lo dejamos donde quedó
#     if time.time() - start > 3:
#         break

def trayMCP(trayectoria, vel = 20):
    """
    Envía una trayectoria generada en variables articulares al robot mediante pyMyCobot.

    Parámetros
    ----------
    trayectoria : vector que contiene los puntos de paso expresados en vectores de variables articulares.
    vel : velocidad [mm/s] entre cada punto.

    Returns
    -------
    q_grados : trayectoria expresada variables articulares [°]
    """
    q_grados = []
    for q in trayectoria:
        q_grados.append(np.degrees(q).tolist())
        mc.send_angles(q_grados[-1], vel)
        while not mc.is_in_position(q_grados[-1], 0):
            mc.resume()
            time.sleep(0.5)
            mc.pause()
    return q_grados

#OK
q_val = np.linspace(0, 45, 5)
# for i in q_val:
#     mc.send_angle(1, i, 30)
#     time.sleep(2)
#     while not mc.is_in_position([i, 0, 0, 0, 0, 0], 0):
#         mc.resume()
#         time.sleep(0.5)
#         mc.pause()
# for j in range(10):
#     print(j)
#     # El while is_in_position sirve para sincronizar las j con la cantidad de veces que se mueve.
#     for i in q_val:
#         mc.send_angles([i, i, i, -i, i/2, i], 30)
#         while not mc.is_in_position([i, i, i, -i, i/2, i], 0):
#             mc.resume()
#             time.sleep(0.5)
#             mc.pause()

    # time.sleep(2)
    # angles = mc.get_angles()
    # print(angles, mc.is_in_position([i, i, 0, 0, 0, 0], 0))
    # while not mc.is_in_position([i, i, i, 0, 0, 0], 0):
    #         mc.resume()
    #         time.sleep(0.5)
    #         print(i)
    #         mc.pause()

# mc.send_angles([i, i, 0, 0, 0, 0], 30)

# mc.send_angles([0, 0, 0, 0, 0, 0], 30)
# time.sleep(5)
# mc.send_angles([45, 45, 0, 0, 0, 0], 30)
# start = time.time()
# for i in range(10):
#      t_act = time.time()
#      angles = mc.get_angles()
#      print(angles, (t_act-start)*1000)
#      time.sleep(80e-3 - (time.time() - t_act))


# Ejemplo: trayectoria cartesiana entre 2 puntos
cobot = myCobot320()
# TA=sm.SE3([[0, -1, 0, 0.135],
#            [0.09983, 0, 0.995, 0.124],
#            [-0.995, 0, 0.09983, 0.154],
#            [0, 0, 0, 1]])
# TB=sm.SE3([[0, -1, 0, 0.135],
#            [0.09983, 0, 0.995, -0.124],
#            [-0.995, 0, 0.09983, 0.154],
#            [0, 0, 0, 1]])

# Trayectoria_c = cobot.genTrCart([TA, TB, TB], 0*np.ones(3))
# # Si hay demasiados puntos los podemos ir salteando:
# qtraj = cobot.q_ref[::100]

# trayMCP(qtraj, 20)

# Ejemplo: trayectoria joint entre 2 puntos
qa = [0, 0, 0, 0, 0, 0]
qb = [0.5, 0.1, 1, 1.5, 0.3, 1.7]

# Podría servir para movimientos más fluidos:
# mc.jog_increment_angle(1, 50, 30)
# mc.jog_increment(2, 30, 30)
# mc.send_angles([0, 0, 0, 0, 0, 45], 30)

# Trayectoria_j = cobot.genTrJoint(np.array([qa, qb, qb]), 0*np.ones(3))
# qtraj_j = cobot.q_ref[::20]
# trayMCP(qtraj_j, 10)

# Movimientos cartesianos
# Para trabajar con coordenadas
# La brida alcanza las coordenadas [-2.3, -153.2, 400.8] en un movimiento cartesiano,
# con la orientación [-120.0, 0.7, 179.73] y una velocidad de 40 mm/s
# mc.send_coords([-2.3, -153.2, 400.8, -120.0, 0.7, 179.73], 40, 1)

# TA_c=sm.SE3([[0,-0.9988,0.04998,-0.30],[-1,0,0,0.08878],[0,0.04998 ,-0.9988,0.09774],[0,0,0,1]])
# TB_c=sm.SE3([[0,-0.9988,0.04998,-0.355],[-1,0,0,0.08878],[0,0.04998 ,-0.9988,0.09774],[0,0,0,1]])
