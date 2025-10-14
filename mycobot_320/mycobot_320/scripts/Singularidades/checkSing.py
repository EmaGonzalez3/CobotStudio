from pymycobot import MyCobotSocket
import time
import numpy as np
from DHRobotGT import myCobot320
import spatialmath as sm

mc = MyCobotSocket('10.42.0.1', 9000)
cobot = myCobot320()

# Antes de mover el robot chequeamos la conexión
if mc.is_controller_connected() != 1:
    print("Please connect the robot arm correctly for program writing")
    exit(0)

def trayMCP(trayectoria, vel = 20, dt = 0.05):
    """
    Envía una trayectoria generada en variables articulares al robot mediante pyMyCobot.

    Parámetros
    ----------
    trayectoria : vector que contiene los puntos de paso expresados en vectores de variables articulares.
    vel : velocidad [mm/s] entre cada punto.
    dt : delay entre puntos [s]

    Returns
    -------
    q_grados : trayectoria expresada variables articulares [°]
    """
    q_grados = []
    for q in trayectoria:
        q_grados.append(np.degrees(q).tolist())
        mc.send_angles(q_grados[-1], vel)
        time.sleep(dt)
        # while not mc.is_in_position(q_grados[-1], 0):
        #     mc.resume()
        #     time.sleep(0.5)
        #     mc.pause()
    return q_grados


def singHombro(step, spd, dt = 0.05):
    """
    Envía una trayectoria al robot mediante pyMyCobot pasando cerca de la singularidad del hombro.

    step : paso de slicing en el generador de trayectoria.
    """
    TA_h=sm.SE3([[1,0,0,-0.100],[0,1,0,0.089],[0,0,1,0.335],[0,0,0,1]])
    TB_h=sm.SE3([[1,0,0,0.100],[0,1,0,0.089],[0,0,1,0.335],[0,0,0,1]])

    qtraj_h = np.empty((0,cobot.n))
    Trayectoria_h = cobot.genTrCart([TA_h, TB_h, TB_h], 0*np.ones(3))
    qtraj_h = cobot.q_ref[::step]
    print(qtraj_h)
    # trayMCP(qtraj_h, spd, dt)

def singCodo(step, spd, dt = 0.05):
    """
    Envía una trayectoria al robot mediante pyMyCobot pasando cerca de la singularidad del codo.

    step : paso de slicing en el generador de trayectoria.
    """
    TA_c=sm.SE3([[0,-0.9988,0.04998,-0.30],[-1,0,0,0.08878],[0,0.04998 ,-0.9988,0.09774],[0,0,0,1]])
    TB_c=sm.SE3([[0,-0.9988,0.04998,-0.355],[-1,0,0,0.08878],[0,0.04998 ,-0.9988,0.09774],[0,0,0,1]])

    qtraj_c = np.empty((0,cobot.n))
    Trayectoria_c = cobot.genTrCart([TA_c, TB_c, TB_c], 0*np.ones(3))
    qtraj_c = cobot.q_ref[::step]
    trayMCP(qtraj_c, spd, dt)

def singMuñeca(step, spd):
    """
    Envía una trayectoria al robot mediante pyMyCobot pasando cerca de la singularidad de la muñeca.

    step : paso de slicing en el generador de trayectoria.
    """
    TA_m=sm.SE3([[0,-1,0,-0.145],[0.09983, 0 ,0.995,0.124],[-0.995,0  ,0.09983,0.154],[0,0,0,1]])
    TB_m=sm.SE3([[0,-1,0,-0.145],[0.09983, 0 ,0.995,-0.124],[-0.995,0  ,0.09983,0.154],[0,0,0,1]])

    qtraj_m = np.empty((0,cobot.n))
    Trayectoria_m = cobot.genTrCart([TA_m, TB_m, TB_m], 0*np.ones(3))
    qtraj_m = cobot.q_ref[::step]
    trayMCP(qtraj_m, spd)

# singCodo(50, 30, 0.03)
# singHombro(30, 50)
singHombro(60, 30, 80e-3)
# singMuñeca(20, 30)