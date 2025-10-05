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

def trayMCP(trayectoria, vel = 20, t_corte = 10):
    """
    Envía una trayectoria generada en variables articulares al robot mediante pyMyCobot.

    Parameters
    ----------
    trayectoria : vector que contiene los puntos de paso expresados en vectores de variables articulares.
    vel : velocidad [mm/s] entre cada punto.
    t_corte : tiempo de corte del movimiento [s]

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


def confHombro(step, spd):
    """
    Envía una trayectoria al robot mediante pyMyCobot cambiando la configuración dada por el hombro.

    step : paso de slicing en el generador de trayectoria.
    """
    q1 = np.radians(np.array([[45, 10, 45, 110.0, 60, 0.0]])) #conf = [1, 1, 1]
    pose1 = cobot.fkine(q1)
    q1a = np.array([cobot.ikine(pose1, [1, 1, 1])[0]])

    Trayectoria_hombro = cobot.genTrJoint(np.array([q1[0], q1a[0], q1a[0]]), 0*np.ones(3))
    qtraj_hombro = cobot.q_ref[::step]
    trayMCP(qtraj_hombro, spd)

def confcodo(step, spd):
    """
    Envía una trayectoria al robot mediante pyMyCobot cambiando la configuración dada por el codo.

    step : paso de slicing en el generador de trayectoria.
    """
    q0 = np.radians(np.array([[0, -30.0, 60.0, -45.0, 30, 0.0]])) #conf = [1, 1, 1]
    pose0 = cobot.fkine(q0)
    q0a = np.array([cobot.ikine(pose0, [1, -1, 1])[0]])

    Trayectoria_codo = cobot.genTrJoint(np.array([q0[0], q0a[0], q0a[0]]), 0*np.ones(3))
    qtraj_codo = cobot.q_ref[::step]
    trayMCP(qtraj_codo, spd)

def confmuñeca(step, spd):
    """
    Envía una trayectoria al robot mediante pyMyCobot cambiando la configuración dada por la muñeca.

    step : paso de slicing en el generador de trayectoria.
    """
    q2 = np.radians(np.array([[70, -40.0, 20.0, -145.0, 20, 0.0]])) #conf = [1, 1, 1]
    pose2 = cobot.fkine(q2)
    q2a = np.array([cobot.ikine(pose2, [1, 1, -1])[0]])

    Trayectoria_muñeca = cobot.genTrJoint(np.array([q2[0], q2a[0], q2a[0]]), 0*np.ones(3))
    qtraj_muñeca = cobot.q_ref[::step]
    trayMCP(qtraj_muñeca, spd)

confcodo(50, 50)