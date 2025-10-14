# controller.py
from pymycobot import MyCobotSocket
import numpy as np
from DHRobotGT import myCobot320
from spatialmath import SE3


class MyCobotController:
    """
    Encapsula la comunicación con un myCobot 320Pi
    y ofrece movimientos articulares y cartesianos
    usando Robotics Toolbox.

    Parámetros
    ----------
    host : str
        IP del robot (Ethernet/Wi‑Fi).
    port : int
        Puerto TCP (por defecto 9000).
    rotar_base : bool
        Si el robot del modelo virtual debe invertir
        el eje‑base (caso habitual en Elephant Robotics).
    tool : SE3
        Transformación de la herramienta (por defecto identidad).
    wobj : SE3
        Work‑object (por defecto base del robot).
    """

    def __init__(self,
                 host: str = "10.42.0.1",
                 port: int = 9000,
                 *,
                 rotar_base: bool = True,
                 tool: SE3 = SE3(),
                 wobj: SE3 = SE3()):

        # Conexión con el robot físico
        self.mc = MyCobotSocket(host, port)

        # Modelo DH para la cinemática y la IK
        self.cobot = myCobot320(rotar_base=rotar_base)

        # Defaults que se pueden sobrescribir en cada comando
        self.tool = tool
        self.wobj = wobj

    # ------------------------------------------------------------------
    #  Métodos públicos
    # ------------------------------------------------------------------
    def move_joint(self,
                   robt,              # RobTarget
                   speed: int,
                   *,
                   tool: SE3 | None = None,
                   wobj: SE3 | None = None):
        """
        Ejecuta un movimiento articular usando cinemática inversa
        basada en Robotics Toolbox y la configuración deseada
        almacenada en el RobTarget.
        """
        tool = tool or self.tool
        wobj = wobj or self.wobj

        # Pose global = wobj * robtarget * inv(tool)
        pose_calc = wobj * robt.pose * tool.inv()

        # IK que respeta la configuración del RobTarget
        q_sol = self.cobot.ikine(pose_calc, robt.config)[0]   # rad

        # Enviar al robot (la API espera grados)
        self.mc.send_angles(np.degrees(q_sol).tolist(), speed)

    def move_cart(self,
                  robt,
                  speed: int,
                  *,
                  tool: SE3 | None = None,
                  wobj: SE3 | None = None,
                  relative: bool = False):
        """
        Movimiento cartesiano en modo punto‑a‑punto
        (usa send_coords de la API de Elephant Robotics).

        Si `relative=True` el movimiento es relativo al punto actual
        (flag = 0 en send_coords); si es False es absoluto (flag = 1).
        """
        tool = tool or self.tool
        wobj = wobj or self.wobj

        pose_calc = wobj * robt.pose * tool.inv()

        # Formato [x y z Rx Ry Rz] con Euler Z‑Y‑X en grados
        pose_xyzrpy = (
            list(pose_calc.t) +
            list(pose_calc.rpy(order="zyx", unit="deg"))
        )

        flag = 0 if relative else 1
        self.mc.send_coords(pose_xyzrpy, speed, flag)

    # ------------------------------------------------------------------
    #  Utilidades opcionales
    # ------------------------------------------------------------------
    def close(self):
        """Cierra el socket con el robot; útil en un bloque try/finally."""
        try:
            self.mc.release_all_servos()
        finally:
            self.mc.close()

    # Permite usar el controlador como contexto “with”
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()

