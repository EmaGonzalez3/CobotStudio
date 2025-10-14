from pymycobot import MyCobotSocket
import time
import numpy as np
from DHRobotGT import myCobot320
from spatialmath import SE3
import itertools

mc = MyCobotSocket('10.42.0.1', 9000)
cobot = myCobot320(rotar_base=True)

class RobTarget:
    def __init__(self, pose, config=None):
        """
        Args:
            pose: Matriz 4x4 (tipo numpy o SE3) que representa la pose cartesiana.
            config: Lista con configuración articular, por ejemplo [1, -1, 1].
        """
        # Convertir SE3 a matriz si es necesario
        if isinstance(pose, SE3):
            self.pose = pose
        elif hasattr(pose, "shape") and pose.shape == (4, 4):
            self.pose = SE3(pose)
        else:
            raise ValueError("La pose debe ser una matriz 4x4 o un objeto SE3.")

        if config is not None:
            if isinstance(config, list) and all(isinstance(x, int) for x in config):
                self.config = config
            else:
                raise ValueError("La configuración debe ser una lista como [1, -1, 1].")
        else:
            self.config = [1, 1, 1]  # Configuración por defecto
        
        self.valid_configs = []

    def find_valid_configs(self, robot_model, tool, wobj):
        T_global = wobj * self.pose * tool.inv()
        # Todas las combinaciones posibles de [±1, ±1, ±1]
        configs = list(itertools.product([1, -1], repeat=3))
        
        valid_solutions = []

        for conf in configs:
            solution = robot_model.ikine(T_global, conf)[0]
            if len(solution) > 0:
                self.valid_configs.append(conf)
    
        if self.valid_configs:
            print("Configuraciones válidas:", ", ".join([str(c) for c in self.valid_configs]))
        else:
            print("⚠️ Ninguna configuración resuelve la IK para esta pose.")

    def as_list(self, degrees=True):
        """
        Devuelve la pose como lista [x, y, z, rx, ry, rz]
        En grados por default (para pymycobot).
        """
        pos = self.pose.t
        rpy = self.pose.rpy(order='xyz', unit='deg' if degrees else 'rad')
        return [*pos, *rpy]

    def as_matrix(self):
        """
        Devuelve la pose como matriz homogénea 4x4 (numpy array).
        """
        return self.pose.A

    def get_SE3(self):
        """
        Devuelve la pose como objeto SE3.
        """
        return self.pose

    def offset(self, dx=0, dy=0, dz=0, wobj=SE3()):
        """
        Desplaza el RobTarget en el marco del wobj, sin cambiar orientación ni configuración.
        
        Args
            dx, dy, dz: Desplazamientos en mm respecto al wobj.
            wobj: Objeto SE3 que representa el marco del wobj (default: identidad).

        Returns:
            Nuevo RobTarget desplazado.
        """
        # Vector de desplazamiento en el marco del wobj
        T_offset_local = SE3(dx, dy, dz) 
        new_pose = T_offset_local * self.pose

        return RobTarget(new_pose, config=self.config.copy())

    def __repr__(self):
        return f"RobTarget(pose=SE3({self.pose.t.tolist()}, rpy={self.pose.rpy(unit='deg').tolist()} deg), config={self.config})"

def matrix_to_pose(T_matrix):
    T = SE3(T_matrix)
    x, y, z = T.t       # Extraer traslación

    # Extraer rotación como RPY (en radianes), orden ZYX
    rx, ry, rz = T.rpy(order='zyx', unit='rad')
    rx, ry, rz = np.rad2deg([rx, ry, rz])   # Convertimos a grados
    return [x, y, z, rx, ry, rz]

def MoveJAngles(q, spd):
    """
    Envía al robot al vector de variables articulares pedido.

    Parameters
    ----------
        q : Array(1,6)
            Vector de variables articulares [rad]
        spd : int (1 - 100)
            Velocidad 
    """
    # mc.send_angles(np.degrees(q).tolist(), spd)

def MoveJoint(robt, spd, tool, wobj=SE3()):
    # La pose se va a tener que calcular como el robtarget desde el wobj para una tool dada.
    # Calcular pose global: wobj * robtarget * inv(tool)
    pose_calc = wobj * robt.pose * tool.inv()

    # Convertir a formato [x, y, z, rx, ry, rz] en grados. En realidad no haría falta:
    # vamos a usar ikine para controlar la conf.
    # pose = list(pose_calc.t) + list(pose_calc.rpy(unit='deg'))

    # Usamos la cinemática inversa de la toolbox para tener control de la config.
    # Acá va a haber que agregar un try porque puede llegar a fallar la resolución de ikine.
    q_pose = cobot.ikine(pose_calc, robt.config)[0]
    mc.send_angles(np.degrees(q_pose).tolist(), spd)

def MoveCart(robt, spd, tool, wobj=SE3()):
    pose_calc = wobj * robt.pose * tool.inv()

    # pose_mycobot = pose_calc*SE3.Rz(np.pi)

    # Convertir a formato [x, y, z, rx, ry, rz] en grados.
    # pose = list(pose_mycobot.t) + list(pose_mycobot.rpy(order='zyx', unit='deg'))
    pose = list(pose_calc.t) + list(pose_calc.rpy(order='zyx', unit='deg'))

    print(pose)

    mc.send_coords(pose, spd, 1)

T = SE3(0.3, 0.1, 0.5) * SE3.OA([0, 1, 0], [0, 0, 1])  # Posición + orientación. Útil para enseñar el wobj.


# wobj: en (100, 100, 100), z hacia abajo
# Z hacia abajo = rotación de 180° alrededor del eje X
# wobj = SE3(100, 100, 100) * SE3.Rx(np.pi)

# tool: 150mm en Z (desde la brida hacia afuera)
# tool = SE3.Rz(np.pi) * SE3(0, 0, 0)
tool = SE3()

########### OK ##############
# rob_nulo = RobTarget(SE3())
# TA=SE3([
#        [ -1,   0,   0,    103.6 ],
#        [  0,  -1,   0,   -89.1  ],
#        [  0,   0,   1,    331.4 ],
#        [  0,   0,   0,   1      ]
#        ])
# target = RobTarget(TA, [-1, -1, -1])
# MoveJoint(target, 30, tool, rob_nulo.pose)
# MoveCart(target.offset(20, 0, 0, rob_nulo.pose), 30, tool, rob_nulo.pose)
########### OK ##############

########### OK ##############
# wobj1 = SE3(100, 100, 100)
# rob_clase1 = RobTarget(SE3(50, -120, 220))
# MoveJoint(rob_clase1, 30, tool, wobj1)
# MoveCart(rob_clase1.offset(0, 20, 0, wobj1), 30, tool, wobj1)
########### OK ##############

########### OK ##############
# wobj2 = SE3(200, 200, 200) * SE3.Rx(np.pi)
# rob_nulo = RobTarget(SE3())
# MoveJoint(rob_nulo, 30, tool, wobj2)
# MoveCart(rob_nulo.offset(0, 20, 0, wobj2), 30, tool, wobj2)
########### OK ##############