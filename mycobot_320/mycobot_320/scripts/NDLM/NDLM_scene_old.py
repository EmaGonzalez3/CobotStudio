import numpy as np
import time
from scripts.CobotStudio_rev4 import  SimManager
from scripts.object_manager_rev1 import CUBE, CYLINDER, MESH

def setup_scene(robot_manager: SimManager):
        """
        Configura la escena de trabajo agregando objetos a una instancia de SimManager.

        Args:
        robot_manager: La instancia de SimManager a la que se agregarán los objetos.
        """
        print("--- Configurando la escena de trabajo en RViz ---")
        altura_base = 0.025 # Base de aluminio del cobot
        altura_mesa = 0.09 # Altura de los bloques de madera a apilar
        brida_base = 0.02

        # Agregamos los objetos según sus dimensiones
        robot_manager.node_obj.add_object("robot_base", pose_init=(38.0e-3, -23.0e-3, -altura_base/2 - brida_base),
                        size=(1.0e-3, 1.0e-3, 1.0e-3),
                        color=(1.0, 1.0, 1.0, 1.0),
                        shape=MESH, movable=False, 
                        mesh="file:///home/ema/colcon_ws/src/mycobot_ros2/mycobot_320/mycobot_320/scripts/NDLM/Base_Cobot.STL",
                        rot_euler=(-np.pi/2, 0, np.pi*1.033))
                
        # robot_manager.node_obj.add_object(name="pieza_aux",
        # pose_init=(0.0, -181.721e-3, 402.32e-3),
        # size=(1.0e-3, 1.0e-3, 1.0e-3),  # escala en metros
        # color=(1.0, 0.0, 0.0, 0.5),
        # shape=MESH,
        # movable=True,
        # mesh="file:///home/ema/colcon_ws/src/mycobot_ros2/mycobot_320/mycobot_320/scripts/EnseñanzaWobj/Palpador_pinza4.STL",
        # rot_euler=(np.pi/2 + 0*np.pi/15, 0, 0))

        robot_manager.node_obj.add_object(name="listón",
        pose_init=(267.0e-3, -125.0e-3,- brida_base),
        size=(40.0e-3, 40.0e-3, 325.0e-3),  # escala en metros
        color=(0.81, 0.68, 0.42, 1.0),
        shape=CUBE,
        movable=False,
        rot_euler=(np.pi/2, 0, -np.pi/7.5))

        robot_manager.node_obj.add_object(name="porta_llavero1",
        pose_init=(235.0e-3, -171.0e-3, - brida_base + 20e-3),
        size=(1.0e-3, 1.0e-3, 1.0e-3),  # escala en metros
        color=(0.0, 1.0, 0.0, 0.5),
        shape=MESH,
        movable=False,
        mesh="file:///home/ema/colcon_ws/src/mycobot_ros2/mycobot_320/mycobot_320/scripts/NDLM/Porta_llavero.STL",
        rot_euler=(np.pi/2, 0, np.pi/2 -np.pi/7.5))

        robot_manager.node_obj.add_object(name="llavero1",
        pose_init=(247.0e-3, -165.721e-3, - brida_base + 40e-3),
        size=(1.0e-3, 1.0e-3, 1.0e-3),  # escala en metros
        color=(243/255, 133/255, 64/255, 1.0),
        shape=MESH,
        movable=True,
        mesh="file:///home/ema/colcon_ws/src/mycobot_ros2/mycobot_320/mycobot_320/scripts/NDLM/Llavero.STL",
        rot_euler=(np.pi/2, 0, np.pi/2 -np.pi/7.5))

        robot_manager.node_obj.add_object(name="porta_llavero2",
        pose_init=(278.0e-3, -62.0e-3, - brida_base + 20e-3),
        size=(1.0e-3, 1.0e-3, 1.0e-3),  # escala en metros
        color=(0.0, 1.0, 0.0, 0.5),
        shape=MESH,
        movable=False,
        mesh="file:///home/ema/colcon_ws/src/mycobot_ros2/mycobot_320/mycobot_320/scripts/NDLM/Porta_llavero.STL",
        rot_euler=(np.pi/2, 0, np.pi/2 -np.pi/7.5))

        robot_manager.node_obj.add_object(name="llavero2",
        pose_init=(290.0e-3, -56.721e-3, - brida_base + 40e-3),
        size=(1.0e-3, 1.0e-3, 1.0e-3),  # escala en metros
        color=(143/255, 33/255, 64/255, 1.0),
        shape=MESH,
        movable=True,
        mesh="file:///home/ema/colcon_ws/src/mycobot_ros2/mycobot_320/mycobot_320/scripts/NDLM/Llavero.STL",
        rot_euler=(np.pi/2, 0, np.pi/2 -np.pi/7.5))

        robot_manager.node_obj.add_object(name="porta_llavero3",
        pose_init=(188.0e-3, -266.0e-3, - brida_base + 20e-3),
        size=(1.0e-3, 1.0e-3, 1.0e-3),  # escala en metros
        color=(0.0, 1.0, 0.0, 0.5),
        shape=MESH,
        movable=False,
        mesh="file:///home/ema/colcon_ws/src/mycobot_ros2/mycobot_320/mycobot_320/scripts/NDLM/Porta_llavero.STL",
        rot_euler=(np.pi/2, 0, np.pi/2 -np.pi/7.5))

        robot_manager.node_obj.add_object(name="llavero3",
        pose_init=(200.0e-3, -260.721e-3, - brida_base + 40e-3),
        size=(1.0e-3, 1.0e-3, 1.0e-3),  # escala en metros
        color=(43/255, 33/255, 164/255, 1.0),
        shape=MESH,
        movable=True,
        mesh="file:///home/ema/colcon_ws/src/mycobot_ros2/mycobot_320/mycobot_320/scripts/NDLM/Llavero.STL",
        rot_euler=(np.pi/2, 0, np.pi/2 -np.pi/7.5))


        time.sleep(1.0)
        print("--- Escena configurada ---")

if __name__ == '__main__':
    print("Ejecutando prueba de configuración de escena...")
    sim = SimManager()
    setup_scene(sim)
    print("Prueba finalizada. Presiona Ctrl+C para salir.")
    try:
        # Mantenemos el script vivo para poder ver la escena en RViz
        while True:
            time.sleep(1)
    except KeyboardInterrupt:
        sim.shutdown()