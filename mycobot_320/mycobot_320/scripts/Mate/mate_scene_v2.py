from Cobot_sdk import *

def setup_scene(robot_manager: ROSManager):
        """
        Configura la escena de trabajo agregando objetos a una instancia de SimManager.

        Args:
        robot_manager: La instancia de SimManager a la que se agregarán los objetos.
        """
        print("--- Configurando la escena de trabajo en RViz ---")

        robot_manager.add_scene_object(name="termo",
        pose_init=(400.0e-3, -100e-3, 240e-3),
        size=(1.0e-3, 1.0e-3, 1.0e-3),  # escala en metros
        color=(37/255, 37/255, 37/255, 1.0),
        shape=Shapes.MESH,
        movable=True,
        mesh="file:///home/ema/colcon_ws/src/mycobot_ros2/mycobot_320/mycobot_320/scripts/Mate/Mallas/BotellaConAgarre.STL",
        rot_euler=(np.pi/2, 0, 1.06))

        robot_manager.add_scene_object(name="mate",
        pose_init=(280.0e-3, -210e-3, 180e-3),
        size=(1.0e-3, 1.0e-3, 1.0e-3),  # escala en metros
        color=(154/255, 114/255, 71/255, 1.0),
        shape=Shapes.MESH,
        movable=False,
        mesh="file:///home/ema/colcon_ws/src/mycobot_ros2/mycobot_320/mycobot_320/scripts/Mate/Mallas/Mate.STL",
        rot_euler=(np.pi/2, 0, -0.48))

        robot_manager.add_scene_object(name="base_termo",
        pose_init=(400.0e-3, -100e-3, 192e-3),
        size=(100.0e-3, 100.0e-3, 46.0e-3),  # escala en metros
        color=(233/255, 169/255, 105/255, 1.0),
        movable=False,
        shape=Shapes.CUBE,
        rot_euler=(0, 0, 1.06))

        robot_manager.add_scene_object(name="mesa",
        pose_init=(0.0, 0.0, 0.0),
        size=(1.0e-3, 1.0e-3, 1.0e-3),  # escala en metros
        color=(0/255, 113/255, 0/255, 1.0),
        shape=Shapes.MESH,
        movable=False,
        mesh="file:///home/ema/colcon_ws/src/mycobot_ros2/mycobot_320/mycobot_320/scripts/Mate/Mallas/Mesa_matera6.STL",
        rot_euler=(np.pi/2, 3.14, 0.0))

        
        time.sleep(1.0)
        print("--- Escena configurada ---")

def run(robot, **kwargs):
    setup_scene(robot)