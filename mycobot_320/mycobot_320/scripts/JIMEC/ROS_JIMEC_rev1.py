from scripts.CobotStudio_rev4 import  SimManager
import time
from scripts.object_manager_rev1 import CUBE, CYLINDER, MESH

def setup_scene(robot_manager: SimManager):
        """
        Configura la escena de trabajo agregando objetos a una instancia de SimManager.

        Args:
        robot_manager: La instancia de SimManager a la que se agregarán los objetos.
        """
        print("--- Configurando la escena de trabajo en RViz ---")
        altura_base = 0.025 # Base de aluminio del cobot
        altura_caja = 0.07 # Altura de los bloques de madera a apilar
        brida_base = 0.02

        # Agregamos los objetos según sus dimensiones
        robot_manager.node_obj.add_object("robot_base", pose_init=(0.0, 0.0, -altura_base/2 - brida_base),
                        size=(0.51, 0.51, altura_base),
                        color=(1.0, 1.0, 1.0, 0.9),
                        shape=CYLINDER, movable=False)
        robot_manager.node_obj.add_object("caja1", pose_init=(0.1, -0.2, altura_caja/2- brida_base),
                        size=(0.04, 0.04, altura_caja),
                        color=(1.0, 0.0, 0.0, 0.9),
                        shape=CUBE, movable=True)
        robot_manager.node_obj.add_object("caja2", pose_init=(0.1, -0.265, altura_caja/2 - 0.015- brida_base),
                size=(0.04, 0.04, altura_caja),
                color=(0.0, 1.0, 0.0, 0.9),
                shape=CUBE, movable=True)
        robot_manager.node_obj.add_object("caja3", pose_init=(0.1, -0.33, altura_caja/2 - 0.015- brida_base),
                size=(0.04, 0.04, altura_caja),
                color=(0.0, 0.0, 1.0, 0.9),
                shape=CUBE, movable=True)

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