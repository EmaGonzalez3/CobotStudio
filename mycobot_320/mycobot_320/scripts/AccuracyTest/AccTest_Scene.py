import numpy as np
import time
from scripts.CobotStudio_rev4 import  SimManager
from scripts.object_manager_rev1 import CUBE, CYLINDER, MESH

def setup_scene(robot_manager: SimManager, caso = 1):
        """
        Configura la escena de trabajo agregando objetos a una instancia de SimManager.

        Args:
        robot_manager: La instancia de SimManager a la que se agregarán los objetos.
        """
        print("--- Configurando la escena de trabajo en RViz ---")
        altura_base = 0.025 # Base de aluminio del cobot
        altura_mesa = 0.05 # Altura de los bloques de madera a apilar
        brida_base = 0.02
    
        # Agregamos los objetos según sus dimensiones
        robot_manager.node_obj.add_object("robot_base", pose_init=(0.0, 0.0, -altura_base/2 - brida_base),
                        size=(0.51, 0.51, altura_base),
                        color=(1.0, 1.0, 1.0, 0.9),
                        shape=CYLINDER, movable=False)
        
        # robot_manager.node_obj.add_object("papel", pose_init=(0.0, -425.00e-3, altura_mesa/2- brida_base),
        #                 size=(300e-3, 300e-3, altura_mesa),
        #                 color=(0.88, 0.88, 0.88, 0.8),
        #                 shape=CUBE, movable=True, rot_euler=np.deg2rad((0, 0, 0)))
        
        if caso == 1:
            posicion_marcador = (0.0, -260.0e-3, 100.0e-3)
        elif caso == 2:
             posicion_marcador = (100.0e-3, 200.0e-3, 100.0e-3)
        elif caso == 3:
             posicion_marcador = (0.0e-3, -182.0e-3, 417.0e-3)
        elif caso == 4:
             posicion_marcador = (0.0e-3, -182.0e-3, 417.0e-3)
        robot_manager.node_obj.add_object(name="marcador",
        pose_init=posicion_marcador,
        size=(18e-3, 18e-3, 100e-3),  # escala en metros
        color=(1.0, 0.0, 0.0, 0.8),
        shape=CYLINDER,
        movable=True,
        rot_euler=(0, 0, 0))


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