#!/usr/bin/env python3
import argparse
import sys
import time
import importlib.util
import os

# --- CORRECCIÓN DE RUTAS (PATH FIX) ---
# 1. Obtenemos la ruta de la carpeta donde está este script ( carpeta 'scripts' )
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# 2. Obtenemos la ruta PADRE ( carpeta 'mycobot_320' )
PARENT_DIR = os.path.dirname(SCRIPT_DIR)

# 3. Agregamos ambas al path de Python
# Esto permite importar archivos vecinos (ej: import CobotStudio)
if SCRIPT_DIR not in sys.path:
    sys.path.append(SCRIPT_DIR)

# Esto permite que los archivos viejos que usan 'from scripts import...' funcionen
if PARENT_DIR not in sys.path:
    sys.path.append(PARENT_DIR)

# --------------------------------------

# Importación de infraestructura
try:
    # Ahora Python encontrará esto ya sea que uses "scripts.CobotStudio" o directo
    from scripts.CobotStudio_rev4 import MyCobotController, ROS_OK
    from scripts.robot_server import RobotServer
    if ROS_OK:
         from scripts.CobotStudio_rev4 import SimManager
    else:
        SimManager = None 
except ImportError as e:
    print(f"CRITICAL ERROR: Falló la importación de librerías base.\nDetalle: {e}")
    print(f"Rutas incluídas en sys.path: {sys.path}")
    sys.exit(1)

def get_angles_wrapper(robot):
    try:
        mc = getattr(robot, "mc", None)
        if mc and hasattr(mc, "get_angles"): return mc.get_angles()
        if hasattr(robot, "get_angles"): return robot.get_angles()
    except: pass
    return None

def load_module_from_path(path_to_file):
    """Carga dinámicamente un archivo .py desde cualquier ruta."""
    if not os.path.exists(path_to_file):
        # Intentamos buscar relativo a la carpeta scripts si no se encuentra directo
        path_relativo = os.path.join(SCRIPT_DIR, path_to_file)
        if os.path.exists(path_relativo):
            path_to_file = path_relativo
        else:
            raise FileNotFoundError(f"No se encontró el archivo: {path_to_file}")

    module_name = os.path.basename(path_to_file).replace('.py', '')
    
    # Agregamos la carpeta del script específico al path
    # (Para que NDLM_llaveros pueda importar NDLM_scene que está a su lado)
    target_dir = os.path.dirname(os.path.abspath(path_to_file))
    if target_dir not in sys.path:
        sys.path.append(target_dir)

    spec = importlib.util.spec_from_file_location(module_name, path_to_file)
    if spec is None or spec.loader is None:
        raise ImportError(f"No se pudo cargar: {path_to_file}")
    
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module

def main():
    parser = argparse.ArgumentParser(description="CobotLauncher: Ejecutor de Rutinas")
    parser.add_argument("filepath", type=str, help="Ruta al archivo .py (ej: NDLM/NDLM_llaveros_rev4.py)")
    parser.add_argument("mode", choices=["sim", "real"], help="Modo: sim o real")
    parser.add_argument("--bridge", action="store_true", help="Activar Server ROS2 (Solo Real)")
    parser.add_argument("--speed", type=int, default=30, help="Velocidad global")

    args = parser.parse_args()

    full_routine_path = os.path.abspath(args.filepath)
    project_directory = os.path.dirname(full_routine_path)

    # --- 1. CARGA DEL SCRIPT ---
    routine_module = None
    try:
        print(f"[*] Cargando script: {args.filepath} ...")
        routine_module = load_module_from_path(args.filepath)
        
        if not hasattr(routine_module, "run"):
            print(f"ERROR: '{args.filepath}' no tiene función 'run(robot, **kwargs)'.")
            sys.exit(1)
            
    except Exception as e:
        print(f"ERROR FATAL cargando el script:\n{e}")
        sys.exit(1)

    # --- 2. SETUP ROBOT & SERVER ---
    robot = None
    server = None
    
    print(f"[*] Inicializando entorno: {args.mode.upper()}")

    try:
        if args.mode == "sim":
            # Solo instanciamos el simulador vacío
            # La rutina se encargará de importar la escena.
            try:
                robot = SimManager(project_path=project_directory)
                print("[Sim] SimManager iniciado (Escena vacía).")
            except RuntimeError as e:
                print(f'ERROR {e}')
                sys.exit(1)

        elif args.mode == "real":
            robot = MyCobotController(project_path=project_directory)
            if args.bridge:
                print("[Bridge] Iniciando RobotServer...")
                server = RobotServer(
                    read_angles_callable=lambda: get_angles_wrapper(robot),
                    port=65432
                )
                server.start()

                if hasattr(robot, "set_server_observer"):
                    robot.set_server_observer(server)
                
                print("\n" + "="*50)
                print("   MODO BRIDGE ACTIVO: ESPERANDO CLIENTE ROS2")
                print("="*50 + "\n")
                
                if server.wait_for_connection():
                    print("[Bridge] Conectado. Iniciando...")
                    time.sleep(1.0)
                else:
                    print("[Bridge] Timeout. Sin cliente.")

    except Exception as e:
        print(f"Error inicializando hardware: {e}")
        return

    # --- 3. EJECUCIÓN ---
    try:
        if robot:
            print(f"[*] Ejecutando rutina...")
            routine_module.run(robot, speed=args.speed)
            
    except KeyboardInterrupt:
        print("\n[!] Detenido por usuario.")
    except Exception as e:
        print(f"[!] Error en rutina: {e}")
    finally:
        if server:
            server.stop()
        if robot and hasattr(robot, 'shutdown'):
            robot.shutdown()
        print("[*] Fin.")

if __name__ == "__main__":
    main()
