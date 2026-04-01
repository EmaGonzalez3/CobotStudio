from Cobot_sdk import *
import time
import datetime
import pandas as pd
from importlib import import_module

def run(robot: BaseRobotController, **kwargs):
    def sendWobj(wobj, pinza, conf = [1, 1, 1]):
        """Enviar el cobot real al wobj calculado"""
        robt0 = RobTarget(SE3(wobj), conf)
        print(robt0.pose)
        robot.MoveJ(robt0, 30, pinza, SE3())

    def cheqWobj(robot: ROSManager, wobj, pinza, conf = [1, 1, 1]):
        ''' Envía el wobj a RViz desde la pose home.'''
        robt0 = RobTarget(SE3(wobj), conf)
        print(robt0.find_valid_configs(pinza, SE3()))
        robot.MoveJ(robt0, speed = 30, tool = pinza, wobj = SE3(), robt_name='wobj')
        time.sleep(2)

    def revisarPoses(poses_list, tool = SE3(), func = 'MoveJ'):
        """
        Envía al robot físico a las poses almacenadas en una lista. 
        Mediante un menu se permite ir al home entre poses o moverse directamente entre ellas.

        Args:
            poses_list (list): Lista de poses a recorrer. Cada pose se lee como [angles, coords] según devuelve el cobot.
            tool (SE3): Herramienta. Por defecto es nula porque las poses guardadas son en la brida.
            func (str): `MoveJ` para ir mediante funciones de la toolbox o `sendAngles`/`sendCoords` mediante la API.
        """
        def pose_alcanzada():
            time.sleep(5)
            print(f'Ángulos alcanzados:\n{robot.mc.get_angles()}')
            print(f'Coordenadas alcanzadas:\n{robot.mc.get_coords()}')
            print(f'Ángulos y coords (check):\n{robot.mc.get_angles_coords()}')
        
        for idx, pose in enumerate(poses_list, 1):
            print(f'\n------------------------- Pose {idx} -------------------------')
            coords = pose[6:]
            print(f'Coords leídas:\n{coords}')
            angles_deg = pose[:6]
            print(f'Ángulos leídos:\n{angles_deg}')
            config = robot.cobot_tb.calc_conf(np.deg2rad(angles_deg)).tolist()
            pose_SE3 = robot.pose_to_matrix(coords)
            robt = RobTarget(pose_SE3, config)
            print(f'Robtarget generado (conversión de coords del cobot):\n{robt.pose}')
            POSE_calc = robot.cobot_tb.fkine(np.deg2rad(angles_deg))
            print(f'POSE_calc - PCD_toolbox(q_cobot):\n{POSE_calc}')
            q_ikine = robot.cobot_tb.ikine(pose_SE3, config)[0]
            print(f'Según la tb se alcanza con los ángulos\n{np.round(np.rad2deg(q_ikine), 2)}')

            while True:
                print("\n¿Qué desea hacer a continuación?")
                print(f"  1. Ir a Home, esperar 3s, y luego ir a la Pose {idx}.")
                print(f"  2. Ir directamente a la Pose {idx} desde la posición actual.")
                print(f"  3. Ir a Home y cerrar.")
                
                choice = input("Ingrese su opción (1, 2 o 3): ").strip()

                if choice == '1':
                    print("\n[Opción 1] Moviendo el robot a la posición Home...")
                    robot.MoveJ_q(np.zeros(6), 30) # Mover a Home
                    
                    print("Llegó a Home. Esperando 3 segundos...")
                    time.sleep(3)
                    
                    print(f"Moviendo a la Pose {idx} con el método {func}...")
                    if func == 'MoveJ':
                        robot.MoveJ(robt, 30, tool, SE3()) # Mover al robtarget final
                        pose_alcanzada()
                    elif func == 'sendAngles':
                        robot.mc.sync_send_angles(angles_deg, 30, 0)
                        pose_alcanzada()
                    elif func == 'sendCoords':
                        robot.mc.sync_send_coords(coords, 30, 0)
                        pose_alcanzada()
                    else:
                        print(f"El método seleccionado {func} no es admisible.")
                        return
                    while True:
                        print("\nCon qué método analizamos el punto?")
                        print("1. Armamos el robt y vamos con MoveJ.")
                        print("2. Vamos con los ángulos que leyó el cobot al grabarlo.")
                        print("3. Vamos con las coordenadas que leyó el cobot al grabarlo.")
                        print("q. Salir del menu.")
                        choice_method = input("Ingrese su opción (1, 2 o 3): ").strip()

                        if choice_method == '1':
                            print("\n[Opción 1] MoveJ...")
                            robot.MoveJ(robt, 30, tool, SE3()) # Mover al robtarget final
                            pose_alcanzada()
                        elif choice_method == '2':
                            print("\n[Opción 2] sendAngles...")
                            robot.mc.sync_send_angles(angles_deg, 30, 0)
                            pose_alcanzada()
                        elif choice_method == '3':
                            print("\n[Opción 3] sendCoords...")
                            robot.mc.sync_send_coords(coords, 30, 0)
                            pose_alcanzada()
                        else:
                            print(f"El método seleccionado no es admisible.")
                            return
                        if choice_method.lower() == 'q':   # salir del menú
                            break

                    break # Salimos del bucle del menú y vamos a la siguiente pose

                elif choice == '2':
                    print(f"\n[Opción 2] Moviendo directamente a la Pose {idx} con el método {func}...")

                    if func == 'MoveJ':
                        robot.MoveJ(robt, 30, tool, SE3())
                    elif func == 'sendAngles':
                        robot.mc.sync_send_angles(angles_deg, 30, 0)
                    elif func == 'sendCoords':
                        robot.mc.sync_send_coords(coords, 30, 0)
                    else:
                        print(f"El método seleccionado {func} no es admisible.")
                        return
                    break # Salimos del bucle del menú y vamos a la siguiente pose

                if choice == '3':
                    print("\n[Opción 3] Enviando a home y finalizando...")
                    robot.MoveJ_q(np.zeros(6), 30) # Mover a Home
                    
                    print("Llegó a Home. Terminando prueba...")
                    return

                else:
                    print("\n*** Opción no válida. Por favor, ingrese 1 o 2. ***")
            robot.MoveJ(robt, 30, tool, SE3())

    def analizarPoses(poses_list):
        """
        Análisis de poses de enseñanza de wobj, guardando los resultados en una lista.

        Args:
            poses_list (list): Lista de poses a recorrer.

        Returns:
            list: Una lista de diccionarios, donde cada diccionario contiene los datos de una pose.
        """
        datos_para_tabla = []
        for idx, pose in enumerate(poses_list, 1):
            # Extracción de datos como en tu función original
            POSE_robot = pose[6:]
            q_robot = pose[:6]
            q_robot_rad = np.deg2rad(q_robot)
            
            # Cálculos
            config = robot.cobot_tb.calc_conf(np.deg2rad(q_robot))
            POSE_robot_SE3 = robot.pose_to_matrix(POSE_robot)
            robt = RobTarget(POSE_robot_SE3, config)
            POSE_calc = robot.cobot_tb.fkine(np.deg2rad(q_robot))
            q_calc = np.rad2deg(robot.cobot_tb.ikine(POSE_calc, config)[0])
            q_robot_recalc = np.rad2deg(robot.cobot_tb.ikine(POSE_robot_SE3, config)[0])

            opciones_formato = {'precision': 2, 'floatmode': 'fixed', 'suppress_small': True}

            resultado_pose = {
                "Pose #": idx,
                "Coords Leídas": np.array2string(np.array(POSE_robot), **opciones_formato),
                "Ángulos Leídos": np.array2string(np.array(q_robot), **opciones_formato),
                "Config": str(config),
                "RobTarget Generado": np.array2string(robt.pose.A.flatten(), **opciones_formato),
                "POSE_calc (FKine)": np.array2string(POSE_calc.A.flatten(), **opciones_formato),
                "q_calc (IKine)": np.array2string(q_calc, **opciones_formato),
                "q_recalc (IKine)": np.array2string(q_robot_recalc, **opciones_formato)
            }
            datos_para_tabla.append(resultado_pose)
            
        return pd.DataFrame(datos_para_tabla)

    def armar_csv(nombre_archivo: str):
        lista_A = getattr(mod_qcoords, 'q_20')
        lista_B = getattr(mod_qcoords, 'q_21')
        lista_C = getattr(mod_qcoords, 'q_22')
        lista_D = getattr(mod_qcoords, 'q_23')
        lista_E = getattr(mod_qcoords, 'q_24')
        lista_F = getattr(mod_qcoords, 'q_25')
        lista_G = getattr(mod_qcoords, 'q_26')
        mod_qcoords = import_module("scripts.EnseñanzaWobj.Datos.Workobjects_qcoordsv8")
        lista_H = getattr(mod_qcoords, 'q_30')



        # Puedes tener cuantas listas quieras
        todas_mis_poses = {
            "Caso 20": lista_A,
            "Caso 21": lista_B,
            "Caso 22": lista_C,
            "Caso 23": lista_D,
            "Caso 24": lista_E,
            "Caso 25": lista_F,
            "Caso 26": lista_G,    
            "Caso 27": lista_H,    
        }

        # 2. Itera, analiza y recolecta los DataFrames
        lista_de_dataframes = []
        for nombre_grupo, poses_list in todas_mis_poses.items():
            print(f"Analizando el grupo: '{nombre_grupo}'...")
            
            # Llama a tu función de siempre
            df_parcial = analizarPoses(poses_list)
            
            # Añade la columna identificadora
            df_parcial['Grupo'] = nombre_grupo
            
            # Guarda el DataFrame parcial en la lista
            lista_de_dataframes.append(df_parcial)

        # 3. Concatena todos los DataFrames en uno solo
        # ignore_index=True es importante para que el índice final sea continuo (0, 1, 2, 3...)
        df_final = pd.concat(lista_de_dataframes, ignore_index=True)

        # Opcional: Mueve la columna 'Grupo' al principio para mayor claridad
        cols = ['Grupo'] + [col for col in df_final if col != 'Grupo']
        df_final = df_final[cols]


        # --- RESULTADOS ---
        print("\n----------- DataFrame Final Combinado -----------")
        print(df_final.to_string())

        # Exporta el DataFrame final a un único archivo CSV
        nombre_archivo_final = nombre_archivo
        df_final.to_csv(nombre_archivo_final, index=False, encoding='utf-8')
        print(f"\n¡Exportación exitosa! El DataFrame combinado se ha guardado en '{nombre_archivo_final}'.")

    def registrar_datos(q_list):
        for i in range (6):
            robot.MoveJ_q(np.zeros(6), 40, 'deg')
            robot.MoveJ_q(np.array(q_list[i]), 30, 'deg')
            time.sleep(5)
            print(f'Ángulos pedidos\n{q_list[i]}')
            print(f'Cobot coords\n{robot.mc.get_angles()}')
            print(f'Pose según toolbox\n{robot.cobot_tb.fkine(np.radians(q_list[i]))}')
            print(f'Cobot pose\n{robot.mc.get_coords()}')

    def calibrar_servos():
        """Calibrar los servos del cobot real en su posición home."""
        robot.MoveJ_q(np.zeros(6), 20, 'deg') # Enviamos al cobot al home
        time.sleep(2)
        print(f'Lectura de los encoders:\n{robot.mc.get_encoders()}') # Vemos qué dicen los encoders
        for i in range (6):
            robot.mc.set_servo_calibration(i+1)
            time.sleep(2)
        print(f'Encoders calibrados:\n{robot.mc.get_encoders()}')

    def corregir_offset(robt):
        # --- Corrección offset ---
        # Asumimos que 'wobj_enseñado' es el que tiene el offset
        # Mueve el robot real a esa pose. Terminará en la posición incorrecta.
        robot.MoveJ(robt, 30, pinza_aux, SE3())

        # Guarda los ángulos a los que llegó
        q_incorrecto = np.deg2rad(robot.mc.get_angles())

        # La función 'mover_callback' para el joystick
        def mover_robot(robt):
            robot.MoveJ(robt, 20, pinza_aux, SE3())

        time.sleep(1)

        print("\n--- Ajuste Manual ---")
        print("Usa el joystick para mover la punta a la esquina REAL.")

        robt_corregido = robot.joystick_adjust(np.deg2rad(robot.mc.get_angles()), 
                                        mover_callback=lambda r: robot.MoveJ(r, 20, SE3(), SE3()))
        
        q_ajustado = robot.mc.get_angles()
        # Pose de la brida donde el robot cree que está bien
        T_brida_incorrecta = robot.cobot_tb.fkine(q_incorrecto)

        # Pose de la brida donde el robot está físicamente bien
        T_brida_correcta = robot.cobot_tb.fkine(np.deg2rad(q_ajustado))

        # T_brida_correcta = T_calibracion * T_brida_incorrecta
        # => T_calibracion = T_brida_correcta * T_brida_incorrecta.inv()

        T_calibracion = T_brida_correcta * T_brida_incorrecta.inv()

        print(f"\nTransformación de Calibración Calculada:\n{T_calibracion}")
        print(f"Offset en [x, y, z]: {T_calibracion.t.round(3)} mm")

    # pinza = SE3(-1.71, 106.91, 28.33) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)
    pinza_palp = SE3(0, 148, 27.5) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)
    pinza = SE3(0, 123, 27.5) * SE3.Rx(-np.pi/2) * SE3.Rz(np.pi)
    pinza_aux = pinza*SE3(0,0,25)
    robot.load_scene('teachWobj_scene_rev0')

    def leer_wobj(robot: BaseRobotController, it_wobj, archivo, version_archivo):
        # Nombre de los datos a grabar
        filename = f'{archivo}_{version_archivo}'
        filename_q = f'{archivo}_{version_archivo}_q'

        ## Enseñanza del wobj y recolección de datos
        wobj_teach = robot.load_wobj(filename, f'wobj{it_wobj}', pinza, True)
        q_wobj = robot.load_data('Workobjects', filename_q, f'wobj{it_wobj}_q')

        return wobj_teach, q_wobj

    def cheq_wobj_qs(robot: ROSManager, it_wobj, archivo, version_archivo):
        robot.view_pose(np.zeros(6), pinza)
        robot.GripperState(0)
        q_load = robot.load_data('Workobjects', f'{archivo}v{version_archivo}', f'q_{it_wobj}')
        q_rads = np.deg2rad(q_load)
        for idx, q in enumerate(q_rads, start=1):
            if idx <= 3:
                robot.show_tf(robot.cobot_tb.fkine(q) * pinza, f'x{idx}')
            else:
                robot.show_tf(robot.cobot_tb.fkine(q) * pinza, f'y{idx-3}')
            time.sleep(1)
            robot.view_pose(q, pinza_aux)
            time.sleep(1)
    
    def send_wobj_to_cobot(robot, wobj):
        ## Chequear el wobj en el robot físico
        sendWobj(wobj, pinza_aux, [1, -1, -1]) # [-1, 1, 1] para enseñanza en vertical!

    # revisarPoses(qcoords_grabados[coord_name], pinza)
    version_archivo = 9
    iteracion_wobj = 51

    # wobj, q_wobj = leer_wobj(robot, iteracion_wobj, 'Workobjects', version_archivo)
    
    ## Chequear puntos en RViz
    cheq_wobj_qs(robot,iteracion_wobj, 'Workobjects_q', version_archivo)

"""
Notas:
cob.MoveJ_q(np.array([135, 25, 30, 20, -30, 0]), 30, 'deg')

wobj15 
Offset en [x, y, z]: [-9.199  1.534 28.009] mm
Si lo hago llegar desde donde quedó ajustado el offset cambia: Offset en [x, y, z]: [ 0.79  -3.483 21.856] mm
Desde MoveJ_q: Offset en [x, y, z]: [-9.911  0.588 29.286] mm

wobj16
Llegando a lo calculado desde 0: Offset en [x, y, z]: [-15.023  -2.646  27.512] mm
Llegando desde la esquina: Offset en [x, y, z]: [-1.48  -6.651 24.885] mm
Desde MoveJ_q: Offset en [x, y, z]: [-8.325  2.395 23.981] mm

wobj17
Desde 0: Offset en [x, y, z]: [12.354  4.259 29.53 ] mm
Desde la esquina: Offset en [x, y, z]: [12.48  -2.029 24.586] mm
Desde el MoveJ_q: Offset en [x, y, z]: [12.126 15.975 32.041] mm

"""

