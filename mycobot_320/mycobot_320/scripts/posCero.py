from pymycobot import MyCobotSocket, MyCobot320Socket
# from pymycobot.mycobot import MyCobot
import time

def recover_robot(mc_socket):
    # --- FUNCIÓN DE RECUPERACIÓN ---
    """Envía un comando para reiniciar el puerto serie en la Raspi y luego enciende los servos."""
    print("Iniciando secuencia de recuperación remota...")
    try:
        # 1. Enviar comando de reinicio de puerto serie
        print("Enviando comando para reiniciar el puerto serie del robot...")
        mc_socket.sock.sendall(b'reset_serial')
        response = mc_socket.sock.recv(1024) # Esperar confirmación
        print(f"Respuesta del servidor: {response.decode()}")
        time.sleep(1) # Esperar a que el puerto se estabilice

        # 2. Ahora, intentar encender los servos
        print("Intentando encender los servos (power_on)...")
        mc_socket.power_on()
        time.sleep(1)

        # 3. Verificar el estado
        power_status = mc_socket.is_power_on()
        print(f"Estado de 'is_power_on': {power_status}")
        if power_status == 1:
            print("¡Recuperación exitosa! El robot está encendido y listo.")
            return True
        else:
            print("La recuperación falló. 'is_power_on' no devolvió 1.")
            return False

    except Exception as e:
        print(f"Ocurrió un error durante la recuperación: {e}")
        return False

def home_recovery():
# # --- SCRIPT PRINCIPAL ---
    mc = MyCobotSocket("10.42.0.1", 9000)

    # ¡Llamar a la función de recuperación aquí!
    if not recover_robot(mc):
        print("No se pudo recuperar el robot. Abortando.")
        exit(1)

    # Ahora el resto de tu código debería funcionar como esperas
    if mc.is_controller_connected() != 1:
        print("El controlador Atom sigue sin responder correctamente.")
        # Aún así, el movimiento podría funcionar
    else:
        print("Conexión con Atom verificada.")

    # Posición home
    print("Enviando a posición home...")
    mc.sync_send_angles([0, 0, 0, 0, 0, 0], 30)
    print("Movimiento completado.")

# Recuperación de parada de emergencia
home_recovery()
mc = MyCobotSocket("10.42.0.1", 9000)
# mc2 = MyCobot320Socket("10.42.0.1", 9000)
mc.clear_error_information()
# mc.power_off()
# time.sleep(1)
mc.power_on()
time.sleep(0.5)
print(mc.is_power_on())
print(mc.get_angles())



# Antes de mover el robot chequeamos la conexión
# if mc.is_controller_connected() != 1:
#     print("Please connect the robot arm correctly for program writing")
#     exit(0)

# Posición home
mc.sync_send_angles([0, 0, 0, 0, 0, 0], 30)

# mc.set_gripper_mode(0)
# mc.set_gripper_state(0, 30) # Abrir pinza
# mc2.set_gripper_state(1, 30) # Cerrar pinza
# print(mc.get_servo_last_pdi(2))

""" Funciones para probar """
# print(f'Robot status: {mc2.get_robot_status()}')
# print(f'Pausa: {mc2.pause()}')
# print(f'Está en movimiento? {mc2.is_moving()}')
# print(f'get_encoders: {mc2.get_encoders()}')
# print(f'is_servo_enable: {mc2.is_all_servo_enable()}')
# print(f'servo_status: {mc2.get_servo_status()}')
# print(f'Versión de firmware: {mc.get_atom_version()}')

# --- FUNCIÓN DE RECUPERACIÓN ---
# def recover_robot_remotely(mc_socket):
#     """Envía un comando para reiniciar el puerto serie en la Raspi y luego enciende los servos."""
#     print("Iniciando secuencia de recuperación remota...")
#     try:
#         # 1. Enviar comando de reinicio de puerto serie
#         print("Enviando comando para reiniciar el puerto serie del robot...")
#         mc_socket.sock.sendall(b'reset_serial')
#         response = mc_socket.sock.recv(1024) # Esperar confirmación
#         print(f"Respuesta del servidor: {response.decode()}")
#         time.sleep(1) # Esperar a que el puerto se estabilice

#         # 2. Ahora, intentar encender los servos
#         print("Intentando encender los servos (power_on)...")
#         mc_socket.power_on()
#         time.sleep(1)

#         # 3. Verificar el estado
#         power_status = mc_socket.is_power_on()
#         print(f"Estado de 'is_power_on': {power_status}")
#         if power_status == 1:
#             print("¡Recuperación exitosa! El robot está encendido y listo.")
#             return True
#         else:
#             print("La recuperación falló. 'is_power_on' no devolvió 1.")
#             return False

#     except Exception as e:
#         print(f"Ocurrió un error durante la recuperación: {e}")
#         return False

# # --- SCRIPT PRINCIPAL ---
# mc = MyCobotSocket("10.42.0.1", 9000)

# # ¡Llamar a la función de recuperación aquí!
# if not recover_robot_remotely(mc):
#     print("No se pudo recuperar el robot. Abortando.")
#     exit(1)

# # Ahora el resto de tu código debería funcionar como esperas
# if mc.is_controller_connected() != 1:
#     print("El controlador Atom sigue sin responder correctamente.")
#     # Aún así, el movimiento podría funcionar
# else:
#     print("Conexión con Atom verificada.")

# # Posición home
# print("Enviando a posición home...")
# mc.sync_send_angles([10, 0, 0, 0, 0, 0], 30)
# print("Movimiento completado.")