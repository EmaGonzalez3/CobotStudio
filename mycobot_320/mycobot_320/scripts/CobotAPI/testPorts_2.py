import time
from pymycobot import MyCobot320

def discover_button_pin():
    """
    Intenta descubrir si el botón de la placa Atom está mapeado a un pin de E/S
    leyendo continuamente los estados de los pines.
    """
    try:
        # Conexión al robot usando los parámetros que proporcionaste
        mc = MyCobot320('/dev/ttyAMA0', 115200)
        print("Conexión con el robot establecida.")
    except Exception as e:
        print(f"Error al conectar con el robot: {e}")
        return

    # Los pines físicos documentados son 2, 5, 22, 23, 32, 33.
    # Probaremos un rango más amplio por si hay pines no documentados.
    # Un rango de 0 a 35 es razonable y seguro para la lectura.
    pins_to_check = list(range(36))
    
    # Configurar todos los pines en modo de entrada para asegurar una lectura correcta y segura.
    # El modo 2 (input_pullup) es a menudo útil para botones.
    # Nota: Si el pin no existe o no es configurable, la API podría ignorarlo o dar un error.
    print("Configurando pines en modo de entrada (input)...")
    for pin in pins_to_check:
        # Usamos set_pin_mode(pin, 0) para modo de entrada simple.
        mc.set_pin_mode(pin, 0) 
        time.sleep(0.01) # Pequeña pausa entre comandos

    print("\n--- Inicio del monitoreo ---")
    print("Presiona y suelta el botón de la placa Atom varias veces.")
    print("El script te notificará si detecta un cambio de estado en algún pin.")
    print("Presiona CTRL+C para detener el script.")

    try:
        # Leer los estados iniciales de todos los pines
        initial_states = {}
        for pin in pins_to_check:
            initial_states[pin] = mc.get_digital_input(pin)

        print(f"\nEstados iniciales: {initial_states}")

        while True:
            for pin in pins_to_check:
                current_state = mc.get_digital_input(pin)
                if current_state != initial_states[pin]:
                    print(f"\n¡CAMBIO DETECTADO!")
                    print(f" -> Pin número {pin} cambió de {initial_states[pin]} a {current_state}")
                    # Actualizamos el estado para no notificar repetidamente
                    initial_states[pin] = current_state
            
            time.sleep(0.05) # Pequeña pausa para no sobrecargar la comunicación

    except KeyboardInterrupt:
        print("\n--- Monitoreo detenido por el usuario ---")
    except Exception as e:
        print(f"\nOcurrió un error durante el monitoreo: {e}")

if __name__ == '__main__':
    discover_button_pin()