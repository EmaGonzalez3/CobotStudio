# ejemplo listo para integrar. Asume que self.mc ya es MyCobot320('/dev/ttyAMA0',115200)
import time
import threading
import subprocess
from pymycobot import MyCobot320

def scan_for_button_pin(mc, pins=range(1,33), timeout=15.0, sample_interval=0.04):
    """
    Escanea una lista de pins preguntando su estado para detectar cuál cambia
    cuando presionás el botón.
    Devuelve (pin, initial_value) o (None, None) si no detecta nada en 'timeout' s.
    """
    deadline = time.time() + timeout
    # leer baseline
    baseline = {}
    for p in pins:
        try:
            baseline[p] = mc.get_digital_input(p)
        except Exception:
            baseline[p] = None
    print("Baseline leída. Ahora presioná el botón (tendrás {:+.0f}s)".format(timeout))
    while time.time() < deadline:
        for p in pins:
            try:
                val = mc.get_digital_input(p)
            except Exception:
                continue
            b = baseline.get(p)
            if b is None:
                baseline[p] = val
                continue
            if val != b:
                print(f"Detectado cambio en pin {p}: {b} -> {val}")
                return p, b
        time.sleep(sample_interval)
    return None, None

def monitor_button(mc, pin, on_press_callback, poll=0.03, debounce=0.12):
    """
    Monitoriza un pin digital y llama on_press_callback() cuando detecta un flanco
    estable (con debounce). Se asume que 'press' es el cambio de 0->1 o 1->0:
    aquí comprobamos la transición que detectaste durante el scan (active_high variable).
    """
    # determinar estado inicial
    try:
        last = mc.get_digital_input(pin)
    except Exception as e:
        print("Error leyendo pin:", e)
        return
    while True:
        try:
            cur = mc.get_digital_input(pin)
        except Exception:
            time.sleep(poll)
            continue
        if cur != last:
            # anti-rebote: esperar un corto periodo y confirmar
            time.sleep(debounce)
            try:
                cur2 = mc.get_digital_input(pin)
            except Exception:
                cur2 = cur
            if cur2 == cur:
                # interpretá el tipo de flanco como prefieras. Aquí llamamos en cualquier cambio.
                if cur != last:
                    # Llamada al callback en hilo separado para no bloquear la lectura
                    threading.Thread(target=on_press_callback, daemon=True).start()
                last = cur2
        time.sleep(poll)

# ejemplo de callback: lanza un script externo
def run_my_script():
    print("Botón presionado: ejecuto script externo.")
    # ajustar ruta y permisos
    subprocess.Popen(["/usr/bin/python3", "/home/pi/mis_proyectos/accion_al_pulsar.py"])
    # si preferís bloquear hasta fin:
    # subprocess.run([...])

# USO (ejemplo)
# --- dentro de tu código donde ya inicializaste self.mc ---
mc = MyCobot320('/dev/ttyAMA0', 115200)

# opcional: intentar configurar pins como input_pullup si el firmware soporta '2'
for try_pin in range(1,6):  # pruebo primero pins Atom 1..5
    try:
        mc.set_pin_mode(try_pin, 2)  # 2 = input_pullup (si está soportado)
    except Exception:
        pass

# escanear para encontrar el pin del botón
pin, baseline = scan_for_button_pin(mc, pins=range(1,33), timeout=12.0)
if pin is None:
    print("No encontré un pin que cambie. Probar ampliar el rango o confirmar firmware Atom.")
else:
    print(f"Usaré pin {pin} para monitorizar el botón.")
    # lanzar monitor en hilo para no bloquear
    t = threading.Thread(target=monitor_button, args=(mc, pin, run_my_script), daemon=True)
    t.start()
    # el hilo seguirá corriendo; tu programa principal puede continuar.
