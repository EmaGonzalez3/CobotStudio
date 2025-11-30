# robot_server_integration.py   <- pegar esto al inicio de tu rutina.py o importarlo
import socket
import threading
import time
import json

class RobotServer:
    """
    Server que publica por TCP la última lectura de ángulos del robot.
    Está pensado para integrarse en el mismo proceso que envía comandos al robot,
    para evitar que varias conexiones abran el puerto serie.
    Parámetros:
      - read_angles_callable: función sin argumentos que devuelve lista de floats (radianes)
      - host, port: bind del server
      - send_rate_hz: frecuencia de envío por socket
    """
    def __init__(self, read_angles_callable, host='0.0.0.0', port=65432, send_rate_hz=20.0):
        self.read_angles = read_angles_callable
        self.host = host
        self.port = port
        self.send_rate_hz = send_rate_hz

        self._latest = {"timestamp": None, "angles": None, "gripper": 0.0}
        self._gripper_val = 0.0 # Variable interna
        self._lock = threading.Lock()
        self._shutdown = threading.Event()

        self._client_connected = threading.Event() 

        self._sock_thread = None
        self._reader_thread = None

    def start(self):
        # Start reader thread (lee hardware)
        self._reader_thread = threading.Thread(target=self._serial_reader_loop, daemon=True)
        self._reader_thread.start()
        # Start socket listener thread
        self._sock_thread = threading.Thread(target=self._socket_server_loop, daemon=True)
        self._sock_thread.start()

    def stop(self):
        self._shutdown.set()
        # close will be handled by threads on their loops finishing

    def wait_for_connection(self, timeout=None):
        """Bloquea la ejecución hasta que un cliente se conecte."""
        print(f"[RobotServer] Esperando conexión de cliente ROS2 en {self.host}:{self.port}...")
        connected = self._client_connected.wait(timeout=timeout)
        if connected:
            print("[RobotServer] ¡Cliente conectado! Iniciando secuencia...")
        else:
            print("[RobotServer] Timeout esperando cliente (o se continuó sin espera).")
        return connected
    
    def set_gripper_state(self, value: float):
        """Recibe un valor (ej: 0 a 100 o radianes) y lo guarda para enviar."""
        with self._lock:
            self._gripper_val = float(value)

    def _serial_reader_loop(self):
        # Lee el robot periódicamente y actualiza _latest
        poll_interval = 1.0 / max(1.0, self.send_rate_hz * 2.0)
        while not self._shutdown.is_set():
            try:
                angles = self.read_angles()
                ts = time.time()
                if angles is not None:
                    with self._lock:
                        self._latest["timestamp"] = ts
                        self._latest["angles"] = list(map(float, angles))
                        self._latest["gripper"] = self._gripper_val
            except Exception as e:
                # opcional: logging
                print(f"[RobotServer] error leyendo ángulos: {e}")
            time.sleep(poll_interval)

    def _handle_client(self, conn, addr):
        print(f"[RobotServer] cliente conectado {addr}")
        self._client_connected.set()  # Marcamos que hay conexión
        try:
            while not self._shutdown.is_set():
                with self._lock:
                    payload = {
                        "timestamp": self._latest["timestamp"],
                        "angles": self._latest["angles"],
                        "gripper": self._latest["gripper"]
                    }
                message = json.dumps(payload, allow_nan=False).encode('utf-8') + b'\n'
                try:
                    conn.sendall(message)
                except (BrokenPipeError, ConnectionResetError):
                    print("[RobotServer] cliente desconectado")
                    break
                time.sleep(1.0 / self.send_rate_hz)
        finally:
            self._client_connected.clear() # Si sale, marcamos desconexión
            try:
                conn.close()
            except:
                pass

    def _socket_server_loop(self):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            s.bind((self.host, self.port))
            s.listen(1)
            s.settimeout(1.0)
            print(f"[RobotServer] escuchando en {self.host}:{self.port}")
            while not self._shutdown.is_set():
                try:
                    conn, addr = s.accept()
                    t = threading.Thread(target=self._handle_client, args=(conn, addr), daemon=True)
                    t.start()
                except socket.timeout:
                    continue

    # Helper público si necesitás acceder a la última lectura desde otro hilo
    def get_latest(self):
        with self._lock:
            return dict(self._latest)

