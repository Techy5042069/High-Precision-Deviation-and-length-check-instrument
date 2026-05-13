# ─────────────────────────────────────────────────────────────────────────────
# nodes.py
#
# Non-blocking communication wrappers for the two Arduino connections.
#
#   SerialNode — USB-serial to the motor Arduino (wired)
#   TCPNode    — WiFi TCP to the sensor Arduino  (wireless)
#
# Both expose the same interface:
#   .connect()         open the connection, start background recv thread
#   .send(msg)         write a command string (newline added automatically)
#   .poll(timeout)     return next queued line, or None if empty
#   .connected         bool — False if the link has dropped
#
# Threading model:
#   A daemon thread reads bytes continuously and pushes complete newline-
#   terminated lines into self.lines (a Queue).  The main thread calls poll()
#   on its own schedule — usually from GantryMaster._poll_loop().
# ─────────────────────────────────────────────────────────────────────────────

import queue
import socket
import threading
import time

import serial


class SerialNode:
    """
    Non-blocking USB-serial wrapper for the motor Arduino.

    The Arduino resets when DTR is toggled (which happens on Serial.open).
    We wait 2 s after opening to let the bootloader finish before sending
    any commands.
    """

    def __init__(self, name: str, port: str, baud: int = 115200):
        """
        Args:
            name:  Human-readable label used in log messages (e.g. 'Motor').
            port:  System serial device path (e.g. 'COM3' or '/dev/ttyACM0').
            baud:  Baud rate — must match the Arduino Serial.begin() call.
        """
        self.name      = name
        self.port      = port
        self.baud      = baud
        self.ser       = None           # pyserial Serial object
        self._lock     = threading.Lock()
        self.lines     = queue.Queue()  # received complete lines
        self.connected = False

    def connect(self):
        """
        Open the serial port and start the background receive thread.
        Blocks for ~2 s to let the Arduino finish its reset cycle.
        """
        print(f"  Connecting to {self.name} on {self.port}...")
        self.ser = serial.Serial(self.port, self.baud, timeout=1)
        time.sleep(2.0)                  # wait for Arduino DTR reset
        self.ser.reset_input_buffer()    # discard bootloader chatter
        self.connected = True
        print(f"  ✓ {self.name} connected")
        threading.Thread(target=self._recv_loop, daemon=True).start()

    def send(self, msg: str):
        """
        Write *msg* + '\\n' to the serial port.
        Thread-safe: can be called from any thread.
        """
        with self._lock:
            try:
                self.ser.write((msg + "\n").encode())
            except Exception as e:
                print(f"  [{self.name}] send error: {e}")

    def _recv_loop(self):
        """
        Background thread: read lines from serial and push to queue.
        Exits (setting connected=False) when the port raises an exception
        (usually because it was closed or the device disconnected).
        """
        try:
            while True:
                line = self.ser.readline().decode(errors="replace").strip()
                if line:
                    self.lines.put(line)
        except Exception:
            pass
        self.connected = False

    def poll(self, timeout: float = 0.0):
        """
        Return the next queued line without blocking longer than *timeout*.
        Returns None if no line is available within the timeout.
        """
        try:
            return self.lines.get(timeout=timeout)
        except queue.Empty:
            return None


class TCPNode:
    """
    Non-blocking TCP client for the sensor Arduino's WiFi server.

    The sensor Arduino runs a single-client TCP server on port 5001.
    TCP_NODELAY is set to minimise latency for TICK commands — each TICK
    triggers an ADC sample that must arrive before the motor advances
    another scan_sample_steps steps.

    Stream handling:
        TCP is a byte stream — a single recv() may contain a partial line
        or multiple lines.  _recv_loop() accumulates into a string buffer
        and splits on '\\n', pushing only complete lines to the queue.
    """

    def __init__(self, name: str, host: str, port: int):
        """
        Args:
            name:  Human-readable label (e.g. 'Sensor').
            host:  Sensor Arduino IP address (e.g. '192.168.137.5').
            port:  TCP port (SENSOR_PORT = 5001).
        """
        self.name      = name
        self.host      = host
        self.port      = port
        self.sock      = None           # connected socket
        self._buf      = ""             # partial-line accumulator
        self._lock     = threading.Lock()
        self.lines     = queue.Queue()  # received complete lines
        self.connected = False

    def connect(self, timeout: int = 10):
        """
        Open the TCP connection and start the background receive thread.

        Args:
            timeout: Connection timeout in seconds.
        """
        print(f"  Connecting to {self.name} at {self.host}:{self.port}...")
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(timeout)
        self.sock.connect((self.host, self.port))
        # Disable Nagle's algorithm — we need TICK commands delivered immediately,
        # not batched into larger segments by the OS.
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.sock.settimeout(None)       # switch recv thread to blocking mode
        self.connected = True
        print(f"  ✓ {self.name} connected")
        threading.Thread(target=self._recv_loop, daemon=True).start()

    def send(self, msg: str):
        """
        Send *msg* + '\\n' over TCP.
        Thread-safe: can be called from any thread.
        """
        with self._lock:
            try:
                self.sock.sendall((msg + "\n").encode())
            except Exception as e:
                print(f"  [{self.name}] send error: {e}")

    def _recv_loop(self):
        """
        Background thread: accumulate TCP stream into lines and push to queue.
        TCP recv() can return any number of bytes — we buffer until '\\n'.
        """
        try:
            while True:
                chunk = self.sock.recv(4096).decode(errors="replace")
                if not chunk:
                    break               # server closed the connection
                self._buf += chunk
                while "\n" in self._buf:
                    line, self._buf = self._buf.split("\n", 1)
                    line = line.strip()
                    if line:
                        self.lines.put(line)
        except Exception:
            pass
        self.connected = False

    def poll(self, timeout: float = 0.0):
        """
        Return the next queued line without blocking longer than *timeout*.
        Returns None if no line is available within the timeout.
        """
        try:
            return self.lines.get(timeout=timeout)
        except queue.Empty:
            return None
