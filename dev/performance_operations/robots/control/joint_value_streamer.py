
import time
import threading
import rtde_receive

class RTDEStateStreamer:

    def __init__(self, robot_ip, poll_delay=0.001):
        self._rtde = rtde_receive.RTDEReceiveInterface(robot_ip)
        self._lock = threading.Lock()
        self._latest_q   = None
        self._latest_tcp = None
        self._running    = False
        self._delay      = poll_delay
        print(f"[RTDEStateStreamer] Initialized for robot at {robot_ip}; running={self._running}")

    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        self._thread.join()

    def _poll_loop(self):
        while self._running:
            q   = self._rtde.getActualQ()
            tcp = self._rtde.getActualTCPPose()
            with self._lock:
                self._latest_q   = q
                self._latest_tcp = tcp
            time.sleep(self._delay)

    def get_latest(self):
        with self._lock:
            # return a tuple (q, tcp) or None if not ready
            if self._latest_q is None:
                return None
            return list(self._latest_q), list(self._latest_tcp)
        

class ABBStateStreamer:

    def __init__(self, robot_ip, poll_delay=0.001):
        raise NotImplementedError("This class is a placeholder for ABB joint value streaming. Implementation needed.")
        # self._rtde = rtde_receive.RTDEReceiveInterface(robot_ip)
        # self._lock = threading.Lock()
        # self._latest_q = None
        # self._running = False
        # self._delay = poll_delay
        print(f"[ABBStateStreamer] Initialized for robot at {robot_ip}; running={self._running}")


    def start(self):
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        self._thread.join()

    def _poll_loop(self):
        while self._running:
            q = self._rtde.getActualQ()
            with self._lock:
                self._latest_q = q
            time.sleep(self._delay)

    def get_latest(self):
        with self._lock:
            return list(self._latest_q) if self._latest_q is not None else None