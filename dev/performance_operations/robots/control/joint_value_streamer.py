
# import time
# import threading
# import rtde_receive

# class RTDEStateStreamer:

#     def __init__(self, robot_ip, poll_delay=0.001):
#         self._rtde = rtde_receive.RTDEReceiveInterface(robot_ip)
#         self._lock = threading.Lock()
#         self._latest_q   = None
#         self._latest_tcp = None
#         self._running    = False
#         self._delay      = poll_delay
#         print(f"[RTDEStateStreamer] Initialized for robot at {robot_ip}; running={self._running}")

#     def start(self):
#         if self._running:
#             return
#         self._running = True
#         self._thread = threading.Thread(target=self._poll_loop, daemon=True)
#         self._thread.start()

#     def stop(self):
#         self._running = False
#         self._thread.join()

#     def _poll_loop(self):
#         while self._running:
#             q   = self._rtde.getActualQ()
#             tcp = self._rtde.getActualTCPPose()
#             with self._lock:
#                 self._latest_q   = q
#                 self._latest_tcp = tcp
#             time.sleep(self._delay)

#     def get_latest(self):
#         with self._lock:
#             # return a tuple (q, tcp) or None if not ready
#             if self._latest_q is None:
#                 return None
#             return list(self._latest_q), list(self._latest_tcp)
        
import time
import threading
import rtde_receive

class RTDEStateStreamer:
    """
    Minimal state streamer with optional sim mode.

    - Normal: pulls q/tcp from a real UR via RTDEReceiveInterface.
    - Sim: never opens RTDE; serves user-provided (or zero) values.
      You can push values using set_sim_state(q[, tcp]).
    """

    def __init__(self, robot_ip, poll_delay=0.001, sim=False, sim_joint_count=6):
        self._lock = threading.Lock()
        self._latest_q   = None
        self._latest_tcp = None
        self._running    = False
        self._delay      = poll_delay
        self._sim        = bool(sim)
        self._thread     = None

        # If sim, don't create RTDE object at all
        if not self._sim:
            self._rtde = rtde_receive.RTDEReceiveInterface(robot_ip)
            mode = "RTDE"
        else:
            self._rtde = None
            # initialize with zeros so get_latest() returns something usable immediately
            self._latest_q   = [0.0] * int(sim_joint_count)
            self._latest_tcp = [-0.09937183964480224, -0.35709644876924446, 0.21218450438745523, -1.43879168415704, -1.5695205093887492, -1.6701313616130904] #[0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
            mode = f"SIM (joints={sim_joint_count})"

        print(f"[RTDEStateStreamer] Initialized for robot at {robot_ip}; mode={mode}; running={self._running}")

    # --- public API ---------------------------------------------------------

    def start(self):
        if self._running:
            return
        self._running = True
        # In sim mode we still spin so consumers see a consistent cadence,
        # but we won't touch hardware.
        self._thread = threading.Thread(target=self._poll_loop, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False
        t = self._thread
        if t and t.is_alive():
            t.join(timeout=1.0)

    def get_latest(self):
        with self._lock:
            # return a tuple (q, tcp) or None if not ready
            if self._latest_q is None:
                return None
            return list(self._latest_q), list(self._latest_tcp)

    def set_sim_state(self, q, tcp=None):
        """
        Sim-only helper: push joint/tcp values into the streamer.

        q   : iterable of joint radians (len = your robot dof)
        tcp : iterable [x, y, z, rx, ry, rz] (optional)
        """
        if not self._sim:
            # ignore in hardware mode to avoid confusion
            return
        with self._lock:
            self._latest_q = list(q)
            if tcp is not None:
                self._latest_tcp = list(tcp)

    # --- internals ----------------------------------------------------------

    def _poll_loop(self):
        while self._running:
            if not self._sim:
                try:
                    q   = self._rtde.getActualQ()
                    tcp = self._rtde.getActualTCPPose()
                    if q is not None and tcp is not None:
                        with self._lock:
                            self._latest_q   = q
                            self._latest_tcp = tcp
                except Exception:
                    # swallow transient network errors; keep last good state
                    pass
            # in sim mode we don’t pull anything here; values are whatever
            # was set via set_sim_state() (or zeros from __init__)
            time.sleep(self._delay)

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