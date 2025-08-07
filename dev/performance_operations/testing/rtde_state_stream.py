import threading
import time
import rtde_receive

class RobotStateCache:
    def __init__(self, robot_ip, poll_delay=0.001):
        self._rtde = rtde_receive.RTDEReceiveInterface(robot_ip)
        self._lock = threading.Lock()
        self._latest_q   = None
        self._latest_tcp = None
        self._running    = False
        self._delay      = poll_delay

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

if __name__ == "__main__":
    cache = RobotStateCache("192.168.1.10")
    cache.start()

    time.sleep(0.1)  # give it a moment to fill

    state = cache.get_latest()
    if state:
        q, tcp = state
        print("Joints:", q)
        print("TCP   :", tcp)
    else:
        print("Still warming up...")

    time.sleep(5)  # Simulate some processing time

    jv, tcp = cache.get_latest()
    if jv is not None:
        print("Updated Joints:", jv)
        print("Updated TCP   :", tcp)
    else:
        print("No joint values available yet.")

    cache.stop()