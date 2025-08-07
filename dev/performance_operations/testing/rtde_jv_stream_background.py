import threading
import time
import rtde_receive

class JointCache:
    def __init__(self, robot_ip, poll_delay=0.001):
        self._rtde = rtde_receive.RTDEReceiveInterface(robot_ip)
        self._lock = threading.Lock()
        self._latest_q = None
        self._running = False
        self._delay = poll_delay   # seconds between polls

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
            q = self._rtde.getActualQ()   # or .receive()/unpack recipe
            with self._lock:
                self._latest_q = q
            time.sleep(self._delay)

    def get_latest(self):
        with self._lock:
            return list(self._latest_q) if self._latest_q is not None else None


if __name__ == "__main__":
    # Example usage
    # Make sure to replace "

    cache = JointCache("192.168.1.10")
    cache.start()

    # … elsewhere in your code …
    current_q = cache.get_latest()
    # time.sleep(0.1)  # Allow some time for the thread to update

    if current_q is not None:
        print("Current joint values:", current_q)
    else:
        print("No joint values available yet.")

    time.sleep(5)  # Simulate some processing time

    second_q = cache.get_latest()
    if second_q is not None:
        print("Second joint values:", second_q)


    # On shutdown:
    cache.stop()