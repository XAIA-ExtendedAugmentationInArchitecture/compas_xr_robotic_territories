# import time
# import numpy as np

# # class MoveJGate:
# #     def __init__(self, urc, speed, accel, nowait=True, min_dt=0.18, min_dq=0.015):
# #         """
# #         urc    : RTDEControlInterface
# #         speed  : rad/s (max joint speed)
# #         accel  : rad/s^2 (max joint accel)
# #         nowait : True -> async moveJ
# #         min_dt : min seconds between sends (rate limit)
# #         min_dq : min max|Δq| (radians) to send (deadband)
# #         """
# #         self.urc = urc
# #         self.speed = float(speed)
# #         self.accel = float(accel)
# #         self.nowait = bool(nowait)
# #         self.min_dt = float(min_dt)
# #         self.min_dq = float(min_dq)
# #         self._last_t = 0.0
# #         self._last_q = None

# #     def maybe_send(self, q_target):
# #         q = np.asarray(q_target, dtype=float)
# #         now = time.perf_counter()

# #         # rate limit
# #         if now - self._last_t < self.min_dt:
# #             return False
# #         # ignore tiny changes
# #         if self._last_q is not None and np.max(np.abs(q - self._last_q)) < self.min_dq:
# #             return False

# #         # non-blocking moveJ
# #         self.urc.moveJ(q.tolist(), self.speed, self.accel, self.nowait)
# #         self._last_q = q
# #         self._last_t = now
# #         return True


import time
import threading
from collections import deque
import numpy as np

class MoveJGate:
    def __init__(self, urc, speed, accel, nowait=True,
                 min_dt=0.18, min_dq=0.015,
                 # blending options (set blend_radius>0 to enable)
                 blend_radius=0.0,      # meters in joint-space moveJ path API (UR expects a radius value; typical 0.005–0.03)
                 blend_batch=4,         # how many recent points to include
                 blend_every=0.35):     # seconds between path flushes
        """
        urc          : RTDEControlInterface
        speed        : rad/s (max joint speed)
        accel        : rad/s^2 (max joint accel)
        nowait       : True -> async moveJ for single-target sends
        min_dt       : min seconds between single-target sends (rate limit)
        min_dq       : min max|Δq| (radians) to send single-target
        blend_radius : >0 enables blended micro-batching via moveJ(path)
        blend_batch  : number of waypoints per micro-batch path (2–6 is typical)
        blend_every  : min seconds between path flushes
        """
        self.urc = urc
        self.speed = float(speed)
        self.accel = float(accel)
        self.nowait = bool(nowait)
        self.min_dt = float(min_dt)
        self.min_dq = float(min_dq)

        self._last_t = 0.0
        self._last_q = None

        # blending
        self.blend_radius = float(blend_radius)
        self.blend_batch = int(max(2, blend_batch))
        self.blend_every = float(blend_every)
        self._buf = deque(maxlen=self.blend_batch)
        self._last_flush = 0.0
        self._path_thread = None
        self._path_lock = threading.Lock()

    def _path_worker(self, path):
        try:
            # path items are [q1..q6, speed, accel, blend]
            self.urc.moveJ(path)  # this blocks inside the thread, not your main loop
        except Exception as e:
            # optional: log e
            pass

    def _try_flush_path(self):
        """Flush a tiny blended path if due and no worker is running."""
        if self.blend_radius <= 0.0:
            return False
        now = time.perf_counter()
        if now - self._last_flush < self.blend_every:
            return False

        # don't start another if one is running
        if self._path_thread and self._path_thread.is_alive():
            return False

        if len(self._buf) < 2:
            return False

        with self._path_lock:
            qs = list(self._buf)
            # build moveJ(path) payload: [q..., speed, accel, blend]
            path = [q.tolist() + [self.speed, self.accel, self.blend_radius] for q in qs]
            # keep last point to seed next batch continuity
            if len(self._buf) > 1:
                last = self._buf[-1]
                self._buf.clear()
                self._buf.append(last)

        t = threading.Thread(target=self._path_worker, args=(path,), daemon=True)
        t.start()
        self._path_thread = t
        self._last_flush = now
        return True

    def maybe_send(self, q_target):
        q = np.asarray(q_target, dtype=float)
        now = time.perf_counter()

        # If blending is enabled, collect the target.
        if self.blend_radius > 0.0:
            self._buf.append(q)
            # Try to flush a blended path; if flushed, we skip single-target send.
            if self._try_flush_path():
                self._last_q = q
                self._last_t = now
                return True  # we launched a blended path

        # Otherwise (or if not flushed), fall back to fast single-target moveJ with gating
        if now - self._last_t < self.min_dt:
            return False
        if self._last_q is not None and np.max(np.abs(q - self._last_q)) < self.min_dq:
            return False

        self.urc.moveJ(q.tolist(), self.speed, self.accel, self.nowait)
        self._last_q = q
        self._last_t = now
        return True