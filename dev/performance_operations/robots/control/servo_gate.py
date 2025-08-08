import time
import numpy as np

class EMA:
    """Tiny exponential smoother for joint targets and for speed/accel commands."""
    def __init__(self, alpha=0.25):
        self.a = float(alpha)
        self.y = None
    def push(self, x):
        x = np.asarray(x, float)
        self.y = x if self.y is None else (1.0 - self.a) * self.y + self.a * x
        return self.y

class ServoJGate:
    """
    No-thread streaming for UR 'servoj':
    - call set_target(q) whenever you have a new joint target (list/np array/Configuration)
    - call tick() every loop iteration (aim >= ~10 Hz; 60–125 Hz is ideal but not required)
    - driver adapts speed/accel based on recent motion; filters targets to kill noise
    """
    def __init__(self, rtde_ctrl, rtde_recv=None,
                 # output caps (UR limits are higher; keep conservative)
                 speed_cap=0.8,          # rad/s max
                 accel_cap=1.5,          # rad/s^2 max
                 # controller params
                 dt_nominal=1/125.0,     # desired stream period
                 lookahead=0.10,         # 0.06–0.15 typical
                 gain=300,               # 200–400 typical
                 # input smoothing
                 target_alpha=0.25,      # EMA on q target (higher = snappier)
                 # adaptive scaling
                 k_speed=1.5,            # scales measured |dq|/dt -> speed
                 tau=0.30,               # accel ≈ speed / tau
                 cmd_alpha=0.35,         # EMA on speed/accel commands
                 # gating
                 min_dt_send=0.008,      # don't send faster than this (sec)
                 min_dq=0.0,             # optional joint deadband before send
                 verbose=False):
        self.ctrl = rtde_ctrl
        self.recv = rtde_recv

        self.speed_cap = float(speed_cap)
        self.accel_cap = float(accel_cap)
        self.dt_nominal = float(dt_nominal)
        self.look = float(lookahead)
        self.gain = int(gain)

        self.smoother = EMA(target_alpha)
        self.cmd_spd_ema = EMA(cmd_alpha)
        self.cmd_acc_ema = EMA(cmd_alpha)

        self.k_speed = float(k_speed)
        self.tau = float(tau)

        self._q_target = None
        self._q_last_sent = None
        self._t_last = None

        self.min_dt_send = float(min_dt_send)
        self.min_dq = float(min_dq)
        self.verbose = bool(verbose)

    def set_target(self, q):
        """Accept list/np-array/Configuration; store filtered target."""
        # unwrap compas_fab Configuration if needed
        if hasattr(q, "joint_values"):
            q = q.joint_values
        q = np.asarray(q, float)
        self._q_target = self.smoother.push(q)

    def _estimate_speed(self, q_now, now):
        """Estimate worst-joint speed from last sent value."""
        if self._q_last_sent is None or self._t_last is None:
            return 0.0
        dt = max(1e-3, now - self._t_last)
        dq = float(np.max(np.abs(q_now - self._q_last_sent)))
        return dq / dt  # rad/s

    def _adapt_cmd_caps(self, v_est):
        # speed command target
        spd_tgt = self.k_speed * float(v_est)
        spd_tgt = np.clip(spd_tgt, 0.2, self.speed_cap)  # keep a low floor to avoid stall
        spd_cmd = self.cmd_spd_ema.push(np.array([spd_tgt]))[0]

        # accel from speed target
        acc_tgt = spd_cmd / self.tau
        acc_tgt = np.clip(acc_tgt, 0.5, self.accel_cap)
        acc_cmd = self.cmd_acc_ema.push(np.array([acc_tgt]))[0]
        return float(spd_cmd), float(acc_cmd)

    def tick(self):
        """
        Send one servoj command if enough time has elapsed.
        Call this every loop iteration; it returns immediately.
        """
        now = time.perf_counter()

        # decide what to send
        if self._q_target is None:
            # no external target: hold current measured joints if receiver exists
            if self.recv is None:
                return False
            q = np.asarray(self.recv.getActualQ(), float)
        else:
            q = self._q_target

        # rate-limit the sends (no busy loop; no thread)
        if self._t_last is not None and (now - self._t_last) < self.min_dt_send:
            return False

        # optional deadband to reduce chatter
        if self._q_last_sent is not None and self.min_dq > 0.0:
            if np.max(np.abs(q - self._q_last_sent)) < self.min_dq:
                # still send periodically so controller stays happy
                # but we can stretch the period a bit (up to 0.2s safely)
                pass

        # compute dt used for this tick (UR is happy with 0.004–0.20)
        if self._t_last is None:
            dt_used = self.dt_nominal
        else:
            dt_used = np.clip(now - self._t_last, 0.004, 0.20)

        # adapt speed/accel to motion magnitude
        v_est = self._estimate_speed(q, now)
        spd_cmd, acc_cmd = self._adapt_cmd_caps(v_est)

        # send
        try:
            self.ctrl.servoJ(q.tolist(), spd_cmd, acc_cmd, dt_used, self.look, self.gain)
        except Exception as e:
            if self.verbose:
                print(f"[ServoJGate] servoJ error: {e}")
            return False

        self._q_last_sent = q
        self._t_last = now
        return True

    def stop(self):
        try:
            self.ctrl.servoStop()
        except Exception:
            pass