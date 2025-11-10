import time
import numpy as np

class EMA:
    def __init__(self, alpha=0.25):
        self.a = float(alpha)
        self.y = None
    def push(self, x):
        x = np.asarray(x, float)
        self.y = x if self.y is None else (1.0 - self.a) * self.y + self.a * x
        return self.y


#TODO: TRY TO LOWER THE GAIN CHECK VALUES
# https://sdurobotics.gitlab.io/ur_rtde/examples/examples.html#speedj-example

class ServoJGate:
    def __init__(self, rtde_ctrl, rtde_recv=None,
                 speed_cap=0.8, accel_cap=1.5,
                 dt_nominal=1/125.0, lookahead=0.10, gain=300,
                 target_alpha=0.25, k_speed=1.5, tau=0.30, cmd_alpha=0.35,
                 min_dt_send=0.010, min_dq=0.0, verbose=False):
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
        self.min_dt_send = float(min_dt_send)
        self.min_dq = float(min_dq)
        self.verbose = bool(verbose)
        self._q_target = None
        self._q_last_sent = None
        self._t_last = None

    def set_target(self, q):
        if hasattr(q, "joint_values"):
            q = q.joint_values
        q = np.asarray(q, float)
        self._q_target = self.smoother.push(q)

    def _estimate_speed(self, q_now, now):
        if self._q_last_sent is None or self._t_last is None:
            return 0.0
        dt = max(1e-3, now - self._t_last)
        dq = float(np.max(np.abs(q_now - self._q_last_sent)))
        return dq / dt

    def _adapt_cmd_caps(self, v_est):
        spd_tgt = np.clip(self.k_speed * float(v_est), 0.2, self.speed_cap)
        spd_cmd = self.cmd_spd_ema.push(np.array([spd_tgt]))[0]
        acc_tgt = np.clip(spd_cmd / self.tau, 0.5, self.accel_cap)
        acc_cmd = self.cmd_acc_ema.push(np.array([acc_tgt]))[0]
        return float(spd_cmd), float(acc_cmd)

    #RESET SESSION ADD
    def reset_session(self, seed_q=None):
        """Clear internal state and optionally seed target to current joints."""
        self.smoother = EMA(self.smoother.a)      # reset EMA
        self.cmd_spd_ema = EMA(self.cmd_spd_ema.a)
        self.cmd_acc_ema = EMA(self.cmd_acc_ema.a)
        self._q_last_sent = None
        self._t_last = None
        if seed_q is None and self.recv is not None:
            try:
                seed_q = np.asarray(self.recv.getActualQ(), float)
            except Exception:
                seed_q = None
        self._q_target = None if seed_q is None else self.smoother.push(np.asarray(seed_q, float))

    def warm_start_hold(self, hold_s=0.4, gain_warm=120, look_warm=0.18):
        """
        For ~hold_s, send the current actual joints with gentle gains.
        Prevents the first 'real' target from yanking the arm.
        """
        if self.recv is None:
            return
        t0 = time.perf_counter()
        while time.perf_counter() - t0 < hold_s:
            q_act = np.asarray(self.recv.getActualQ(), float)
            now = time.perf_counter()
            # compute dt
            dt_used = self.dt_nominal if self._t_last is None else float(np.clip(now - self._t_last, 0.004, 0.20))
            try:
                # conservative speed/acc during warm
                self.ctrl.servoJ(q_act.tolist(), 0.25, 0.75, dt_used, look_warm, gain_warm)
            except Exception:
                break
            self._q_last_sent = q_act
            self._t_last = now
            time.sleep(max(0.0, self.min_dt_send - 0.002))

    def tick(self):
        now = time.perf_counter()
        if self._q_target is None:
            if self.recv is None:
                return False
            q = np.asarray(self.recv.getActualQ(), float)
        else:
            q = self._q_target

        if self._t_last is not None and (now - self._t_last) < self.min_dt_send:
            return False

        if self._t_last is None:
            dt_used = self.dt_nominal
        else:
            dt_used = float(np.clip(now - self._t_last, 0.004, 0.20))

        v_est = self._estimate_speed(q, now)
        spd_cmd, acc_cmd = self._adapt_cmd_caps(v_est)

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