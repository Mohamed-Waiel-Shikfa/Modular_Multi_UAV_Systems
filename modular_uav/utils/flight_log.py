"""Flight log for any backend, on top of the shared Logger (utils/Logger.py).

Samples every module at cfg.log_rate Hz in Logger's 20-float state layout:
pos (world, m), quat (x, y, z, w), roll/pitch/yaw (rad, extrinsic xyz), vel (world, m/s),
body rates (body frame, rad/s; upstream PyBullet logs world-frame rates here), rotor RPM.
Control targets: target position (world, m), the rest zero.
"""
import numpy as np
from scipy.spatial.transform import Rotation

from modular_uav.utils.Logger import Logger


class FlightLog:
    def __init__(self, cfg, n, freq, duration):
        """n modules, freq: Hz of log() calls, duration: s, expected (preallocates)."""
        self.on = cfg.plot or cfg.save_log
        self.cfg, self.n, self.freq = cfg, n, freq
        self.every = max(1, int(round(freq / cfg.log_rate)))
        if self.on:
            self.logger = Logger(logging_freq_hz=cfg.log_rate, output_folder=cfg.output_folder, num_drones=n,
                                 duration_sec=int(np.ceil(duration)) + 1)

    def log(self, k, sim, rpm, targets):
        """k: call index, rpm: (n, 4), targets: (n, 3) target positions, m."""
        if not self.on or k % self.every:
            return
        t = k / self.freq
        for i in range(self.n):
            pos, vel, R, w = sim.state(i)
            state = np.hstack([pos, sim.quat_xyzw(i), Rotation.from_matrix(R).as_euler("xyz"), vel, w, rpm[i]])
            self.logger.log(drone=i, timestamp=t, state=state, control=np.hstack([targets[i], np.zeros(9)]))

    def finish(self):
        if not self.on:
            return
        lg = self.logger
        k = int(lg.counters.min())                  # drop unused preallocated samples
        lg.timestamps, lg.states, lg.controls = lg.timestamps[:, :k], lg.states[:, :, :k], lg.controls[:, :, :k]
        if self.cfg.save_log:
            lg.save()
            print(f"flight log saved in {self.cfg.output_folder}/")
        if self.cfg.plot:
            lg.plot()
