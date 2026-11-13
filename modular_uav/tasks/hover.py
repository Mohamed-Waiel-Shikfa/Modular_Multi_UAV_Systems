"""task: hover. One module (caged or bare, per cfg.cage) hovers on the simulator's own stock PID,
gains untouched: DSLPIDControl in PyBullet, PIDControl in MuJoCo. Checks that the cage model
flies with the stock controller. The PID runs at hover_ctrl_freq, physics at freq (the RPM is
held between control steps). Passes if the final position error is below hover_tol.
"""
import copy

import numpy as np

from modular_uav import sims
from modular_uav.dock.mission import start_height
from modular_uav.utils.flight_log import FlightLog


def run(cfg):
    if cfg.sim == "both":                     # no trace to compare here: run in each, pass if both pass
        codes = []
        for name in ("pybullet", "mujoco"):
            c = copy.copy(cfg)
            c.sim = name
            codes.append(run(c))
        return max(codes)
    target = np.array(cfg.hover_target, float)
    sim = sims.make(cfg, [[0.0, 0.0, start_height(cfg)]])
    pid = sim.stock_pid()
    hold = cfg.freq // cfg.hover_ctrl_freq
    flog = FlightLog(cfg, 1, cfg.freq, cfg.hover_duration)
    every = int(round(cfg.print_every * cfg.freq))
    rpm = np.zeros(4)
    for k in range(int(cfg.hover_duration * cfg.freq)):
        if k % hold == 0:
            rpm = np.asarray(pid(0, hold / cfg.freq, target), float)
        flog.log(k, sim, rpm[None], target[None])
        sim.step(rpm[None])
        if every and k % every == every - 1:
            print(f"t={(k + 1) / cfg.freq:5.2f}s pos={np.round(sim.state(0)[0], 3)}")
    err = np.linalg.norm(sim.state(0)[0] - target)
    sim.close()
    flog.finish()
    ok = err < cfg.hover_tol
    print(f"\n  [{'PASS' if ok else 'FAIL'}] {cfg.cage} cage on {cfg.sim}: final position error "
          f"{100 * err:.1f} cm (< {100 * cfg.hover_tol:g} cm)")
    return 0 if ok else 1
