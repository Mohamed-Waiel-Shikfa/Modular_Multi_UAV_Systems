"""task: dock, the runner: one or more missions in one simulation.

  include: []            one mission, this scenario (the usual case)
  include: [a, b, ...]   one mission per scenario file, side by side in the same simulation: each
                         keeps its own layout, cages, phases and checks; they start, fly and land at
                         the same time. Placed on a square-ish grid, footprints (ground, staging,
                         circle) apart by include_gap, so they do not touch or feel each other's downwash.
  sim: both              the whole thing in PyBullet, then in MuJoCo, then a plot of how far the two
                         runs drift apart over time (viz/diff.py).

The run exits 0 only if every check of every mission passes.
"""
import copy
import itertools
from pathlib import Path

import numpy as np

from modular_uav import sims
from modular_uav.config.loader import derive
from modular_uav.dock.mission import Mission, planned_duration
from modular_uav.utils.flight_log import FlightLog
from modular_uav.viz.plume import Plume


def missions(cfg):
    if not cfg.include:
        return [Mission(cfg)]
    out = [Mission(derive(cfg, f), name=Path(f).with_suffix("").as_posix().removeprefix("scenarios/"))
           for f in cfg.include]
    # Square-ish grid: ceil(sqrt(n)) columns, filled row by row (+x right, +y up, first row at the top).
    # Column widths and row heights are the widest / tallest footprint in them, include_gap apart.
    cols = int(np.ceil(np.sqrt(len(out))))
    fp = [m.footprint() for m in out]
    rows = [range(r, min(r + cols, len(out))) for r in range(0, len(out), cols)]
    w = [max(fp[i][1][0] - fp[i][0][0] for i in range(c, len(out), cols)) for c in range(cols)]
    h = [max(fp[i][1][1] - fp[i][0][1] for i in r) for r in rows]
    cx = np.cumsum([0] + [a + cfg.include_gap for a in w])[:-1] + np.array(w) / 2
    cy = -(np.cumsum([0] + [a + cfg.include_gap for a in h])[:-1] + np.array(h) / 2)
    centre = np.array([(cx[-1] + w[-1] / 2) / 2, (cy[-1] - h[-1] / 2) / 2])   # grid middle -> origin
    for r, row in enumerate(rows):
        for i in row:
            lo, hi = fp[i]
            m, c = out[i], i - r * cols
            m.offset = np.array([cx[c], cy[r]]) - centre - (lo + hi) / 2
    return out


def run_world(cfg):
    """(exit code, missions) of one simulation run."""
    ms = missions(cfg)
    for m in ms:
        print(m.describe())
    print(f"physics {cfg.physics}, downwash {cfg.downwash_model}, module collisions {cfg.module_collisions}")
    starts, welds, yaws, kinds, mtypes, no_contact = [], [], [], [], {}, []
    first = 0
    for n, m in enumerate(ms):
        starts.append(m.starts())
        yaws += list(m.yaws)
        kinds += [(n, k) for k in m.kinds]
        mtypes.update({(n, k): t for k, t in m.mtypes.items()})
        welds += [(first + i, first + j, o, q) for i, j, o, q in m.welds]
        no_contact += [(first + i, first + j) for i, j in m.no_contact]
        first += m.N
    if cfg.module_collisions == "off":                     # across missions too
        bounds = np.cumsum([0] + [m.N for m in ms])
        no_contact += [(i, j) for a, b in itertools.combinations(range(len(ms)), 2)
                       for i in range(bounds[a], bounds[a + 1]) for j in range(bounds[b], bounds[b + 1])]
    sim = sims.make(cfg, np.vstack(starts), welds, cfg.freq, yaws, mtypes, kinds, no_contact)
    if cfg.plume != "off":
        Plume(sim, cfg, cfg.freq)
    first = weld_first = 0
    for m in ms:
        m.attach(sim, first, weld_first)
        first, weld_first = first + m.N, weld_first + len(m.welds)
    N = first
    flog = FlightLog(cfg, N, cfg.freq, max(planned_duration(m.cfg) for m in ms))
    live = None
    if cfg.live_plot:
        from modular_uav.viz.live import LivePlot
        live = LivePlot(cfg, ms)
    rpm, targets = np.zeros((N, 4)), np.vstack(starts)
    k = 0
    while k < cfg.max_t * cfg.freq:
        out = [m.step(k) for m in ms]
        if all(o is None for o in out):
            break
        for m, o in zip(ms, out):
            sl = slice(m.first, m.first + m.N)
            if o is None:
                rpm[sl] = 0                                    # done (landed) or aborted: motors off
            else:
                rpm[sl], targets[sl] = o
        flog.log(k, sim, rpm, targets)
        sim.step(rpm)
        if live is not None:
            live.update(k)
        k += 1
    sim.close()
    flog.finish()
    if live is not None:
        live.finish()
    codes = [m.report() for m in ms]
    return max(codes), ms


def run(cfg):
    if cfg.sim != "both":
        return run_world(cfg)[0]
    from modular_uav.viz.diff import plot
    runs = {}
    for name in ("pybullet", "mujoco"):
        c = copy.copy(cfg)
        c.sim = name
        print(f"\n===== {name} =====")
        runs[name] = run_world(c)
    plot(cfg, {name: ms for name, (_, ms) in runs.items()})
    return max(code for code, _ in runs.values())
