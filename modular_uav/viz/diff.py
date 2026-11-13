"""sim: both. How far the PyBullet and MuJoCo runs of the same scenario drift apart over time.

Both runs record each mission's trace at log_rate from t = 0 (same freq, same start), so samples
line up by index. For each mission, against sim time:
  CoM distance, cm          |CoM_pybullet - CoM_mujoco| (assembly CoM; approach centroid before the snap)
  attitude difference, deg  angle of R_pybullet^T R_mujoco
  worst module distance, cm max over modules of |P_i,pybullet - P_i,mujoco|
Shaded bands: the PyBullet phases; a red tick wherever the two runs are in different phases.
Saved as output_folder/diff_<scenario>.png; shown in a window too with plot: true.
"""
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation


def series(a, b):
    """Aligned difference series of two traces (lists of (t, phase, com, target, rpy, P))."""
    n = min(len(a), len(b))
    t = np.array([r[0] for r in a[:n]])
    d_com = np.array([np.linalg.norm(x[2] - y[2]) for x, y in zip(a[:n], b[:n])])
    d_att = np.array([np.degrees((Rotation.from_euler("xyz", x[4]).inv() * Rotation.from_euler("xyz", y[4])).magnitude())
                      for x, y in zip(a[:n], b[:n])])
    d_mod = np.array([np.linalg.norm(x[5] - y[5], axis=1).max() for x, y in zip(a[:n], b[:n])])
    same = np.array([x[1] == y[1] for x, y in zip(a[:n], b[:n])])
    return t, 100 * d_com, d_att, 100 * d_mod, same, [r[1] for r in a[:n]]


def _name(cfg):
    """scenarios/tiling/kagome/fail.yaml -> tiling/kagome/fail."""
    return Path(cfg.scenario or "run").with_suffix("").as_posix().split("scenarios/")[-1]


def plot(cfg, runs):
    import matplotlib
    if not cfg.plot:
        matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    pb, mj = runs["pybullet"], runs["mujoco"]
    fig, ax = plt.subplots(3, 1, sharex=True, figsize=(9, 7))
    print("\nPyBullet vs MuJoCo:")
    for n, (a, b) in enumerate(zip(pb, mj)):
        t, d_com, d_att, d_mod, same, phases = series(a.trace, b.trace)
        name = a.name or _name(cfg)
        for axis, y in zip(ax, (d_com, d_att, d_mod)):
            axis.plot(t, y, f"C{n}", label=name)
        if n == 0:                                         # phase bands of the first mission
            start = 0
            for i in range(1, len(phases) + 1):
                if i == len(phases) or phases[i] != phases[start]:
                    for axis in ax:
                        axis.axvspan(t[start], t[i - 1], color="0.5" if len(set(phases[:i])) % 2 else "0.8", alpha=0.12, lw=0)
                    ax[0].text((t[start] + t[i - 1]) / 2, 1.02, phases[start], transform=ax[0].get_xaxis_transform(),
                               ha="center", fontsize=7, rotation=0)
                    start = i
        if (~same).any():
            ax[0].plot(t[~same], np.zeros((~same).sum()), "r|", ms=8)
        if len(t):
            print(f"  {name}: CoM max {d_com.max():.2f} cm (final {d_com[-1]:.2f}), attitude max {d_att.max():.2f} deg, "
                  f"worst module max {d_mod.max():.2f} cm; phases differ for {(~same).mean():.0%} of the run")
    for axis, lab in zip(ax, ("CoM distance, cm", "attitude difference, deg", "worst module distance, cm")):
        axis.set_ylabel(lab)
        axis.grid(alpha=0.3)
    ax[2].set_xlabel("sim time, s")
    if len(pb) > 1:
        ax[0].legend(fontsize=8)
    fig.suptitle("PyBullet vs MuJoCo: disagreement over time", fontsize=11)
    fig.tight_layout()
    out = Path(cfg.output_folder)
    out.mkdir(parents=True, exist_ok=True)
    path = out / f"diff_{_name(cfg).replace('/', '_')}.png"
    fig.savefig(path, dpi=120)
    print(f"  diff plot saved: {path}")
    if cfg.plot:
        plt.show()
    plt.close(fig)
