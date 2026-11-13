"""Live plot (live_plot: true): a matplotlib window that follows the run as it flies.

One line per mission (assembly CoM, or the approach centroid before the snap), dashed = target:
  height z, m;  horizontal distance to the target, cm;  tilt, deg.
The title shows each mission's phase. Refreshed live_plot_rate times per second of sim time from
the missions' traces (sampled at log_rate), so it costs nothing between refreshes.
"""
import numpy as np


class LivePlot:
    def __init__(self, cfg, missions):
        import matplotlib.pyplot as plt
        self.plt, self.ms = plt, missions
        self.every = max(1, int(round(cfg.freq / cfg.live_plot_rate)))
        plt.ion()
        self.fig, self.ax = plt.subplots(3, 1, sharex=True, figsize=(8, 7))
        self.fig.canvas.manager.set_window_title(f"live plot: {cfg.scenario or 'simulate.py'} on {cfg.sim}")
        labels = ["height z, m", "horizontal error, cm", "tilt, deg"]
        self.lines = []
        for n, m in enumerate(missions):
            col = f"C{n}"
            name = m.name or "assembly"
            self.lines.append((self.ax[0].plot([], [], col, label=name)[0], self.ax[0].plot([], [], col + "--", lw=1)[0],
                               self.ax[1].plot([], [], col)[0], self.ax[2].plot([], [], col)[0]))
        for a, lab in zip(self.ax, labels):
            a.set_ylabel(lab)
            a.grid(alpha=0.3)
        self.ax[2].set_xlabel("sim time, s")
        if len(missions) > 1:
            self.ax[0].legend(loc="upper right", fontsize=8)
        self.fig.tight_layout()

    def update(self, k):
        if k % self.every:
            return
        phases = []
        for m, (z, zt, e, tilt) in zip(self.ms, self.lines):
            if not m.trace:
                continue
            t = np.array([r[0] for r in m.trace])
            com = np.array([r[2] for r in m.trace])
            tgt = np.array([r[3] for r in m.trace])
            rpy = np.array([r[4] for r in m.trace])
            z.set_data(t, com[:, 2])
            zt.set_data(t, tgt[:, 2])
            e.set_data(t, 100 * np.linalg.norm(com[:, :2] - tgt[:, :2], axis=1))
            tilt.set_data(t, np.degrees(np.arccos(np.clip(np.cos(rpy[:, 0]) * np.cos(rpy[:, 1]), -1, 1))))
            phases.append(f"{m.name + ': ' if m.name else ''}{m.trace[-1][1]}")
        for a in self.ax:
            a.relim()
            a.autoscale_view()
        self.ax[0].set_title("   ".join(phases), fontsize=9)
        self.fig.canvas.draw_idle()
        self.plt.pause(0.001)

    def finish(self):
        self.plt.ioff()
        if self.plt.get_backend().lower() != "agg":
            self.plt.show()
