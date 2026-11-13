"""Live aero isolines in the simulator viewer (plume: dw | gnd | drag | all, needs gui).

Each backend draws the segments with Sim.draw_plume() in its own viewer.
Isolines of equal felt force, one per level in cfg.plume_levels (multiples of one module's weight W):
  dw    under each module, the surface where a module would feel L of downwash from it, drawn as a
        vertical oval (x-z plane) and a ring at its widest point. Uses the configured model
        (physics/downwash.py, original or regularised) at hover thrust T = W. The model is
        axisymmetric, so F = L has the closed form r(dz) = |b| sqrt(2 ln(F0(dz) / L)) where the
        centreline force F0(dz) > L. It depends only on the coefficients: computed once and moved
        with the modules. Each oval is one module's wake; overlapping wakes add up in the sims, not here.
  gnd   a horizontal outline around the modules at the height where ground effect at hover thrust
        reaches L: h = PROP_RADIUS / 4 sqrt(GND_EFF_COEFF W / L) (upstream _groundEffect). Levels
        above the value at the clip height GND_EFF_H_CLIP never occur and are not drawn.
  drag  an arrow from each module along its drag force (upstream _drag), 5 cm per factor of 10
        above the lowest level, coloured by its size.
  all   all three.
Colour is viridis on log10(L / W): dark purple = weakest level, bright yellow = strongest.
Forces are in the world frame (PyBullet applies downwash and drag in the body frame); rotor
heights are taken equal to the CoM height. Units: m, s, N.
"""
import numpy as np

from modular_uav.physics import downwash

N_DZ = 12                   # samples down each side of an oval
N_RING = 16                 # segments per ring
VIRIDIS = np.array([[68, 1, 84], [59, 82, 139], [33, 145, 140], [94, 201, 98], [253, 231, 37]]) / 255
COMPONENTS = {"dw": "downwash", "gnd": "ground effect", "drag": "drag", "all": "all"}


def viridis(v):
    """(K,) in [0, 1] -> (K, 3) rgb."""
    x = np.clip(v, 0, 1) * (len(VIRIDIS) - 1)
    i = np.minimum(x.astype(int), len(VIRIDIS) - 2)
    return VIRIDIS[i] + (x - i)[:, None] * (VIRIDIS[i + 1] - VIRIDIS[i])


def loop(p):
    """Closed polyline (K, 3) -> segments (K, 2, 3)."""
    return np.stack([p, np.roll(p, -1, 0)], 1)


def ring(r, z):
    t = 2 * np.pi * np.arange(N_RING) / N_RING
    return loop(np.column_stack([r * np.cos(t), r * np.sin(t), np.full(N_RING, z)]))


class Plume:
    def __init__(self, sim, cfg, freq):
        """Wrap sim.step: every freq / plume_rate steps, call sim.draw_plume with the isolines."""
        e = sim.env
        self.sim, self.cfg, self.comp = sim, cfg, cfg.plume
        self.levels = np.array(cfg.plume_levels, float)
        self.W, self.R = e.M * e.G, e.PROP_RADIUS
        self.GE, self.HCLIP, self.DRAG = e.GND_EFF_COEFF, e.GND_EFF_H_CLIP, np.asarray(e.DRAG_COEFF, float)
        self.every, self.k = max(1, int(freq // cfg.plume_rate)), 0
        self.wake = [self.oval(L * self.W, e) for L in self.levels]      # per level, module frame
        self.wake_rgb = [np.repeat(self.colour(L), len(s), 0) for L, s in zip(self.levels, self.wake)]
        print(f"plume: {COMPONENTS[self.comp]} isolines ({cfg.downwash_model} downwash, physics {cfg.physics}). "
              f"Each line joins points of equal force, as a multiple of one module's weight W = {self.W:.3f} N: "
              + ", ".join(f"{L:g} W" for L in self.levels) + ". Dark purple is the weakest, bright yellow the strongest.")
        step = sim.step

        def step_and_draw(rpm):
            step(rpm)
            if self.k % self.every == 0:
                self.draw(np.asarray(rpm, float).reshape(-1, 4))
            self.k += 1
        sim.step = step_and_draw

    def colour(self, level):
        """Colour of a force level (x W): log scale from the lowest level (purple) to the highest (yellow)."""
        lo, hi = np.log10(self.levels[[0, -1]])
        return viridis((np.log10(np.atleast_1d(level)) - lo) / max(hi - lo, 1e-9))

    def oval(self, L, e):
        """Segments (K, 2, 3) of the surface where one hovering module's downwash equals L (N), around (0, 0, 0)."""
        dz = np.geomspace(1e-4, 5.0, 2000)
        F0 = downwash.force(dz, 0.0, e, self.W, self.cfg)             # centreline force at hover thrust
        inside = np.flatnonzero(F0 > L)
        if len(inside) == 0:
            return np.zeros((0, 2, 3))
        dz = np.geomspace(dz[inside[0]], dz[inside[-1]], N_DZ)
        F0 = downwash.force(dz, 0.0, e, self.W, self.cfg)
        b = e.DW_COEFF_2 * dz + e.DW_COEFF_3
        if self.cfg.downwash_model == "regularised":
            w = e.L if self.cfg.dw_min_width is None else self.cfg.dw_min_width
            b = np.maximum(b, w) if w > 0 else b
        r = np.abs(b) * np.sqrt(2 * np.log(np.maximum(F0 / L, 1)))
        side = np.column_stack([r, -dz])                     # right half, top tip to bottom tip
        half = np.vstack([side, side[::-1] * [-1, 1]])       # then up the left half
        xz = np.column_stack([half[:, 0], np.zeros(len(half)), half[:, 1]])
        k = np.argmax(r)
        return np.vstack([loop(xz), ring(r[k], -dz[k])])

    def segments(self, P, V, rpm):
        """All isoline segments (S, 2, 3) and their colours (S, 3)."""
        segs, rgb = [], []
        if self.comp in ("dw", "all"):
            for s, c in zip(self.wake[::-1], self.wake_rgb[::-1]):       # strongest first: the viewer may truncate
                if len(s):
                    segs.append((s[None] + P[:, None, None]).reshape(-1, 2, 3))
                    rgb.append(np.tile(c, (len(P), 1)))
        if self.comp in ("gnd", "all"):
            (x0, y0), (x1, y1) = P[:, :2].min(0) - 0.3, P[:, :2].max(0) + 0.3
            for L in self.levels:
                h = self.R / 4 * np.sqrt(self.GE / L)          # force / W = GE (R / 4h)^2 at hover thrust
                if h > self.HCLIP:
                    segs.append(loop(np.array([[x0, y0, h], [x1, y0, h], [x1, y1, h], [x0, y1, h]])))
                    rgb.append(np.repeat(self.colour(L), 4, 0))
        if self.comp in ("drag", "all"):
            f = self.DRAG * (-2 * np.pi / 60 * rpm.sum(1))[:, None] * V     # N, per module
            m = np.linalg.norm(f, axis=1) / self.W
            on = m > self.levels[0]
            u = f[on] / np.linalg.norm(f[on], axis=1, keepdims=True)
            segs.append(np.stack([P[on], P[on] + 0.05 * (1 + np.log10(m[on] / self.levels[0]))[:, None] * u], 1))
            rgb.append(self.colour(m[on]).reshape(-1, 3))
        if not segs:
            return np.zeros((0, 2, 3)), np.zeros((0, 3))
        segs, rgb = np.vstack(segs), np.vstack(rgb)
        above = segs[:, :, 2].max(1) >= 0                      # wakes reach metres down: cut at the floor
        return segs[above], rgb[above]

    def legend(self, P):
        """Colour bar beside the modules: positions (n, 3) and (position, text, rgb) labels."""
        n = len(self.levels)
        pos = np.column_stack([np.full(n, P[:, 0].max() + 0.5), np.full(n, P[:, 1].min()),
                               max(P[:, 2].min(), 0.2) + 0.1 * np.arange(n)])
        labels = [(q + [0.05, 0, 0], f"{L:g} W" + (" (weakest)" if i == 0 else " (strongest)" if i == n - 1 else ""), c)
                  for i, (q, L, c) in enumerate(zip(pos, self.levels, self.colour(self.levels)))]
        labels.append((pos[-1] + [0, 0, 0.12], f"{COMPONENTS[self.comp]}: equal force / module weight W", np.ones(3)))
        return pos, labels

    def draw(self, rpm):
        st = [self.sim.state(i) for i in range(len(rpm))]
        P, V = np.array([s[0] for s in st]), np.array([s[1] for s in st])
        ok = np.isfinite(P).all(1) & np.isfinite(V).all(1)
        if ok.any():
            self.sim.draw_plume(*self.segments(P[ok], V[ok], rpm[ok]), self.legend(P[ok]))
