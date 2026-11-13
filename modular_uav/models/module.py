"""Module types: one cage around one CF2X, scaled by Froude similarity. Level: L0 rigid frame.

A layout can mix module types (tilings mix triangles, squares, hexagons, ...; several scenarios
in one simulation mix cages). Each type is a cage (rods in the body frame) and a CF2X scaled by k:

    lengths (arm L, propeller radius, mesh, rotor positions)   x k
    mass x k^3, inertia x k^5, kf x k^4, km x k^5, drag coefficient x k^3,
    downwash DW_COEFF_1 x k^3, DW_COEFF_3 x k, ground-effect clip height x k,
    MAX_RPM = sqrt(T/W m g / (4 kf)) with the module's own mass (x k^-1/2 for the bare drone)

so thrust-to-weight, the ground-effect and downwash shapes and the docking manoeuvre are
dynamically similar at any size (time scales as sqrt(k)). k = 1 is the stock CF2X, and every
coefficient is then the simulator's own. drone_scale: fit picks, for each type, the largest k
that still clears its cage by cage_clearance.

Body frame: x forward, z up, origin at the CoM. Units: m, kg, s.
"""
from dataclasses import dataclass

import numpy as np

from modular_uav.models import cage

# exponent of k for each per-drone simulator attribute (both backends use these names)
FROUDE = dict(KF=4, KM=5, L=1, PROP_RADIUS=1, DRAG_COEFF=3, GND_EFF_H_CLIP=1, DW_COEFF_1=3, DW_COEFF_3=1)
# default cage colours, one per module type (RGBA); cage_color overrides them all
PALETTE = [(0.96, 0.62, 0.16, 1), (0.16, 0.66, 0.72, 1), (0.86, 0.33, 0.52, 1), (0.52, 0.74, 0.22, 1),
           (0.36, 0.50, 0.92, 1), (0.70, 0.70, 0.74, 1)]


@dataclass
class ModuleType:
    name: str            # e.g. "box", "hex", "6-gon"
    vertices: np.ndarray  # cage vertices, body frame, m
    rods: list           # cage rods [(a, b)], body frame, m
    cage_mass: float     # kg
    scale: float         # k, drone length scale (1 = CF2X)
    color: tuple         # cage RGBA
    rod_radius: float    # m

    @property
    def mass(self):
        return cage.CF2X["mass"] * self.scale ** 3 + (self.cage_mass if self.rods else 0)

    @property
    def J(self):
        """Inertia about the CoM, kg m^2 (3, 3)."""
        drone = np.diag([cage.CF2X[k] for k in ("ixx", "iyy", "izz")]) * self.scale ** 5
        J = drone + cage.rods_inertia(self.rods, self.cage_mass)
        assert np.allclose(J, np.diag(np.diag(J)), atol=1e-12)   # symmetric cage, no products of inertia
        return J

    @property
    def bottom(self):
        """m, lowest point below the CoM (the CoM height when resting on the ground)."""
        if not self.rods:
            return -cage.CF2_MESH_BOX[0, 2] * self.scale
        return -self.vertices[:, 2].min()

    @property
    def radius(self):
        """m, horizontal radius of the cage (or the drone mesh) about the CoM."""
        pts = self.vertices if self.rods else cage.mesh_vertices() * self.scale
        return float(np.linalg.norm(pts[:, :2], axis=1).max())


def margin(vertices, k):
    """m, clearance of a CF2X scaled by k inside the convex cage (negative: it sticks out)."""
    return cage.hull_margin(vertices, cage.mesh_vertices() * k)


def fit_scale(vertices_of_k, clearance):
    """Largest k (to 0.01) whose CF2X clears the cage vertices_of_k(k) by clearance."""
    lo, hi = 0.1, 20.0
    for _ in range(40):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if margin(vertices_of_k(mid), mid) >= clearance else (lo, mid)
    return np.floor(lo * 100) / 100


def drone_scale(cfg, vertices_of_k):
    if cfg.drone_scale == "fit":
        return fit_scale(vertices_of_k, cfg.cage_clearance)
    return float(cfg.drone_scale)


def types(cfg, kinds):
    """One ModuleType per kind in kinds (layout.kinds): cfg's own cage (kind 0), or the n-gon prism
    of a tiling (kind n)."""
    out = {}
    for n, kind in enumerate(sorted(set(kinds))):
        color = tuple(cfg.cage_color) if cfg.cage_color else PALETTE[n % len(PALETTE)]
        if cfg.cage == "tiling":
            # prism grows with its drone (height prism_height x k), so a big drone still fits inside
            verts = lambda k, n=kind: cage.prism_vertices(cage.polygon(n, cfg.tile_edge / (2 * np.sin(np.pi / n)),
                                                                       np.pi / 2), cfg.prism_height * max(k, 1))
            k = drone_scale(cfg, verts)
            v = verts(k)
            rods = cage.prism_edges(v)
            mass = cfg.rod_density * sum(np.linalg.norm(b - a) for a, b in rods)
            out[kind] = ModuleType(f"{kind}-gon", v, rods, mass, k, color, cfg.cage_rod_radius)
        else:
            v = cage.vertices(cfg) if cfg.cage != "none" else np.zeros((0, 3))
            k = drone_scale(cfg, lambda k: v) if cfg.cage != "none" else float(cfg.drone_scale.replace("fit", "1"))
            out[kind] = ModuleType(cfg.cage, v, cage.edges(cfg), cfg.cage_mass, k, color, cfg.cage_rod_radius)
    return out


def check(cfg, mtypes):
    """Error message if a module type's drone sticks out of its cage by less than cage_clearance."""
    for t in mtypes.values():
        if not t.rods:
            continue
        m = margin(t.vertices, t.scale)
        if m < cfg.cage_clearance - 1e-9:
            return (f"{t.name} cage clears its CF2X (scale {t.scale:g}) by {1000 * m:.1f} mm, less than "
                    f"cage_clearance = {1000 * cfg.cage_clearance:.1f} mm: make the cage bigger or the drone smaller")
    return None


def describe(mtypes, counts=None):
    """One line per type: name, count, scale, mass, max thrust ratio to weight is constant (2.25)."""
    lines = []
    for kind, t in mtypes.items():
        c = f"{counts[kind]} x " if counts is not None else ""
        lines.append(f"  {c}{t.name}: drone scale {t.scale:g}, module mass {1000 * t.mass:.0f} g, "
                     f"cage radius {100 * t.radius:.1f} cm")
    return "\n".join(lines)
