"""Caged CF2X module: a stock CF2X with thin rods along the edges of a box, a regular
tetrahedron, a triangular or hexagonal prism, or a regular dodecahedron, rigidly fixed
to the frame. Level: L0 rigid frame.

Only mass and inertia change; every other coefficient (kf, km, drag, downwash,
...) and the stock PID gains are the CF2X's. The cage is centred on the CoM, so
the CoM does not move. Inertia = CF2X (Forster 2015) + the rods as uniform thin
rods, mass split by length, I = sum of integral(|r|^2 E - r r^T) dm about the CoM.
Joints/connectors at the vertices are not modelled. Every cage vertex and edge
midpoint is a future connection point.

Body frame: x forward, z up, origin at the CoM. Units: m, kg.
Defaults (fields.py), each the smallest such size that clears the CF2 mesh by >= 5 mm:
  box       120 x 120 x 50 mm
  tetra     310 mm edges, apex up, horizontal base below, one base vertex on +x
  triangle  230 mm edges x 50 mm prism, one vertex on +y (an edge along x, so rows run along x)
  hex       70 mm edges x 50 mm prism, flat faces on +-x (neighbours along x)
  dodeca    60 mm edges, two opposite vertices on z (Garanger et al., T-RO 2026,
            "The Dodecacopter": the rotor axis runs through two opposite vertices, and those
            vertices belong to the cube inscribed in the dodecahedron); one neighbour on +x
3 g cage, 1 mm rods.
"""
import functools
import itertools
from importlib.resources import files

import numpy as np
from scipy.spatial import ConvexHull

# CF2 mesh bounding box, body frame, m (x -0.055..0.051, y -0.051..0.047, z -0.013..0.019)
CF2_MESH_BOX = np.array([[-0.055, -0.051, -0.013], [0.051, 0.047, 0.019]])
CF2X = dict(mass=0.027, ixx=1.4e-5, iyy=1.4e-5, izz=2.17e-5)   # kg, kg m^2 (Forster 2015; both sims)


def tetra_vertices(a):
    """Regular tetrahedron of edge a, centroid at the origin: 3 base vertices (the first on +x), then the apex."""
    h, r = a * np.sqrt(2 / 3), a / np.sqrt(3)       # height; base circumradius. Centroid h/4 above the base
    return np.array([[r * np.cos(t), r * np.sin(t), -h / 4] for t in 2 * np.pi / 3 * np.arange(3)]
                    + [[0, 0, 3 * h / 4]])


def prism_vertices(ring, h):
    """Vertical prism of height h over the horizontal polygon `ring` (n, 2): bottom ring, then top ring."""
    return np.vstack([np.column_stack([ring, np.full(len(ring), z)]) for z in (-h / 2, h / 2)])


def polygon(n, r, start):
    """Regular n-gon, circumradius r, first vertex at angle `start` (rad), counter-clockwise."""
    t = start + 2 * np.pi / n * np.arange(n)
    return r * np.column_stack([np.cos(t), np.sin(t)])


PHI = (1 + np.sqrt(5)) / 2
# Dodecahedron frame: cube diagonal (1, 1, 1) -> z (rotor axis through two opposite vertices),
# lattice direction (1, -1, 0) -> x. Rows are the new x, y, z axes in the textbook coordinates.
DODECA_R = np.array([[1, -1, 0] / np.sqrt(2), [1, 1, -2] / np.sqrt(6), [1, 1, 1] / np.sqrt(3)])


def dodeca_vertices(a):
    """Regular dodecahedron of edge a, centre at the origin, in the module frame (see DODECA_R).
    Textbook coordinates: the cube (+-1, +-1, +-1) plus the cyclic permutations of (0, +-1/phi, +-phi), edge 2/phi."""
    cube = list(itertools.product((-1, 1), repeat=3))
    rest = [np.roll([0, s / PHI, t * PHI], k) for s, t in itertools.product((-1, 1), repeat=2) for k in range(3)]
    return np.array(cube + rest) @ DODECA_R.T * (a * PHI / 2)


def lattice(cfg):
    """3-D neighbour vectors of a tetra or dodeca cage, module frame, m: 3 pairwise at 60 deg, the first
    two horizontal, the third pointing up. Integer combinations of them are the slots (an fcc lattice)."""
    if cfg.cage == "tetra":
        v = tetra_vertices(cfg.tetra_edge)
        return v[1:] - v[0]                                  # one cage edge apart: neighbours share a vertex
    # dodeca (Garanger 2026, Fact II.4): a neighbour sits at g + h for a cube edge {g, h}, so the two
    # share vertices g and h and their faces through g, h are coplanar.
    return np.array([[2, 0, -2], [0, 2, -2], [2, 2, 0]]) @ DODECA_R.T * (cfg.dodeca_edge * PHI / 2)


def flat_pitch(cfg):
    """m, neighbour distance along x of a flat layout on a triangular lattice (hex, dodeca, triangle edge-to-edge)."""
    if cfg.cage == "hex":
        return np.sqrt(3) * cfg.hex_edge                     # flat to flat: neighbours share a face
    if cfg.cage == "triangle":
        return cfg.triangle_edge
    return np.linalg.norm(lattice(cfg)[0])                   # dodeca: the horizontal fcc vectors run along 0, 60, 120 deg


@functools.lru_cache(1)
def _dodeca_symmetries():
    """The 60 rotations of the regular dodecahedron (its symmetry group S^D, Garanger 2026 Def. II.1),
    in the module frame: each maps the top vertex and one of its neighbours onto an adjacent pair."""
    v = dodeca_vertices(1.0)
    top = np.argmax(v[:, 2])
    adj = lambda i: [j for j in range(20) if np.isclose(np.linalg.norm(v[j] - v[i]), 1.0)]
    frame = lambda a, b: np.column_stack([v[a], v[b], np.cross(v[a], v[b])])
    F0 = np.linalg.inv(frame(top, adj(top)[0]))
    return [frame(a, b) @ F0 for a in range(20) for b in adj(a)]


def dodeca_tilts():
    """Rotations of S^D that tilt the rotor axis (z) 41.8 deg onto one of the 3 vertices next to the top
    one, the smallest such rotation for each: list of 3 (R, horizontal unit tilt direction)."""
    out = {}
    for R in _dodeca_symmetries():
        a = R[:, 2]
        if np.isclose(a[2], np.sqrt(5) / 3):                 # cos 41.81 deg
            k = tuple(a.round(6))
            if k not in out or np.trace(R) > np.trace(out[k]):
                out[k] = R
    return [(R, R[:2, 2] / np.linalg.norm(R[:2, 2])) for R in out.values()]


def vertices(cfg):
    if cfg.cage == "box":
        half = np.array(cfg.box_size) / 2
        return np.array([half * s for s in itertools.product((-1, 1), repeat=3)])
    if cfg.cage == "triangle":
        return prism_vertices(polygon(3, cfg.triangle_edge / np.sqrt(3), np.pi / 2), cfg.prism_height)
    if cfg.cage == "hex":
        return prism_vertices(polygon(6, cfg.hex_edge, np.pi / 6), cfg.prism_height)
    if cfg.cage == "dodeca":
        return dodeca_vertices(cfg.dodeca_edge)
    return tetra_vertices(cfg.tetra_edge)


def edges(cfg):
    """Cage rods as (start, end) points, m. []: no cage."""
    if cfg.cage == "none":
        return []
    v = vertices(cfg)
    pairs = itertools.combinations(v, 2)
    if cfg.cage == "box":
        return [(a, b) for a, b in pairs if np.count_nonzero(a != b) == 1]
    if cfg.cage in ("triangle", "hex"):
        return prism_edges(v)
    if cfg.cage == "dodeca":
        return [(a, b) for a, b in pairs if np.isclose(np.linalg.norm(b - a), cfg.dodeca_edge)]
    return list(pairs)


def prism_edges(v):
    """Rods of a prism from prism_vertices: the two rings and the verticals."""
    n = len(v) // 2
    return ([(v[r + k], v[r + (k + 1) % n]) for r in (0, n) for k in range(n)]
            + [(v[k], v[n + k]) for k in range(n)])


def hull_margin(vertices, points):
    """m, smallest distance from the points to the faces of the convex hull of vertices (negative: outside)."""
    hull = ConvexHull(vertices)
    # hull.equations: n . x + d <= 0 inside, |n| = 1, so -(n . x + d) is the distance to that face
    return -(points @ hull.equations[:, :3].T + hull.equations[:, 3]).max()


def bottom(cfg):
    """m, lowest cage point below the CoM (the CoM height when resting on the ground)."""
    if cfg.cage == "none":
        return -CF2_MESH_BOX[0, 2]
    return -vertices(cfg)[:, 2].min()


def rods_inertia(rods, mass):
    """Inertia tensor about the origin of uniform thin rods, mass split by length, kg m^2."""
    if not rods:
        return np.zeros((3, 3))
    lengths = np.array([np.linalg.norm(b - a) for a, b in rods])
    S = np.zeros((3, 3))    # second moment, integral of x x^T dm
    for (a, b), m in zip(rods, mass * lengths / lengths.sum()):
        u = b - a
        S += m * (np.outer(a, a) + (np.outer(a, u) + np.outer(u, a)) / 2 + np.outer(u, u) / 3)
    return np.trace(S) * np.eye(3) - S


def mass_properties(cfg):
    """Caged module: mass (kg) and principal inertia (ixx, iyy, izz), kg m^2."""
    if cfg.cage == "none":
        return CF2X["mass"], (CF2X["ixx"], CF2X["iyy"], CF2X["izz"])
    J = rods_inertia(edges(cfg), cfg.cage_mass)
    assert np.allclose(J, np.diag(np.diag(J)), atol=1e-12)   # symmetric cage, no products of inertia
    return CF2X["mass"] + cfg.cage_mass, tuple(np.array([CF2X["ixx"], CF2X["iyy"], CF2X["izz"]]) + np.diag(J))


@functools.lru_cache(1)
def mesh_vertices():
    """CF2 visual mesh vertices, body frame, m (the cf2_*.obj files shipped with MuJoCo-drones-gym)."""
    folder = files("multi_drone_mujoco") / "assets" / "cf2"
    return np.vstack([[[float(x) for x in line.split()[1:4]] for line in (folder / f"cf2_{i}.obj").read_text().splitlines()
                       if line.startswith("v ")] for i in range(7)])


def check_clearance(cfg):
    """Error message if the cage does not clear every CF2 mesh vertex by cfg.cage_clearance, else None.
    Defaults: box clears by 5.3 mm, tetra by 7.5 mm, triangle by 6.5 mm, hex by 5.9 mm, dodeca by 11.6 mm."""
    if cfg.cage == "none":
        return None
    margin = hull_margin(vertices(cfg), mesh_vertices())
    if margin < cfg.cage_clearance:
        return (f"{cfg.cage} cage clears the CF2 mesh by {1000 * margin:.1f} mm, "
                f"less than cage_clearance = {1000 * cfg.cage_clearance:.1f} mm: make the cage bigger")
    return None
