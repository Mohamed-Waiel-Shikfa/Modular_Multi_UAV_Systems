"""Docked structures: module positions, orientations and the neighbour pairs that get welded.

All positions are in the assembly frame (origin at the assembly CoM after centring,
axes = the assembly axes, which are the body axes of every upright, unturned module), m.

A layout is a shape (cfg.layout, sized by cfg.size) on the lattice of the cage:

  lattice   cages                        neighbours                       slot pitch
  box       box                          full face in a row; full face    box_size x, y
                                         (same-parity rows, lined up) or
                                         half a face (offset rows)
  tri       hex, dodeca (flat),          one pitch apart, rows offset     hex sqrt(3) x edge, dodeca
            triangle (edge)              half a pitch                     sqrt(2) phi x edge, triangle edge
  face      triangle (face)              share a face; modules alternate  edge / sqrt(3)
                                         pointing +y and -y
  fcc       tetra, dodeca (3-D)          one neighbour vector apart       tetra edge, dodeca as tri

Flat shapes are rows of modules, top (+y) row first, each row centred (see the `rows` field for how
rows sit against each other on each lattice):

  shape      rows for size n (box, tri)          face-docked triangles
  rows       the `rows` field                    the `rows` field (triangles per row)
  square     n, n, ..., n (n rows)               2n, ..., 2n (n rows)
  triangle   1, 2, ..., n                        1, 3, ..., 2n - 1
  rhombus    1, 2, ..., n, ..., 2, 1             1, 3, ..., 2n - 1, 2n - 1, ..., 3, 1
  hexagon    n, n + 1, ..., 2n - 1, ..., n       2n + 1, ..., 4n - 1, 4n - 1, ..., 2n + 1

3-D shapes (fcc): pyramid = tetrahedral stack of n layers, n(n+1)(n+2)/6 modules (4, 10, 20, ...).

Then, on any shape: fill (solid, hollow = the hull's boundary, frame = the hull's edges) picks which
modules of the shape are kept, and fractal L repeats the result L levels deep: a copy at every module,
scaled up just enough not to overlap. Sierpinski triangle = triangle 2, fractal L (3^L modules);
Sierpinski tetrahedron = pyramid 2, fractal L (4^L); Sierpinski carpet = square 3, frame, fractal L (8^L).

Garanger et al. 2026, Fig. 13: quadrotor = dodeca rhombus 2; hexarotor = hexagon 2 + frame;
6-DOF hexarotor = that + tilt: alternate; tetrahedron quad/decarotor = pyramid 2/3;
tetrahedron hexadecarotor = pyramid 2, fractal 2 (= pyramid 4, frame).

Modules are numbered layer by layer from the bottom, then row by row from the +y row,
left to right (-x to +x). layout_map() draws the numbering.
"""
import itertools
import sys
from collections import namedtuple

import numpy as np
from scipy.spatial import ConvexHull, QhullError

from modular_uav.dock import tilings
from modular_uav.models import cage

# p (N, 3) positions, pairs [(i, j)] docked neighbours, Q (N, 3, 3) orientation body -> assembly,
# kinds (N,) module type of each module (0 for a single-cage layout; polygon sides for a tiling)
Layout = namedtuple("Layout", "p pairs Q kinds")

FLAT_ROWS = {   # shape -> rows for size n: (box and tri lattices, face-docked triangles)
    "square": (lambda n: [n] * n, lambda n: [2 * n] * n),
    "triangle": (lambda n: list(range(1, n + 1)), lambda n: list(range(1, 2 * n, 2))),
    "rhombus": (lambda n: list(range(1, n)) + list(range(n, 0, -1)),
                lambda n: list(range(1, 2 * n, 2)) + list(range(2 * n - 1, 0, -2))),
    "hexagon": (lambda n: list(range(n, 2 * n)) + list(range(2 * n - 2, n - 1, -1)),
                lambda n: list(range(2 * n + 1, 4 * n, 2)) + list(range(4 * n - 1, 2 * n, -2))),
}


def lattice_kind(cfg):
    if cfg.cage == "box":
        return "box"
    if cfg.cage == "triangle":
        return "face" if cfg.triangle_connect == "face" else "tri"
    if cfg.cage == "tetra" or (cfg.cage == "dodeca" and cfg.layout in ("pyramid", "sierpinski")):
        return "fcc"
    return "tri"


def flat(counts, kind, cfg):
    """Centred rows -> positions (N, 3), yaw turned (N,) bool, neighbour distance (None: box rule),
    row band (N,) (the row's y, for numbering)."""
    base = (counts[0] - 1) / 2 % 1                       # 0 or 0.5: where the first row's cells sit
    cells = []
    for k, n in enumerate(counts):
        x = np.arange(n) - (n - 1) / 2
        # tri: each row half a slot off the one above; face: every row on the first row's grid.
        # A centred row off its grid moves half a slot to +x.
        grid = (base + k / 2) % 1 if kind == "tri" else base
        if kind != "box" and x[0] % 1 != grid:
            x = x + 0.5
        cells += [(xa, k, a) for a, xa in enumerate(x)]
    x, k = np.array([c[0] for c in cells]), np.array([c[1] for c in cells], float)
    turned = np.zeros(len(x), bool)
    if kind == "box":
        w, h = cfg.box_size[:2]
        return np.column_stack([x * w, -k * h, 0 * x]), turned, None, -k
    if kind == "tri":
        d = cage.flat_pitch(cfg)
        return np.column_stack([x * d, -k * d * np.sqrt(3) / 2, 0 * x]), turned, d, -k
    s = cfg.triangle_edge
    h = s * np.sqrt(3) / 2
    # face: the first triangle of the top row points up (+y), then up and down alternate along a row
    # and between rows (an up triangle's base sits on the top edge of the down one below it)
    turned = (np.round(x + (counts[0] - 1) / 2) + k) % 2 == 1
    y = -k * h + np.where(turned, h / 6, -h / 6)         # centroid sits h/3 above a base, h/3 below a top edge
    return np.column_stack([x * s / 2, y, 0 * x]), turned, s / np.sqrt(3), -k


def space(n, e):
    """fcc cells i e0 + j e1 + k e2 with i + j + k < n (a tetrahedral stack of n layers)."""
    return np.array([i * e[0] + j * e[1] + k * e[2] for k in range(n) for i in range(n) for j in range(n)
                     if i + j + k < n], float)


def fill(p, how):
    """Mask of the modules kept by `fill`: solid (all), hollow (on the convex hull's boundary: a flat
    shape's outline, a 3-D shape's surface) or frame (on the hull's edges: the outline, or the 3-D edge
    skeleton, i.e. on 2+ faces)."""
    if how == "solid":
        return np.ones(len(p), bool)
    dims = [a for a in range(3) if np.ptp(p[:, a]) > 1e-9]
    try:
        hull = ConvexHull(p[:, dims])
    except (QhullError, ValueError):
        return np.ones(len(p), bool)                      # collinear: every module is on the outline
    planes = np.unique(hull.equations.round(6), axis=0)
    on = np.abs(p[:, dims] @ planes[:, :-1].T + planes[:, -1]) < 1e-6
    return on.sum(1) >= (2 if how == "frame" and len(dims) == 3 else 1)


def fractal(p, turned, levels, overlap):
    """Self-similar copies: level L+1 puts a copy of level L at every module of the base shape, scaled
    by the smallest integer s >= 2 at which no copies overlap (Sierpinski triangle: triangle 2, s = 2;
    carpet: square 3 + frame, s = 3). Works on positions, so any lattice closed under integer scaling."""
    base = p - p[0]                                       # a lattice point at the origin
    for s in range(2, 64):
        q = (s * base[:, None] + base[None]).reshape(-1, 3)
        if not overlap(q):
            break
    out = base
    for _ in range(levels - 1):
        out = (s * out[:, None] + base[None]).reshape(-1, 3)
    return out, np.tile(turned, len(out) // len(p))

def tilts(p):
    """tilt: alternate. Rotations (N, 3, 3): modules sorted by azimuth about the vertical axis
    through the CoM take turns tilting their rotor axis +tangential and -tangential, each picking the
    41.8 deg cage symmetry with the most tangential tilt of that sign. Modules on the axis stay upright."""
    Q = np.tile(np.eye(3), (len(p), 1, 1))
    r = p[:, :2] - p[:, :2].mean(0)
    off = np.flatnonzero(np.linalg.norm(r, axis=1) > 1e-6)
    options = cage.dodeca_tilts()
    for n, i in enumerate(off[np.argsort(np.arctan2(r[off, 1], r[off, 0]))]):
        t = np.array([-r[i, 1], r[i, 0]]) / np.linalg.norm(r[i]) * (1 if n % 2 == 0 else -1)
        Q[i] = max(options, key=lambda o: o[1] @ t)[0]
    return Q


def build(cfg):
    """Layout(positions, neighbour pairs, orientations, module kinds) for cfg."""
    if cfg.cage == "tiling":
        return _tiling(cfg)
    kind, n = lattice_kind(cfg), cfg.size
    turned = band = None
    if kind == "fcc":
        e = cage.lattice(cfg)
        p = space(n, e)
        d = np.linalg.norm(e[0])
    else:
        counts = cfg.rows if cfg.layout == "rows" else FLAT_ROWS[cfg.layout][kind == "face"](n)
        p, turned, d, band = flat(counts, kind, cfg)
    if turned is None:
        turned = np.zeros(len(p), bool)
    m = fill(p, cfg.fill)
    p, turned, band = p[m], turned[m], None if band is None else band[m]
    if cfg.fractal > 1:
        p, turned = fractal(p, turned, cfg.fractal, lambda q: _overlaps(q, d, cfg))
        band = None
    if _overlaps(p, d, cfg):
        sys.exit(f"config error: layout {cfg.layout} on {cfg.cage} cages puts two modules on top of each other")
    p = p - p.mean(0)
    row = p[:, 1].round(4) if band is None else band
    order = np.lexsort((p[:, 0].round(4), -row, p[:, 2].round(4)))   # layer, +y row, x
    p, turned = p[order], turned[order]
    pairs = _pairs(p, d, cfg)
    Q = np.tile(np.eye(3), (len(p), 1, 1))
    Q[turned] = np.diag([-1.0, -1.0, 1.0])                # 180 deg about z
    if cfg.tilt == "alternate":
        Q = tilts(p)
    _check(cfg, p, pairs)
    return Layout(p, pairs, Q, np.zeros(len(p), int))


def _tiling(cfg):
    """cage: tiling. Tiles of cfg.tiling: the centre tile and `size` rings of docked neighbours."""
    p, pairs, yaw, sides = tilings.build(cfg.tiling, cfg.size, cfg.tile_edge)
    m = fill(p, cfg.fill)
    keep = np.flatnonzero(m)
    new = {i: n for n, i in enumerate(keep)}
    pairs = [(new[i], new[j]) for i, j in pairs if i in new and j in new]
    p, yaw, sides = p[keep], yaw[keep], sides[keep]
    p = p - p.mean(0)
    order = np.lexsort((p[:, 0].round(4), -p[:, 1].round(4)))
    rank = np.argsort(order)
    p, yaw, sides = p[order], yaw[order], sides[order]
    pairs = sorted(tuple(sorted((rank[i], rank[j]))) for i, j in pairs)
    c, s = np.cos(yaw), np.sin(yaw)
    Q = np.zeros((len(p), 3, 3))
    Q[:, 0, 0], Q[:, 0, 1], Q[:, 1, 0], Q[:, 1, 1], Q[:, 2, 2] = c, -s, s, c, 1
    _check(cfg, p, pairs)
    return Layout(p, pairs, Q, sides)


def _pairs(p, d, cfg):
    if d is not None:
        return [(i, j) for i, j in itertools.combinations(range(len(p)), 2)
                if np.isclose(np.linalg.norm(p[j] - p[i]), d)]
    u = p[:, :2] / cfg.box_size[:2]                       # box: slot units
    pairs = []
    for i, j in itertools.combinations(range(len(p)), 2):
        dx, dy = np.abs(u[j] - u[i]).round(6)
        if (dy == 0 and dx == 1) or (dy == 1 and dx < 1):    # full face in a row, full or half face across rows
            pairs.append((i, j))
    return pairs


def _overlaps(p, d, cfg):
    """Two modules closer than their cages allow (box: overlapping boxes; lattices: under one pitch)."""
    i, j = np.triu_indices(len(p), 1)
    delta = np.abs(p[j] - p[i])
    if d is None:
        w, h = cfg.box_size[:2]
        return bool(np.any((delta[:, 0] < w - 1e-9) & (delta[:, 1] < h - 1e-9) & (delta[:, 2] < 1e-9)))
    return bool(np.any(np.linalg.norm(delta, axis=1) < 0.99 * d))


def _check(cfg, p, pairs):
    """At least 2 modules, every module reachable through docked neighbours."""
    if len(p) < 2:
        sys.exit(f"config error: layout {cfg.layout} (size {cfg.size}) has {len(p)} module; it needs 2 or more")
    seen, todo = {0}, [0]
    while todo:
        i = todo.pop()
        for a, b in pairs:
            for x, y in ((a, b), (b, a)):
                if x == i and y not in seen:
                    seen.add(y)
                    todo.append(y)
    if len(seen) < len(p):
        sys.exit(f"config error: layout {cfg.layout} on {cfg.cage} cages is not connected: modules "
                 f"{sorted(set(range(len(p))) - seen)} touch no docked neighbour of module 0 "
                 f"(check rows, frame or triangle_connect)")


def layout_map(p, Q=None):
    """Module indices drawn at their xy positions, one block per layer (z), top row first."""
    p = p.copy()
    turned = np.zeros(len(p), bool) if Q is None else np.isclose(Q[:, 0, 0], -1) & np.isclose(Q[:, 2, 2], 1)
    if turned.any() and (~turned).any() and np.ptp(p[:, 2]) == 0:
        # face-docked triangles: up and down centroids of one row differ by h/3; draw them on one line
        off = np.abs(p[turned, None, 1] - p[None, ~turned, 1]).min()
        p[:, 1] += np.where(turned, -off / 2, off / 2)
    # column pitch: the smallest x gap between two modules of one row (3 characters on screen)
    rows = [np.sort(p[np.isclose(p[:, 1].round(4), y) & np.isclose(p[:, 2].round(4), z), 0])
            for z in np.unique(p[:, 2].round(4)) for y in np.unique(p[:, 1].round(4))]
    gaps = [np.diff(r).min() for r in rows if len(r) > 1]
    qx = min(gaps) / 2 if gaps else 1
    layers = np.unique(p[:, 2].round(4))
    lines = []
    for n, z in enumerate(layers):
        idx = np.flatnonzero(p[:, 2].round(4) == z)
        if len(layers) > 1:
            lines.append(f"  layer {n} (z {z:+.3f} m from the CoM)")
        for y in sorted(set(p[idx, 1].round(4)), reverse=True):
            row = [" "] * (int(3 * np.ptp(p[:, 0]) / qx) + 3)
            for i in idx[p[idx, 1].round(4) == y]:
                c = int(round(3 * (p[i, 0] - p[:, 0].min()) / qx))
                row[c:c + 2] = f"{i:>2}"
            lines.append("    " + "".join(row).rstrip())
    return "\n".join(lines)


def fail_mask(text, p):
    """'1,0,0', '[1, 0, 0]', '100' or 'edge' (the -x edge of the bottom layer) -> bool array, one per module."""
    n = len(p)
    if text == "edge":
        bottom = np.isclose(p[:, 2], p[:, 2].min())
        return bottom & np.isclose(p[:, 0], p[bottom, 0].min())
    digits = [c for c in str(text) if c not in "[](), "]
    if len(digits) != n or set(digits) - {"0", "1"}:
        sys.exit(f"config error: fail_mask needs {n} values of 0 or 1 (one per module, map order) or edge, "
                 f"got {text!r}")
    return np.array(digits) == "1"
