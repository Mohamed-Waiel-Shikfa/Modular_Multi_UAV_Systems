"""Which module pairs may collide (module_collisions).

  off       no module touches another (the old default: cages pass through each other)
  all       every pair collides, docked neighbours too (contact then fights the welds)
  undocked  every pair collides except those whose cages touch in the docked structure: docked
            neighbours, and modules that meet at a single rod or vertex (diagonal boxes, vertex-
            sharing tetrahedra, face-docked triangles round a vertex). So modules can no longer
            fly through each other on the way in, and the docked structure is contact-free.

Touching = two rods of the two cages, placed in the docked layout, closer than both rod radii
plus 2 mm. Exact segment-to-segment distances, after a bounding-sphere filter.
"""
import itertools

import numpy as np

SLACK = 0.002   # m, on top of the two rod radii


def _segment_distance(a0, a1, b0, b1):
    """Distances between segments a (n, 3) and b (n, 3) pairwise along the first axis."""
    u, v, w = a1 - a0, b1 - b0, a0 - b0
    a, b, c = (u * u).sum(1), (u * v).sum(1), (v * v).sum(1)
    d, e = (u * w).sum(1), (v * w).sum(1)
    den = a * c - b * b
    s = np.where(den > 1e-12, np.clip((b * e - c * d) / np.where(den > 1e-12, den, 1), 0, 1), 0)
    t = np.clip((b * s + e) / np.where(c > 1e-12, c, 1), 0, 1)
    s = np.clip((b * t - d) / np.where(a > 1e-12, a, 1), 0, 1)
    return np.linalg.norm(w + s[:, None] * u - t[:, None] * v, axis=1)


def touching(p, Q, mtypes):
    """Pairs (i, j) whose cages touch when docked at positions p (N, 3), orientations Q (N, 3, 3)."""
    rods = [np.array([[q @ a + pi, q @ b + pi] for a, b in t.rods]) for pi, q, t in zip(p, Q, mtypes)]
    reach = [np.linalg.norm(t.vertices, axis=1).max() if t.rods else 0 for t in mtypes]
    out = []
    for i, j in itertools.combinations(range(len(p)), 2):
        if not len(rods[i]) or not len(rods[j]):
            continue
        gap = mtypes[i].rod_radius + mtypes[j].rod_radius + SLACK
        if np.linalg.norm(p[j] - p[i]) > reach[i] + reach[j] + gap:
            continue
        A, B = np.repeat(rods[i], len(rods[j]), 0), np.tile(rods[j], (len(rods[i]), 1, 1))
        if _segment_distance(A[:, 0], A[:, 1], B[:, 0], B[:, 1]).min() < gap:
            out.append((i, j))
    return out


def pairs(cfg, p, Q, mtypes):
    """Module pairs with contact switched off, for cfg.module_collisions."""
    if cfg.module_collisions == "all":
        return []
    if cfg.module_collisions == "off":
        return list(itertools.combinations(range(len(p)), 2))
    return touching(p, Q, mtypes)
