"""Edge-to-edge tilings of regular polygons: the heterogeneous flat structures (cage: tiling).

A tiling is named by its vertex configuration, the polygons met going round every vertex
(Archimedean tilings: every vertex alike). Every tile is a module: an n-gon prism cage of edge
tile_edge, docked face to face with each neighbour it shares an edge with.

  tiling       polygons                 one vertex          name
  4.4.4.4      squares                  4 squares           square
  3.3.3.3.3.3  triangles                6 triangles         triangular
  6.6.6        hexagons                 3 hexagons          hexagonal
  3.6.3.6      triangles, hexagons      alternating         trihexagonal (kagome)
  3.4.6.4      triangles, squares, hex  tri sq hex sq       rhombitrihexagonal
  3.3.3.4.4    triangles, squares       3 tri, 2 sq         elongated triangular
  3.3.4.3.4    triangles, squares       tri tri sq tri sq   snub square
  4.8.8        squares, octagons        sq oct oct          truncated square
  4.6.12       squares, hex, 12-gons    sq hex 12-gon       truncated trihexagonal
  3.12.12      triangles, 12-gons       tri 12-gon 12-gon   truncated hexagonal

Each tiling is stored as a unit cell (two lattice vectors and its non-triangle tiles, edge 1).
The triangles are not stored: every triangle of these tilings sits on the edge of a bigger tile,
so they are the outward equilateral triangles on tile edges that land on free space. The region
is the centre tile (the biggest polygon nearest the origin) and `size` rings of docked neighbours.

Units: tile_edge, m. Frame: z up; a tile's yaw turns the module's canonical prism (one vertex on +y).
"""
import itertools
from collections import namedtuple

import numpy as np

Tile = namedtuple("Tile", "n centre angle")   # angle: direction of the first vertex, rad

S3 = np.sqrt(3)
_r = lambda n: 1 / (2 * np.sin(np.pi / n))            # circumradius of an n-gon of edge 1


def _ring(n, d, angles, extra=np.pi / 4):
    """n-gons at distance d along the given directions, turned to face back (squares: an edge faces the centre)."""
    return [Tile(n, d * np.array([np.cos(a), np.sin(a)]), a + extra) for a in angles]


CELLS = {   # name: (a1, a2, tiles other than triangles); edge 1
    "4.4.4.4": ((1, 0), (0, 1), [Tile(4, np.zeros(2), np.pi / 4)]),
    "3.3.3.3.3.3": ((1, 0), (0.5, S3 / 2), []),
    "6.6.6": ((S3, 0), (S3 / 2, 1.5), [Tile(6, np.zeros(2), np.pi / 6)]),
    "3.6.3.6": ((S3, 1), (0, 2), [Tile(6, np.zeros(2), np.pi / 6)]),
    "3.4.6.4": ((1 + S3, 0), ((1 + S3) / 2, (1 + S3) * S3 / 2),
                [Tile(6, np.zeros(2), np.pi / 6)] + _ring(4, (S3 + 1) / 2, [0, np.pi / 3, 2 * np.pi / 3])),
    "3.3.3.4.4": ((1, 0), (0.5, 1 + S3 / 2), [Tile(4, np.zeros(2), np.pi / 4)]),
    "3.3.4.3.4": ((np.sqrt(2 + S3), 0), (0, np.sqrt(2 + S3)),
                  [Tile(4, np.zeros(2), np.pi / 4 + np.pi / 12),
                   Tile(4, np.full(2, np.sqrt(2 + S3) / 2), np.pi / 4 - np.pi / 12)]),
    "4.8.8": ((1 + np.sqrt(2), 0), (0, 1 + np.sqrt(2)),
              [Tile(8, np.zeros(2), np.pi / 8), Tile(4, np.full(2, (1 + np.sqrt(2)) / 2), 0.0)]),
    "4.6.12": ((3 + S3, 0), ((3 + S3) / 2, (3 + S3) * S3 / 2),
               [Tile(12, np.zeros(2), np.pi / 12)] + _ring(4, (2 + S3) / 2 + 0.5, [0, np.pi / 3, 2 * np.pi / 3])
               + [Tile(6, np.array([(3 + S3) / 2, (3 + S3) * S3 / 6]) * f, 0.0) for f in (1, 2)]),
    "3.12.12": ((2 + S3, 0), ((2 + S3) / 2, (2 + S3) * S3 / 2), [Tile(12, np.zeros(2), np.pi / 12)]),
}
NAMES = tuple(CELLS)


def corners(t):
    a = t.angle + 2 * np.pi * np.arange(t.n) / t.n
    return t.centre + _r(t.n) * np.column_stack([np.cos(a), np.sin(a)])


def _inside(pt, t):
    """pt strictly inside tile t (convex polygon)."""
    c = corners(t)
    e = np.roll(c, -1, 0) - c
    return np.all(e[:, 0] * (pt[1] - c[:, 1]) - e[:, 1] * (pt[0] - c[:, 0]) > 1e-6)


def _patch(name, reach):
    """Tiles of the tiling within `reach` cells of the origin, triangles included, edge 1."""
    a1, a2, cell = CELLS[name]
    a1, a2 = np.array(a1, float), np.array(a2, float)
    tiles = [Tile(t.n, t.centre + i * a1 + j * a2, t.angle)
             for i, j in itertools.product(range(-reach, reach + 1), repeat=2) for t in cell]
    if name == "3.3.3.3.3.3":
        tiles = [Tile(3, c + i * a1 + j * a2, ang) for i, j in itertools.product(range(-reach, reach + 1), repeat=2)
                 for c, ang in ((np.array([0.5, S3 / 6]), np.pi / 2), (np.array([1.0, S3 / 3]), -np.pi / 2))]
        return tiles
    tris = {}
    for t in tiles:
        c = corners(t)
        for p, q in zip(c, np.roll(c, -1, 0)):
            mid, out = (p + q) / 2, (p + q) / 2 - t.centre
            out /= np.linalg.norm(out)
            centroid = mid + out * S3 / 6              # inradius of a unit triangle
            if any(_inside(centroid, u) for u in tiles):
                continue
            key = tuple(np.round(centroid, 5))
            tris.setdefault(key, Tile(3, centroid, np.arctan2(out[1], out[0])))   # apex points away from t
    return tiles + list(tris.values())


def build(name, size, edge):
    """Centre tile and `size` rings of edge-sharing neighbours: positions (N, 3), pairs, yaws (N,), sides (N,)."""
    tiles = _patch(name, size + 2)
    edge_of = {}
    for i, t in enumerate(tiles):
        c = np.round(corners(t), 4)
        for p, q in zip(c, np.roll(c, -1, 0)):
            edge_of.setdefault(frozenset((tuple(p), tuple(q))), []).append(i)
    adj = {i: set() for i in range(len(tiles))}
    for owners in edge_of.values():
        for i, j in itertools.combinations(owners, 2):
            adj[i].add(j)
            adj[j].add(i)
    big = max(t.n for t in tiles)
    start = min((i for i, t in enumerate(tiles) if t.n == big), key=lambda i: np.linalg.norm(tiles[i].centre))
    ring, seen = [start], {start}
    for _ in range(size):
        ring = [j for i in ring for j in adj[i] if j not in seen]
        seen.update(ring)
    idx = sorted(seen)
    pick = {i: n for n, i in enumerate(idx)}
    pairs = sorted({tuple(sorted((pick[i], pick[j]))) for i in idx for j in adj[i] if j in pick})
    p = np.array([[*tiles[i].centre, 0.0] for i in idx]) * edge
    # yaw that turns the canonical prism (first vertex on +y) onto the tile, folded into one sector
    yaw = np.array([(tiles[i].angle - np.pi / 2 + np.pi / tiles[i].n) % (2 * np.pi / tiles[i].n) - np.pi / tiles[i].n
                    for i in idx])
    return p, pairs, yaw, np.array([tiles[i].n for i in idx])


def vertex_configurations(name, reach=2):
    """Sorted polygon sides around each vertex near the origin (for the tests)."""
    tiles = _patch(name, reach + 2)
    around = {}
    for t in tiles:
        for v in np.round(corners(t), 4):
            around.setdefault(tuple(v), []).append(t.n)
    return [sorted(ns) for v, ns in around.items() if np.linalg.norm(v) < reach]
