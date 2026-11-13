"""Cage model, layouts and downwash: pure functions, no simulator."""
from types import SimpleNamespace

import numpy as np
import pytest

from modular_uav.config import load
from modular_uav.dock import layouts
from modular_uav.models import cage
from modular_uav.physics import downwash


def cfg(*argv):
    return load(list(argv))


def test_tetra_cage_inertia_closed_form():
    """A regular tetrahedral rod frame (edge a, mass m) has I = 5 m a^2 / 36 on every axis."""
    c = cfg("--cage", "tetra", "--layout", "pyramid")
    np.testing.assert_allclose(cage.rods_inertia(cage.edges(c), c.cage_mass),
                               5 * c.cage_mass * c.tetra_edge ** 2 / 36 * np.eye(3), atol=1e-12)


@pytest.mark.parametrize("name, J", [("box", (2.348017e-05, 2.348017e-05, 3.734138e-05)),
                                     ("tetra", (5.404167e-05, 5.404167e-05, 6.174167e-05))])
def test_mass_properties_match_the_old_fork_urdfs(name, J):
    c = cfg("--cage", name, "--layout", "pyramid" if name == "tetra" else "square")
    m, got = cage.mass_properties(c)
    assert m == pytest.approx(0.030)
    np.testing.assert_allclose(got, J, rtol=1e-6)


def test_dodeca_cage():
    """Regular dodecahedron (Garanger 2026): 20 vertices, 30 rods, a vertex straight up (rotor axis),
    isotropic inertia, and every lattice neighbour shares exactly two vertices."""
    c = cfg("--cage", "dodeca", "--layout", "rows")
    v, rods = cage.vertices(c), cage.edges(c)
    assert (len(v), len(rods)) == (20, 30)
    np.testing.assert_allclose(v[np.argmax(v[:, 2])], [0, 0, np.linalg.norm(v[0])], atol=1e-12)
    J = cage.rods_inertia(rods, c.cage_mass)
    np.testing.assert_allclose(J, J[0, 0] * np.eye(3), atol=1e-12)
    for d in cage.lattice(c):
        assert np.isclose(np.linalg.norm(v[:, None] + d - v[None], axis=2), 0).sum() == 2


def test_clearance():
    assert cage.check_clearance(cfg()) is None
    for name, layout in (("tetra", "pyramid"), ("triangle", "triangle"), ("hex", "rows"), ("dodeca", "rows")):
        assert cage.check_clearance(cfg("--cage", name, "--layout", layout)) is None
    assert "bigger" in cage.check_clearance(cfg("--cage", "dodeca", "--layout", "rows", "--dodeca_edge", "0.05"))
    assert "bigger" in cage.check_clearance(cfg("--box_size", "0.1", "0.1", "0.05"))
    assert "bigger" in cage.check_clearance(cfg("--cage", "tetra", "--layout", "pyramid", "--tetra_edge", "0.25"))


@pytest.mark.parametrize("argv, n, pairs", [
    (("--layout", "rows", "--rows", "2"), 2, 1), ((), 9, 12), (("--layout", "rhombus"), 9, 16),
    (("--layout", "rows", "--rows", "3,2,2,3"), 10, 16),        # brick, face, brick
    (("--layout", "square", "--fill", "frame"), 8, 8),
    (("--layout", "square", "--fill", "frame", "--fractal", "2"), 64, 88),     # Sierpinski carpet
    (("--layout", "triangle", "--size", "2", "--fractal", "3"), 27, 39),        # brick Sierpinski triangle
    (("--cage", "tetra", "--layout", "pyramid", "--size", "2"), 4, 6),
    (("--cage", "tetra", "--layout", "pyramid", "--size", "3"), 10, 24),
    (("--cage", "tetra", "--layout", "pyramid", "--size", "4"), 20, 60),
    (("--cage", "tetra", "--layout", "pyramid", "--size", "5"), 35, 120),
    (("--cage", "tetra", "--layout", "pyramid", "--size", "2", "--fractal", "2"), 16, 30),
    (("--cage", "tetra", "--layout", "pyramid", "--size", "5", "--fill", "hollow"), 34, 108),   # one interior module dropped
    (("--cage", "tetra", "--layout", "pyramid", "--size", "4", "--fill", "frame"), 16, 30),
    (("--cage", "triangle", "--layout", "triangle", "--size", "3"), 6, 9),
    (("--cage", "triangle", "--triangle_connect", "face", "--layout", "triangle", "--size", "3"), 9, 9),
    (("--cage", "triangle", "--triangle_connect", "face", "--layout", "hexagon", "--size", "1"), 6, 6),
    (("--cage", "triangle", "--triangle_connect", "face", "--layout", "hexagon", "--size", "2"), 24, 30),
    (("--cage", "hex", "--layout", "rows", "--rows", "2"), 2, 1),
    (("--cage", "hex", "--layout", "hexagon", "--size", "2"), 7, 12),
    (("--cage", "hex", "--layout", "triangle", "--size", "2", "--fractal", "2"), 9, 12),
    (("--cage", "hex", "--layout", "hexagon", "--size", "2", "--fill", "frame", "--fractal", "2"), 36, 42),
    (("--cage", "dodeca", "--layout", "rhombus", "--size", "2"), 4, 5),
    (("--cage", "dodeca", "--layout", "hexagon", "--size", "2", "--fill", "frame"), 6, 6),
    (("--cage", "dodeca", "--layout", "pyramid", "--size", "3"), 10, 24),
    (("--cage", "dodeca", "--layout", "pyramid", "--size", "2", "--fractal", "3"), 64, 126)])
def test_layout_sizes(argv, n, pairs):
    p, pr, _, _ = layouts.build(cfg(*argv))
    assert (len(p), len(pr)) == (n, pairs)
    np.testing.assert_allclose(p.mean(0)[:2], 0, atol=1e-12)


def test_face_triangles_alternate_and_tilts_are_fully_actuated():
    p, _, Q, _ = layouts.build(cfg("--cage", "triangle", "--triangle_connect", "face", "--layout", "hexagon", "--size", "1"))
    assert np.isclose(Q[:, 0, 0], -1).tolist() == [False, True, False, True, False, True]   # up, down, up / down, up, down
    p, _, Q, _ = layouts.build(cfg("--cage", "dodeca", "--layout", "hexagon", "--size", "2", "--fill", "frame", "--tilt", "alternate"))
    np.testing.assert_allclose(np.degrees(np.arccos(Q[:, 2, 2])), 41.81, atol=0.01)
    a = Q[:, :, 2]                                                       # one thrust axis per module
    B6 = np.vstack([a.T, np.cross(p, a).T])
    assert np.linalg.matrix_rank(B6) == 6                                # 6-DOF
    np.testing.assert_allclose(a.sum(0)[:2], 0, atol=1e-12)             # no net side force at equal thrust


def test_disconnected_layout_exits():
    with pytest.raises(SystemExit):
        layouts.build(cfg("--cage", "triangle", "--triangle_connect", "face", "--layout", "triangle", "--fill", "frame"))


def test_grid_numbering_and_edge_mask():
    p, _, _, _ = layouts.build(cfg())
    np.testing.assert_allclose(p[:3, :2], [[-0.12, 0.12], [0, 0.12], [0.12, 0.12]])   # +y row first, -x to +x
    assert layouts.fail_mask("edge", p).astype(int).tolist() == [1, 0, 0, 1, 0, 0, 1, 0, 0]
    assert layouts.fail_mask("[0, 1, 0, 0, 0, 0, 0, 0, 0]", p).sum() == 1


ENV = SimpleNamespace(DW_COEFF_1=2267.18, DW_COEFF_2=0.16, DW_COEFF_3=-0.11, PROP_RADIUS=2.31348e-2, L=0.0397)


def test_downwash_original_is_the_upstream_formula():
    c = cfg("--downwash_model", "original")
    dz, dxy = 0.3, 0.05
    a = ENV.DW_COEFF_1 * (ENV.PROP_RADIUS / (4 * dz)) ** 2
    b = ENV.DW_COEFF_2 * dz + ENV.DW_COEFF_3
    assert downwash.force(dz, dxy, ENV, 0.3, c) == pytest.approx(a * np.exp(-0.5 * (dxy / b) ** 2))
    assert downwash.force(-0.1, 0.0, ENV, 0.3, c) == 0 and downwash.force(0.1, 11, ENV, 0.3, c) == 0


def test_downwash_regularised_bounds():
    c, T = cfg(), 0.294
    dz = np.array([1e-9, 1e-3, 0.02, 0.05, 0.2, 1.0])
    F = downwash.force(dz, 0.0, ENV, T, c)
    assert np.all(np.isfinite(F)) and F.max() <= T + 1e-12          # capped at the upper thrust
    assert F[0] < 1e-6                                              # ramps to 0 at the rotor plane
    beside = downwash.force(0.03, 0.12, ENV, T, c)                  # neighbour slightly higher, one slot away
    assert beside < 0.01 * T
    off = cfg("--dw_thrust_cap", "false", "--dw_dz_ramp", "0", "--dw_min_width", "0")
    assert downwash.force(0.02, 0.0, ENV, T, off) == pytest.approx(downwash.force(0.02, 0.0, ENV, T, cfg("--downwash_model", "original")))


@pytest.mark.parametrize("name", __import__("modular_uav.dock.tilings", fromlist=["NAMES"]).NAMES)
def test_tilings_have_their_vertex_configuration(name):
    """Every vertex near the origin is surrounded by exactly the polygons the name lists."""
    from modular_uav.dock import tilings
    want = sorted(int(x) for x in name.split("."))
    confs = tilings.vertex_configurations(name)
    assert confs and all(c == want for c in confs)
    assert all(sum(180 * (n - 2) / n for n in c) == pytest.approx(360) for c in confs)


def test_tiling_layout_and_froude_scaled_types():
    from modular_uav.models import module
    c = cfg("--cage", "tiling", "--layout", "rings", "--size", "2", "--drone_scale", "fit")
    lay = layouts.build(c)
    T = module.types(c, lay.kinds)
    assert sorted(T) == [3, 4, 6] and len(lay.p) == 19 and len(lay.pairs) == 24
    assert module.check(c, T) is None
    k = {n: t.scale for n, t in T.items()}
    assert k[3] < k[4] < k[6]                                    # bigger tiles carry bigger drones
    tri, hexa = T[3], T[6]
    drone = lambda t: t.mass - t.cage_mass
    assert drone(hexa) / drone(tri) == pytest.approx((hexa.scale / tri.scale) ** 3)   # Froude: mass x k^3


def test_undocked_contact_pairs():
    """Docked neighbours and corner-touching boxes never collide; boxes two slots apart do."""
    from modular_uav.dock import contact
    from modular_uav.models import module
    c = cfg()                                                   # 3 x 3 box grid
    lay = layouts.build(c)
    T = [module.types(c, [0])[0]] * len(lay.p)
    off = set(contact.touching(lay.p, lay.Q, T))
    assert set(lay.pairs) <= off                                # 12 face neighbours
    assert (0, 4) in off and (0, 2) not in off and len(off) == 12 + 8   # + 8 diagonal corner pairs


def test_capacity_allocation_shares_hover_by_capacity():
    """Two rotors of different strength under the CoM-balanced hover: each runs at the same fraction."""
    from modular_uav.dock.controller import Controller
    c = cfg()
    f_max = np.array([1.0, 1.0, 1.0, 1.0, 4.0, 4.0, 4.0, 4.0])
    xy = np.array([[-1.2, .1], [-1.2, -.1], [-1, -.1], [-1, .1], [.3, .1], [.3, -.1], [.1, -.1], [.1, .1]])
    xy[:, 0] -= (f_max @ xy[:, 0]) / f_max.sum()                 # CoM where capacity-weighted thrust balances
    ctrl = Controller(c, xy, np.tile([-1, 1, -1, 1], 2) * 0.01, 1.0, np.eye(3) * 0.01, 9.81, f_max, 2.5, 0.01)
    assert ctrl.general
    t = ctrl(np.zeros(3), np.zeros(3), np.eye(3), np.zeros(3), np.zeros(3), 0.0)
    np.testing.assert_allclose(t / f_max, (t / f_max)[0], rtol=1e-6)
    assert t.sum() == pytest.approx(9.81)
