"""Backends on the upstream simulators: the caged model loads with the right mass, and hovers on the stock PID."""
import pytest

from modular_uav import sims
from modular_uav.config import load
from modular_uav.tasks import run


@pytest.mark.parametrize("sim", ["pybullet", "mujoco"])
@pytest.mark.parametrize("cage", ["box", "tetra", "triangle", "hex", "dodeca"])
def test_caged_module_mass(sim, cage):
    c = load(["--sim", sim, "--cage", cage, "--task", "hover"])
    s = sims.make(c, [[0, 0, 0.1]])
    try:
        assert s.M == pytest.approx(0.030)
        assert s.J[0, 0] > 2e-5                     # bare CF2X is 1.4e-5
    finally:
        s.close()


@pytest.mark.parametrize("sim", ["pybullet", "mujoco"])
@pytest.mark.parametrize("cage", ["box", "tetra", "triangle", "hex", "dodeca", "none"])
def test_hover_task_passes(sim, cage):
    assert run(load(["--sim", sim, "--cage", cage, "--task", "hover", "--print_every", "0"])) == 0


@pytest.mark.parametrize("sim", ["pybullet", "mujoco"])
@pytest.mark.parametrize("contact", [True, False])
def test_modules_collide_unless_excluded(sim, contact):
    """A module dropped (motors off) onto another lands on its cage; with the pair excluded it falls through."""
    import numpy as np
    c = load(["--sim", sim, "--cage", "box", "--task", "hover"])
    s = sims.make(c, [[0, 0, 0.03], [0, 0, 0.3]], no_contact=[] if contact else [(0, 1)])
    try:
        for _ in range(240):
            s.step(np.zeros((2, 4)))
        z = s.state(1)[0][2]
        assert (z > 0.05) if contact else (z < 0.04)     # resting on the other cage: 0.06-0.08 m; fallen through: 0.03
    finally:
        s.close()
