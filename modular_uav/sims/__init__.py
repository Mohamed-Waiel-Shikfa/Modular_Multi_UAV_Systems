"""Simulator backends. Both expose the same Sim interface, so tasks never import a simulator:

  Sim(cfg, starts, welds, freq, rpys, mtypes, kinds, no_contact)
    starts: (N, 3) start positions, m. welds: (i, j, offset, quat) per docked pair,
    offset = module j's position in module i's frame, m; quat = j's orientation in i's frame,
    x, y, z, w. freq: Hz, physics = control. rpys: (N, 3) start attitudes, rad (None: all 0).
    mtypes: {kind: ModuleType} (models/module.py); kinds: (N,) the type of each module.
    no_contact: (i, j) module pairs that never collide.
  M, J, G, KF, KM, MAX_RPM   module 0 as upstream parsed it (kg, kg m^2, m/s^2, N/(rad/s)^2, ...)
  ROTORS                     (4, 2) rotor x, y of a scale-1 module, body frame, m, backend rotor order
  drone(i)                   module i: M, J, KF, KM, MAX_RPM, L, PROP_RADIUS, ROTORS (scaled), G
  env                        the underlying upstream env (coefficients for the plume)
  state(i) -> pos (world), vel (world), R (body to world), w (body rates, body frame)
  quat_xyzw(i), step(rpm (N, 4)), snap(which=None), draw_plume(segs, rgb, legend), close()
  set_attitude(i, R)         turn module i in place to R (body to world), body rates zeroed

The simulators themselves are the upstream repos, untouched (submodules).
"""


def make(cfg, starts, welds=(), freq=None, yaws=None, mtypes=None, kinds=None, no_contact=()):
    """mtypes/kinds default to cfg's own single module type."""
    freq = freq or cfg.freq
    rpys = None if yaws is None else [[0, 0, y] for y in yaws]
    if mtypes is None:
        from modular_uav.models import module
        mtypes, kinds = module.types(cfg, [0]), [0] * len(starts)
    if cfg.sim == "pybullet":
        from modular_uav.sims.pybullet_sim import Sim
    else:
        from modular_uav.sims.mujoco_sim import Sim
    return Sim(cfg, starts, list(welds), freq, rpys, mtypes, list(kinds), list(no_contact))
