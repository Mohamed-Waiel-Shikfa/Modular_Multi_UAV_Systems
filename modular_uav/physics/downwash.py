"""Downwash between modules: the simulators' fitted model and our regularised one.

Both simulators ship the same model (Zhou, DSL) for a module dz below and dxy beside
an upper module (dz > 0, dxy < 10 m), as a downward force in N:

    original     F = a exp(-0.5 (dxy / b)^2),  a = DW_COEFF_1 (PROP_RADIUS / (4 dz))^2,  b = DW_COEFF_2 dz + DW_COEFF_3

a ~ 1/dz^2 is singular at dz -> 0, so roundoff between modules at equal height gave
forces of 1e12 N and up (docs/dock.md, "Findings"). The regularised model adds three
bounds, each switchable from the config (fields dw_*):

    regularised  F = min(a, T) exp(-0.5 (dxy / b')^2) min(dz / ramp, 1),  b' = max(b, w)

  - dw_thrust_cap: the peak a is capped at the upper module's thrust T = KF sum(rpm^2): the
    force is that module's jet hitting the lower one, and the jet's momentum flux equals T;
  - dw_dz_ramp: a module within `ramp` of another's rotor plane sits beside its wake, not
    in it, so the force fades to 0 at dz = 0;
  - dw_min_width: b is floored at w (default the arm length L): a jet is at least as wide as
    the rotors that make it. The fitted b is negative below dz = 0.69 m on CF2X.

Mixed module sizes (Froude scale k_upper above, k_lower below; models/module.py): the ramp is a
length, so it grows with the upper drone (ramp x k_upper; w already does, through L), and a lower
module smaller than the upper one intercepts only part of its jet: F x min(1, (k_lower / k_upper)^2),
the ratio of their rotor disk areas. Both factors are 1 when every module has the same size.

The simulators run `original` through their own untouched _downwash; this module is
used for `regularised` and by the plume drawing. Works on scalars or numpy arrays.
"""
import numpy as np


def force(dz, dxy, env, thrust, cfg, model=None, k_upper=1.0, k_lower=1.0):
    """Downward force (N, >= 0) on a module dz (m) below and dxy (m) beside an upper module of thrust T (N).

    env: the simulator env (DW_COEFF_1..3, PROP_RADIUS, L of the upper module). model: overrides
    cfg.downwash_model. k_upper, k_lower: Froude scales of the two modules (regularised only).
    """
    dz, dxy = np.asarray(dz, float), np.asarray(dxy, float)
    model = model or cfg.downwash_model
    on = (dz > 0) & (dxy < 10)
    dzs = np.where(on, dz, 1.0)                                     # keep the formula finite where off
    a = env.DW_COEFF_1 * (env.PROP_RADIUS / (4 * dzs)) ** 2
    b = env.DW_COEFF_2 * dzs + env.DW_COEFF_3
    if model == "regularised":
        w = env.L if cfg.dw_min_width is None else cfg.dw_min_width
        if w > 0:
            b = np.maximum(b, w)
        if cfg.dw_thrust_cap:
            a = np.minimum(a, thrust)
        if cfg.dw_dz_ramp > 0:
            a = a * np.minimum(dzs / (cfg.dw_dz_ramp * k_upper), 1)
        a = a * min(1.0, (k_lower / k_upper) ** 2)
    F = a * np.exp(-0.5 * (dxy / b) ** 2)
    return np.where(on, F, 0.0)
