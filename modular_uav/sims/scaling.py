"""Per-drone coefficients for mixed module types, shared by both backends.

Both upstream envs hold one set of drone coefficients (self.KF, self.KM, self.L, ...) and read
them inside per-drone methods (_physics(rpm, i), _groundEffect, _drag). PerDrone swaps drone i's
Froude-scaled values in while those upstream methods run (models/module.py gives the exponents),
and clips each drone's RPM to its own MAX_RPM. With one module type at scale 1 every value is the
upstream one, so the physics is unchanged.
"""
from contextlib import contextmanager
from types import SimpleNamespace

import numpy as np

from modular_uav.models.module import FROUDE
from modular_uav.physics import downwash


class PerDrone:
    """Mixin, listed before the upstream aviary class. Call _setup_drones after the parent constructor."""

    def _setup_drones(self, mtypes, kinds):
        """mtypes: {kind: ModuleType}; kinds: (N,) the kind of each drone."""
        base = {a: getattr(self, a) for a in FROUDE}
        self._coef, self._k = [], [mtypes[kind].scale for kind in kinds]
        for kind in kinds:
            t = mtypes[kind]
            c = {a: base[a] * t.scale ** e for a, e in FROUDE.items()}
            c["MAX_RPM"] = np.sqrt(self.THRUST2WEIGHT_RATIO * t.mass * self.G / (4 * c["KF"]))
            self._coef.append(c)

    @contextmanager
    def _drone(self, i):
        saved = {a: getattr(self, a) for a in self._coef[i]}
        for a, v in self._coef[i].items():
            setattr(self, a, v)
        try:
            yield
        finally:
            for a, v in saved.items():
                setattr(self, a, v)

    def drone(self, i, mtype, rotors):
        """Drone i for the controllers: M, J, KF, KM, MAX_RPM, L, PROP_RADIUS, ROTORS (scaled), G."""
        c = self._coef[i]
        return SimpleNamespace(M=mtype.mass, J=mtype.J, KF=c["KF"], KM=c["KM"], MAX_RPM=c["MAX_RPM"], L=c["L"],
                               PROP_RADIUS=c["PROP_RADIUS"], ROTORS=rotors * mtype.scale, G=self.G)

    def _physics(self, rpm, nth_drone):
        with self._drone(nth_drone):
            return super()._physics(rpm, nth_drone)

    def _groundEffect(self, rpm, nth_drone):
        with self._drone(nth_drone):
            return super()._groundEffect(rpm, nth_drone)

    def _drag(self, rpm, nth_drone):
        with self._drone(nth_drone):
            return super()._drag(rpm, nth_drone)

    def _preprocessAction(self, action):
        a = np.reshape(action, (self.NUM_DRONES, 4))
        return np.vstack([np.clip(a[i], 0, c["MAX_RPM"]) for i, c in enumerate(self._coef)])

    def _dw_force(self, nth_drone):
        """N, regularised downwash on drone nth_drone from every drone above it (each with its own coefficients)."""
        F = 0.0
        for i in range(self.NUM_DRONES):
            if i == nth_drone:
                continue
            dz = self.pos[i, 2] - self.pos[nth_drone, 2]
            dxy = np.linalg.norm(self.pos[i, :2] - self.pos[nth_drone, :2])
            with self._drone(i):
                thrust = self.KF * np.sum(self.last_clipped_action[i, :] ** 2)
                F += float(downwash.force(dz, dxy, self, thrust, self.cfg, k_upper=self._k[i], k_lower=self._k[nth_drone]))
        return F
