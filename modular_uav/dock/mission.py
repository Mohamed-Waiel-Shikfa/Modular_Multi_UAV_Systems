"""task: dock. Caged modules take off, dock into cfg.layout in the air, fly the mission, land.

Level: L0 docking ("magic glue": every docked neighbour pair is welded at its exact slot
offset once all pairs are within snap_gap of it). Aero effects follow cfg.physics and
cfg.downwash_model. World frame: z up. Units: m, s, rad.

Phases: APPROACH (each module flies its own geometric controller: from the ground to a
staging point, the layout scaled up by start_spacing / neighbour distance, then contract to
the slots) -> SNAP -> HOVER, then optionally
  failure: CLIMB to fail_z -> HOLD -> FAILURE (masked modules at `outage` x their commanded
           thrust for outage_t; nobody is told) -> RECOVER (hover where it ends up, heading
           free until stable) -> FINAL hover
  circle:  CIRCLE (laps of a circle of `radius` starting at the hover point, speed ramped up
           and down so it starts and ends at rest)
-> LAND (land_speed, land_speed_low below land_slow_below, motors off on touchdown).
After the snap one controller flies all rotors of the assembly (not the L1 "PID per module").

A Mission is one scenario. Several missions can share one simulation (`include`, dock/world.py),
each at its own offset, with its own phases, controllers and checks.

Module orientation Q_i (body -> assembly, from the layout): a module turned about z (face-docked
triangles, tiles) flies its whole approach at that heading. A tilted module (tilt: alternate) cannot
hover tilted on its own, so it approaches upright and is turned to Q_i at the snap, in place ("magic
glue", L0: the real system would assemble on the ground or with an actuated connector).

Mixed modules (tilings, drone_scale): the hover point is the mass-weighted CoM, each module flies
the approach with its own mass, inertia and rotors, and the assembly controller shares thrust by
`allocation`.
"""
import itertools
import sys

import numpy as np
from scipy.optimize import linear_sum_assignment
from scipy.spatial.transform import Rotation

from modular_uav.dock import checks, contact, layouts
from modular_uav.dock.controller import YAW_SIGN, Controller
from modular_uav.models import module


def tilt_deg(R):
    return np.degrees(np.arccos(np.clip(R[2, 2], -1, 1)))


def yaw_of(R):
    return np.arctan2(R[1, 0], R[0, 0])


def start_height(cfg, bottom=None):
    """z0, or the module resting on its cage bottom, rounded up to the cm above a 5 mm gap."""
    if cfg.z0 is not None:
        return cfg.z0
    if bottom is None:
        bottom = next(iter(module.types(cfg, [0]).values())).bottom
    return np.ceil(round((bottom + 0.005) * 100, 6)) / 100     # box, triangle, hex 0.03; tetra 0.07; dodeca 0.09


def planned_duration(c):
    """s, rough mission length (log preallocation only)."""
    t = c.rise_t + c.close_t + c.hover_t + 2 + max(c.meet_z, c.fail_z) / c.land_speed_low
    if c.failure:
        t += abs(c.fail_z - c.meet_z) / c.climb_rate + c.hold_t + c.outage_t + 5 + c.final_t
    return t + (c.laps * c.lap_t if c.circle else 0)


class Mission:
    def __init__(self, cfg, name=""):
        c = self.cfg = cfg
        self.name, self.tag = name, f"[{name}] " if name else ""
        lay = layouts.build(c)
        self.mtypes = module.types(c, lay.kinds)
        msg = module.check(c, self.mtypes)
        if msg:
            sys.exit(f"config error: {self.tag}{msg}")
        self.kinds, self.Q, self.pairs = lay.kinds, lay.Q, lay.pairs
        T = [self.mtypes[k] for k in lay.kinds]
        self.m = np.array([t.mass for t in T])
        p = lay.p - (self.m @ lay.p) / self.m.sum()          # about the mass-weighted CoM
        self.p, self.N = p, len(p)
        self.offs = [p[j] - p[i] for i, j in self.pairs]      # slot offset of j from i, assembly frame
        self.mask = layouts.fail_mask(c.fail_mask, p) if c.failure else None
        Q = self.Q
        self.upright = np.isclose(Q[:, 2, 2], 1)
        # Approach orientation: Q itself if upright (a heading), else upright at heading 0, turned at the snap
        self.A = np.where(self.upright[:, None, None], Q, np.eye(3))
        self.yaws = np.arctan2(self.A[:, 1, 0], self.A[:, 0, 0])

        # Staging: the layout scaled by s0 around (0, 0, cz0), lowest cage >= stage_clearance up;
        # big modules spread further, so no two neighbours' cages overlap on the way in.
        R = np.array([t.radius for t in T])
        R3 = np.array([np.linalg.norm(t.vertices, axis=1).max() if t.rods else t.radius for t in T])
        b = np.array([t.bottom for t in T])
        s0 = c.stage_spacing / min(np.linalg.norm(o) for o in self.offs) if self.pairs else 1.0
        # bounding spheres of neighbours stage_gap apart at the staging points (inactive for one cage type)
        s0 = max([s0] + [(R3[i] + R3[j] + c.stage_gap) / np.linalg.norm(o) for (i, j), o in zip(self.pairs, self.offs)])
        self.s0 = s0
        self.cz0 = max(c.meet_z, c.stage_clearance + np.max(b - s0 * p[:, 2]))
        stage = s0 * p[:, :2]
        if np.ptp(p[:, 2]) == 0:
            ground = stage                                    # flat layout: rise straight up
        else:
            # Stacked layout: start on a square grid, each module on the cell nearest its staging point.
            n = int(np.ceil(np.sqrt(self.N)))
            pitch = max(c.start_spacing, 2 * R.max() + 0.05)
            cells = pitch * (np.array(list(itertools.product(range(n), repeat=2)), float) - (n - 1) / 2)
            rows, cols = linear_sum_assignment(np.linalg.norm(stage[:, None] - cells[None], axis=2))
            ground = cells[cols[np.argsort(rows)]]
        self.ground = ground
        self.z0 = np.array([start_height(c, bi) for bi in b])
        self.rest_z = np.max(b - p[:, 2])                    # CoM height with the lowest cages on the ground
        # welds in module i's frame: j's position and orientation relative to i
        self.welds = [(i, j, Q[i].T @ o, Rotation.from_matrix(Q[i].T @ Q[j]).as_quat())
                      for (i, j), o in zip(self.pairs, self.offs)]
        self.no_contact = contact.pairs(c, p, Q, [t for t in T])
        self.radius = R
        self.offset = np.zeros(2)

    def footprint(self):
        """xy box (min, max) the mission uses about its own origin: ground, staging, circle, plus cage radii."""
        r = self.radius.max()
        pts = [self.ground, self.s0 * self.p[:, :2], self.p[:, :2]]
        if self.cfg.circle:
            ext = np.abs(self.p[:, :2]).max()
            pts.append(np.array([[-2 * self.cfg.radius - ext, -self.cfg.radius - ext], [ext, self.cfg.radius + ext]]))
        pts = np.vstack(pts)
        return pts.min(0) - r, pts.max(0) + r

    def describe(self):
        c = self.cfg
        what = f"{c.tiling} tiling" if c.cage == "tiling" else f"{c.cage} cages"
        lines = [f"{self.tag}{c.layout} of {what} on {c.sim}: {self.N} modules, {len(self.pairs)} docked neighbour "
                 f"pairs (index map, +y up, +x right)\n{layouts.layout_map(self.p, self.Q)}"]
        if len(self.mtypes) > 1 or next(iter(self.mtypes.values())).scale != 1:
            counts = {k: int(np.sum(self.kinds == k)) for k in self.mtypes}
            lines.append(f"{self.tag}module types:\n" + module.describe(self.mtypes, counts))
        turned = ~np.isclose(self.yaws, 0) & (self.cfg.cage != "tiling")
        if turned.any():
            lines.append(f"{self.tag}turned 180 deg about z: {np.flatnonzero(turned).tolist()}")
        if not self.upright.all():
            lines.append(f"{self.tag}rotor axis tilted {np.degrees(np.arccos(self.Q[~self.upright, 2, 2].min())):.1f} "
                         f"deg, turned at the snap: {np.flatnonzero(~self.upright).tolist()}")
        if self.mask is not None:
            lines.append(f"{self.tag}failing modules: {np.flatnonzero(self.mask).tolist()} at {c.outage:.0%} "
                         f"thrust for {c.outage_t} s")
        if self.offset.any():
            lines.append(f"{self.tag}placed at x {self.offset[0]:+.2f} m, y {self.offset[1]:+.2f} m")
        return "\n".join(lines)

    def starts(self):
        return np.column_stack([self.ground + self.offset, self.z0])

    def attach(self, sim, first, weld_first):
        """Bind to modules first .. first + N - 1 and welds weld_first .. of a shared simulation."""
        c, self.sim, self.first = self.cfg, sim, first
        self.ids = list(range(first, first + self.N))
        self.weld_ids = list(range(weld_first, weld_first + len(self.welds)))
        dt = 1 / c.freq
        D = [sim.drone(i) for i in self.ids]
        self.D = D
        self.n_rot = len(sim.ROTORS)
        self.kf = np.array([d.KF for d in D])
        # One per module, each with its own integral. Stiffer than the assembly loop: a module docking
        # under another feels more downwash the closer it gets (about 0.73 W/m in a depth-4 pyramid),
        # which beats the wn = 2.5 spring (0.64 W/m) and leaves only the slow integral to close the gap.
        self.solo = [Controller(c, d.ROTORS, YAW_SIGN * d.KM / d.KF, d.M, d.J, d.G, d.KF * d.MAX_RPM ** 2,
                                c.module_wn, dt, att_scale=t.scale)
                     for d, t in zip(D, (self.mtypes[k] for k in self.kinds))]
        r, Q = self.p, self.Q
        J_asm = sum(q @ d.J @ q.T + d.M * (ri @ ri * np.eye(3) - np.outer(ri, ri)) for q, d, ri in zip(Q, D, r))
        rotors = np.vstack([ri + np.column_stack([d.ROTORS, np.zeros(self.n_rot)]) @ q.T for ri, q, d in zip(r, Q, D)])
        axes = np.repeat(Q[:, :, 2], self.n_rot, axis=0)
        yaw_coef = np.concatenate([YAW_SIGN * d.KM / d.KF for d in D])
        f_max = np.repeat([d.KF * d.MAX_RPM ** 2 for d in D], self.n_rot)
        k_eff = float(self.m @ np.array([self.mtypes[k].scale for k in self.kinds]) / self.m.sum())
        self.ctrl = Controller(c, rotors if not self.upright.all() else rotors[:, :2], yaw_coef, self.m.sum(), J_asm,
                               D[0].G, f_max, c.pos_wn, dt, axes=None if self.upright.all() else axes,
                               att_scale=k_eff)
        self.seq = (["HOVER"] + (["CLIMB", "HOLD", "FAILURE", "RECOVER", "FINAL"] if self.mask is not None else [])
                    + (["CIRCLE"] if c.circle else []) + ["LAND", "DONE"])
        self.phase, self.t_phase = "APPROACH", 0.0
        self.p_d = np.array([*self.offset, c.meet_z])
        self.v_d, self.a_d, self.yaw_d = np.zeros(3), np.zeros(3), 0.0
        self.log, self.fail_log, self.circ = {}, dict(max_tilt=0.0, z_start=0.0, z_min=np.inf), dict(err=0.0)
        self.every = int(round(c.print_every * c.freq))
        self.done = False
        self.trace = []                                      # (t, phase, pos, p_d, rpy, P) at log_rate

    def approach(self, t):
        """Approach targets (N, 3) at time t: ground -> staging over rise_t, then contract over close_t."""
        c, p = self.cfg, self.p
        f = min(t / c.rise_t, 1)
        g = np.clip((t - c.rise_t) / c.close_t, 0, 1)
        s, cz = self.s0 - (self.s0 - 1) * g, self.cz0 + (c.meet_z - self.cz0) * g
        xy = (1 - f) * self.ground + f * s * p[:, :2] + self.offset
        return np.column_stack([xy, (1 - f) * self.z0 + f * (cz + s * p[:, 2])])

    def com(self, states):
        P = np.array([s[0] for s in states])
        V = np.array([s[1] for s in states])
        return P, (self.m @ P) / self.m.sum(), (self.m @ V) / self.m.sum()

    def step(self, k):
        """Rotor RPM (N, 4) and per-module targets (N, 3) for physics step k, or None once done."""
        if self.done:
            return None
        c, sim, freq = self.cfg, self.sim, self.cfg.freq
        t = k / freq
        tp = t - self.t_phase
        N, offs, pairs = self.N, self.offs, self.pairs
        if self.phase == "APPROACH":
            P = np.array([sim.state(i)[0] for i in self.ids])
            gap = max((np.linalg.norm(P[j] - P[i] - o) for (i, j), o in zip(pairs, offs)), default=0.0)
            if gap < c.snap_gap and t > c.rise_t:
                R_a = sim.state(self.ids[0])[2] @ self.A[0].T   # assembly attitude as the modules fly it
                for i in np.flatnonzero(~self.upright):
                    sim.set_attitude(self.ids[i], R_a @ self.Q[i])
                sim.snap(self.weld_ids)
                if c.carry_integral and self.ctrl.KI > 0:      # carry the learnt downwash over
                    self.ctrl.i_err = (self.m @ np.array([s.KI * s.i_err for s in self.solo])) / self.m.sum() / self.ctrl.KI
                self.log["snap"] = dict(t=t, gap_cm=100 * gap)
                print(f"{self.tag}t={t:5.2f}s SNAP: all {len(pairs)} neighbour offsets within {100 * gap:.1f} cm "
                      f"at CoM z={(self.m @ P[:, 2]) / self.m.sum():.3f} m")
                self.phase, self.t_phase, tp = "HOVER", t, 0.0
            elif t > c.rise_t + c.close_t + c.snap_timeout:
                errs = [P[j] - P[i] - o for (i, j), o in zip(pairs, offs)]
                w = int(np.argmax([np.linalg.norm(e) for e in errs]))
                print(f"{self.tag}t={t:5.2f}s NO SNAP: worst neighbour offset error still {100 * gap:.1f} cm, "
                      f"modules {pairs[w]}, error {np.round(100 * errs[w], 1)} cm")
                self.done = True
                return None
            else:
                tgt, tgt_v = self.approach(t), (self.approach(t + 1 / freq) - self.approach(t)) * freq
                states = [sim.state(i) for i in self.ids]
                thrust = np.array([self.solo[n](*states[n], tgt[n], self.yaws[n], tgt_v[n]) for n in range(N)])
                self._record(k, t, states, tgt.mean(0))
                return np.sqrt(thrust / self.kf[:, None]), tgt

        states = [sim.state(i) for i in self.ids]
        P, pos, vel = self.com(states)
        _, _, R0, w0 = states[0]
        R, w = R0 @ self.Q[0].T, self.Q[0] @ w0              # assembly attitude and body rates
        p_d = self.p_d
        err = np.linalg.norm(pos - p_d)
        phase = self.phase
        nxt = self.seq[self.seq.index(phase) + 1] if phase != "DONE" else None
        log, fl, mask = self.log, self.fail_log, self.mask
        tag = self.tag

        if phase == "HOVER" and tp >= c.hover_t:
            slot_err = max(np.linalg.norm(R.T @ (P[i] - P[0]) - (self.p[i] - self.p[0])) for i in range(N))
            log["hover"] = dict(err_cm=100 * err, slot_cm=100 * slot_err)
            print(f"{tag}t={t:5.2f}s HOVER done: error {100 * err:.1f} cm, worst slot error {100 * slot_err:.2f} cm")
            self.phase, self.t_phase = nxt, t
        elif phase == "CLIMB":
            dz = c.fail_z - c.meet_z
            self.p_d = p_d = np.array([*self.offset, c.meet_z + np.sign(dz) * min(c.climb_rate * tp, abs(dz))])
            if p_d[2] == c.fail_z and err < 0.03 and np.linalg.norm(vel) < 0.05:
                print(f"{tag}t={t:5.2f}s HOLD at z={pos[2]:.3f} m")
                self.phase, self.t_phase = nxt, t
        elif phase == "HOLD" and tp >= c.hold_t:
            print(f"{tag}t={t:5.2f}s FAILURE: modules {np.flatnonzero(mask).tolist()} at {c.outage:.0%} thrust "
                  f"for {c.outage_t} s, z={pos[2]:.3f} m")
            fl["z_start"] = pos[2]
            self.phase, self.t_phase = nxt, t
        elif phase == "FAILURE":
            fl["max_tilt"] = max(fl["max_tilt"], tilt_deg(R))
            fl["z_min"] = min(fl["z_min"], pos[2])
            if tp >= c.outage_t:
                self.p_d = p_d = np.array([pos[0], pos[1], max(pos[2], c.meet_z)])
                log["failure"] = dict(max_tilt=fl["max_tilt"], drop=fl["z_start"] - fl["z_min"])
                print(f"{tag}t={t:5.2f}s RECOVER: tilt {tilt_deg(R):.0f} deg (max {fl['max_tilt']:.0f}), "
                      f"speed {np.linalg.norm(vel):.2f} m/s, pos {np.round(pos, 2)}, hold z={p_d[2]:.2f} m")
                self.phase, self.t_phase = nxt, t
        elif phase == "RECOVER":
            fl["z_min"] = min(fl["z_min"], pos[2])
            if (err < c.recover_pos_tol and np.linalg.norm(vel) < c.recover_vel_tol
                    and tilt_deg(R) < c.recover_tilt_deg and np.linalg.norm(w) < c.recover_rate_tol):
                log["recover"] = dict(time=tp, z_min=fl["z_min"])
                print(f"{tag}t={t:5.2f}s STABLE after {tp:.2f} s of recovery (lowest CoM z {fl['z_min']:.2f} m)")
                self.phase, self.t_phase, self.yaw_d = nxt, t, yaw_of(R)
        elif phase == "FINAL" and tp >= c.final_t:
            log["final"] = dict(err_cm=100 * err)
            print(f"{tag}t={t:5.2f}s FINAL hover done: error {100 * err:.1f} cm")
            self.phase, self.t_phase = nxt, t
        elif phase == "CIRCLE":
            circ = self.circ
            if "c" not in circ:
                circ["c"] = p_d - [c.radius, 0, 0]            # start on the circle at angle 0
                print(f"{tag}t={t:5.2f}s CIRCLE: {c.laps:g} laps, radius {c.radius} m, about {c.laps * c.lap_t:.0f} s")
            T = c.laps * c.lap_t
            tau = min(tp / T, 1)
            # Speed follows 1 - cos: starts and ends at rest, peak = 2 x average.
            th = 2 * np.pi * c.laps * (tau - np.sin(2 * np.pi * tau) / (2 * np.pi))
            thd = 2 * np.pi * c.laps / T * (1 - np.cos(2 * np.pi * tau)) if tau < 1 else 0.0
            thdd = (2 * np.pi) ** 2 * c.laps / T ** 2 * np.sin(2 * np.pi * tau) if tau < 1 else 0.0
            e_t, e_n = np.array([-np.sin(th), np.cos(th), 0]), np.array([np.cos(th), np.sin(th), 0])
            self.p_d = p_d = circ["c"] + c.radius * e_n
            self.v_d = c.radius * thd * e_t
            self.a_d = c.radius * (thdd * e_t - thd ** 2 * e_n)
            circ["err"] = max(circ["err"], np.linalg.norm(pos - p_d))
            if tau >= 1:
                log["circle"] = dict(err_cm=100 * circ["err"])
                print(f"{tag}t={t:5.2f}s CIRCLE done: worst tracking error {100 * circ['err']:.1f} cm")
                self.phase, self.t_phase = nxt, t
        elif phase == "LAND":
            speed = c.land_speed if p_d[2] > c.land_slow_below else c.land_speed_low
            p_d[2] = max(p_d[2] - speed / freq, 0.0)
            if pos[2] < self.rest_z + 0.015 and abs(vel[2]) < 0.1:
                print(f"{tag}t={t:5.2f}s LANDED at {np.round(pos, 3)}; motors off")
                self.phase, self.t_phase = nxt, t
        elif phase == "DONE" and tp >= 1:
            log["land"] = dict(z=pos[2], tilt=tilt_deg(R), speed=np.linalg.norm(vel))
            self.done = True
            return None

        if self.phase == "DONE":
            thrust = np.zeros(N * self.n_rot)
        else:
            yaw = yaw_of(R) if self.phase == "RECOVER" else self.yaw_d
            thrust = self.ctrl(pos, vel, R, w, self.p_d, yaw, self.v_d, self.a_d)
            if self.phase == "FAILURE":
                thrust *= np.repeat(np.where(mask, c.outage, 1.0), self.n_rot)
        rpm = np.sqrt(thrust.reshape(N, self.n_rot) / self.kf[:, None])
        if self.every and k % self.every == 0:
            print(f"{tag}t={t:5.2f}s {self.phase:8s} pos={np.round(pos, 3)} tilt={tilt_deg(R):5.1f} deg")
        self._record(k, t, states, self.p_d, pos, R)
        return rpm, self.p_d + self.p @ R.T

    def _record(self, k, t, states, target, pos=None, R=None):
        """Assembly trace at log_rate, for the live plot and the sim: both diff."""
        if k % max(1, int(round(self.cfg.freq / self.cfg.log_rate))):
            return
        P, com, _ = self.com(states)
        if R is None:
            R = states[0][2] @ self.A[0].T
        self.trace.append((t, self.phase, com, np.array(target, float), Rotation.from_matrix(R).as_euler("xyz"), P))

    def report(self):
        if self.tag:
            print(f"\n{self.tag.strip()}", end="")
        return checks.report(checks.evaluate(self.cfg, self.log, self.N, self.mask, self.rest_z))
