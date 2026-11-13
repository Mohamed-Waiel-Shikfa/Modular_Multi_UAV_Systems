"""Geometric SE(3) controller (Lee et al. 2010) for one module or N rigidly joined modules.

Attitude error = rotation vector of R_d^T R, so it pushes hardest when upside down
instead of vanishing at 180 deg. The wrench [f, Mx, My, Mz] is mapped to rotor thrusts
through the allocation matrix B: roll/pitch torque first, then collective, then yaw with
what is left. A roll/pitch demand beyond reach goes bang-bang (rotors on the pushing side
full, the rest off).

That prioritised split assumes identical upright rotors placed symmetrically about the CoM. Any
other assembly (tilted rotors, mixed drone sizes, an off-centre CoM) takes the general path: B is the
wrench map ([Fz, M] for upright rotors, the full 6 x n with [R^T F, M] for tilted ones), and the
thrusts are the weighted least-norm solution t = W^1/2 pinv(B W^1/2) w, clipped to [0, f_max].
allocation: capacity weights each rotor by its maximum thrust (every rotor works at the same fraction
of its capacity, so big drones carry big loads); equal uses W = I. The attitude loop is unchanged, so
a tilted assembly still flies like an underactuated vehicle (the sixth DOF is not used yet).

Attitude gains are Froude-scaled with the drone (att_scale = k): natural frequencies x k^-1/2, so a
big, slow module is not asked for more angular acceleration than its rotors give.

Frame: origin at the (assembly) CoM, axes = the assembly axes. Units: m, s, N.
"""
import numpy as np
from scipy.spatial.transform import Rotation

YAW_SIGN = np.array([-1, 1, -1, 1])   # rotor reaction torque signs, both sims: -t0 + t1 - t2 + t3


class Controller:
    def __init__(self, cfg, rotor_xy, yaw_coef, m, J, g, f_max, wn, dt, axes=None, att_scale=1.0):
        """rotor_xy: (n, 2) or (n, 3) rotor positions, m. yaw_coef: (n,) reaction torque per newton of
        thrust, signed (km / kf x spin sign), m. f_max: N, scalar or (n,). axes: (n, 3) rotor thrust axes
        (None: all +z). wn: position loop natural frequency, rad/s. dt: s per call."""
        self.m, self.J, self.g, self.dt = m, J, g, dt
        n = len(rotor_xy)
        self.f_max = np.broadcast_to(np.asarray(f_max, float), (n,)).copy()
        att = np.array([cfg.att_wn_rp, cfg.att_wn_rp, cfg.att_wn_yaw]) / np.sqrt(att_scale)
        self.KR = att ** 2
        self.KW = 2 * cfg.att_zeta * att
        self.max_tilt = np.radians(cfg.max_tilt_deg)
        # PID: KI (s^-3) on the integral of the position error removes the steady sag under
        # downwash; KI = 0.256 wn^3 (4 at wn = 2.5) stays well inside KI < KP KD.
        self.KP, self.KD, self.KI = wn ** 2, 2 * cfg.pos_zeta * wn, cfg.ki_ratio * wn ** 3
        self.i_err = np.zeros(3)                            # m s, integral of position error, capped so KI i_err <= g
        self.n = n
        self.tilted = axes is not None and not np.allclose(axes, [0, 0, 1])
        yaw_coef = np.asarray(yaw_coef, float)
        if not self.tilted:
            B = np.vstack([np.ones(n), rotor_xy[:, 1], -rotor_xy[:, 0], yaw_coef])
            self.B_pinv = np.linalg.pinv(B)
            # Identical rotors symmetric about the CoM: collective spreads evenly, so torque and
            # collective can be traded off separately (the prioritised path below).
            self.general = not (np.allclose(self.B_pinv[:, 0], 1 / n) and np.ptp(self.f_max) == 0)
        else:
            r = np.column_stack([rotor_xy, np.zeros(n)]) if rotor_xy.shape[1] == 2 else rotor_xy
            B = np.vstack([axes.T, (np.cross(r, axes) + yaw_coef[:, None] * axes).T])
            self.general = True
        if self.general:
            sw = np.sqrt(self.f_max if cfg.allocation == "capacity" else np.ones(n))
            self.alloc = sw[:, None] * np.linalg.pinv(B * sw)       # weighted least-norm: t = alloc @ wrench

    def __call__(self, pos, vel, R, w, p_d, yaw_d, v_d=np.zeros(3), a_d=np.zeros(3)):
        """Rotor thrusts (n,), N."""
        if self.KI > 0:
            self.i_err = np.clip(self.i_err + (pos - p_d) * self.dt, -self.g / self.KI, self.g / self.KI)
        a = a_d - self.KP * (pos - p_d) - self.KD * (vel - v_d) - self.KI * self.i_err
        a_xy_max = (a[2] + self.g) * np.tan(self.max_tilt) if a[2] > -self.g else 0
        if np.linalg.norm(a[:2]) > a_xy_max:
            a[:2] *= a_xy_max / np.linalg.norm(a[:2])
        F = self.m * (a + [0, 0, self.g])                   # desired force, world frame
        b3 = F / np.linalg.norm(F)
        b2 = np.cross(b3, [np.cos(yaw_d), np.sin(yaw_d), 0])
        b2 /= np.linalg.norm(b2)
        R_d = np.column_stack([np.cross(b2, b3), b2, b3])
        e_R = Rotation.from_matrix(R_d.T @ R).as_rotvec()   # body frame, |e_R| <= pi
        M = self.J @ (-self.KR * e_R - self.KW * w) + np.cross(w, self.J @ w)
        if self.general:
            wrench = np.concatenate([R.T @ F, M]) if self.tilted else np.concatenate([[F @ R[:, 2]], M])
            return np.clip(self.alloc @ wrench, 0, self.f_max)
        # Priority: roll/pitch torque, then collective, then yaw (weak on a quadrotor,
        # and a post-tumble spin would otherwise eat all the thrust).
        t_rp = self.B_pinv[:, 1:3] @ M[:2]
        f_max = self.f_max[0]
        if np.ptp(t_rp) > f_max:
            # Demand beyond reach (e.g. de-spinning after a tumble): rotors on the pushing
            # side full, the rest off. Most torque along M, half the net thrust of pinv + clip.
            base = np.where(t_rp > 0, f_max, 0.0)
        else:
            base = t_rp + np.clip(F @ R[:, 2] / self.n, -t_rp.min(), f_max - t_rp.max())
        t_yaw = self.B_pinv[:, 3] * M[2]
        room = np.where(t_yaw > 0, f_max - base, -base) / np.where(t_yaw == 0, np.inf, t_yaw)
        return np.clip(base + np.clip(room.min(), 0, 1) * t_yaw, 0, f_max)
