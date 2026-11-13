"""PyBullet backend: upstream gym-pybullet-drones, extended by subclassing only.

CagedAviary(PerDrone, CtrlAviary) changes, all from the config:
  - the module model: one URDF per module type, generated at run time from the upstream
    cf2x.urdf: the type's mass, inertia and cage rods, the drone scaled by its Froude scale k, and
    the CF2X drawn from the seven coloured CF2 meshes MuJoCo uses (the upstream cf2.dae renders
    black). drone_model stays DroneModel.CF2X, so every CF2X branch upstream (mixer, torques, PID)
    applies; drones whose type differs from drone 0's are reloaded with their own URDF;
  - per-drone coefficients and RPM limits (sims/scaling.py); identical to upstream for k = 1;
  - _downwash: `original` calls the upstream method; `regularised` uses physics/downwash.py.
Sim wraps it in the backend interface used by the tasks (see sims/__init__.py).

PyBullet: quaternion x, y, z, w; ang_v in the world frame; prop links mirrored in y
relative to MuJoCo's _physics(), hence the ROTORS table below.
"""
import hashlib
import re
import tempfile
import time
from importlib.resources import files
from pathlib import Path

import numpy as np
import pybullet as p
from scipy.spatial.transform import Rotation

from gym_pybullet_drones.envs.CtrlAviary import CtrlAviary
from gym_pybullet_drones.utils.enums import DroneModel, Physics

from modular_uav.models import cage
from modular_uav.sims.scaling import PerDrone
from modular_uav.utils.utils import sync

PHYSICS = {"base": Physics.PYB, "gnd": Physics.PYB_GND, "drag": Physics.PYB_DRAG, "dw": Physics.PYB_DW,
           "gnd_drag_dw": Physics.PYB_GND_DRAG_DW}
ASSETS = Path(str(files("gym_pybullet_drones") / "assets"))
CF2_OBJ = Path(str(files("multi_drone_mujoco") / "assets" / "cf2"))
# the seven CF2 mesh parts and their MuJoCo-drones-gym material colours (base_aviary.py)
CF2_PARTS = [(0.792, 0.820, 0.933), (0.109, 0.184, 0.0), (0.969, 0.878, 0.6), (0.631, 0.659, 0.678),
             (0.898, 0.898, 0.898), (0.102, 0.102, 0.102), (1, 1, 1)]
DITHER = np.random.default_rng(0).uniform(-0.5, 0.5, (8, 3))   # colour bar swatch points, x swatch size
POINTS_PER_SEG = 4          # isoline drawn as dots: PyBullet has one call for many points, not lines


def caged_urdf(mtype):
    """Write the URDF of a module type (cached by content) and return its absolute path."""
    src = (ASSETS / "cf2x.urdf").read_text()
    old = ('<mass value="0.027"/>\n'
           '      <inertia ixx="1.4e-5" ixy="0.0" ixz="0.0" iyy="1.4e-5" iyz="0.0" izz="2.17e-5"/>')
    assert old in src and cage.CF2X["mass"] == 0.027, "upstream cf2x.urdf inertial changed: update pybullet_sim.caged_urdf"
    k, J = mtype.scale, np.diag(mtype.J)
    new = (f'<!-- CF2X at scale {k:g} + {1000 * mtype.cage_mass:g} g {mtype.name} cage (models/module.py) -->\n'
           f'      <mass value="{mtype.mass:g}"/>\n'
           f'      <inertia ixx="{J[0]:.6e}" ixy="0.0" ixz="0.0" iyy="{J[1]:.6e}" iyz="0.0" izz="{J[2]:.6e}"/>')
    rgba = " ".join(f"{c:g}" for c in mtype.color)
    rods = f"\n    <!-- {mtype.name} cage: {len(mtype.rods)} rods -->\n"
    for a, b in mtype.rods:
        u = b - a
        length = np.linalg.norm(u)
        u, mid = u / length, (a + b) / 2
        pitch, yaw = np.arccos(np.clip(u[2], -1, 1)), np.arctan2(u[1], u[0])   # Rz(yaw) Ry(pitch) z = u
        origin = f'<origin rpy="0 {pitch:.6f} {yaw:.6f}" xyz="{mid[0]:.6f} {mid[1]:.6f} {mid[2]:.6f}"/>'
        geom = f'<geometry><cylinder radius="{mtype.rod_radius}" length="{length:.6f}"/></geometry>'
        rods += (f'    <visual>{origin}{geom}<material name="cage"><color rgba="{rgba}"/></material></visual>\n'
                 f'    <collision>{origin}{geom}</collision>\n')
    parts = [f'    <visual><origin rpy="0 0 0" xyz="0 0 0"/><geometry><mesh filename="{CF2_OBJ / f"cf2_{i}.obj"}" '
             f'scale="{k:g} {k:g} {k:g}"/></geometry><material name="cf2_{i}"><color rgba="{r} {g} {b} 1"/>'
             f'</material></visual>\n' for i, (r, g, b) in enumerate(CF2_PARTS)]
    out, n = re.subn(r"    <visual>.*?</visual>\n", parts[0], src, count=1, flags=re.S)
    assert n == 1, "upstream cf2x.urdf visual changed: update pybullet_sim.caged_urdf"
    # Upstream _parseURDFParameters reads base_link's children by index (inertial, visual, collision),
    # so the other mesh parts and the rods go after its own collision element.
    out = (out.replace(old, new).replace('<cylinder radius=".06" length=".025"/>',
                                         f'<cylinder radius="{0.06 * k:g}" length="{0.025 * k:g}"/>')
           .replace("  </link>", "".join(parts[1:]) + rods + "  </link>", 1)
           .replace('<robot name="cf2">', f'<robot name="cf2x_{mtype.name}">'))
    for s in ("0.028 -0.028", "-0.028 -0.028", "-0.028 0.028", "0.028 0.028"):   # rotor arms x k
        x, y = (float(v) * k for v in s.split())
        out = out.replace(f'xyz="{s} 0"', f'xyz="{x:g} {y:g} 0"')
    path = Path(tempfile.gettempdir()) / "modular_uav" / f"cf2x_{mtype.name}_{hashlib.sha1(out.encode()).hexdigest()[:10]}.urdf"
    path.parent.mkdir(exist_ok=True)
    if not path.exists():
        path.write_text(out)
    return path


class CagedAviary(PerDrone, CtrlAviary):
    def __init__(self, cfg, mtypes, kinds, **kw):
        self.cfg, self._kinds = cfg, list(kinds)
        self._urdfs = {kind: caged_urdf(t) for kind, t in mtypes.items()}
        super().__init__(drone_model=DroneModel.CF2X, physics=PHYSICS[cfg.physics], **kw)
        self._setup_drones(mtypes, kinds)

    def _parseURDFParameters(self):
        # Upstream joins assets/ with self.URDF; joining an absolute path yields that path.
        self.URDF = str(self._urdfs[self._kinds[0]])
        return super()._parseURDFParameters()

    def _housekeeping(self):
        super()._housekeeping()            # loads every drone from drone 0's URDF
        for i, kind in enumerate(self._kinds):
            if kind != self._kinds[0]:
                p.removeBody(int(self.DRONE_IDS[i]), physicsClientId=self.CLIENT)
                self.DRONE_IDS[i] = p.loadURDF(str(self._urdfs[kind]), self.INIT_XYZS[i, :],
                                               p.getQuaternionFromEuler(self.INIT_RPYS[i, :]),
                                               flags=p.URDF_USE_INERTIA_FROM_FILE, physicsClientId=self.CLIENT)

    def _downwash(self, nth_drone):
        if self.cfg.downwash_model == "original":
            return super()._downwash(nth_drone)
        F = self._dw_force(nth_drone)
        if F > 0:
            p.applyExternalForce(self.DRONE_IDS[nth_drone], 4, forceObj=[0, 0, -F], posObj=[0, 0, 0],
                                 flags=p.LINK_FRAME, physicsClientId=self.CLIENT)


class Sim:
    """N modules, welded pairs on snap(). See sims/__init__.py for the interface."""
    ROTORS = 0.028 * np.array([[1, -1], [-1, -1], [-1, 1], [1, 1]])   # prop links of cf2x.urdf, body x, y, m

    def __init__(self, cfg, starts, welds, freq, rpys=None, mtypes=None, kinds=None, no_contact=()):
        self.gui, self.freq, self.welds = cfg.gui, freq, welds
        self.mtypes, self.kinds = mtypes, kinds
        self.env = e = CagedAviary(cfg, mtypes, kinds, num_drones=len(starts), gui=cfg.gui, record=cfg.record,
                                   initial_xyzs=np.asarray(starts), pyb_freq=freq, ctrl_freq=freq,
                                   initial_rpys=None if rpys is None else np.asarray(rpys, float),
                                   user_debug_gui=False, output_folder=cfg.output_folder)
        self.obs, _ = e.reset()
        self.M, self.J, self.G, self.KF, self.KM, self.MAX_RPM = e.M, e.J, e.G, e.KF, e.KM, e.MAX_RPM
        self.ids = e.getDroneIds()
        for i, j in no_contact:
            p.setCollisionFilterPair(self.ids[i], self.ids[j], -1, -1, 0, physicsClientId=e.CLIENT)
        if cfg.gui:
            # Set once, then the mouse owns the camera: re-setting it from getDebugVisualizerCamera
            # every step (which reads back a frame late) fought the mouse.
            starts = np.asarray(starts)
            p.resetDebugVisualizerCamera(min(1.5 + 4 * np.ptp(starts[:, :2]), 8), 30, -25,
                                         [*starts[:, :2].mean(0), 0.5], physicsClientId=e.CLIENT)
        self.start, self.k = time.time(), 0
        self.plume_ids = []

    def drone(self, i):
        return self.env.drone(i, self.mtypes[self.kinds[i]], self.ROTORS)

    def state(self, i):
        e = self.env
        R = Rotation.from_quat(e.quat[i]).as_matrix()
        return e.pos[i].copy(), e.vel[i].copy(), R, R.T @ e.ang_v[i]

    def quat_xyzw(self, i):
        return np.asarray(self.env.quat[i], float)

    def step(self, rpm):
        self.obs, *_ = self.env.step(np.asarray(rpm).reshape(-1, 4))
        if self.gui:
            sync(self.k, self.start, 1 / self.freq)
        self.k += 1

    def stock_pid(self):
        """Upstream DSLPIDControl (gains untouched) -> f(i, dt, target_pos) = rpm (4,)."""
        from gym_pybullet_drones.control.DSLPIDControl import DSLPIDControl
        e = self.env
        ctrls = [DSLPIDControl(drone_model=DroneModel.CF2X) for _ in range(e.NUM_DRONES)]
        for c in ctrls:
            c.GRAVITY = e.GRAVITY       # weight of the caged module; upstream reads the bare cf2x.urdf mass
        return lambda i, dt, target: ctrls[i].computeControl(dt, e.pos[i], e.quat[i], e.vel[i], e.ang_v[i],
                                                             np.asarray(target, float))[0]

    def snap(self, which=None):
        """Weld the docked pairs: all of them, or the weld indices in `which`."""
        for n in range(len(self.welds)) if which is None else which:
            i, j, o, q = self.welds[n]
            p.createConstraint(self.ids[i], -1, self.ids[j], -1, p.JOINT_FIXED, [0, 0, 0],
                               parentFramePosition=list(o), childFramePosition=[0, 0, 0],
                               parentFrameOrientation=list(q), physicsClientId=self.env.CLIENT)

    def set_attitude(self, i, R):
        e = self.env
        p.resetBasePositionAndOrientation(self.ids[i], e.pos[i].tolist(), Rotation.from_matrix(R).as_quat().tolist(),
                                          physicsClientId=e.CLIENT)
        p.resetBaseVelocity(self.ids[i], e.vel[i].tolist(), [0, 0, 0], physicsClientId=e.CLIENT)
        e._updateAndStoreKinematicInformation()

    def draw_plume(self, segs, rgb, legend):
        """Isolines as debug points, POINTS_PER_SEG per segment, all in one call
        (one debug line per segment would be thousands of calls per redraw)."""
        if not self.gui:
            return
        t = np.linspace(0, 1, POINTS_PER_SEG)[None, :, None]
        xyz = (segs[:, :1] + t * (segs[:, 1:] - segs[:, :1])).reshape(-1, 3)
        pos, labels = legend
        bar = (pos[:, None] + 0.04 * DITHER[None]).reshape(-1, 3)        # colour bar swatches
        cols = np.vstack([np.repeat(rgb, POINTS_PER_SEG, 0),
                          np.repeat([c for _, _, c in labels[:len(pos)]], len(DITHER), 0)])
        ids = self.plume_ids or [-1] * (1 + len(labels))
        c = self.env.CLIENT
        self.plume_ids = [p.addUserDebugPoints(np.vstack([xyz, bar]).tolist(), cols.tolist(), pointSize=3,
                                               replaceItemUniqueId=ids[0], physicsClientId=c)]
        self.plume_ids += [p.addUserDebugText(t, list(q), list(col), textSize=1.1, replaceItemUniqueId=i,
                                              physicsClientId=c) for (q, t, col), i in zip(labels, ids[1:])]

    def close(self):
        self.env.close()
