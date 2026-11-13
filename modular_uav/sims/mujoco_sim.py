"""MuJoCo backend: upstream MuJoCo-drones-gym, extended by subclassing only.

CagedAviary(PerDrone, BaseAviary) changes, from the config:
  - the module model: upstream builds its MJCF from DRONE_PARAMS[CF2X]; drone 0's caged mass and
    inertia are swapped into that table only while the parent constructor runs, so every
    derived constant (HOVER_RPM, MAX_RPM, ...) is computed upstream, then the model is
    rebuilt with each drone's own mass, inertia, Froude scale (meshes, collision cylinder,
    rotor sites) and cage rods, one inactive weld per docked pair and the contact excludes;
  - per-drone coefficients and RPM limits (sims/scaling.py); identical to upstream for k = 1;
  - _downwash: `original` calls the upstream method; `regularised` uses physics/downwash.py;
  - step: with gui, draw the viewer and pace to wall-clock (upstream never calls render()).
Sim wraps it in the backend interface used by the tasks (see sims/__init__.py).

MuJoCo: quaternion w, x, y, z; free-joint ang_v in the body frame.
"""
import re
import time

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

from multi_drone_mujoco.envs import base_aviary
from multi_drone_mujoco.utils.enums import ActionType, DroneModel, Physics

from modular_uav.sims.scaling import PerDrone

PHYSICS = {"base": Physics.MJC, "gnd": Physics.MJC_GND, "drag": Physics.MJC_DRAG, "dw": Physics.MJC_DW,
           "gnd_drag_dw": Physics.MJC_GND_DRAG_DW}


class CagedAviary(PerDrone, base_aviary.BaseAviary):
    def __init__(self, cfg, mtypes, kinds, welds=(), no_contact=(), **kw):
        """welds: (i, j, offset, quat_xyzw) per docked pair: module j's pose in module i's frame, m.
        no_contact: (i, j) pairs that never collide."""
        self.cfg = cfg
        stock = base_aviary.DRONE_PARAMS[DroneModel.CF2X]
        t0 = mtypes[kinds[0]]
        ixx, iyy, izz = np.diag(t0.J)
        base_aviary.DRONE_PARAMS[DroneModel.CF2X] = {**stock, "mass": t0.mass, "ixx": ixx, "iyy": iyy, "izz": izz}
        try:
            super().__init__(drone_model=DroneModel.CF2X, physics=PHYSICS[cfg.physics],
                             act_type=ActionType.RPM, gui=cfg.gui, record=cfg.record,
                             output_folder=cfg.output_folder, **kw)
            xml = base_aviary._generate_aviary_xml(num_drones=self.NUM_DRONES, drone_model=DroneModel.CF2X,
                                                   init_xyzs=self.INIT_XYZS, init_rpys=self.INIT_RPYS,
                                                   obstacles=False, vision=False, timestep=self.SIM_TIMESTEP)
        finally:
            base_aviary.DRONE_PARAMS[DroneModel.CF2X] = stock
        n = self.NUM_DRONES
        scales = sorted({mtypes[k].scale for k in kinds} - {1.0})
        meshes = "".join(f'    <mesh file="{base_aviary.CF2_MESH_DIR}/cf2_{i}.obj" name="cf2_vis_{i}_k{k:g}" '
                         f'scale="{k:g} {k:g} {k:g}"/>\n' for k in scales for i in range(7))
        assert xml.count("<asset>") == 1, "upstream MJCF layout changed: update mujoco_sim.CagedAviary"
        xml = xml.replace("<asset>", "<asset>\n" + meshes.rstrip("\n"))
        for d in range(n):
            t = mtypes[kinds[d]]
            rgba = " ".join(f"{c:g}" for c in t.color)
            rods = "".join(f'      <geom name="drone{d}_cage{k}" type="capsule" size="{t.rod_radius}" '
                           f'fromto="{a[0]} {a[1]} {a[2]} {b[0]} {b[1]} {b[2]}" '
                           f'rgba="{rgba}" contype="1" conaffinity="1"/>\n'
                           for k, (a, b) in enumerate(t.rods))
            anchor = f'<site name="drone{d}_center"'
            assert xml.count(anchor) == 1, "upstream MJCF layout changed: update mujoco_sim.CagedAviary"
            head, tail = xml.split(f'<body name="drone{d}" ', 1)
            body, rest = tail.split(f'<site name="drone{d}_prop3"', 1)
            line, rest = rest.split("\n", 1)
            body = _scale_body(body + f'<site name="drone{d}_prop3"' + line, t)
            xml = head + f'<body name="drone{d}" ' + body.replace(anchor, rods.lstrip() + "      " + anchor) + "\n" + rest
        # relpose = module j's pose in module i's frame: x y z qw qx qy qz
        eq = "".join(f'    <weld body1="drone{i}" body2="drone{j}" active="false" '
                     f'relpose="{o[0]} {o[1]} {o[2]} {q[3]} {q[0]} {q[1]} {q[2]}" solref="0.01 1"/>\n'
                     for i, j, o, q in welds)
        excl = "".join(f'    <exclude body1="drone{i}" body2="drone{j}"/>\n' for i, j in no_contact)
        extra = (f"  <equality>\n{eq}  </equality>\n" if eq else "") + (f"  <contact>\n{excl}  </contact>\n" if excl else "")
        self.model = mujoco.MjModel.from_xml_string(xml.replace("</mujoco>", extra + "</mujoco>"))
        self.data = mujoco.MjData(self.model)
        mujoco.mj_forward(self.model, self.data)
        self._updateAndStoreKinematicInformation()
        self._wall_start, self._cam_set = None, False
        self._setup_drones(mtypes, kinds)

    def reset(self, seed=None, options=None):
        self._wall_start, self._cam_set = None, False
        return super().reset(seed=seed, options=options)

    def step(self, action):
        out = super().step(action)
        if self.render_mode == "human":
            # Live viewer: draw this step and pace to wall-clock (sim time = step_counter / SIM_FREQ)
            self.render()
            if not self._cam_set and self._viewer is not None:
                # MuJoCo orbits about cam.lookat, which starts at the model centre: aim it at the modules
                xy, ext = self.INIT_XYZS[:, :2], np.ptp(self.INIT_XYZS[:, :2])
                cam = self._viewer.cam
                cam.lookat[:] = [*xy.mean(0), 0.5]
                cam.distance, cam.azimuth, cam.elevation = min(1.5 + 4 * ext, 8), 30, -25
                self._cam_set = True
            if self._wall_start is None:
                self._wall_start = time.time() - self.step_counter / self.SIM_FREQ
            lag = self.step_counter / self.SIM_FREQ - (time.time() - self._wall_start)
            if lag > 0:
                time.sleep(lag)
        return out

    def _downwash(self, nth_drone):
        if self.cfg.downwash_model == "original":
            return super()._downwash(nth_drone)
        body = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_BODY, f"drone{nth_drone}")
        self.data.xfrc_applied[body, 2] -= self._dw_force(nth_drone)


def _scale_body(body, t):
    """One drone body of the upstream MJCF with module type t: its mass and inertia, and at scale k
    the scaled meshes, collision cylinder and rotor sites."""
    J = np.diag(t.J)
    body, n = re.subn(r'mass="[^"]*" diaginertia="[^"]*"',
                      f'mass="{t.mass:g}" diaginertia="{J[0]:.6e} {J[1]:.6e} {J[2]:.6e}"', body)
    assert n == 1, "upstream MJCF drone inertial changed: update mujoco_sim._scale_body"
    k = t.scale
    if k == 1:
        return body
    body = re.sub(r'mesh="cf2_vis_(\d)"', rf'mesh="cf2_vis_\1_k{k:g}"', body)
    scale = lambda m: " ".join(f"{float(x) * k:g}" for x in m.group(2).split())
    body = re.sub(r'(_collision" type="cylinder" size=")([^"]*)', lambda m: m.group(1) + scale(m), body)
    return re.sub(r'(_prop\d" pos=")([^"]*)', lambda m: m.group(1) + scale(m), body)


class Sim:
    """N modules, welded pairs on snap(). See sims/__init__.py for the interface."""

    def __init__(self, cfg, starts, welds, freq, rpys=None, mtypes=None, kinds=None, no_contact=()):
        self.mtypes, self.kinds = mtypes, kinds
        self.env = e = CagedAviary(cfg, mtypes, kinds, welds=welds, no_contact=no_contact, num_drones=len(starts),
                                   sim_freq=freq, ctrl_freq=freq, initial_xyzs=np.asarray(starts),
                                   initial_rpys=None if rpys is None else np.asarray(rpys, float))
        e.reset()
        self.M, self.J, self.G, self.KF, self.KM, self.MAX_RPM = e.M, e.J, e.G, e.KF, e.KM, e.MAX_RPM
        self.ROTORS = e.L / np.sqrt(2) * np.array([[1, 1], [-1, 1], [-1, -1], [1, -1]])   # see upstream _physics()

    def drone(self, i):
        return self.env.drone(i, self.mtypes[self.kinds[i]], self.ROTORS)

    def state(self, i):
        e = self.env
        R = Rotation.from_quat(e.quat[i], scalar_first=True).as_matrix()
        return e.pos[i].copy(), e.vel[i].copy(), R, e.ang_v[i].copy()

    def quat_xyzw(self, i):
        return np.roll(np.asarray(self.env.quat[i], float), -1)

    def step(self, rpm):
        self.env.step(np.asarray(rpm).flatten())

    def stock_pid(self):
        """Upstream PIDControl (gains untouched, reads mass and inertia from the env) -> f(i, dt, target_pos) = rpm (4,)."""
        from multi_drone_mujoco.control.pid_control import PIDControl
        e = self.env
        ctrls = [PIDControl(e) for _ in range(e.NUM_DRONES)]
        return lambda i, dt, target: ctrls[i].computeControl(
            control_timestep=dt, cur_pos=e.pos[i], cur_quat=e.quat[i], cur_vel=e.vel[i], cur_ang_vel=e.ang_v[i],
            target_pos=np.asarray(target, float))[0].flatten()

    def snap(self, which=None):
        """Weld the docked pairs: all of them, or the weld indices in `which`."""
        if self.env.model.neq:
            self.env.data.eq_active[slice(None) if which is None else list(which)] = 1

    def set_attitude(self, i, R):
        e = self.env
        joint = mujoco.mj_name2id(e.model, mujoco.mjtObj.mjOBJ_JOINT, f"drone{i}_joint")
        q, v = e.model.jnt_qposadr[joint], e.model.jnt_dofadr[joint]
        e.data.qpos[q + 3:q + 7] = Rotation.from_matrix(R).as_quat(scalar_first=True)
        e.data.qvel[v + 3:v + 6] = 0
        mujoco.mj_forward(e.model, e.data)
        e._updateAndStoreKinematicInformation()

    def draw_plume(self, segs, rgb, legend):
        """Isolines as line geoms in the viewer's user scene, colour bar boxes beside the
        modules, and the scale printed top left."""
        vw = self.env._viewer
        if vw is None:
            return
        pos, labels = legend
        scn = vw.user_scn
        with vw.lock():
            n = 0
            for q, (_, _, c) in zip(pos, labels):
                mujoco.mjv_initGeom(scn.geoms[n], mujoco.mjtGeom.mjGEOM_BOX, np.full(3, 0.02), q,
                                    np.eye(3).flatten(), np.append(c, 1).astype(np.float32))
                n += 1
            for (a, b), c in zip(segs[:scn.maxgeom - n], rgb):
                g = scn.geoms[n]
                mujoco.mjv_initGeom(g, mujoco.mjtGeom.mjGEOM_LINE, np.zeros(3), np.zeros(3), np.eye(3).flatten(),
                                    np.append(c, 1).astype(np.float32))
                mujoco.mjv_connector(g, mujoco.mjtGeom.mjGEOM_LINE, 2, a, b)
                n += 1
            scn.ngeom = n
        vw.set_texts((mujoco.mjtFont.mjFONT_NORMAL, mujoco.mjtGridPos.mjGRID_TOPLEFT,
                      "\n".join(t for _, t, _ in labels[::-1]), None))

    def close(self):
        self.env.close()
