# Simulator reference: gym-pybullet-drones and MuJoCo-drones-gym

Reference notes on the two drone simulators used by this project: **gym-pybullet-drones** (PyBullet) and **MuJoCo-drones-gym** (MuJoCo). It covers how they work inside, what they model and what they do not, how they compare, the bugs found in them, and how the project builds on them.

Related documents:
- `docs/architecture.md`: the declarative design of `modular_uav/` and `simulate.py`.
- `docs/dock.md`: the docking scenarios, their results, and the downwash findings and fix.

## Evidence tags

Every claim carries a tag that says where it came from. Check the tag before citing a claim in the thesis.

- **[measured]**: run on the development laptop (Intel Iris Xe, CPU only, no NVIDIA GPU). The run conditions are written next to the number. Unless dated otherwise, measurements are from 2026-10-31, on the simulators as they were then.
- **[code]**: read from the source files. Paths are relative to `codebase/`. Line numbers refer to the upstream commits checked out today (PyBullet `f83fbdd`, MuJoCo `ca170dc`).
- **[docs]**: from the MuJoCo documentation (fetched via Context7).
- **[computed]**: worked out by hand from a formula in the code.
- **[assumption]**: reasoning or general knowledge that was not checked here. Verify it before citing it.

## Contents

1. [How the project uses the simulators](#1-how-the-project-uses-the-simulators)
2. [Setup and environment](#2-setup-and-environment)
3. [Running the upstream examples](#3-running-the-upstream-examples)
4. [Inside BaseAviary (PyBullet)](#4-inside-baseaviary-pybullet)
5. [Open loop vs feedback, and sync()](#5-open-loop-vs-feedback-and-sync)
6. [Choosing frequencies](#6-choosing-frequencies)
7. [Physics modes](#7-physics-modes)
8. [The DSL PID controller](#8-the-dsl-pid-controller)
9. [Drone models and the caged module](#9-drone-models-and-the-caged-module)
10. [Aerodynamic models in code](#10-aerodynamic-models-in-code)
11. [MuJoCo-drones-gym overview](#11-mujoco-drones-gym-overview)
12. [File map of both repos](#12-file-map-of-both-repos)
13. [Deep dives: air, integrators, force application, APIs](#13-deep-dives-air-integrators-force-application-apis)
14. [Docking: constraints, welds and recompile](#14-docking-constraints-welds-and-recompile)
15. [The controllers explained](#15-the-controllers-explained)
16. [Reinforcement learning in both repos](#16-reinforcement-learning-in-both-repos)
17. [Bugs and caveats](#17-bugs-and-caveats)
18. [Open questions](#18-open-questions)
19. [Glossary](#19-glossary)

---

## 1. How the project uses the simulators

### 1.1 Upstream, untouched
- Both simulators are the **unmodified upstream repos**, checked out as git submodules of `codebase/` (`.gitmodules` points upstream) **[code]**:

  | Folder | Upstream | Commit |
  |---|---|---|
  | `gym-pybullet-drones/` | `learnsyslab/gym-pybullet-drones` (formerly `utiasDSL`) | `f83fbdd` |
  | `MuJoCo-drones-gym/` | `tau-intelligence/MuJoCo-drones-gym` | `ca170dc` |

- No file inside either folder is edited. All project code lives in the package `modular_uav/` and extends the simulators **by subclassing only**.
- Earlier fork-only work (sandbox scripts, caged URDFs and their enum entries, the downwash patch inside `BaseAviary`, the MuJoCo live-viewer patch, the `modular-aviary` branches) is no longer in the simulator folders. It survives in the history of the `Mohamed-Waiel-Shikfa` forks. Section 1.3 says where each piece lives now.

### 1.2 Subclass hooks used
One backend per simulator, both exposing the same `Sim` interface (`modular_uav/sims/__init__.py`), so tasks never import a simulator directly **[code]**.

| Backend | Subclass | Hooks overridden | What changes |
|---|---|---|---|
| PyBullet, `sims/pybullet_sim.py` | `CagedAviary(CtrlAviary)` | `_parseURDFParameters` | points `self.URDF` at a caged CF2X URDF generated at run time from the upstream `cf2x.urdf` (cage mass, inertia and rods added) |
| | | `_downwash` | `original`: calls the upstream method. `regularised`: uses `physics/downwash.py` |
| MuJoCo, `sims/mujoco_sim.py` | `CagedAviary(BaseAviary)` | `__init__` | swaps the caged mass and inertia into `DRONE_PARAMS[CF2X]` only while the parent constructor runs (so `HOVER_RPM`, `MAX_RPM` etc. are derived upstream), restores the table, then rebuilds the model with cage rods (capsules), one inactive weld per docked pair and the contact excludes |
| | | `_downwash` | as PyBullet |
| | | `step` | with `gui: true`: calls `render()` and sleeps until wall-clock time catches up with sim time (upstream `step()` never draws) |

- `drone_model` stays `CF2X` in both, so every upstream CF2X branch (mixer, torque law, stock PID) applies unchanged.
- `sync()`, `str2bool()` and `Logger` are copied unchanged from gym-pybullet-drones (`f83fbdd`, MIT licence) into `modular_uav/utils/`, so both backends share them.

### 1.3 Where the earlier modifications live now

| Earlier (fork) | Now |
|---|---|
| `CF2X_BOX` / `CF2X_TETRA` enum entries, `cf2x_box.urdf`, `make_caged_urdfs.py` | `modular_uav/models/cage.py`, plus a URDF generated at run time (PyBullet) or a `DRONE_PARAMS` swap + MJCF rods (MuJoCo). Field `cage: box \| tetra \| triangle \| hex \| dodeca \| none` |
| Regularised downwash patched into both `BaseAviary` files | `modular_uav/physics/downwash.py`. Field `downwash_model: original \| regularised`, with `dw_thrust_cap`, `dw_dz_ramp`, `dw_min_width` |
| MuJoCo `--gui` live-viewer patch in `base_aviary.py` | `CagedAviary.step` in `sims/mujoco_sim.py`. Field `gui: true` |
| `--physics pyb_*` / `mjc_*` | Field `physics: base \| gnd \| drag \| dw \| gnd_drag_dw`, mapped to each simulator's `Physics` enum |
| `sandbox/caged_hover.py` | `task: hover` (`scenarios/hover/*.yaml`) |
| `sandbox/dock/...` scripts | `task: dock` (`scenarios/box/...`, `scenarios/tetra/...`); see `docs/dock.md` |
| `sandbox/manual_step.py` (open-loop ±2 % thrust experiment), `sandbox/PID.py` (copy of upstream `pid.py`) | dropped; results kept in sections 5.2 and 3.1 |

### 1.4 Running
One master script runs everything. Every parameter is a field that can be set in a scenario YAML or as a command-line flag of the same name. Precedence: **flag > YAML > default**. All fields are documented in `scenarios/master.yaml` (generated from `modular_uav/config/fields.py`). The run exits 0 only if all its self-checks pass.

```bash
conda activate drones
cd codebase
python simulate.py --scenario scenarios/hover/box.yaml                 # PyBullet (default sim)
python simulate.py --scenario scenarios/hover/box.yaml --sim mujoco
python simulate.py --scenario scenarios/hover/none.yaml --gui          # bare CF2X, live viewer
python simulate.py --scenario scenarios/box/3x3/fail.yaml --sim mujoco --downwash_model original
python simulate.py --scenario scenarios/box/pair/base.yaml --physics base
python simulate.py -h                                                  # short help; --help all, --help WORD
```

- **[measured, 2026-11-07]** `scenarios/hover/box.yaml` (one box-caged CF2X, stock PID, target [0, 0, 1] m, 10 s, physics 240 Hz, PID 48 Hz, `physics: gnd_drag_dw`) passes on both: final position error 0.5 cm in PyBullet, 1.8 cm in MuJoCo.
- Defaults that matter: `sim: pybullet`, `physics: gnd_drag_dw`, `downwash_model: regularised`, `freq: 240` (physics = control rate for the dock task), `module_collisions: false`.

---

## 2. Setup and environment

### 2.1 Python environment
- Everything runs in the conda env `drones` (`~/miniforge3/envs/drones`, Python 3.12). `pip install -r requirements.txt` installs both submodules **editable**, plus numpy, scipy, matplotlib, pyyaml and pytest **[code]**.
  - PyBullet: `gym-pybullet-drones 2.2.0`, `pybullet 3.2.7`, `stable_baselines3 2.9.0`.
  - MuJoCo: package `mj-drones-gym 1.0.0`, installed with extras `[rl,marl,viz]`, which add `mujoco 3.17.0` and `pettingzoo 1.27.0`. The `gpu` extra (CUDA JAX) is skipped: no NVIDIA GPU. `pip check` reported no conflicts.
- `gym-pybullet-drones/.venv` is an older, separate setup. Use `drones`.

### 2.2 Viewer notes
- MuJoCo viewer: left-drag rotates, right-drag pans, scroll zooms. Double-click a body to select it, then Ctrl + right-drag pushes it. Space pauses.
- `Failed to load plugin 'libdecor-gtk.so'` is a harmless Wayland warning.
- PyBullet console noise (OpenGL, threads, Gymnasium float32 warnings) is ignorable.

---

## 3. Running the upstream examples

The upstream examples still run from inside each submodule. They use the stock models only (no cage).

```bash
conda activate drones
cd codebase/gym-pybullet-drones
python gym_pybullet_drones/examples/pid.py                 # GUI by default
python gym_pybullet_drones/examples/pid.py --gui False --plot False --duration_sec 3
cd ../MuJoCo-drones-gym
python multi_drone_mujoco/examples/pid.py                  # headless only, no flags
python multi_drone_mujoco/examples/downwash.py
```

- Upstream MuJoCo examples have no `--gui` flag and never open a live window **[code]**. The README GIFs were rendered offscreen by `render_demos.py` / `render_demos2.py` (`render_mode="rgb_array"`). For a live MuJoCo window, use `simulate.py --sim mujoco --gui` (section 1.2).
- The MuJoCo examples write CSVs to `results/` in the current folder.

### 3.1 PyBullet `examples/pid.py`
- **[measured]** Headless 3 s run: exit 0, 3 drones flying circles at about 6× real time.
- CLI flags: `--num_drones`, `--physics {pyb, pyb_gnd, pyb_drag, pyb_dw, pyb_gnd_drag_dw}`, `--simulation_freq_hz 240 --control_freq_hz 48`, `--gui`, `--plot`, `--record_video`, `--duration_sec`.
- What it does:
  1. Builds circular waypoints (`TARGET_POS`), one per control step.
  2. Creates `CtrlAviary` (action = 4 RPMs per drone).
  3. Creates one `DSLPIDControl` per drone.
  4. Loop: `env.step(action)`, then `computeControlFromState(state=obs[j], target_pos=...)` gives the next RPMs.
  5. `Logger` records the states and plots them.

### 3.2 The one idea that carries through everything
**Env = physics. Controller = project code. They meet only at `step(action)`.**

---

## 4. Inside BaseAviary (PyBullet)

File: `gym-pybullet-drones/gym_pybullet_drones/envs/BaseAviary.py` (1147 lines).

### 4.1 Template method
`BaseAviary` owns the physics. Subclasses only fill in hooks:

| Hook | `CtrlAviary` returns |
|---|---|
| `_actionSpace()` | Box `(N,4)`, RPM in `[0, MAX_RPM]` |
| `_observationSpace()` / `_computeObs()` | `(N,20)` state vectors |
| `_preprocessAction(a)` | clips to `[0, MAX_RPM]` |
| `_computeReward / Terminated / Truncated / Info` | dummies |

The project's `CagedAviary` additionally overrides `_parseURDFParameters` (line 982) and `_downwash` (line 782); see 1.2.

### 4.2 Two clocks (lines 77–81) **[code]**
- `PYB_FREQ` (default 240 Hz) is the physics integrator rate.
- `CTRL_FREQ` is how often `step()` is called. Defaults: **240 in `CtrlAviary`**; `pid.py` passes 48.
- `PYB_STEPS_PER_CTRL = PYB_FREQ / CTRL_FREQ`. The constructor raises an error if it doesn't divide exactly (line 80).
- With 240/48, **one `step()` = 5 physics ticks = 1/48 s**, and the action is held constant across them (zero-order hold, see 6.2).

### 4.3 `step()` walk-through (lines 258–383; physics loop 342–372) **[code]**
```
action -> _preprocessAction -> clip -> reshape (N,4)
repeat PYB_STEPS_PER_CTRL times:
    if the physics mode needs fresh state: _updateAndStoreKinematicInformation()
    for each drone: _physics(rpm) [+ _groundEffect | _drag | _downwash per mode]
    p.stepSimulation()          # skipped in Physics.DYN
update kinematics -> obs, reward, terminated, truncated, info
```

### 4.4 Rotor law, `_physics()` (line 676) **[code]**
```
F_i = KF * rpm_i^2                         (N, at prop link i, along local +z)
tau_z = KM * (-rpm0^2 + rpm1^2 - rpm2^2 + rpm3^2)   (N*m, on link 4 = centre)
```
- Units: RPM in rev/min. KF and KM come from the URDF.
- `LINK_FRAME`: forces rotate with the drone, which is how tilting moves it sideways.
- Roll and pitch torque come out of the 4 forces at offset arms. Yaw comes from the alternating CW/CCW reaction torques.
- This function hard-codes 4 rotors. A modular structure needs a general allocation matrix **B (6×k)** instead.
- Prop link positions in `cf2x.urdf` are ±0.028 m in body x and y. They are mirrored in y relative to the rotor order of MuJoCo's `_physics()`, so the project keeps a per-backend `ROTORS` table **[code]**.

### 4.5 The 20-float state vector, `_getDroneStateVector()` (line 540) **[code]**
```
[0:3]   pos xyz            m, world
[3:7]   quaternion x,y,z,w body -> world
[7:10]  roll, pitch, yaw   rad
[10:13] linear velocity    m/s, world
[13:16] angular velocity   rad/s, world frame (not body)
[16:20] last action        RPM
```
- `obs` has shape **(NUM_DRONES, 20)**: one row per drone. `obs[0][2]` = drone 0's z. With 1 drone it is still 2-D, `(1,20)`, so the code works for any N. The action is `(N,4)` for the same reason.
- The project's `FlightLog` (`modular_uav/utils/flight_log.py`) uses this layout for both backends, but logs **body-frame** rates in `[13:16]`.

### 4.6 Quaternions
- A 4-number encoding of a 3-D rotation. Physics engines use it because Euler angles suffer gimbal lock at ±90° pitch and compose badly. `[7:10]` gives the same orientation as roll/pitch/yaw.
- **xyzw** is PyBullet's storage order (scalar last). **MuJoCo uses wxyz** (scalar first). Mixing them up gives a silently wrong rotation. The `Sim` interface exposes `quat_xyzw(i)` on both backends.
- "Body → world" means it rotates a vector written in the drone's frame into the world frame:
  ```python
  R = np.array(p.getMatrixFromQuaternion(obs[0][3:7])).reshape(3,3)
  thrust_world = R @ [0, 0, 1]
  ```

### 4.7 Numbers from a first hover experiment **[measured]**
- CF2X params as loaded: m 0.027 kg, arm 0.0397 m, Ixx = Iyy 1.4e-5, Izz 2.17e-5 kg·m², kf 3.16e-10, km 7.94e-12, thrust/weight 2.25, prop radius 0.0231 m.
- Hover RPM = √(m·g / (4·KF)) = √(0.027·9.8 / (4·3.16e-10)) ≈ **14468**. Matches `env.HOVER_RPM`.
- The drone spawns at z = `COLLISION_H/2 − COLLISION_Z_OFFSET + 0.1` ≈ **0.1125 m** (line 196) and stays exactly there at hover RPM in `Physics.PYB`: thrust = weight, no ground effect, so net force is 0.
- Pitfall: 48 steps at `CtrlAviary`'s default 240 Hz is **0.2 s**, not 1 s. Use `range(env.CTRL_FREQ)` for 1 s.

---

## 5. Open loop vs feedback, and sync()

### 5.1 Why an open-loop drone keeps climbing
- At 1.02× hover RPM for 1 s it accelerates upward. Back at hover RPM, net force is 0, so **acceleration** is 0, but the velocity it already has continues (no drag in ideal mode, apart from PyBullet's default damping, 13.1). Newton's first law.
- **"Hover RPM" means "hold the current velocity"**, not "stay put".
- Thrust ∝ rpm², so 1.02× RPM is 1.04× thrust. To scale thrust, scale RPM by `sqrt`.

### 5.2 The open-loop fix and its limit **[measured]**
+2 % thrust for 1 s, then −2 % for 1 s, then hover (CF2X, `Physics.PYB`, 240 Hz):
```
t=1s z=0.209 vz=0.192   climbing
t=2s z=0.300 vz=-0.008  braked, almost stopped
t=3s z=0.292 vz=-0.008
t=4s z=0.284 vz=-0.008  slowly sinking
```
- The leftover −8 mm/s comes from asymmetric timing and discretisation **[assumption: not proven; a step-halving test would confirm]**.
- Open-loop commands accumulate every small error and never correct it. Feedback control measures and pushes back.
- The script (`sandbox/manual_step.py`) was dropped in the restructure.

### 5.3 `sync()` (`gym_pybullet_drones/utils/utils.py:10`, copied to `modular_uav/utils/utils.py`) **[code]**
```python
def sync(i, start_time, timestep):
    if timestep > .04 or i % int(1/(24*timestep)) == 0:
        elapsed = time.time() - start_time
        if elapsed < i*timestep:
            time.sleep(timestep*i - elapsed)
```
- Problem: one step takes about 0.01 ms to compute but represents 4.2 ms of sim time, so a GUI run would finish in a blink.
- It compares against an **absolute target** (`i*timestep` since start), not "sleep one timestep each loop". Errors don't add up, and a slow step makes the next sleep shorter.
- It only checks about **24 times per second** (every `1/(24·dt)` steps; every 10th step at 240 Hz), because `time.sleep` is imprecise for tiny durations. When control runs slower than 25 Hz, it checks every step.
- If the computer is slower than real time, it never sleeps; it just runs slow.
- **It is for watching only.** It has no effect on physics. The PyBullet backend calls it only when `gui: true`. The MuJoCo backend paces in `CagedAviary.step` instead, with the same absolute-target logic, every step.

---

## 6. Choosing frequencies

### 6.1 Benchmark **[measured]** (PyBullet, 4 drones, 5 s sim, hover RPM)

| Physics / control | × real time |
|---|---|
| 240 / 48 Hz | **101×** |
| 1000 / 200 Hz | 26× |
| 5000 / 1000 Hz | 5× |

Cost is linear in frequency.

### 6.2 Zero-order hold (ZOH)
A digital controller outputs a value at discrete ticks. Between ticks the last command is **held constant**, like a staircase. "Zero-order" means a degree-0 (constant) polynomial between samples; a first-order hold would ramp. Real flight controllers and ESCs work this way. In PyBullet it's the loop that reuses `clipped_action` for `PYB_STEPS_PER_CTRL` ticks. In `task: hover` the RPM is held for `freq / hover_ctrl_freq` physics steps.

### 6.3 How to choose
- **Physics rate: set by the fastest dynamics.** For a 27 g Crazyflie that's roll/pitch (tens of ms). 240 Hz ≈ 4 ms steps resolves it. 5 kHz is slightly more precise but changes almost nothing, at 20× the cost.
- **Control rate: copy the real hardware.** Crazyflie position loop ≈ 100 Hz, attitude 250–500 Hz, and motors lag by tens of ms **[assumption: typical firmware values]**. A 1 kHz sim controller relies on reaction speed the real robot doesn't have.
- **Ratio:** 3–5× is a sensible default. Ratio 1 just steps the integrator. The dock task runs ratio 1 (`freq` = physics = control).
- **When to raise it:** contacts and stiff joints, e.g. **docking** (500–1000 Hz may be needed). Heavier structures are slower, so they're easier.
- **Convergence rule:** double the frequency. If the metric doesn't change, the frequency was high enough.
- **Why speed matters:** shape search and allocation sweeps will run thousands of episodes. 100× vs 5× is overnight vs weeks. RL needs millions of steps.

---

## 7. Physics modes

### 7.1 The modes **[code]**

| Field `physics` | PyBullet enum | MuJoCo enum | Adds | Model |
|---|---|---|---|---|
| `base` | `PYB` | `MJC` | nothing (ideal rotors) | thrust ∝ rpm² |
| (not exposed) | `DYN` | `DYN` | **replaces** the engine with hand-written Newton–Euler | explicit integration, **no collisions or constraints** |
| `gnd` | `PYB_GND` | `MJC_GND` | ground effect | per-prop thrust × (1 + c·(r/4h)²) below a height |
| `drag` | `PYB_DRAG` | `MJC_DRAG` | rotor drag | ∝ Σ rotor speed × velocity |
| `dw` | `PYB_DW` | `MJC_DW` | downwash from drones above | Gaussian in horizontal offset, decays with vertical gap |
| `gnd_drag_dw` | `PYB_GND_DRAG_DW` | `MJC_GND_DRAG_DW` | all three | sum |

`DYN` is not a `physics` choice in `simulate.py` because it cannot dock (7.3, item 5). Downwash under `dw` / `gnd_drag_dw` is further selected by `downwash_model` (10.4).

### 7.2 Cost **[measured]** (PyBullet, 4 drones, 240/48 Hz, original downwash)

| Mode | × real time |
|---|---|
| pyb | 104× |
| pyb_drag | 48× |
| pyb_dw | 42× |
| pyb_gnd | 40× |
| dyn | 21× |
| pyb_gnd_drag_dw | 24× |

### 7.3 Why not always use everything
1. **Cost:** about 4× slower.
2. **Attribution:** if a dock fails with every effect on, the cause is unclear. The fidelity ladder exists for this: L0 = `base`, L1 = add effects one at a time and measure each.
3. **Models are fitted to a single Crazyflie.** The thesis modules aren't Crazyflies. "All features" doesn't mean "realistic"; a model can be confidently wrong.
4. **Irrelevant effects:** ground effect only matters near the floor (for a CF2, below about 10–20 cm; see 10.2). At cruise altitude it computes almost zero, just more slowly.
5. **`dyn` can't do collisions or constraints**, so it can't dock. Use it only to cross-check the integrator.

### 7.4 Mapping to the thesis pipeline
- Stages 1–5 (planning): no sim, or `base`.
- Stage 6 (docking): `base` (L0 magic glue), then `dw`. Downwash matters when approaching from above.
- Stage 7 (flight): `base` → `dw` → `gnd_drag_dw`, reported step by step.
- The dock scenarios currently default to `gnd_drag_dw` with `regularised` downwash; see `docs/dock.md`.

---

## 8. The DSL PID controller

File: `gym-pybullet-drones/gym_pybullet_drones/control/DSLPIDControl.py`.

### 8.1 PID in one paragraph
`e = target − current`. Command = **P·e** (push in proportion to the error) + **I·∫e** (removes steady offsets, e.g. a wrong mass) + **D·ė** (brakes before overshooting). Without D it oscillates. Without I it settles slightly off.

### 8.2 Why a cascade
A quadrotor's props all push along its own up-axis. It has no sideways thrusters. To go right it must **tilt** right. So the controller works in order:
1. **Outer loop (position): what total force is needed?** One 3-D vector, e.g. "up to hold the weight, plus a bit right".
2. Split the vector: **its length** → total thrust; **its direction** → how much to tilt.
3. **Inner loop (attitude): what torque reaches that tilt?** → 3 torques.
4. **Mixer: which motors give that total + torque?** e.g. speed up the left motors to roll right.

The inner loop must be faster than the outer one: the drone can't move right before it has tilted.

```
target_pos -> [position PID] -> desired thrust vector (world, N)
                 magnitude -> collective thrust
                 direction -> desired roll/pitch
              -> [attitude PID] -> 3 torques -> [mixer] -> 4 PWM -> 4 RPM
```

### 8.3 Position loop, `_dslPIDPositionControl` (lines 156–217) **[code]**
```python
pos_e = target_pos - cur_pos ; vel_e = target_vel - cur_vel
target_thrust = P*pos_e + I*∫pos_e + D*vel_e + [0, 0, m*g]     # lines 201–203
```
- Gains (lines 41–43): P = [.4,.4,1.25], I = [.05,.05,.05], D = [.2,.2,.5].
- The output is a **world-frame force (N)**. `+[0,0,mg]` is **gravity feedforward**: the PID only corrects around hover.
- D uses the velocity error, which is measured directly, so it's cleaner than differencing position.
- `scalar_thrust = dot(target_thrust, current body z)`: only the component along where the drone actually points counts.
- The direction becomes the desired body z. Body x is built to match the desired yaw via cross products, then converted to target roll/pitch/yaw.

### 8.4 Anti-windup
- The I term accumulates error over time. If the drone sits on the ground, is blocked, or the motors saturate, the error stays large and the integral grows huge ("winds up"). Once the drone is free, that stored integral keeps pushing and causes a big overshoot.
- Fix: **clip the integral** (lines 198–199). `clip(∫e, −2, 2)`, and **±0.15 for z**, tighter because a vertical overshoot hits the floor or ceiling.

### 8.5 Attitude loop, `_dslPIDAttitudeControl` (lines 219–270) **[code]**
```python
rot_matrix_e = R_target^T R_cur - R_cur^T R_target
rot_e = [e[2,1], e[0,2], e[1,0]]          # vee map: skew matrix -> 3-vector
target_torques = -P*rot_e + D*rate_e + I*∫rot_e      # clipped to ±3200 (line 267)
```
- The error is computed on rotation matrices, not by subtracting Euler angles, so it has no ±180° wraparound problem. This is the geometric controller of Lee et al. 2010 **[assumption: citation from memory]**.
- Gains (lines 44–46): P_TOR = [70000, 70000, 60000], I_TOR = [0, 0, 500], D_TOR = [20000, 20000, 12000]. They're large because these torques are in **PWM units**, Crazyflie-firmware style.

### 8.6 Mixer and PWM (lines 268–270)
```python
pwm = thrust + MIXER_MATRIX @ [tau_roll, tau_pitch, tau_yaw]
rpm = 0.2685*pwm + 4070.3          # PWM clipped to [20000, 65535]
```
CF2X mixer rows (roll, pitch, yaw): m0 `[-.5,-.5,-1]`, m1 `[-.5,.5,1]`, m2 `[.5,.5,-1]`, m3 `[.5,-.5,1]`. CF2P: `[0,-1,-1]`, `[1,0,1]`, `[0,1,-1]`, `[-1,0,1]`. PWM→RPM is a linear fit of the real Crazyflie motors.

**Thesis link:** the mixer is a hard-coded 4×3 inverse of the geometry. For k rotors at arbitrary positions, replace it with **B (6×k)** and solve `B u = wrench`, `0 ≤ u ≤ ū`. That is `A(C,P) = {Bu}` in the formulation. **The fly stage = this cascade with the mixer replaced by allocation.**

### 8.7 Gotchas **[code]**
- It raises an error unless the model is CF2X or CF2P (line 40). The gains are tuned for 27 g.
- It reads mass from the bare `cf2x.urdf` itself, not from the env. With a caged module (30 g) its gravity feedforward would be 10 % low. The project's `Sim.stock_pid()` therefore overwrites `ctrl.GRAVITY` with the env's caged weight and leaves the gains untouched (`sims/pybullet_sim.py`).
- `cur_ang_vel` is unused. Rates come from differencing Euler angles (noisy).
- `computeControlFromState` (`BaseControl.py:55`) just slices the 20-float state.

### 8.8 Verified hover **[measured]** (bare CF2X, target z = 1 m, control 48 Hz)
```
t=0.5 z=0.724 vz=0.828
t=1.0 z=0.941 vz=0.192
t=2.0 z=1.003
t=3.0 z=1.006 vz=0.000 rpm=14467   <- found hover RPM by itself
t=6.0 z=1.005
```
A small steady offset (+5 mm) remains because the z integral is tiny (I = 0.05, clip 0.15).

### 8.9 Suggested experiments (not run)
- Set D_z to 0 → overshoot and oscillation.
- Set I to 0 → steady offset.
- Target `[1,0,1]` and print roll/pitch → it tilts first, then moves.

---

## 9. Drone models and the caged module

### 9.1 Names
- **DSL = Dynamic Systems Lab**, Angela Schoellig's lab at the University of Toronto Institute for Aerospace Studies (UTIAS). They wrote gym-pybullet-drones and the controller. The repo has since moved to the `learnsyslab` GitHub organisation.
- **Attitude = orientation** (roll, pitch, yaw). Position = where; attitude = which way it's rotated.

### 9.2 Stock models (`utils/enums.py`, URDFs) **[code]**

| | CF2X | CF2P | RACE |
|---|---|---|---|
| Real thing | Crazyflie 2.0, X | Crazyflie 2.0, + | generic racer |
| Mass | 27 g | 27 g | 830 g (PyBullet), 250 g (MuJoCo) |
| Arm | 3.97 cm | 3.97 cm | 10.9 cm |
| kf | 3.16e-10 | same | 8.47e-9 |
| Thrust/weight | 2.25 | same | 4.17 |
| Prop radius | 2.3 cm | same | 12.7 cm |
| DSLPIDControl | yes | yes | no (raises) |

- **X vs +:** same drone, "front" rotated by 45°. X: roll uses all 4 motors (±0.5 in the mixer). +: roll uses 2 motors (0, ±1).
- **RACE copies CF2's drag/gnd/dw coefficients exactly.** Nobody measured them, so treat them as placeholders. RACE values in the table are PyBullet's except where noted.

### 9.3 What a model is made of
In PyBullet everything is in one URDF (`<properties>` + mass, inertia, geometry), parsed by `_parseURDFParameters()`. Upstream, a new model = a new URDF + one enum entry. In MuJoCo the same numbers live in `DRONE_PARAMS` (11.2, item 5). The project avoids new enum entries: it keeps `CF2X` and changes the parameters through the hooks in 1.2.

| Parameter | Type | How to get it |
|---|---|---|
| Mass | measure | scale, or sum of parts in CAD |
| Inertia | derive | CAD export; check with a pendulum test if needed |
| Arm / rotor positions | derive | CAD |
| Collision shape | derive | box or cylinder around the frame |
| kf (thrust/rpm²) | measure | thrust stand, fit a parabola; or the prop datasheet |
| km (torque/rpm²) | measure | stand with a torque sensor; km/kf ≈ 0.01–0.03 m **[assumption]** |
| Max RPM / thrust-to-weight | measure | stand at full throttle and voltage |
| Prop radius | derive | read it off the prop |
| gnd_eff_coeff | measure / literature | hover at known heights |
| Drag coefficients | measure | fly at known speeds, or wind tunnel |
| Downwash coefficients | measure, **hard** | one drone under another at many offsets |
| PWM→RPM | measure | tachometer sweep; Crazyflie-specific |
| Motor lag | measure | **not modelled at all** (RPM changes instantly) |

### 9.4 Generic vs tuned to CF2
- **Generic (reusable and realistic):** rigid-body dynamics, the rpm² propeller law (only the coefficients change), gravity, collisions, constraints (docking), the *form* of the ground-effect equation, the cascade *structure*, the geometric attitude error.
- **CF2-specific (redo for a new drone):** all PID gains, the mixer, PWM↔RPM and PWM limits, the downwash model (form and coefficients), drag coefficients. `HOVER_RPM` and `MAX_RPM` update automatically if the mass and kf are right.

### 9.5 The caged CF2X (`modular_uav/models/cage.py`) **[code]**
- A stock CF2X with thin rods along the edges of a box, a regular tetrahedron, a triangular or hexagonal prism, or a regular dodecahedron, rigidly fixed to the frame and centred on the CoM. Level: L0 rigid frame. Body frame: x forward, z up, origin at the CoM.
- **Only mass and inertia change.** kf, km, drag, downwash coefficients and the stock PID gains stay the CF2X's. Inertia = CF2X (Forster 2015) + the rods as uniform thin rods, mass split by length. Connectors at the vertices are not modelled.
- Field `cage: box | tetra | triangle | hex | dodeca | none`. Defaults: box 120 × 120 × 50 mm (`box_size`), tetra 310 mm edges (`tetra_edge`, apex up, one base vertex on +x), triangle prism 230 mm edges (`triangle_edge`, one vertex on +y) and hex prism 70 mm edges (`hex_edge`, flat faces on ±x), both 50 mm tall (`prism_height`), dodecahedron 60 mm edges (`dodeca_edge`, two opposite vertices on z as in the Dodecacopter, Garanger et al. 2026), cage mass 3 g (`cage_mass`), rod radius 1 mm (`cage_rod_radius`). Each size is the smallest (rounded up to the cm) that clears the CF2 mesh by at least `cage_clearance` = 5 mm (box clears by 5.3 mm, tetra by 7.5 mm, triangle by 6.5 mm, hex by 5.9 mm, dodeca by 11.6 mm). A dodeca module may be turned by any of the dodecahedron's 60 symmetries (`tilt`); since the cage maps onto itself, only the CF2X and its rotor axis turn.
- Resulting module **[computed, defaults]**:

  | `cage` | Rods | Mass | Ixx = Iyy | Izz | Lowest point below CoM |
  |---|---|---|---|---|---|
  | none | 0 | 27 g | 1.400e-5 kg·m² | 2.170e-5 kg·m² | 13 mm (mesh) |
  | box | 12 | 30 g | 2.348e-5 kg·m² | 3.734e-5 kg·m² | 25 mm |
  | tetra | 6 | 30 g | 5.404e-5 kg·m² | 6.174e-5 kg·m² | 63 mm |
  | triangle | 9 | 30 g | 3.027e-5 kg·m² | 5.074e-5 kg·m² | 25 mm |
  | hex | 18 | 30 g | 2.199e-5 kg·m² | 3.459e-5 kg·m² | 25 mm |
  | dodeca | 30 | 30 g | 2.694e-5 kg·m² | 3.464e-5 kg·m² | 84 mm (a vertex) |

- Hover RPM of a 30 g module = √(0.030·9.8 / (4·3.16e-10)) ≈ **15 250** **[computed]**, about 5 % above the bare CF2X.
- PyBullet: one URDF per module type, written to the system temp folder (`modular_uav/cf2x_<type>_<hash>.urdf`), cached by content. The upstream `cf2.dae` renders black in PyBullet (its materials are ignored), so the generated URDF draws the CF2X from the seven coloured CF2 meshes MuJoCo-drones-gym ships, with MuJoCo's material colours: both simulators show the same drone, and the cage (field `cage_color`, by default one colour per module type, amber first) stands out against it. The first mesh replaces the dae; the other six and the rods go after `base_link`'s own collision element, because upstream `_parseURDFParameters` reads children by index. An assertion fails loudly if the upstream `cf2x.urdf` inertial or visual block changes.
- **Module types and Froude scaling [code, `models/module.py`, `sims/scaling.py`]:** a drone of scale k has lengths × k (arm, propeller, meshes, rotor positions, collision cylinder), mass × k³, inertia × k⁵, kf × k⁴, km × k⁵, drag coefficient × k³, downwash `DW_COEFF_1` × k³ and `DW_COEFF_3` × k, ground-effect clip height × k, and MAX_RPM from its own module weight (× k^-1/2 for the bare drone). Thrust-to-weight stays 2.25 and every motion is dynamically similar with time × √k. Both upstream envs keep one set of drone coefficients and read them inside per-drone methods, so a mixin swaps each drone's values in around `_physics`, `_groundEffect` and `_drag`, and clips each drone's RPM to its own MAX_RPM. At k = 1 every value is the upstream one.
- MuJoCo: rods are capsule geoms inserted before each `drone<i>_center` site of the upstream MJCF; an assertion guards that anchor.

### 9.6 Plan for the thesis modules
- Reuse: URDF/MJCF format, rigid-body physics, the rpm² law, the ground-effect form, the cascade structure, fixed-joint docking.
- Derive: mass, inertia and rotor placement from CAD, per module and per assembled structure.
- Cite or measure: kf, km and max thrust for a real motor+prop, e.g. from public thrust-stand data **[assumption]**.
- Can't cheaply get: inter-module downwash. L0/L1 use the fitted CF2 model (original or regularised), flagged as a gap; L2 = learnt downwash.
- Must write new: a general **B (6×k) allocation** controller, and gains scaled to mass and inertia.

---

## 10. Aerodynamic models in code

File: `BaseAviary.py`, lines 712–811 (`_groundEffect` 712, `_drag` 751, `_downwash` 782). Each one adds an `applyExternalForce` on top of the base thrust. MuJoCo ports the same equations (11.2).

### 10.1 How empirical are they?
All three are a physics-shaped formula with coefficients fitted to Crazyflie experiments. None is generic theory.

| Effect | Shape from theory? | Coefficients |
|---|---|---|
| Ground effect | mostly: Cheeseman–Bennett (1955) `T_IGE/T = 1/(1−(R/4z)²)`; the code uses the small-term form `(R/4z)²` | `gnd_eff_coeff = 11.37`, fitted (theory ≈ 1) |
| Drag | partly: rotor drag ∝ rotor speed × body velocity (blade flapping / induced drag) | fitted (Forster 2015, ETH system ID) |
| Downwash | barely: a Gaussian bump chosen to fit data | all 3 fitted |

The citations (Cheeseman–Bennett, Forster 2015, Shi et al. "Neural Lander") are **[assumption: from memory]**. Verify before citing.

### 10.2 Ground effect
```python
h = clip(z of each prop link, GND_EFF_H_CLIP, inf)
gnd = KF*rpm^2 * GND_EFF_COEFF * (PROP_RADIUS/(4*h))^2     # per prop, local +z
```
- Per prop, so a tilted drone near the floor gets a restoring roll torque (realistic). MuJoCo uses the CoM height for all 4 props instead (11.2).
- Falls off as 1/h². For a CF2 (R = 2.31 cm, coeff 11.37) **[computed]**: 26 % extra at the 3.8 cm height clip, 15 % at 5 cm, 3.8 % at 10 cm, 0.95 % at 20 cm, 0.15 % at 50 cm.
- The fitted coefficient (11.37) makes the CF2 ground effect much stronger than plain theory (about 6× the "2.4 % at 5 cm" a theory-sized coefficient gives). It matters for the first ~10 cm: takeoff, landing, and modules docking near the floor.
- Off when roll or pitch > 90°. Bigger props reach proportionally higher.

### 10.3 Drag
```python
drag = R^T * (-DRAG_COEFF * Σ(rotor rad/s) * v_world)       # at the centre link
```
- Rotor drag, linear in v, scaled by prop speed. Realistic only at low speed; quadratic body drag is **missing**. It uses the previous step's RPM (negligible lag).

### 10.4 Downwash
Upstream (`downwash_model: original`):
```python
for each other drone above (delta_z > 0, delta_xy < 10 m):
    alpha = DW1 * (R/(4*delta_z))^2
    beta  = DW2*delta_z + DW3            # 0.16*dz - 0.11
    F_z   = -alpha * exp(-0.5*(delta_xy/beta)^2)
```
- Picture: a widening cone of pushed-down air, strongest directly below.
- Limits:
  - Only pushes down at the CoM, with **no torque**. Real partial coverage tips the drone, and that's what makes docking from above hard.
  - Ignores the upper drone's RPM.
  - `beta` is **negative below 0.69 m** of vertical gap, so the plume width is meaningless at close range, which is exactly docking range.
  - Written as world −z but applied in `LINK_FRAME` in PyBullet (wrong direction when tilted, a small bug).
  - One coefficient set (same drone type only).
  - **Singular as delta_z → 0** (17.1).

Project version (`downwash_model: regularised`, the default; `modular_uav/physics/downwash.py`) **[code]**:
```
F = min(alpha, T) * exp(-0.5 (delta_xy / beta')^2) * min(delta_z / ramp, 1),   beta' = max(beta, w)
```
- `dw_thrust_cap: true`: the peak is capped at the upper module's thrust `T = KF Σ rpm²` (N); the jet's momentum flux equals T.
- `dw_dz_ramp: 0.05` (m): the force fades in linearly over the first 5 cm below the upper rotor plane; 0 disables the ramp.
- `dw_min_width: null` (m): `beta` is floored at the arm length `L`; 0 disables the floor.
- Same applicability test as upstream (`delta_z > 0`, `delta_xy < 10 m`). Both backends apply it as a downward force at the CoM; PyBullet still uses `LINK_FRAME` (same frame caveat as upstream).
- The same function draws the live isolines (`plume` field). Findings and results: `docs/dock.md`.

### 10.5 `dyn` mode (line 812)
It computes `ω̇ = J⁻¹(τ − ω×Jω)` and `v̇ = F/m − g`, integrates with explicit Euler, then teleports the body with `resetBasePositionAndOrientation`. No collisions or constraints. Use it only as a textbook cross-check.

---

## 11. MuJoCo-drones-gym overview

Package `multi_drone_mujoco`. Core file: `envs/base_aviary.py` (1168 lines). It is **a near line-by-line port of gym-pybullet-drones**.

### 11.1 What carries over unchanged

| Concept | PyBullet | MuJoCo repo |
|---|---|---|
| Env class | `BaseAviary` + hooks | same names, same template |
| Clocks | `pyb_freq`, `PYB_STEPS_PER_CTRL` | `sim_freq`, `SIM_STEPS_PER_CTRL` (line 352, must divide) |
| Step loop | ZOH, substeps, per-drone physics | identical (`step()`, line 528) |
| Action | RPM `(N,4)` | RPM, flat `(4N,)` also accepted |
| State | 20 floats | 20 floats, same layout, but quaternion wxyz and free-joint angular velocity in the **body** frame |
| Rotor law, coefficients | KF·rpm², CF2 numbers | same |
| Physics modes | `PYB*`, `DYN` | `MJC*`, `DYN`, same equations and coefficients |
| Cascade PID | `DSLPIDControl` | `PIDControl`, `DSLPIDControl` (re-tuned) |
| `sync` | `utils.sync` | none upstream; the project's `CagedAviary.step` paces when `gui: true` |

### 11.2 Differences (gotchas)
1. **Quaternion order is wxyz.** `obs[i][3:7] = [w,x,y,z]`. Label variables `quat_wxyz` / `quat_xyzw` and convert at boundaries. Level drone: MuJoCo `[1,0,0,0]` vs PyBullet `[0,0,0,1]`.
2. **MuJoCo's built-in fluid drag is on**: `<option integrator="RK4" density="1.225" viscosity="1.8e-5">` (line 220). See 13.1.
3. **Integrator: RK4.** See 13.2.
4. **Forces are applied at the body centre** via `xfrc_applied`. Roll/pitch torque is computed by hand (`±F·L/√2`). Props are marker `site`s, not bodies. Ground effect uses CoM height for all 4 props (no tilt asymmetry). Downwash is applied in the world frame (avoids PyBullet's frame bug). See 13.3.
5. **No URDF.** Parameters live in the `DRONE_PARAMS` dict (`base_aviary.py:34`); `_generate_aviary_xml` (line 101) builds the MJCF as an f-string. That's just a different input format. RACE here is **250 g** vs PyBullet's **830 g**, so never compare RACE across simulators.
6. **The viewer needs `render()`.** Upstream `step()` doesn't draw, so `gui=True` alone opens nothing that updates. The project's `CagedAviary.step` calls `render()` each step when `render_mode == "human"`. **[measured, on the earlier fork patch with the same logic]** sim time matched wall time second by second (t = 9 s at 10 s wall); headless runs are unaffected.
7. **The controller takes the env** (13.4).
8. **Docking needs a different mechanism** (section 14).

### 11.3 Test runs **[measured]** (CF2X, 240/48 Hz, `MJC`)
- `PIDControl` → target [0.5,0,1], after 6 s: [0.539, 0, 0.999].
- `DSLPIDControl` → [0.559, 0, 1.006].
- The xy error is probably slow convergence; only 6 s were run **[assumption]**.
- Example `pid.py`: hover settles at z = 1.016 (height error 0.0165 m); velocity tracking reaches 4/4 waypoints; the 3-drone part is broken (17.1).

### 11.4 Speed **[measured]** (4 drones, 5 s)

| Setting | MuJoCo | PyBullet |
|---|---|---|
| 240/48 ideal | 65× | 101× |
| 240/48 all effects | 21× | 24× |
| 1000/200 ideal | 15× | 26× |
| 1000/200 all effects | 5× | — |

Despite the README, it's **slower here**. The engine is fast, but the step loop is per-drone Python with `mj_name2id` lookups every substep **[code]**, and RK4 does 4 force evaluations per step.

---

## 12. File map of both repos

### 12.1 PyBullet `assets/`

| File | Purpose |
|---|---|
| `cf2x.urdf`, `cf2p.urdf` | Crazyflie X / +: mass, inertia, prop links, all coefficients |
| `racer.urdf` | 830 g racer (aero coefficients copied from CF2) |
| `cf2.dae` | Crazyflie mesh (visual) |
| `box.urdf` | box used only by `examples/debug.py` |
| `architrave.urdf` | decorative structure, not referenced by any Python |
| `beta-presets-bak.txt`, `eeprom.bin` | Betaflight SITL config / saved settings |
| `beta-traj.csv` | trajectory (pos/vel/acc/jerk) for `beta.py` |
| `clone_bfs.sh` | builds N Betaflight SITL binaries |
| `ffmpeg_png2mp4.sh` | frames → mp4 |
| `helix.*`, `marl.gif`, `rl.gif` | README images |

The caged URDF is not here: it is generated at run time (9.5).

### 12.2 PyBullet `envs/`

| File | Purpose |
|---|---|
| `BaseAviary.py` | the engine: setup, step loop, physics modes, state, render, video |
| `CtrlAviary.py` | control work: RPM in, 20-float state out (the base of the project's `CagedAviary`) |
| `VelocityAviary.py` | velocity commands, internal PID; for high-level planners |
| `BaseRLAviary.py` | RL base: action types, action buffer, kin/RGB obs, optional obstacles |
| `HoverAviary.py` | RL: one drone to [0,0,1] |
| `MultiHoverAviary.py` | RL: leader–follower heights |
| `BetaAviary.py` | **real Betaflight firmware** in SITL over UDP (firmware-in-the-loop) |

### 12.3 PyBullet `control/` (details in section 15)

| File | Purpose |
|---|---|
| `BaseControl.py` | interface + URDF reader; `computeControlFromState` → `computeControl` |
| `DSLPIDControl.py` | cascade PID (CF2 only) |
| `CTBRControl.py` | collective thrust + body rates (for Betaflight) |
| `MRACControl.py` | model reference adaptive control |

### 12.4 PyBullet `examples/`

| File | Purpose |
|---|---|
| `pid.py` | N drones circling with DSLPID |
| `pid_velocity.py` | velocity commands via `VelocityAviary` |
| `downwash.py` | one drone passes over another |
| `learn.py` / `play.py` | train PPO / fly a trained policy |
| `beta.py` | Betaflight SITL flying `beta-traj.csv` |
| `mrac.py` | MRAC demo |
| `debug.py` | raw PyBullet scratch: two weightless boxes for axis checks |

### 12.5 File-to-file comparison

| PyBullet | MuJoCo | Note |
|---|---|---|
| `assets/*.urdf` | `assets/cf2/*.obj` + `DRONE_PARAMS` | params in Python; RACE differs |
| `BaseAviary.py` | `envs/base_aviary.py` | same structure; wxyz, fluid on, `xfrc_applied` at CoM, RK4, wind hook, built-in action types |
| `CtrlAviary.py` | — | `BaseAviary(act_type=RPM)` plays this role |
| `VelocityAviary.py` | `velocity_aviary.py` | MuJoCo's is an RL task |
| `BaseRLAviary.py` | — | absorbed into `base_aviary.py` |
| `HoverAviary.py` | `hover_aviary.py` | 1:1 port |
| `MultiHoverAviary.py` | `multi_hover_aviary.py` | each drone to its own height |
| `BetaAviary.py` | — | no firmware-in-the-loop |
| — | `formation_aviary.py`, `race_aviary.py`, `fly_through_aviary.py` | extra RL tasks |
| — | `multi_agent_aviary.py` | PettingZoo wrapper |
| `BaseControl` + `DSLPIDControl` | `pid_control.py` + `dsl_pid_control.py` | MuJoCo's take `env` |
| `CTBRControl`, `MRACControl` | — | missing |
| `pid.py`, `downwash.py`, `learn.py`, `play.py` | same names | ports |
| `pid_velocity.py`, `beta.py`, `mrac.py`, `debug.py` | — | missing |
| — | `train_rl.py`, `train_all_envs.py`, `render_demos*.py` (repo root) | RL scripts, GIF makers |

**Summary:** PyBullet is broader in **control** (firmware SITL, CTBR, adaptive). MuJoCo is broader in **RL tooling** (tasks, wind, randomization, vectorization).

### 12.6 MuJoCo-only files
- `wrappers/__init__.py`, `DomainRandomizationWrapper`: per episode, mass ±20 %, inertia ±20 %, kf/km ±15 %, arm ±5 %, max RPM ±10 %, plus sensor noise (pos 2 mm, vel 1 cm/s, gyro/acc bias). Useful for controller robustness tests.
- `wrappers/wind.py`, `WindField`: CONSTANT, GUST, DRYDEN, SINUSOIDAL, COMBINED. `wind_wrapper.py` plugs it in. `BaseAviary.set_wind` (line 1147) is the hook.
- `wrappers/obstacles.py`: procedural FOREST, URBAN, INDOOR, GATES, RANDOM worlds → XML bodies.
- `wrappers/curriculum.py`: RL difficulty scaling.
- `rendering/shared.py`, `SharedStaticRenderer`: one GL context renders cameras for many envs.
- `vec/shared_render_vec.py`, `SharedRenderVecEnv`: SB3 VecEnv, N envs over M processes.
- `vectorized/__init__.py`, `MJXVectorAviary`: GPU batch sim via MJX (`jit(vmap(step))`). Thrust only, no gnd/drag/dw **[code, skimmed]**. JAX sees **CPU only** here.
- `bench/`: render-path benchmarks (baseline, compare, calibrate, verify, env). **Not** a PyBullet-vs-MuJoCo comparison.

### 12.7 Project package `modular_uav/`
Details in `docs/architecture.md`.

| Path | Purpose |
|---|---|
| `config/fields.py` | every field (name, default, units, choices); `master.yaml` is generated from it |
| `config/loader.py` | YAML + CLI loader and validation |
| `sims/pybullet_sim.py`, `sims/mujoco_sim.py` | the two backends (1.2) |
| `models/cage.py` | caged CF2X geometry, mass properties, mesh clearance check |
| `physics/downwash.py` | original and regularised downwash, shared by both backends and the plume drawing |
| `dock/` | task `dock`: layouts, controller, mission phases, self-checks |
| `tasks/hover.py` | task `hover`: one module on the stock PID |
| `viz/plume.py` | live isolines of equal aero force |
| `utils/` | `sync`, `str2bool`, `Logger` (copied from upstream), `FlightLog` |

---

## 13. Deep dives: air, integrators, force application, APIs

### 13.1 Built-in air
- MuJoCo **[docs]**: "Proper simulation of fluid dynamics is beyond the scope of MuJoCo". There are two **stateless** models, enabled by density/viscosity > 0:
  - **Inertia model** (active here): each body becomes its equivalent inertia box. Quadratic drag `f = −2ρ r_j r_k |v| v` + viscous `−6βπ r_eq v`, plus matching rotational terms. Optional `wind` vector.
  - **Ellipsoid model** (`fluidshape="ellipsoid"` per geom): adds added mass, Magnus and Kutta lift; designed for insect flight and swimming.
- Stateless means the air never moves, so there's **no downwash, no ground effect, no rotor aerodynamics**. The hand-written models are still needed.
- PyBullet's `pyb` / `base` is not ideal either: PyBullet applies a default **linear damping** to every body (non-physical).

Hidden-drag test **[measured]**: hover RPM, start vx = 1 m/s, after 3 s:

| Setup | vx (m/s) |
|---|---|
| PyBullet `pyb` default | **0.797** |
| PyBullet, `changeDynamics(linearDamping=0, angularDamping=0)` | 1.000 |
| MuJoCo `mjc` default | **0.849** |
| MuJoCo, `model.opt.density = viscosity = 0` | 1.000 |

- Both "ideal" modes have hidden drag. PyBullet's is stronger and not physics-based.
- In MuJoCo, `MJC_DRAG` = fluid drag + Forster drag, so **drag is counted twice**.
- For a clean L0, disable it in both. For L1 in MuJoCo, the fluid drag is a real formula, but don't stack `MJC_DRAG` on top.
- Neither is disabled by the project backends today. No config field controls it yet (18).

### 13.2 RK4 vs Euler
- RK4 is 4th-order accurate for smooth motion (free flight), with much less error per step.
- It costs about 4 force evaluations per step, which is part of why MuJoCo is slower here. Its accuracy advantage disappears through contacts.
- MuJoCo **[docs]** recommends `implicit`/`implicitfast` when fluid forces are on (stability with velocity-dependent forces).
- At 240 Hz for a 27 g drone both are accurate enough, so the integrator isn't a deciding factor. Switch with `env.model.opt.integrator = mujoco.mjtIntegrator.mjINT_IMPLICITFAST`.

### 13.3 Force at the CoM vs at the props
- For one rigid body, forces at 4 points = one net force + one torque about the CoM. It's **exactly equivalent**, not an approximation.
- The only real loss is per-prop ground-effect asymmetry when tilted near the floor: a few % of a few % of thrust, within ~5 cm.
- For the thesis it matters little: a B-matrix force application can apply each rotor force at its own site, e.g. `mj_applyFT` at each rotor site.

### 13.4 "The controller takes the env"
- PyBullet: `DSLPIDControl(drone_model=DroneModel.CF2X)`. The controller opens `cf2x.urdf` itself to read mass and kf. It never sees the env (hence the `GRAVITY` override in 8.7). It offers `computeControlFromState(state, target)`.
- MuJoCo: `PIDControl(env)`. It reads `env.M`, `env.KF`, … from the env object, so it sees the caged mass with no override. The caller passes the pieces: `computeControl(dt, env.pos[0], env.quat[0], env.vel[0], env.ang_v[0], target)`.
- The MuJoCo way suits randomized or modular params: the controller sees the actual values.

---

## 14. Docking: constraints, welds and recompile

### 14.1 PyBullet
`p.createConstraint(...)` adds a fixed joint between any two bodies at any time; `removeConstraint` undoes it. Fully dynamic. The project's `Sim.snap()` creates one `JOINT_FIXED` constraint per docked pair at the snap **[code]**.

### 14.2 MuJoCo: pre-declared welds
The model is compiled and fixed. The classic trick is to declare a `<weld>` equality for each possible pair or slot, inactive, then toggle `data.eq_active[i] = 1`.

The project's MuJoCo backend does this: the model is rebuilt with one inactive `<weld>` per docked pair of the chosen layout (`relpose` = module j's pose in module i's frame, `solref="0.01 1"`), and `Sim.snap()` activates all of them at once **[code]**. Only the welds the layout needs are declared, so the count stays small.

Benchmark **[measured]** (36 free boxes, 240 Hz, `MjSpec`; `spec.memory = 256 MB` set for ≥ 3600 welds):

| Welds declared | Compile | µs/step, all off | µs/step, 35 on | × real time | Model |
|---|---|---|---|---|---|
| 0 | 2 ms | 10 | — | ~400× | 0.1 MB |
| 36 | 1 ms | 11 | 43 | ~390× | 0.1 MB |
| 360 | 1 ms | 11 | 44 | ~380× | 0.2 MB |
| 3,600 | 3–4 ms | 13–14 | 45–53 | ~300× | 0.8 MB |
| 36,000 | 30 ms | 47 | 74 | ~90× | 6.6 MB |
| 360,000 | 0.27 s | 325 | 328 | ~13× | 66 MB |

- Inactive welds aren't free: about **0.9 µs per 1000 welds per step**, linear (all are scanned).
- Active welds cost about 1 µs each (solver work).
- **Memory wall:** at ≥ 36k welds, compile **fails with "Could not allocate memory"** under default sizing, because the arena is sized for the worst case. Fix: `<size memory="256M"/>` / `spec.memory`.
- Unusable around **3–5 M welds** (≈ real time), with RAM the limit for batches **[extrapolated, not measured]**.
- These were bare boxes. Drones add fixed per-body cost, but the per-weld cost is the same.

### 14.3 MuJoCo: recompile on dock
- `model, data = spec.recompile(model, data)` **[docs]** edits the model and keeps positions and velocities (MuJoCo ≥ 3.2).
- **[measured]** About **1.5 ms per added weld** (36-box model, 1.2–2.0 ms range).
- So: pay about 1.5 ms once per dock, then every later step runs at the speed of a model with only the real welds. Docks happen tens of times per episode, so the overhead is negligible.
- Caveats: full drone models with meshes will recompile slower (guess: up to tens of ms) **[assumption]**. `model` and `data` are **new objects** after recompile, so the env must refresh its references. Upstream MuJoCo-drones-gym doesn't do this, and the project backend doesn't use recompile today.

### 14.4 Verdict
Pre-declaring is fine up to about 10k welds, and the current dock layouts need far fewer. Beyond that, or for open-ended assembly where the pairs aren't known in advance, recompile per dock. Docking doesn't disqualify MuJoCo. Its soft contacts should also make docking contact better behaved than PyBullet's **[assumption: not tested]**.

---

## 15. The controllers explained

All of them answer: given state and target, what command to send?

### 15.1 `BaseControl.py`: skeleton
- Stores `KF`, `KM`, `GRAVITY`; `computeControlFromState` slices the state; `computeControl` is empty (subclasses fill it); `setPIDCoefficients`, `_getURDFParameter`. Subclass it for a modular controller.

### 15.2 `DSLPIDControl.py`
Fixed-gain cascade PID tuned for 27 g (section 8). If the mass changes, it's wrong, and only the I term slowly compensates (the project patches `GRAVITY`, 8.7).

### 15.3 `CTBRControl.py`: Collective Thrust + Body Rates
- **CTBR** is the command language of real flight controllers (Betaflight): total thrust + desired spin rates about the drone's own axes. Think pilot sticks: throttle + "rotate this fast".
- The firmware's fast inner loop (~1–8 kHz) **[assumption]** tracks the rates and drives the motors.
- The code is only the outer half: PD on position → desired acceleration → desired attitude quaternion → `body_rates = 2·K·q_error`. Gains P = [3,3,8], D = [2.5,2.5,5], K_rates = [5,5,1] **[code]**.
- Returns `(thrust, p, q, r)`, **not RPM**. Used only by `beta.py`, which sends it over UDP to Betaflight SITL to test against the real firmware.

### 15.4 `MRACControl.py`: Model Reference Adaptive Control
- Idea:
  1. Write a **reference model**: an imaginary ideal drone that behaves as desired. That's `Xm`, simulated alongside the real one.
  2. Each step, `e = real state − ideal state`.
  3. Nudge the controller gains in the direction that shrinks `e` (`Kx += Kx_dot·dt`, `Kr += Kr_dot·dt`).
  4. The gains drift until the real drone behaves like the ideal one.
- **PID = fixed gains tuned offline. MRAC = gains that tune themselves online.**
- In code: `_compute_K` linearises around hover (12 states, 4 inputs) and uses **pole placement** (`ct.place`, poles −1…−12) for the initial gains and the reference model `Am = A − BK`. `solve_lyapunov` gives `P`, the stability guarantee for the adaptation law. `Gamma = 5e-3` are the adaptation rates.
- `mrac.py`: one drone, ground → [0,0,1], 120 Hz control, 15 s. **[measured]** It settles at **z = 1.00, xy within 1 cm**.
- `mrac.py` has **no argparse**: `--gui/--plot/--duration_sec` are silently ignored. It always opens the GUI, and at the end a plot window that blocks until it is closed. **[code + measured: a "headless" run hung at the plot]**
- **Why it matters:** each dock or payload pickup changes mass and inertia. Adaptive control answers "fly a structure whose parameters aren't known exactly" (L2: in-flight mass estimation).

### 15.5 The project's controller
The dock task uses its own geometric SE(3) controller (`modular_uav/dock/controller.py`, gains `pos_wn`, `att_wn_rp`, … in the `controller` field group), per module before the snap and on the assembly after. `task: hover` uses the stock controllers unchanged (`DSLPIDControl` in PyBullet, `PIDControl` in MuJoCo). See `docs/dock.md`.

---

## 16. Reinforcement learning in both repos

### 16.1 PettingZoo
The multi-agent version of Gymnasium: dicts of per-agent obs, actions and rewards (`env.step({"drone_0": a0, ...})`). Needed for **MARL**, where each drone learns its own policy from its own view. "Parallel" means all agents act at the same time. MuJoCo's `multi_agent_aviary.py` provides it. PyBullet has none; its multi-agent setup is one centralised policy.

### 16.2 Shared setup
stable-baselines3 **PPO**, `MlpPolicy`, kinematic state input (camera optional).

### 16.3 PyBullet **[code]**
- Tasks: `HoverAviary` (→ [0,0,1]), `MultiHoverAviary` (stacked heights, `1/(i+1)` m above start).
- Default action `ONE_D_RPM` (one number for all motors, the easiest case). Others: RPM, PID, VEL, ONE_D_PID.
- Obs = state + an **action buffer** (the last ½ s of actions).
- Reward: `max(0, 2 − dist⁴)`.
- Episode: 8 s; truncated if > 1.5 m away in xy, > 2 m high, or tilted > 0.4 rad.
- `learn.py`: up to 1e7 steps, stopping early on an eval reward threshold.

### 16.4 MuJoCo **[code]**
- Tasks: Hover, MultiHover, Velocity, FlyThrough (waypoints), Race (gates), Formation, MultiAgent (PettingZoo).
- Action: normalised [−1,1]⁴ → RPM.
- Rewards are shaped penalties: −height error, −0.1·xy error, −0.05·|v|, −0.05·tilt, a bonus near the target, **−100 on crash**. Formation: −0.5·|spacing error| per pair. Race: +20 per gate + a speed bonus. FlyThrough: +10 per waypoint.
- `train_all_envs.py`: PPO on all 6 envs, but `STEPS_PER_ENV = 5000`, so it's a **smoke test** (hover needs ~1e6+). `learn.py` is closer to a real run.

### 16.5 State
- **Neither repo contains a trained policy.** PyBullet's `examples/results/` only holds flight logs from local runs.
- PyBullet's RL is minimal but a known reference (the original paper's setup). MuJoCo's is broader and better tooled but untested.
- Thesis relevance: optional. It matters only for learnt downwash or learnt docking (L2), where randomisation, wind and PettingZoo help.

---

## 17. Bugs and caveats

### 17.1 Downwash singularity (both simulators) **[measured]**
- `alpha = DW1·(R/(4·delta_z))²` blows up as `delta_z → 0⁺`. Drones side by side at the same height: as soon as one rises a fraction of a millimetre, it counts as "above" the others. With `delta_z ≈ 1e-4 m`, alpha ≈ 2267·(0.023/0.0004)² ≈ 7.5 M N on a 27 g drone.
- MuJoCo: only `mjc_dw` / `mjc_gnd_drag_dw` with ≥ 2 drones go unstable (1 instability warning each; every other mode had 0). MuJoCo prints `Nan, Inf or huge value in QACC` and **auto-resets**, so the drones look frozen. This is why the 3-drone part of upstream `examples/pid.py` never takes off (errors stuck at 0.7/0.9/1.1).
- PyBullet `PYB_DW`, same setup: one drone reached **vz = 4.9 m/s within 5 steps**. Silent failure, no NaN check.
- **Status:** the upstream code is unchanged, so `downwash_model: original` still has this bug. The project default `downwash_model: regularised` (10.4) bounds it with the thrust cap, the `dw_dz_ramp` fade-in and the `dw_min_width` floor. Record the chosen bounds as an L0/L1 modelling choice. Findings and results: `docs/dock.md`. The upstream examples, which don't go through the project backends, are still affected.

### 17.2 Other caveats
- Double drag in MuJoCo `MJC_DRAG`; hidden damping in PyBullet `PYB` (13.1).
- `render_demos.py` / `render_demos2.py` (MuJoCo repo root) write to a hard-coded `/root/multi_drone_mujoco/demo_gifs`.
- `examples/play.py` (both) needs a trained policy, and none exist.
- PyBullet `examples/mrac.py` ignores CLI flags (no argparse), always opens the GUI, and blocks on the final plot.
- Upstream MuJoCo examples have no live viewer (11.2, item 6).
- MJX vectorized env: thrust only, and CPU-only here.
- `architrave.urdf`: unused by any Python.
- PyBullet downwash frame bug and negative `beta` (10.4).
- `DSLPIDControl` reads the bare URDF mass, not the env's (8.7).
- Running the upstream examples leaves untracked output in the submodules (e.g. `results/`, `MUJOCO_LOG.TXT`), which makes them show as modified in `git status` of `codebase/`.

---

## 18. Open questions

- **Simulator choice:** still open. Both backends run every scenario through the same interface, so the comparison can be made on the dock scenarios directly.
- **Hidden drag (13.1):** should the backends disable PyBullet's default damping and MuJoCo's fluid model for a clean L0? No field exists for this yet.
- **Docking mechanism comparison:** PyBullet `createConstraint` vs MuJoCo pre-declared welds vs `spec.recompile`, on full drone models (14.3 recompile time with meshes is an assumption).
- **Adaptive control:** MRAC vs PID with a mid-flight payload pickup, to test the L2 in-flight mass estimation idea.
- **General module model:** a URDF/MJCF module plus an N-rotor B-matrix allocation controller (8.6, 9.6).
- **Downwash at docking range:** the regularised bounds are modelling choices with no data behind them at dz < 0.69 m (10.4).
- **Not yet explored:** `beta.py` (needs Betaflight SITL via `clone_bfs.sh`), RL training runs, MuJoCo wrappers in practice.

---

## 19. Glossary

| Term | Meaning |
|---|---|
| ZOH (zero-order hold) | hold the last command constant until the next control tick |
| PID | proportional + integral + derivative feedback on an error |
| Anti-windup | clipping the integral so stored error can't cause overshoot |
| Cascade | outer loop (position) feeds the target of an inner, faster loop (attitude) |
| Attitude | orientation: roll, pitch, yaw |
| Quaternion | 4-number rotation; PyBullet xyzw, MuJoCo wxyz |
| Gravity feedforward | adding m·g so the controller only corrects around hover |
| Vee map | turning a skew-symmetric rotation-error matrix into a 3-vector |
| Mixer | fixed matrix from (thrust, torques) to per-motor commands |
| Allocation matrix B | general 6×k map from k rotor thrusts to wrench; the mixer's generalisation |
| Wrench | force + moment (6-vector) |
| CTBR | collective thrust + body rates command format |
| MRAC | model reference adaptive control: gains adapt online to match a reference model |
| Pole placement | choosing feedback gains so the closed loop has chosen eigenvalues |
| Lyapunov | energy-like function proving stability; here it shapes the adaptation law |
| SITL | software-in-the-loop: real firmware running against the simulator |
| PettingZoo / MARL | multi-agent Gym API / multi-agent RL |
| PPO | proximal policy optimization, the RL algorithm used in both repos |
| RK4 | 4th-order Runge–Kutta integrator |
| Fluid model (MuJoCo) | stateless drag/lift on bodies from density and viscosity |
| Linear damping (PyBullet) | default non-physical velocity bleed on bodies |
| Equality / weld | MuJoCo constraint gluing two bodies; can be toggled with `eq_active` |
| `spec.recompile` | MuJoCo: edit the model and recompile while keeping state |
| MJX | MuJoCo in JAX, for batched GPU simulation |
| Domain randomization | randomizing physical params per episode for robustness |
| Dryden | standard aerospace turbulence model |
| Ground effect / downwash | extra lift near the floor / downward push from a rotor above |
| Regularised downwash | the project's bounded downwash model (`downwash_model: regularised`) |
| Caged module | a CF2X inside a rigid rod cage (box, tetrahedron, triangular or hexagonal prism, dodecahedron), `modular_uav/models/cage.py` |
| Field | one run parameter: a key in a scenario YAML and a CLI flag of the same name |
| Snap | the moment docked pairs are welded ("magic glue" at L0) |
| L0 / L1 / L2 | the thesis fidelity ladder: idealised / simple physics / state of the art |
