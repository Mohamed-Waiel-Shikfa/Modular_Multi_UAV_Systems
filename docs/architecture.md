# Architecture: a declarative simulator

Every experiment is one run of one master script, `simulate.py`, described by data: a scenario YAML file, plus command-line flags. Nothing is configured by editing code, and nothing in the simulators is edited at all.

## The three rules

1. **The simulators stay upstream.** `gym-pybullet-drones/` and `MuJoCo-drones-gym/` are git submodules pinned to upstream commits (learnsyslab `f83fbdd`, tau-intelligence `ca170dc`), with no local changes. Project code builds on top of them by subclassing their envs and never patches their files. To try a newer upstream, move the submodule pointer and rerun the scenarios.
2. **Every feature is a field.** Each change or new feature (the cage, the regularised downwash, the docking layouts, the failure segment, the plume overlay, the plots, the controller gains, the pass/fail thresholds) is a field in `modular_uav/config/fields.py`. A field has a name, a default, a type, an optional list of choices, and a one-line explanation. The original simulator behaviour always stays reachable (for example `downwash_model: original`, `cage: none`, `physics: base`).
3. **One name everywhere.** A field is the same word in YAML and on the command line: `meet_z: 1.2` in a file is `--meet_z 1.2` as a flag. Precedence: **flag > scenario file > default**.

## How a run is built

```
simulate.py --scenario scenarios/box/3x3/fail.yaml --sim mujoco
   │
   ├─ config/loader.py   defaults (fields.py) ← scenario YAML ← flags; type checks, choices,
   │                     cross-field checks (layout fits cage, plume needs gui, ...)
   ├─ tasks/__init__.py  task: dock → dock/mission.py,  task: hover → tasks/hover.py
   │
   └─ sims.make(cfg, …)  sim: pybullet → sims/pybullet_sim.py,  sim: mujoco → sims/mujoco_sim.py
                         both expose the same Sim interface (state, step, snap, draw_plume, …)
```

The task code never imports a simulator. It sees one `Sim` interface (documented in `modular_uav/sims/__init__.py`): module mass and inertia, rotor positions, `state(i)` as position, velocity, body-to-world rotation and body rates, `step(rpm)`, `snap()` to weld docked pairs, `stock_pid()`, `draw_plume()` and `close()`. Each backend converts its simulator's conventions to it (quaternion order, rate frames, rotor order).

## Package layout

```
codebase/
  simulate.py              master script (the only entry point)
  scenarios/
    master.yaml            every field at its default, with its explanation (generated)
    <cage>/master.yaml     short view for one cage: its fields, its layouts, the main switches (generated)
    box/<layout>/<variant>.yaml       pair, 3x3, diamond, rows × base, fail, circle
    tetra/<layout>/<variant>.yaml     4, 10, recursive × base, fail, circle
    triangle/<layout>/<variant>.yaml  edge3, edge6, face4, face6 × base, fail, circle
    hex/<layout>/<variant>.yaml       pair, 7 × base, fail, circle
    dodeca/<layout>/<variant>.yaml    quad, hexa, hexa6dof, tetraquad, tetradeca, tetrahexadeca × base, fail, circle
    tiling/<layout>/<variant>.yaml    rhombitri, kagome, snub, elongated, octagons, dodecagons × base, fail, circle
    world/<name>.yaml                 several scenarios in one simulation (include): paper, zoo
    hover/<cage>.yaml                 one module on the stock PID: every cage, and none
  modular_uav/
    config/   fields.py (the registry), loader.py (YAML + CLI merge), master.py (renders master.yaml, the per-cage
              sub-masters and docs/fields.html), help.py (-h, --help all, --help WORD; fields_page.html template)
    sims/     pybullet_sim.py, mujoco_sim.py: CagedAviary subclasses + the Sim wrapper;
              scaling.py: per-drone coefficients (mixed module sizes), shared by both
    models/   cage.py: cage geometry, mass, inertia, clearance check; module.py: module types
              (a cage + a Froude-scaled CF2X), drone_scale: fit
    physics/  downwash.py: original and regularised downwash, shared by both backends
    dock/     layouts.py (shapes on lattices, fill, fractal), tilings.py (polygon tilings), contact.py
              (collision pairs), controller.py (geometric SE(3), weighted allocation), mission.py (one
              scenario's phases), world.py (the runner: includes, sim: both), checks.py
    tasks/    task registry, hover.py
    viz/      plume.py (live aero isolines in the viewer), live.py (live_plot), diff.py (sim: both)
    utils/    Logger.py, utils.py (copied from gym-pybullet-drones, shared), flight_log.py
  tests/      pytest: config layer, models, both backends
  docs/       architecture.md (this file), dock.md, simulators.md, fields.html (generated, searchable field reference)
```

Each file has one job and stays small. The largest, `dock/mission.py`, is the phase machine of the docking mission.

## How the simulators are extended without touching them

| Feature | PyBullet (`sims/pybullet_sim.py`) | MuJoCo (`sims/mujoco_sim.py`) |
|---|---|---|
| Caged module | `CagedAviary(CtrlAviary)` overrides `_parseURDFParameters` to point `self.URDF` at a URDF generated from upstream `cf2x.urdf`, with the cage's mass, inertia and rods | `CagedAviary(BaseAviary)` swaps the caged mass and inertia into `DRONE_PARAMS[CF2X]` while the parent constructor runs, then rebuilds the MJCF with the rods |
| Model enum | stays `DroneModel.CF2X`, so every upstream CF2X branch (mixer, torques, PID) applies | same |
| Downwash | `_downwash` override: `original` calls the upstream method, `regularised` uses `physics/downwash.py` | same |
| Docking welds | `p.createConstraint(JOINT_FIXED)` per docked pair on `snap()`, with the pair's relative orientation | one inactive `<weld>` per pair in the rebuilt MJCF (`relpose` with orientation), activated on `snap()` |
| Mixed module types | one URDF per type (scaled CF2 meshes, the type's rods); drones of other types reloaded in `_housekeeping` | each drone body rewritten in the MJCF (mass, inertia, scaled meshes, collision cylinder, rotor sites) |
| Per-drone coefficients | `PerDrone` mixin (`sims/scaling.py`): `KF`, `KM`, `L`, `PROP_RADIUS`, drag, downwash and ground-effect lengths swapped in around upstream `_physics`, `_groundEffect`, `_drag`; RPM clipped per drone | same mixin |
| Module heading, tilt | `initial_rpys`; `set_attitude` uses `resetBasePositionAndOrientation` | `initial_rpys`; `set_attitude` writes the free joint's `qpos` |
| Module contact | `setCollisionFilterPair` off for the pairs `dock/contact.py` lists (field `module_collisions`) | `<exclude>` for the same pairs |
| Live viewer | upstream GUI + `sync()` | `step` override renders and paces to wall-clock |
| Stock PID | `DSLPIDControl`, with `GRAVITY` set to the caged weight | `PIDControl(env)`, reads mass and inertia from the env |

Each hook asserts the upstream detail it relies on (the `cf2x.urdf` inertial block, the MJCF `drone{i}_center` site), so an upstream change fails loudly instead of silently.

## Adding something

- **A parameter:** add a `Field` to `config/fields.py`, read it as `cfg.<name>`, run `python -m modular_uav.config.master` to regenerate `scenarios/master.yaml` and the sub-masters `scenarios/<cage>/master.yaml`. A field that only matters for some cages lists them in `cages=(...)`; sub-masters show the `module` and `layout` fields of their cage plus the switches in `master.SUB_SWITCHES`. `tests/test_config.py` fails if you forget.
- **A cage:** its vertices, rods and neighbour vectors in `models/cage.py` (`vertices`, `edges`, `lattice`), its size field(s) with `cages=(...)`, and its layouts in `fields.CAGE_LAYOUTS`. Both backends draw the rods from `cage.edges`, so they need no change.
- **A layout:** a shape is a row recipe in `dock/layouts.py` (`FLAT_ROWS`) or an fcc cell rule (`space`); lattices are in `lattice_kind` / `flat`. Module orientations (`Q`, body to assembly) come out of `build` with the positions, and the mission, the welds and the assembly controller all take them from there.
- **A scenario:** a new YAML file under `scenarios/`, starting with a short comment block that says what it is and how to run it. Only list the fields that differ from the defaults.
- **A task:** a module with `run(cfg) -> exit code`, registered in `tasks/__init__.py`, plus a `task` choice in `fields.py`.
- **A simulator feature:** a method override in the backend's `CagedAviary`, switched by a field, with the upstream behaviour as one of the choices.

## Checking

```bash
conda activate drones
python -m pytest                      # config, models, both backends (about a minute)
python simulate.py --scenario scenarios/tetra/recursive/fail.yaml --sim mujoco
```

Every run ends with PASS/FAIL self-checks and exits 0 only if all pass, so a shell loop over `scenarios/` is a regression suite.
