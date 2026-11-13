# Task-Oriented Modular Multi-UAV Systems

Software for a senior thesis at the School of Computer Science, Carnegie Mellon University in Qatar, by Mohamed Waiel Shikfa.

The question is how a group of small UAVs can attach to one another, form a structure that suits a task, and fly it. A payload is given with its mass and the motion it must follow. The software decides where to grip it, what structure of modules can lift it, which module goes where, and how the modules dock and fly together.

## How the work is organised

The task is split into seven stages, from the task to the flight:

1. Task: the payload and the motion it must follow.
2. Attach: where to grip it.
3. Wrench: the force and moment needed.
4. Shape: which structure of modules provides them.
5. Allocate: which module fills which slot.
6. Assemble: approach and dock.
7. Fly: control and coordinate the structure.

Stages 1–6 form the superstructure, and stage 7 operates it. Each stage is built in three levels of realism, from an idealised version to state of the art. The first milestone is the idealised version of all seven stages running end to end. Stages 6 and 7 run today, at the idealised level.

## A declarative simulator

Every experiment is one command: the master script `simulate.py`, a scenario file, and any flags you want to change.

```bash
conda activate drones
pip install -r requirements.txt          # once: both simulators (editable) + numpy, scipy, matplotlib, pyyaml, pytest

python simulate.py --scenario scenarios/box/3x3/fail.yaml
python simulate.py --scenario scenarios/box/3x3/fail.yaml --sim mujoco --gui
python simulate.py --scenario scenarios/tetra/recursive/circle.yaml --size 5 --plot
python simulate.py --scenario scenarios/box/pair/base.yaml --downwash_model original
python simulate.py -h                    # short help; --help all, --help WORD (search), docs/fields.html
```

- **Two simulators, both untouched.** [gym-pybullet-drones](https://github.com/learnsyslab/gym-pybullet-drones) (PyBullet) and [MuJoCo-drones-gym](https://github.com/tau-intelligence/MuJoCo-drones-gym) (MuJoCo) are git submodules at upstream commits. Everything of ours is in `modular_uav/` and builds on them by subclassing. `sim: pybullet` or `sim: mujoco` picks one.
- **Every feature is a field.** The caged modules, the docking layouts, the regularised downwash model, the failure and circle segments, the live aero overlay, the plots, the controller gains and the pass/fail thresholds are all switchable. The original simulator behaviour is always one of the choices.
- **Same name in the file and on the command line.** `meet_z: 1.2` in a scenario file is `--meet_z 1.2` as a flag. A flag wins over the file, and the file wins over the default.
- **`scenarios/master.yaml`** lists every field with its default and an explanation. **`scenarios/<cage>/master.yaml`** is the short view for one cage (its geometry and layouts plus the main switches). Every scenario file starts with a short comment that says what it runs.

Each run ends with PASS/FAIL self-checks and exits 0 only if all of them pass.

## What runs today

- **Docking** (`task: dock`): caged Crazyflie modules take off, dock in the air into a shape (rows, square, triangle, rhombus, hexagon, pyramid, Sierpinski, or their outline) on the lattice of their cage: box, tetrahedron, triangular prism (edge- or face-docked), hexagonal prism, or dodecahedron, including all six Dodecacopter configurations of Garanger et al. (T-RO 2026) with its tilted-rotor 6-DOF hexarotor, or a mixed tiling of triangles, squares, hexagons, octagons and 12-gons whose drones are Froude-scaled to fit each tile (so the thrust allocation shares load between 30 g and 10 kg modules); shapes can be hollowed, framed or repeated fractally, and several scenarios can fly side by side in one simulation, then hover, survive module failures or fly a circle, and land. Docking welds the modules together in the simulator.
- **Comparing the simulators** (`sim: both`): the same run in PyBullet and MuJoCo, then a plot of how far they drift apart; `live_plot: true` plots any run as it flies.
- **Hover check** (`task: hover`): one caged module flies on the simulator's stock PID, to check the module model.

## Files

- `simulate.py`: the master script.
- `scenarios/`: `master.yaml`, one sub-master per cage, the scenario files, and `world/` (several scenarios in one simulation).
- `modular_uav/`: the code (config, simulator backends, module model, downwash, docking, plume overlay, shared utilities).
- `tests/`: `python -m pytest`.
- `docs/architecture.md`: how the declarative design works and how to extend it.
- `docs/dock.md`: the docking scenarios: phases, controller, results, and the downwash findings and fix.
- `docs/simulators.md`: reference notes on both simulators.
- `gym-pybullet-drones/`, `MuJoCo-drones-gym/`: the upstream simulators (submodules; clone with `--recurse-submodules`).

## Licence

MIT. See `LICENSE`. The simulators are separate projects under their own terms.
