# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Status

This repository is new and holds no pipeline code yet. It is the code base for the Fall 2026 senior thesis, "Task-Oriented Modular Multi-UAV Systems".

Nothing has been built yet.

Before December 2026 the goal is "Milestone 1": the level-0 pipeline running end to end (see the fidelity ladder below). No build, lint or test commands exist yet. Add them here when the environment exists. Do not invent them before then.

## The pipeline

A task (a payload that must move along a motion) becomes a flying structure. Seven stages, in order:

1. **Task**: payload and the motion it must follow.
2. **Attach**: where to grip the payload.
3. **Wrench**: the force and moment the attachment must produce.
4. **Shape**: which superstructure of modules can supply that wrench.
5. **Allocate**: which module fills which slot.
6. **Assemble**: approach and dock the modules.
7. **Fly**: control and coordinate the assembled structure.

Stages 1–5 plan and stage 6 executes the superstructure. Together they are **Level A, form the superstructure**. Stage 7 is **Level B, operate it**, including coordinating several structures.

The execute and fly stages can be refined and decomposed further as needed, with notifying the user.

Four shared constraints touch every stage: downwash and aerodynamics, energy and power budget, collision avoidance, and structural integrity. A choice at stage 4 must still hold at stages 6 and 7, so planning and execution are one problem, not two.

## Formulation

The deck writes the whole problem as one optimisation.

- Given: payload `O = (m, I, ∂O)` (mass, inertia, surface); motion `ξ(t)`; `N` modules with thrust limit `ū` and battery `E`.
- Choose: attachment points `P ⊂ ∂O`; structure `C`; module-to-slot assignment `σ`; docking order `π`; trajectories `x(t)` and thrusts `u(t)`.
- Minimise: `J = time + energy` over the mission.
- Subject to:
  - `R(O, ξ) ⊆ A(C, P)`: the wrenches the task needs lie inside the set the structure can produce, where `A = {B u : 0 ≤ u ≤ ū}` and `B` maps rotor thrusts onto the payload.
  - `flyable(C, π)`: every partial structure during assembly can fly.
  - `collision-free(x)`.
  - `loads(C, x) ≤ limits`: downwash and connector loads stay safe.
  - `energy(u) ≤ E`.

The unknowns mix discrete choices (sets, orders, counts) with continuous quantities (trajectories, thrusts). The constraints couple them, so the problem is intractable as one piece. The approach is to split it along the seven stages and check each stage's answer against the stages after it.

## Fidelity ladder

Each stage is built up in three levels, and each step up is measured against the level below:

- **L0, idealised**: known box payload, hand-picked attachment points, hover-only thrust, a fixed shape library, nearest-module allocation, "magic glue" docking (snap when within ε of the slot), ideal thrust with no downwash.
- **L1, simple model with physics**: real force and moment from Newton–Euler along `ξ(t)`, arbitrary rigid meshes, symmetric attachment points about the CoM, a wrench-set check in the shape search, optimal assignment (Hungarian), a scripted approach handing over to PD docking, PID per module with the simulator's downwash model on.
- **L2, state of the art per stage**: in-flight mass and CoM estimation, grasp-quality search, worst-case envelope with margins, configuration optimisation, coalition formation with energy, docking with learnt downwash, robust control and multi-robot path planning.

Fall 2026 target: all of L0 end to end, plus two L1 pieces (Newton–Euler force and moment, and per-module PID with downwash on). Spring 2027 climbs the stages that matter most, then configuration optimisation, coalition formation and learnt-downwash docking. Gold dashed cells in the deck are stretch goals.

## Simulator

Two simulator repos are forked under `Mohamed-Waiel-Shikfa` and cloned as submodules. We will experiment with both until we work out which one is better:

- `gym-pybullet-drones/`: fork of learnsyslab/gym-pybullet-drones (PyBullet). Own `.venv`.
- `MuJoCo-drones-gym/`: fork of tau-intelligence/MuJoCo-drones-gym (MuJoCo). Own `.venv`.

The choice between them is open. The rest of this section describes the earlier PyBullet choice.

The testbed is **gym-pybullet-drones** (utiasDSL; Panerati et al., IROS 2021), MIT licence. Reasons it was chosen over the nine other environments compared in the prospectus:

- runs on a laptop CPU; Isaac Sim lists an RTX 4080 with 16 GB as its minimum
- ground effect, drag and downwash are switchable physics modes (`PYB_GND`, `PYB_DRAG`, `PYB_DW`) on real Crazyflie models (CF2X, CF2P)
- Python throughout; our modules are defined as URDF and planners plug in through the Gymnasium API
- docking is physics: PyBullet's `createConstraint` and `removeConstraint` add and remove a fixed joint between two bodies mid-simulation

The known costs: plain visuals, and a simple downwash model that must be refined later.

## Things to keep in mind

- Keep units, coordinate frames, timing and failure modes explicit in code and docs, as the thesis work requires.
- Keep the level of each module written down (L0, L1 or L2), so results are never compared across levels by mistake.
- The licence file in this folder restricts use to verifying the author's thesis results. Keep any reuse inside that scope.
- Do not commit or push unless the user asks.
