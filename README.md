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

Stages 1–6 form the superstructure, and stage 7 operates it. Each stage is built in three levels of realism, from an idealised version to state of the art. The first milestone is the idealised version of all seven stages running end to end.

The simulator is [gym-pybullet-drones](https://github.com/utiasDSL/gym-pybullet-drones), with models of the Crazyflie quadrotors. Modules dock in simulation by adding and removing a fixed joint between two bodies.

## Status

This is the start of the project. The repository holds documentation and the licence, and no pipeline code yet. The current work is the simulation environment, a parameterised module model, and payload and connection models.

## Files

- `README.md`: this file.
- `CLAUDE.md`: guidance for AI coding assistants working here, with the full formulation and the fidelity ladder.
- `liscence`: the licence.

## Licence

Use is limited to verifying the results of the author's thesis work. See `liscence`.
