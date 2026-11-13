"""Task-Oriented Modular Multi-UAV Systems: declarative experiments on top of the upstream simulators.

Entry point: simulate.py at the repository root. Layout:
  config/   field registry (every parameter), YAML + CLI loader, master.yaml renderer
  sims/     simulator backends (subclasses of the upstream envs, one Sim interface)
  models/   module models (the caged CF2X)
  physics/  aero models shared by both backends (downwash)
  dock/     task dock: layouts, controller, mission phases, self-checks
  tasks/    task registry and the other tasks (hover)
  viz/      live viewer overlays (plume isolines)
  utils/    shared Logger and helpers (copied from gym-pybullet-drones), flight log
"""
