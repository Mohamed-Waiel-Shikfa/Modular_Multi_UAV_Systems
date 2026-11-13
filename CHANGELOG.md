# Changelog

Format: [Keep a Changelog](https://keepachangelog.com/en/1.1.0/). Versioning: [Semantic Versioning](https://semver.org/).
Below 1.0.0 the scenario-file fields and the Python API may change in any minor release.
1.0.0 is reserved for the first release where all seven stages (task to flight) run end to end at the idealised level.

## [0.1.0] - 2026-11-13

First public release. Stages 6 (assemble) and 7 (fly) run at the idealised level; stages 1 to 5 are not implemented yet.

### Added
- Declarative master script `simulate.py`: a scenario file plus command-line flags, each field named the same in both; every run ends with PASS/FAIL self-checks.
- Two simulator backends, PyBullet and MuJoCo, built on upstream gym-pybullet-drones and MuJoCo-drones-gym (git submodules, unmodified); `sim: both` compares them.
- Docking task: caged Crazyflie modules dock in the air on box, tetrahedron, triangular-prism, hexagonal-prism and dodecahedron lattices, including the six Dodecacopter configurations, and on mixed tilings with Froude-scaled drones.
- Fractal, hollow and framed shapes, module-failure and circle-flight segments, several scenarios in one world.
- Regularised downwash model, with the original simulator behaviour kept as a choice.
- Hover check for a single caged module, live plotting and an aero overlay.
- 96 scenario files, a short help system (`-h`, `--help all`, `--help WORD`) and a field reference (`docs/fields.html`).
- 197 tests (`python -m pytest`) and documentation in `docs/`.
