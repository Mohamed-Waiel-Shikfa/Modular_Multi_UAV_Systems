# Docking scenarios (`task: dock`)

Caged CF2X modules take off, dock into a rigid structure in the air, and fly it as one assembly. Depending on the scenario, the assembly then hovers, survives module failures or flies a circle, and lands. These are the first pipeline experiments: stages 6 (Assemble) and 7 (Fly) at L0. Each one is a scenario YAML run by the master script (see `architecture.md`).

## Scenario files

```
scenarios/
  master.yaml           every field, default and explanation
  <cage>/master.yaml    short view for one cage: its geometry, its layouts, the main switches
  box/                  box cages (120 x 120 x 50 mm): full faces, or half faces between offset rows
    pair/               2 side by side         (layout: rows, rows [2])
    3x3/                9 in a square          (layout: square, size 3)
    diamond/            9 in rows 1 2 3 2 1    (layout: rhombus, size 3; brick bond)
    rows/               10 in rows 3 2 2 3     (layout: rows; brick, face, brick)
  tetra/                tetrahedron cages (310 mm edges), docked vertex to vertex
    4/                  pyramid of 3 + 1       (layout: pyramid, size 2)
    10/                 pyramid of 6 + 3 + 1   (size 3)
    recursive/          pyramid of any size    (size 4 by default)
  triangle/             triangular prisms (230 mm edges, 50 mm tall)
    edge3/, edge6/      triangle of 3 / 6, all pointing +y, sharing vertical edges (triangle_connect: edge)
    face4/              triangle of 4 tiles, middle one turned 180 deg (triangle_connect: face)
    face6/              hexagon of 6 tiles, alternately turned (layout: hexagon, size 1)
  hex/                  hexagonal prisms (70 mm edges, 50 mm tall), docked face to face
    pair/               2 side by side         (layout: rows, rows [2])
    7/                  hexagon of 2 + 3 + 2   (layout: hexagon, size 2)
  dodeca/               dodecahedra (60 mm edges), the Dodecacopter (Garanger et al. 2026, Fig. 13)
    quad/               (a) rhombus of 4               (layout: rhombus, size 2)
    hexa/               (b) ring of 6                  (layout: hexagon, size 2, fill: frame)
    hexa6dof/           (c) the ring, rotors tilted    (+ tilt: alternate)
    tetraquad/          (d) tetrahedron of 3 + 1       (layout: pyramid, size 2)
    tetradeca/          (e) tetrahedron of 6 + 3 + 1   (layout: pyramid, size 3)
    tetrahexadeca/      (f) 4 tetraquads, 16 modules   (layout: pyramid, size 2, fractal: 2)
  tiling/               mixed polygon prisms (230 mm edges), each tile the largest drone that fits it
    rhombitri/          3.4.6.4: 6 triangles, 6 squares, 7 hexagons (layout: rings, size 2)
    kagome/             3.6.3.6: 6 triangles, 7 hexagons
    snub/               3.3.4.3.4: 8 triangles, 5 squares
    elongated/          3.3.3.4.4: 10 triangles, 5 squares
    octagons/           4.8.8: 5 octagons, 4 squares (size 1)
    dodecagons/         4.6.12: one 12-gon, 6 squares, 6 hexagons (size 1)
  world/                several scenarios in one simulation (include)
    paper.yaml          the six Dodecacopter configurations at once (46 modules)
    zoo.yaml            one assembly of every cage, all flying a circle (51 modules)
  hover/                one module on the stock PID: every cage, and none (task: hover)
```

Each layout folder holds the same three variants:

| Variant | Fields set | After the snap |
|---|---|---|
| `base.yaml` | | hover, land |
| `fail.yaml` | `failure: true`, `fail_mask` | hover, climb to fail_z, chosen modules at 50 % thrust for 0.5 s, recover, hover 3 s, land |
| `circle.yaml` | `circle: true` | hover, 2 laps of a 1 m circle, land |

`failure` and `circle` are independent switches, so one run can do both. Layouts are not limited to these files: `--size` grows any shape (the recursive forms of the paper's configurations: `dodeca/hexa --size 3` is a ring of 12, `dodeca/tetrahexadeca --size 3` the 64-module Sierpinski tetrahedron), and `--rows 3,4,5,4,3` builds any row pattern.

Level: L0 docking ("magic glue" welds). Aero effects follow `physics` (default `gnd_drag_dw`: ground effect, drag and downwash) and `downwash_model` (default `regularised`). After the snap, one controller flies all the rotors. This is not the L1 "PID per module".

## Run

From `codebase/`:

```bash
conda activate drones
python simulate.py --scenario scenarios/box/3x3/fail.yaml --fail_mask 1,0,0,1,0,0,1,0,0
python simulate.py --scenario scenarios/tetra/recursive/base.yaml --size 5 --sim mujoco
python simulate.py --scenario scenarios/tetra/10/circle.yaml --gui --radius 1.5 --laps 3
python simulate.py --scenario scenarios/box/3x3/base.yaml --downwash_model original   # the upstream model: blows up
python simulate.py --help
```

Each run prints the module index map, a timeline and PASS/FAIL checks. It exits 0 only if every check passes. Every value is a field (`scenarios/master.yaml` lists all of them with explanations); the ones most used here:

| Field | Default | Meaning |
|---|---|---|
| `sim` | `pybullet` | `pybullet` or `mujoco` |
| `gui` | `false` | open the viewer, run in real time. The camera is free in both simulators (PyBullet: Ctrl + left drag rotates, Ctrl + middle drag pans, wheel zooms) and does not follow the assembly |
| `plume` | `off` (`dw` if given bare) | with `gui`: draw live isolines of equal aero force (see below) |
| `physics` | `gnd_drag_dw` | `base`, `gnd`, `drag`, `dw` or `gnd_drag_dw` |
| `downwash_model` | `regularised` | `original` (upstream) or `regularised` (with `dw_thrust_cap`, `dw_dz_ramp`, `dw_min_width`) |
| `cage`, `layout` | `box`, `square` | cage type (`tiling` mixes polygon prisms); shape: `rows`, `square`, `triangle`, `rhombus`, `hexagon` (flat), `pyramid` (tetra, dodeca), `rings` (tilings) (see "Layouts") |
| `size`, `rows`, `fill`, `fractal` | 3, [2, 3, 2], `solid`, 1 | shape size; row counts for `layout: rows`; keep all modules, the hull's boundary (`hollow`) or its edges (`frame`); levels of self-similar repetition |
| `tiling`, `tile_edge`, `drone_scale` | `3.4.6.4`, 0.23 m, `1` | tilings: which one, the shared polygon edge; drone size as a Froude scale, or `fit` (the largest drone each cage holds) |
| `include`, `include_gap` | [], 0.5 m | scenario files to fly together in one simulation (laid out on a square-ish grid), and the clear space between their footprints |
| `module_collisions` | `undocked` | `off`, `all`, or every pair except those whose cages touch when docked |
| `live_plot`, `sim: both` | false | plot the run while it flies; run both simulators and plot their disagreement |
| `triangle_connect`, `tilt` | `edge`, `none` | triangle cages: share vertical edges (`edge`) or alternate and share faces (`face`); dodeca: `alternate` tilts the rotors 41.8° |
| `start_spacing` | 0.5 m (1.0 m in the pair files) | ground distance between neighbouring module centres at the start of a stacked layout (a flat layout starts at the staging points, see `stage_spacing`) |
| `stage_spacing`, `stage_gap` | 0.01 m, 0.05 m | distance between neighbouring module centres at the staging points before closing in to dock; least clearance between neighbouring cages' bounding spheres there |
| `z0` | null: 0.03 m (box, triangle, hex), 0.07 m (tetra), 0.09 m (dodeca) | start height of the module CoMs |
| `meet_z` | 1.0 m | docking height of the assembly CoM |
| `rise_t`, `close_t` | 3 s, 4 s | climb to the staging points; contract to the slots |
| `hover_t` | 1 s | hover after the snap |
| `failure`, `fail_mask` | false, `edge` | add the failure segment; failing modules, one 0/1 per module in map order (`[1,0,...]` and `100...` also work), or `edge` |
| `fail_z`, `hold_t`, `outage`, `outage_t`, `final_t` | 1.0 m, 1 s, 0.5, 0.5 s, 3 s | climb here, hover, keep this fraction of thrust for this long, hover after recovery |
| `circle`, `radius`, `laps`, `lap_t` | false, 1.0 m, 2, 10 s | add the circle segment; peak speed is twice the average |
| `plot`, `save_log` | false, false | plot the flight log at the end (shared Logger); save it as .npy in `output_folder` |

Controller gains (`pos_wn`, `module_wn`, `ki_ratio`, `att_wn_*`, ...), landing speeds and every self-check threshold (`check_*`) are fields too.

The old flag names map to fields by replacing `-` with `_` (`--meet-z` became `--meet_z`), except `--fail`, which is now `fail_mask` (plus `failure: true`).

Default failure masks: pair `0,1`; 3×3 `1,0,0,1,0,0,1,0,0` (left column); diamond `0,1,0,1,0,0,1,0,0` (left side); tetra 4 `1,0,0,0`; tetra 10 and recursive: every module on the −x edge of the bottom layer (3 modules for tetra 10, n for size n); box rows `edge` (the two −x modules); triangle edge3 `1,0,0`, edge6 `1,1,0,1,0,0` (the left edge), face4 `1,0,0,0`, face6 `edge` (2 modules); hex pair `0,1`, hex 7 `edge` (module 2); dodeca quad `0,1,0,0`, hexa and hexa6dof `1,0,0,0,0,0`, tetraquad `1,0,0,0`, tetradeca and tetrahexadeca the +y edge of the bottom layer (3 and 4 modules). One module at 50 % inside a 10- or 20-module pyramid barely registers (under 1° of tilt, 2–4 cm of drop), so the larger pyramids default to a whole edge (`fail_mask: edge`).

## Layouts and module numbers

A layout is a **shape** (`layout`, grown by `size`) on the **lattice** of the cage. Four lattices cover every cage:

| Lattice | Cages | Neighbours | Pitch |
|---|---|---|---|
| box | box | in a row: full face. Across rows: full face if the two rows have the same parity (they line up), half a face if not (half a cage off, brick bond) | 0.12 m |
| triangular | hex, dodeca (flat), triangle (`edge`) | 6 around each module; each row sits half a slot off the next | hex 121 mm, dodeca 137 mm, triangle 230 mm |
| face | triangle (`face`) | 3 around each module, sharing faces; modules alternate pointing +y and −y (turned 180° about z) | 133 mm (edge / √3) |
| fcc (3-D) | tetra, dodeca (`pyramid`) | 12 around each module; horizontal layers are triangular | tetra 310 mm, dodeca 137 mm |

Flat shapes are rows of modules, top (+y) row first, each row centred. A row that the centring puts off its lattice moves half a slot to +x (triangular: a row with the parity of the one above; face: a row with the other parity of the first row). Row counts for size n:

| Shape | box, triangular | face-docked triangles | Paper example |
|---|---|---|---|
| `rows` | the `rows` field | the `rows` field (triangles per row) | |
| `square` | n × n | n rows of 2n (a parallelogram strip) | |
| `triangle` | 1, 2, …, n | 1, 3, …, 2n − 1 (n² tiles) | |
| `rhombus` | 1, 2, …, n, …, 2, 1 | 1, 3, …, 2n − 1, 2n − 1, …, 1 | quadrotor = dodeca rhombus 2 |
| `hexagon` | n, …, 2n − 1, …, n | 2n + 1, …, 4n − 1, 4n − 1, …, 2n + 1 (6n² tiles) | hexarotor = dodeca hexagon 2, `fill: frame` |

3-D shapes (fcc): `pyramid` is a tetrahedral stack of n layers, n(n+1)(n+2)/6 modules (tetraquad = 2, tetradecarotor = 3).

Two fields then work on any shape:

- **`fill`** picks which modules of the shape are kept: `solid` (all), `hollow` (those on the convex hull's boundary: a flat shape's outline, a 3-D shape's surface; pyramid 5 hollow drops its one interior module) or `frame` (those on the hull's edges: the outline, or a 3-D shape's edge skeleton; hexagon 2 → the ring of 6, square n → a square ring, pyramid 4 → 16 modules).
- **`fractal: L`** repeats the filled shape L levels deep: level L + 1 puts a copy of level L at every module of the base shape, scaled up by the smallest integer at which no copies overlap. It works on positions, so on any lattice closed under integer scaling (box, triangular, fcc). Sierpinski triangle = `triangle 2, fractal L` (3^L); Sierpinski tetrahedron = `pyramid 2, fractal L` (4^L; L = 2 is the paper's hexadecarotor, four tetraquads at the corners of a 4-layer tetrahedron, the same 16 modules as `pyramid 4, fill: frame`); Sierpinski carpet = `square 3, fill: frame, fractal L` (8^L); a ring of rings = `hexagon 2, fill: frame, fractal 2` (36).

Every layout is checked: no two modules overlap, and every module is reachable through docked neighbours. A failing one (for example `fill: frame` on face-docked triangles, whose edge rows lose their turned tiles) stops with a config error.

Modules are numbered layer by layer from the bottom, then row by row from the +y row, left to right (−x to +x). Every run prints the map, and lists the modules that are turned or tilted.

```
box/rows (3 2 2 3)     triangle/face6        dodeca/hexa          dodeca/tetradeca
   0     1     2       0  1  2   (1, 3, 5      0     1           layer 0       layer 1     layer 2
      3     4          3  4  5    turned)   2           3         0  1  2         6  7         9
      5     6                                  4     5              3  4           8
   7     8     9                                                     5
```

- **Box:** 1 slot = 0.12 m, the cage width.
- **Tetra:** a regular tetrahedron, apex up, one base vertex on +x. Modules sit one cage edge vector (0.31 m) apart and share one vertex. Tetra cages take the 3-D shapes only (their flat lattice runs along ±y, not x).
- **Triangle:** a 50 mm prism over an equilateral triangle with one vertex on +y (an edge along x). `triangle_connect: edge`: every module points +y and neighbours share a vertical cage edge, one edge length apart. `triangle_connect: face`: down tiles are the same module turned 180° about z (the CF2X is symmetric under that turn, so it is the same vehicle), and neighbours share a full face.
- **Hex:** flat faces on ±x, neighbours √3 × 70 = 121 mm apart along x and at ±60°, sharing a full face.
- **Dodeca:** the Dodecacopter module (Garanger et al., T-RO 2026). The rotor axis (z) runs through two opposite vertices, which belong to the cube inscribed in the dodecahedron. A neighbour sits at g + h for an edge {g, h} of that cube (the paper's restricted connection set, Fact II.4), so the two modules share vertices g and h and their faces through them are coplanar; centres are √2 φ × 60 = 137 mm apart. The slots form an fcc lattice whose horizontal planes are triangular, so the flat shapes and the 3-D ones both apply. A dodecahedron rests on a vertex, so a single module or a pair tips over on landing; the scenarios use 3 or more modules not in a line.
- **Tilt (dodeca, `tilt: alternate`):** every rotation of the dodecahedron's symmetry group (60 of them) maps the cage onto itself, so a turned module still docks (Fact II.3). Three of them tilt the rotor axis 41.8° onto a vertex next to the top one, at azimuths 7.8°, 127.8° and 247.8°. Around a hexagonal ring the modules alternate between +tangential and −tangential tilt, picking the closest of the three; each is 37.8° off the tangent. Each tilt direction is used twice, so at equal thrust the side forces and yaw torques cancel, and the rotor axes span all 6 DOF (rank 6, checked in `tests/test_models.py`). A tilted CF2X cannot hover on its own, so tilted modules approach upright and are turned in place at the snap (L0 "magic glue"). The assembly controller then maps the body force and moment through the full 6 × n wrench matrix (least-norm thrusts, clipped), but still flies the attitude like an underactuated vehicle: the sixth DOF is available, not yet used. Downwash stays vertical (not modelled for tilted rotors). 70.5° tilts are also cage symmetries, but at that tilt a 30 g module lifts only 0.75 of its weight, so they are not offered.
- **Tiling (`cage: tiling`):** an edge-to-edge tiling of regular polygons (`dock/tilings.py`), named by its vertex configuration: the polygons met going round every vertex. Ten are built in: the three regular ones (4.4.4.4, 3.3.3.3.3.3, 6.6.6) and seven mixed Archimedean ones (3.6.3.6, 3.4.6.4, 3.3.3.4.4, 3.3.4.3.4, 4.8.8, 4.6.12, 3.12.12). Each is stored as a unit cell of its non-triangle tiles; every triangle of these tilings sits on the edge of a bigger tile, so the triangles are the outward equilateral triangles on tile edges that land on free space (a test checks every vertex configuration). `layout: rings` takes the centre tile (the biggest polygon nearest the origin) and `size` rings of docked neighbours, so the region is always connected. Every tile is a module: an n-gon prism of edge `tile_edge`, turned to its tile, docked face to face with every tile it shares an edge with.
- **Module types and drone scale:** a layout may mix module types (`models/module.py`): each is a cage plus a CF2X scaled by Froude similarity with scale k (lengths × k, mass × k³, inertia × k⁵, kf × k⁴, km × k⁵, drag × k³, MAX_RPM from the module's own weight, so thrust-to-weight stays 2.25). `drone_scale: fit` gives each type the largest k that clears its cage by `cage_clearance`; tiling prisms grow to `prism_height` × k so a big drone fits. In the default 3.4.6.4 that makes triangles k = 1.03 (33 g), squares 1.97 (212 g) and hexagons 3.54 (1.2 kg). Both backends take one module type per drone (`sims/scaling.py`): per-drone URDF (PyBullet) or MJCF body (MuJoCo), and per-drone coefficients swapped in around the upstream `_physics`, `_groundEffect` and `_drag`; at k = 1 every value is the upstream one.
- **Recursive:** `size` sets everything else: module count, positions, neighbour pairs (fcc pyramid: 6 for size 2, 24 for 3, 60 for 4, 120 for 5), welds, mask length, ground grid and staging height.

## Phases

World frame, z up. Units are m, s and rad. Physics and control run at 240 Hz.

| Phase | What happens | Controller |
|---|---|---|
| Approach | Each module flies from the ground to its staging point over rise-t. Staging points are the layout scaled up so neighbours sit start-spacing apart, lifted so the lowest cage is at least 0.3 m up. The layout then contracts to the slots over close-t. Flat layouts start right under their staging points; turned triangles fly at their 180° heading from the start. Stacked (fcc) layouts start on a square ground grid, each module assigned to its nearest cell by Hungarian assignment (`scipy.optimize.linear_sum_assignment`). | Geometric controller per module, with velocity feedforward |
| Snap | Once every docked neighbour pair is within 3 cm of its slot offset (the full 3D error), every pair is welded at its exact offset and relative orientation in one step. Tilted modules (`tilt: alternate`) are turned in place to their tilt first. | |
| Hover | Hold (0, 0, meet-z) for hover-t. | Assembly controller |
| Climb + hold (fail) | Climb or descend at 1 m/s to fail-z, then hover for hold-t. | Assembly controller |
| Failure (fail) | For outage-t, each failing module gets `outage` × the thrust the controller commands. No module is told. | Assembly controller |
| Recover (fail) | Full thrust returns. Hover at the x, y where recovery starts, at z = max(current z, meet-z). Heading is left free until stable. | Assembly controller |
| Final hover (fail) | Hover there for final-t. | Assembly controller |
| Circle (circle) | Laps of a horizontal circle that starts at the hover point. Speed follows 1 − cos, so it starts and ends at rest. Velocity and acceleration are fed forward. | Assembly controller |
| Land | Descend at 1 m/s, then 0.3 m/s below 1 m, until the lowest cages touch. The motors cut on touchdown. | Assembly controller |

"Stable" means position error < 5 cm, speed < 5 cm/s, tilt < 3° and body rate < 0.1 rad/s.

The approach used to run each simulator's stock PID. MuJoCo's stock `PIDControl` has a very soft position loop (P = 0.4 s⁻², ωn ≈ 0.6 rad/s), and the tetra cage triples the module's roll/pitch inertia. Together they left tetra modules 1–2 cm off target five seconds after the approach ended, so the snap never triggered. Each module now flies the same geometric controller as the assembly (with N = 1), so both simulators approach identically. The stock PIDs are still used by `task: hover` (`scenarios/hover/`).

## Docking ("magic glue")

- **One weld per docked neighbour pair:** 1 for the pair, 12 for the 3×3, 16 for the diamond, 6/24/60/120 for tetra depth 2/3/4/5. This matches where the connectors would be. An earlier version welded every module to module 0. On the 10-module pyramid in PyBullet, the far modules then sagged 0.64 cm on their long lever arms. Pair welds bring that to 0.00 cm.
- **PyBullet:** `createConstraint(JOINT_FIXED)` from module i to module j, with the parent frame at j's 3D offset. The constraints close the last 3 cm themselves.
- **MuJoCo:** the model is rebuilt with one inactive `<weld body1="drone{i}" body2="drone{j}" relpose="dx dy dz 1 0 0 0">` per pair, with `solref` 0.01 s. Snap sets `data.eq_active[:] = 1`.
- **Both:** contact between modules is turned off from the start. Docked cages touch, and that contact would fight the welds.

## Assembly controller

A geometric SE(3) controller (Lee et al. 2010) acts on the assembly CoM, the mean of the module positions. Its axes are module 0's body axes.

- **Position loop:** PID, ωn = 2.5 rad/s, ζ = 0.9, plus an integral term KI = 4 s⁻³ (stable while KI < KP·KD = 28) capped so that KI·∫e ≤ g. Without the integral, a steady downwash force F leaves a steady sag F/(m·KP): 0.22 W gave 34 cm in `box/pair`. During the approach each module has its own controller and integral, with a stiffer loop (ωn = 4 rad/s, same ζ, KI = 0.256 ωn³ = 16.4 s⁻³). A module docking under another feels more downwash the closer it gets, about 0.73 W/m for module 4 under module 12 in `tetra/recursive`. That beats the ωn = 2.5 spring (m·KP = 0.64 W/m), and the module crept in at 2 cm/s and missed the snap by 0.7 cm. At the snap the assembly's integral starts at the mean of the module integrals, so it keeps the downwash it has already learnt (otherwise `tetra/10` hovered 5.1 cm low one second after the snap). Commanded tilt is capped at 35°. Desired velocity and acceleration are fed forward (used by the circle and the approach).
- **Attitude loop:** ωn = 20 rad/s for roll and pitch, 8 rad/s for yaw, ζ = 0.8. The gains are scaled by the assembly inertia J = N·J_module + Σ m(|r|²I − r rᵀ) over the 3D module offsets r.
- **Attitude error:** the rotation vector of R_dᵀR. Unlike the vee-map error, it stays large near 180°.
- **Allocation:** B maps the 4N rotor thrusts to [f, Mx, My, Mz]. Rotor heights do not matter, since all thrust is along body z. Roll/pitch torque is served first, then collective thrust, then yaw from what is left. A roll/pitch demand the rotors cannot meet goes bang-bang: the rotors on the pushing side run at full thrust and the rest switch off. The collective spreads evenly because every layout's rotors are centred on the CoM; the code asserts this.
- **Rotor positions:** PyBullet's prop links are mirrored in y relative to MuJoCo's `_physics()`, so each backend carries its own `ROTORS` table.

## Collisions between modules (`module_collisions`)

Until 2026-11-08 module–module contact was off by default, so cages passed through each other whenever two approach paths crossed (stacked layouts start on a ground grid and can cross). Docked cages touch, and contact there would fight the welds, which is why it was off. The default is now `undocked`: every pair of modules collides, except the pairs whose cages touch in the docked structure (`dock/contact.py`: any two rods closer than both radii plus 2 mm once docked). That covers docked neighbours and modules that only meet at a rod or a vertex (diagonal boxes in a grid, face-docked triangles round a vertex). `off` restores the old behaviour, `all` makes every pair collide. Modules of different scenarios in one world always collide. With `undocked`, the old scenarios give the same numbers as with `off`: no approach path in them actually crosses another module's cage. The 1 mm rods are thin, so a fast enough crossing can still tunnel through in one 1/240 s step (PyBullet and MuJoCo both use discrete contact).

## Several scenarios in one simulation (`include`)

A scenario file with `include: [a.yaml, b.yaml, ...]` flies each listed scenario at the same time in one simulation (`dock/world.py`). Each keeps its own layout, cages, module types, phases, controllers and checks; they share the space and the clock. They are placed on a square-ish grid (ceil(sqrt(n)) columns, filled row by row), each footprint (ground grid, staging, circle and the cages' radii) `include_gap` (0.5 m) clear of the next, so they neither touch nor feel each other's downwash. Fields set in the world file and on the command line override every included scenario (`circle: true` makes them all fly the circle); the run, physics, plume and logging fields always come from the world file. Status lines carry the scenario name, and the run exits 0 only if every scenario passes. `world/paper.yaml` flies the six Dodecacopter configurations together (46 modules), and `world/zoo.yaml` one assembly of every cage (51 modules). Each cage also has `scenarios/<cage>/run_all.yaml`, which includes every `base.yaml` of that cage's layouts in one simulation (for example `box/run_all.yaml` flies the pair, 3 × 3, diamond and rows grids together). Isolation was checked: in the zoo every scenario reproduces its standalone numbers exactly (for example box 3 × 3 circle 3.7 cm, tetra 4 snap 6.37 s, 3.4.6.4 tiling circle 1.9 cm).

## Plots

- **`live_plot: true`**: a matplotlib window that follows the run as it flies: height, horizontal error and tilt of each assembly (approach centroid before the snap), with the target dashed and the phases in the title, refreshed `live_plot_rate` (5) times per second of sim time.
- **`sim: both`**: the whole run in PyBullet, then in MuJoCo, then `diff_<scenario>.png` in `output_folder`: CoM distance (cm), attitude difference (deg) and worst module distance (cm) between the two runs over time, with the phases shaded and red ticks wherever the two runs are in different phases. It also prints the maxima. With `plot: true` the figure opens in a window too.

Example, `tiling/kagome/fail --sim both`: the two simulators agree to the millimetre through the approach, the snap and the hover. The failure (two 1.2 kg hexagons at half thrust) then sends them apart chaotically, up to 62° and 91 cm, and they settle in different places. Both runs still pass every check. So results that pass through a large upset are simulator-specific, while everything before it is not.

## Mixed module sizes and downwash

The regularised downwash was fitted and bounded for equal CF2X modules. In the first mixed tiling (3.4.6.4 with `drone_scale: fit`), the 33 g triangles were slammed to the ground at t ≈ 6.6 s, straight down, just as they closed in beside the 1.2 kg hexagons. Without aero (`physics: base`) the same tiling docks, hovers and flies the circle at 3.4 cm, so downwash was the whole cause. The hexagon drone is 3.5× larger, so its plume floor (its arm length) reaches the neighbouring triangle with a force above the triangle's whole thrust margin. Two Froude-consistent corrections, both 1 when every module has the same size (so no earlier result moves):

- the dz ramp is a length, so it scales with the upper drone: `dw_dz_ramp` × k_upper;
- a lower module smaller than the upper one intercepts only part of its jet: F × min(1, (k_lower / k_upper)²), the ratio of their rotor disk areas.

With them, every mixed tiling in `scenarios/tiling/` passes on both simulators. One limit remains, and it is physical: in 3.12.12 with `drone_scale: fit`, 33 g triangles sit beside 10 kg 12-gons (k = 7.2). The intercepted jet is still about 2 N on a 0.32 N module, and the triangles are blown off. 3.12.12 docks with `physics: base`. With `drone_scale: 1` it fails for another physical reason: a 30 g CF2X cannot steer a 0.9 m 12-gon cage with about 200× its own inertia.

The results sections below keep the field names of their date: `depth` is now `size`, `grid` 3 × 3 is `square` 3, `diamond` is `rhombus` 3, `lattice_rows` is `rows`, `frame: true` is `fill: frame`, `sierpinski` n is `pyramid 2, fractal n` (or `triangle 2, fractal n` flat).

## Results (2026-11-01, old fork scripts, aero effects off: `physics: base`)

All 37 runs pass every check: 18 scenarios × 2 simulators, plus `tetra/recursive/base` with depth 5 in both. These runs were made with the old fork scripts, which matched `physics: base`. The two simulators agree to the millimetre except for circle tracking with tetra cages (see the table).

| Layout | Snap | Slot error after hover | `fail` (default mask) | `circle` worst error (PyBullet / MuJoCo) |
|---|---|---|---|---|
| box/pair | 6.87 s, 1 weld | 0.00 cm | max tilt 7°, lowest CoM 0.70 m, stable after 1.81 s | 0.8 / 0.9 cm |
| box/3x3 | 6.69 s, 12 welds | 0.00 cm | max tilt 3–4°, lowest 0.81 m, stable after 1.58 s | 0.8 / 0.9 cm |
| box/diamond | 6.72 s, 16 welds | 0.00 cm | max tilt 4°, lowest 0.82 m, stable after 1.57 s | 0.8 / 0.9 cm |
| tetra/4 | 6.37 s, 6 welds | 0.00 cm | max tilt 2°, lowest 0.85 m, stable after 1.48 s | 0.8 / 2.1 cm |
| tetra/10 | 6.37 s, 24 welds | 0.00 cm | 3-module edge: max tilt 1°, lowest 0.85 m, stable after 1.46 s | 0.8 / 2.1 cm |
| tetra/recursive (depth 4, 20 modules) | 6.38 s, 60 welds | 0.00 cm | 4-module edge: max tilt 0°, lowest 0.91 m, stable after 1.20 s | 0.8 / 2.1 cm |
| tetra/recursive, depth 5 (35 modules) | 6.38 s, 120 welds | 0.00 cm | | |

MuJoCo's circle error is larger for tetra cages. Its model sets `density` and `viscosity`, so MuJoCo applies air drag to the large tetra rods, and PyBullet's `PYB` mode has no drag. That is an observation, not checked further.

Harsher failures (PyBullet, current engine, all checks pass):

| Run | Result |
|---|---|
| `box/pair/fail.yaml --physics base --outage 0 --outage_t 1 --fail_z 12` | flips 136°, 8.4 m/s at the end of the failure, lowest CoM 2.24 m, stable after 3.73 s |
| `box/3x3/fail.yaml --physics base --fail_mask 1,1,1,0,0,0,0,0,0 --outage 0 --fail_z 3` | top row off for 0.5 s: max tilt 10°, lowest CoM 1.86 m, stable after 2.17 s |
| `box/diamond/fail.yaml --physics base --fail_mask 1,1,1,0,0,0,0,0,0 --outage 0 --fail_z 3` | top three off for 0.5 s: max tilt 6°, lowest CoM 1.86 m, stable after 2.16 s |
| `box/3x3/fail.yaml --physics base --fail_mask 1,0,0,0,0,0,0,0,0 --outage 0 --outage_t 1 --fail_z 6` | one corner off for 1 s: max tilt 4°, lowest CoM 5.75 m, stable after 0.40 s |
| `tetra/10/fail.yaml --physics base --outage 0 --outage_t 1 --fail_z 4` | bottom −x edge (3 modules) off for 1 s: max tilt 2°, lowest CoM 2.52 m, stable after 2.02 s |

The bigger the assembly, the less one failing module matters. Two modules lose half their thrust when one is off, and a fully failed pair flips. Nine modules lose a ninth. In a tetra pyramid the failing edge sits low and the inertia is large, so it hardly tilts at all and mostly sinks. The failure checks therefore only ask for more than 2° of tilt or more than 5 cm of drop.

## Results with ground effect, drag and downwash on (2026-11-05)

`PYB_GND_DRAG_DW` and `MJC_GND_DRAG_DW`, regularised downwash, PID loops (see "Fix" below). All 38 runs pass: 18 scenarios × 2 simulators, plus `tetra/recursive/base` with depth 5 in both. These are the defaults now (`physics: gnd_drag_dw`, `downwash_model: regularised`); see "Results after the restructure" for the rerun on the upstream simulators. The two simulators agree except for circle tracking with tetra cages, as before.

| Layout | Snap | Hover error / slot error | `fail` (default mask) | `circle` worst error (PyBullet / MuJoCo) |
|---|---|---|---|---|
| box/pair | 6.87 s | 0.1 / 0.00 cm | max tilt 7°, lowest CoM 0.71 m, stable after 3.88 s | 3.7 / 3.7 cm |
| box/3x3 | 6.69 s | 0.1 / 0.00 cm | max tilt 3–4°, lowest 0.82 m, stable after 3.14 s | 3.7 / 3.7 cm |
| box/diamond | 6.72 s | 0.1 / 0.00 cm | max tilt 4°, lowest 0.83 m, stable after 3.10 s | 3.7 / 3.7 cm |
| tetra/4 | 6.37 s | 0.1 / 0.00 cm | max tilt 2°, lowest 0.86 m, stable after 2.81 s | 3.7 / 7.1 cm |
| tetra/10 | 6.37 s | 0.1 / 0.00 cm | max tilt 1°, lowest 0.86 m, stable after 1.66 s | 3.7 / 7.1 cm |
| tetra/recursive (depth 4) | 8.03 s | 0.3 / 0.00 cm | max tilt 0°, lowest 0.91 m, stable after 1.46 s | 3.8 / 7.1 cm |
| tetra/recursive, depth 5 | 8.02 s | 0.7 / 0.01 cm | | |

Against the aero-off results above: slots stay rigid as before, and recovery from a failure takes longer (up to 3.9 s instead of 1.2–1.8 s) because the integral overshoots. Circle tracking is worse, 3.7 instead of 0.8 cm, and 7.1 instead of 2.1 cm for tetra cages in MuJoCo, still under the 10 cm check: the integral adds phase lag. The recursive pyramid snaps 1.6 s later, because the lower modules have to push up into the downwash of the layer above.

## Results after the restructure (2026-11-07)

Same scenarios, now run as `python simulate.py --scenario ...` on the **unmodified upstream** simulators (gym-pybullet-drones `f83fbdd`, MuJoCo-drones-gym `ca170dc`), with the cage, the regularised downwash and the live-viewer pacing supplied from `modular_uav/` by subclassing. Defaults: `physics: gnd_drag_dw`, `downwash_model: regularised`, no `gui`. All 40 dock runs pass (18 scenarios × 2 simulators, plus `tetra/recursive/base` at depth 5 on both), and so do the 6 hover runs (`scenarios/hover/{box,tetra,none}.yaml` × 2).

Every number in the 2026-11-05 table above is reproduced: the same snap times (6.87, 6.69, 6.72, 6.37, 6.37 s; recursive 8.03 s PyBullet, 8.04 s MuJoCo), hover errors, slot errors (0.00 cm, 0.01 cm at depth 5), failure tilts, lowest CoM heights, recovery times and circle errors (3.7 cm; 7.1 cm for tetra in MuJoCo, 3.8 cm for the PyBullet recursive pyramid). So the port changed nothing in the physics or the control.

| Run | Result |
|---|---|
| `hover/box`, `hover/tetra`, `hover/none` (stock PID, 10 s at 1 m) | final error 0.5 cm (PyBullet `DSLPIDControl`), 1.7–1.9 cm (MuJoCo `PIDControl`) |
| `box/3x3/base.yaml --downwash_model original` | fails in both, exactly as in "Findings" below: PyBullet throws the modules tens of metres (worst pair 38 m apart, no snap); MuJoCo prints `Nan, Inf or huge value in QACC at DOF 8 ... Time = 3.0042`, resets, and never snaps |

## Triangle, hex and dodecahedron cages (2026-11-07)

Three new cages (`triangle`, `hex`, `dodeca`), two layout kinds for them (`pyramid` reused, `rows` new) and one sub-master per cage (`scenarios/<cage>/master.yaml`). Defaults as above. All 42 new dock runs pass: 21 scenarios × 2 simulators. So do the 10 sub-master runs (each runs as is) and the 6 new hover runs (pytest). The 18 older scenarios still pass with the same numbers (snap at 6.87, 6.69, 6.72, 6.37 s; circle 3.7 cm, 7.1 cm for tetra in MuJoCo).

| Scenario | Snap | Hover / slot error | Failure (`fail`) | Circle error PyBullet / MuJoCo |
|---|---|---|---|---|
| triangle/3 | 6.59 s, 3 welds | 0.1 / 0.00 cm | `1,0,0`: max tilt 5°, lowest 0.80 m, stable after 3.30 s | 3.7 / 4.0 cm |
| triangle/6 | 6.59 s, 9 welds | 0.1 / 0.00 cm | −x edge (3): max tilt 3°, lowest 0.73 m, stable after 3.73 s | 3.7 / 4.0 cm |
| hex/pair | 6.87 s, 1 weld | 0.1 / 0.00 cm | `0,1`: max tilt 7°, lowest 0.71 m, stable after 3.89 s | 3.7 / 3.7 cm |
| hex/7 | 6.69 s, 12 welds | 0.1 / 0.00 cm | module 2: max tilt 2°, lowest 0.94 m, stable after 1.26 s | 3.7 / 3.6 cm |
| dodeca/quad | 6.67 s, 5 welds | 0.1 / 0.00 cm | `0,1,0,0`: max tilt 4°, lowest 0.88 m, stable after 1.62 s | 3.7 / 4.4 cm |
| dodeca/7 | 6.67 s, 12 welds | 0.1 / 0.00 cm | module 2: max tilt 2°, lowest 0.94 m, stable after 1.25 s | 3.7 / 4.4 cm |
| dodeca/pyramid (`hover_t: 5`) | 6.82 s, 6 welds | 0.6 / 0.00 cm | `1,0,0,0`: stable after 3.17 s, lowest 0.83 m | 4.3 / 4.8 cm |

Both simulators agree to the millimetre except for circle tracking, where MuJoCo's air drag on the rods adds a little (as for tetra).

**Finding: the dodecahedron pyramid sits in its own downwash.** Stacked dodecahedra are close: the top module is 112 mm above each bottom module and 79 mm beside it (tetra pyramid: 253 mm and 179 mm). The regularised downwash then puts 0.14 W on each bottom module, against about zero in the tetra pyramid. After the snap the assembly sags 12 cm, and the integral needs about 4 s to pull it back, so with the default `hover_t: 1` the hover check failed (11.9 cm) on both simulators, and the circle that follows missed its 10 cm check (12.8 cm). With `physics: base` the same run hovers at 0.1 cm, and `physics: dw` alone reproduces the sag, so downwash is the whole cause. The three `dodeca/pyramid` files set `hover_t: 5`. The same applies to the real Dodecacopter: in its 3-D configurations the upper rotors blow onto the lower modules.

## Shapes, lattices, face-docked triangles and the six Dodecacopter configurations (2026-11-08)

Layouts became shapes on the cage's lattice (`layout`, `size`, `rows`, `frame`; see "Layouts"); `triangle_connect: face` and `tilt: alternate` added per-module orientation (turned 180° about z, or tilted 41.8°), carried through the welds, the assembly inertia and the controller. The old box and tetra scenarios were migrated (`grid` 3 × 3 → `square` 3, `diamond` → `rhombus` 3, `depth` → `size`) and reproduce their numbers exactly (snap 6.87, 6.69, 6.72, 6.37 s and 8.03/8.04 s; circle 3.7 cm, 7.1 cm for tetra in MuJoCo). All 124 dock runs pass: 57 scenarios and the 5 sub-masters, on both simulators. The hover check of each cage passes on both (pytest).

| Scenario | Snap | Hover / slot error | Failure (`fail`) | Circle error PyBullet / MuJoCo |
|---|---|---|---|---|
| box/rows (3 2 2 3) | 6.72 s, 16 welds | 0.1 / 0.00 cm | two −x modules: max tilt 2°, lowest 0.91 m, stable after 1.49 s | 3.7 / 3.7 cm |
| triangle/edge3 | 6.56 s, 3 welds | 0.1 / 0.00 cm | `1,0,0`: max tilt 5°, lowest 0.80 m, stable after 3.30 s | 3.7 / 4.1 cm |
| triangle/edge6 | 6.56 s, 9 welds | 0.1 / 0.00 cm | left edge (3): max tilt 3°, lowest 0.73 m, stable after 3.73 s | 3.7 / 4.1 cm |
| triangle/face4 | 6.67 s, 3 welds | 0.1 / 0.00 cm | `1,0,0,0`: max tilt 4°, lowest 0.86 m, stable after 2.85 s | 3.7 / 4.1 cm |
| triangle/face6 | 6.67 s, 6 welds | 0.1 / 0.00 cm | two −x modules: max tilt 3–4°, lowest 0.82 m, stable after 3.13 s | 3.7 / 4.1 cm |
| dodeca/quad (a) | 6.67 s, 5 welds | 0.1 / 0.00 cm | `0,1,0,0`: max tilt 4°, lowest 0.88 m, stable after 1.62 s | 3.7 / 4.4 cm |
| dodeca/hexa (b) | 6.67 s, 6 welds | 0.1 / 0.00 cm | `1,0,0,0,0,0`: max tilt 2°, lowest 0.93 m, stable after 1.35 s | 3.7 / 4.4 cm |
| dodeca/hexa6dof (c) | 6.67 s, 6 welds | 0.1 / 0.00 cm (MuJoCo 0.3 / 0.09 cm) | `1,0,0,0,0,0`: max tilt 5°, lowest 0.92 m, stable after 1.47 s | 7.1 / 7.7 cm |
| dodeca/tetraquad (d) | 6.82 s, 6 welds | 0.6 / 0.00 cm | `1,0,0,0`: max tilt 4°, lowest 0.83 m, stable after 3.17 s | 4.3 / 4.8 cm |
| dodeca/tetradeca (e) | 8.44 s (MuJoCo 8.52 s), 24 welds | 0.1 / 0.00 cm | +y bottom edge (3): max tilt 2°, lowest 0.78 m, stable after 3.76 s | 7.3 / 5.4 cm |
| dodeca/tetrahexadeca (f) | 6.84 s, 30 welds | 1.3 / 0.00 cm (MuJoCo 0.03 cm) | +y bottom edge (4): max tilt 1°, lowest 0.84 m, stable after 3.20 s | 8.6 / 5.2 cm |

The three stacked dodeca configurations keep `hover_t: 5` (their downwash sag, see the 2026-11-07 finding). Their circle errors (5–9 cm) are the closest to the 10 cm check of any scenario: the integral that holds up the lower layers also lags the circle.

**Recursive forms.** Larger sizes fly too (PyBullet, `base`): `dodeca/quad --size 3` (9 modules), `dodeca/hexa6dof --size 3` (ring of 12, tilted; slot error 0.07 cm). The level-3 Sierpinski tetrahedron (`dodeca/tetrahexadeca --size 3`, 64 modules, 126 welds) does not dock with the defaults: module 34 stays 17 cm below its slot under the stack above, and a longer `snap_timeout` makes it worse (29 cm). With `physics: base` it docks at 6.68 s, so downwash is the cause, the same mechanism that set `module_wn: 4` for the tetra pyramids. `--module_wn 6` docks it at 7.94 s and hovers at 0.1 cm (slot error 0.07 cm). The default stays 4, so no other scenario changes; the file headers give the flag.

The 6-DOF hexarotor flies, but less well than the plain hexarotor: circle 7.1 instead of 3.7 cm, and the soft MuJoCo welds give way by 0.09 cm under the tilted rotors' internal side forces. Each tilted rotor spends cos 41.8° = 75 % of its thrust on lift, so the ring hovers at 1.34 × the thrust of the plain one; the paper reports the same (its 6-DOF hexarotor needed 0.58 of full input against 0.31). The controller does not use the sixth DOF yet (attitude still tilts to move sideways), so this configuration is a fully actuated vehicle flown as an underactuated one.

## Fill, fractal, mixed tilings, worlds, collisions and the two plots (2026-11-08)

`fill` and `fractal` replaced `frame` and `sierpinski` (see "Layouts"); `cage: tiling` added mixed polygon prisms with Froude-scaled drones; `include` runs several scenarios in one simulation; `module_collisions` now defaults to `undocked`; `live_plot` and `sim: both` were added; the downwash got its two mixed-size corrections. The PyBullet CF2X is now drawn from coloured meshes, and the cages default to amber, so they stand apart. All 166 dock runs pass: 75 scenarios, the 6 sub-masters and the 2 worlds, on both simulators; 191 tests pass. With collisions on, every older scenario reproduces its numbers exactly (snap 6.87, 6.69, 6.72, 6.37 s and 8.02/8.03 s; circle 3.7 cm, 7.1 cm for tetra in MuJoCo; dodeca and triangle as in the table above).

| Tiling (`drone_scale: fit`) | Modules | Snap | Hover / slot error | Failure (`fail`) PyBullet | Circle error PyBullet / MuJoCo |
|---|---|---|---|---|---|
| rhombitri 3.4.6.4 | 6 tri (33 g), 6 sq (212 g), 7 hex (1.2 kg) | 6.78 s | 0.2 / 0.00 cm | 3 hexagons: max 3°, lowest 0.81 m, stable after 3.19 s | 1.9 / 2.3 cm |
| kagome 3.6.3.6 | 6 tri, 7 hex | 6.51 s | 0.1 / 0.00 cm (MuJoCo 0.07) | 2 hexagons: max 3°, lowest 0.85 m, stable after 2.86 s (MuJoCo: 6°, 0.26 m, 7.83 s) | 1.8 / 2.6 cm |
| snub 3.3.4.3.4 | 8 tri, 5 sq | 6.76 s | 0.1 / 0.00 cm | 2 squares: max 3°, lowest 0.84 m, stable after 3.00 s | 2.4 / 1.9 cm |
| elongated 3.3.3.4.4 | 10 tri, 5 sq | 6.81 s | 0.1 / 0.00 cm | 2 squares: max 2°, lowest 0.85 m, stable after 2.91 s | 2.4 / 2.0 cm |
| octagons 4.8.8 | 4 sq, 5 oct (2.6 kg) | 6.45 s | 0.1 / 0.00 cm | 1 octagon: max 3°, lowest 0.91 m, stable after 1.49 s | 2.0 / 3.7 cm |
| dodecagons 4.6.12 | 6 sq, 6 hex, 1 12-gon (10 kg) | 6.67 s | 0.1 / 0.00 cm (MuJoCo 0.02) | 3 modules: max 3°, lowest 0.93 m, stable after 1.46 s | 3.1 / 4.8 cm |

The capacity allocation makes failures in big mixed assemblies small: one 1.2 kg hexagon at half thrust in the 8.9 kg rhombitri tilted it only 1–2° and dropped it under 5 cm, so the `fail` file fails the three −x hexagons. In kagome, MuJoCo's recovery from two failing hexagons is much deeper than PyBullet's (lowest CoM 0.26 against 0.85 m); `sim: both` shows the two runs identical until the failure and chaotic after it (see "Plots").

## Failure modes and limits

- The CoM is assumed to be the mean of the module positions, which holds only for equal module masses. Mass and inertia are given to the controller, not estimated (estimating them is L2).
- No module knows about a failure. Fault detection is not modelled.
- All modules snap at once when the last pair closes. There is no docking order (π in the formulation) yet.
- Welds cannot break: connector loads are not modelled or limited. In a tetra pyramid every load passes through single vertices.
- Collisions between modules are off, so approach paths may pass modules through each other. Stacked layouts start on a ground grid and fly diagonally to staging, and those paths can cross.
- The default physics is ground effect, drag and the regularised downwash (`physics: gnd_drag_dw`, `downwash_model: regularised`). The downwash model is still a fitted Gaussian, with a cap and a ramp added, not measured near-field data (see the fix below).
- The pass/fail thresholds were tuned on these runs. The `gui: true` path has not been rerun since the restructure.

## Live aero isolines (`plume`)

Draws lines of equal felt force in the simulator's own viewer while the scenario runs. It needs `gui: true`. `modular_uav/viz/plume.py` wraps `sim.step` and redraws `plume_rate` (20) times per second of sim time. Each backend (`modular_uav/sims/*_sim.py`) draws the segments with `draw_plume()`: PyBullet as dotted lines (one debug-points call, 4 dots per segment), MuJoCo as line geoms in the viewer's user scene.

```bash
python simulate.py --scenario scenarios/box/3x3/base.yaml --gui --plume          # downwash
python simulate.py --scenario scenarios/tetra/4/base.yaml --gui --plume all
```

- **Levels and colour:** one isoline per level, as a multiple of one module's weight W = M g (0.294 N for the caged CF2X). **Dark purple = 0.02 W (weakest), teal = 0.1 W, bright yellow = 0.5 W (strongest).** Downwash is capped at the upper module's thrust (W at hover, see the fix below), so no level at or above 1 W exists. A labelled colour bar stands beside the modules. MuJoCo also prints the scale top left, and the terminal prints it at start.
- **`dw`, downwash:** under each module, the surface where another module would feel that level of downwash from it, drawn as a vertical oval (x–z plane) and a ring at its widest point. The regularised model is axisymmetric, so the surface has a closed form, r(Δz) = β √(2 ln(min(α, T) s/L)) with α = DW_COEFF_1 (R/(4Δz))², β = max(DW_COEFF_2 Δz + DW_COEFF_3, arm length) and s = min(Δz / dw_dz_ramp, 1). It is drawn at hover thrust T = W, for whichever `downwash_model` is configured (the drawing finds where the centreline force exceeds each level numerically, so it also works for `original`). It depends only on the coefficients, so it is computed once and moved with the modules. Each oval is one module's wake: the simulators add overlapping wakes, the drawing does not. Each oval's top tip sits at Δz = L·dw_dz_ramp/W (1 mm for 0.02 W, 2.5 cm for 0.5 W), where the ramp first lets the force reach L. The parts below the floor are not drawn.
- **`gnd`, ground effect:** a horizontal outline around the modules at the height where ground effect at hover thrust reaches the level, h = R/4 √(GND_EFF_COEFF / L): 0.02 W at 13.8 cm, 0.1 W at 6.2 cm. 0.5 W lies below the clip height `GND_EFF_H_CLIP` (3.8 cm), never occurs and is not drawn.
- **`drag`:** an arrow from each module along its drag force, 5 cm per factor of 10 above 0.02 W, coloured by its size. Drag has no spatial field.
- **Cost:** the segments take under 1 ms per redraw (about 1100 segments for the 3×3, about 4200 for 35 modules). This replaces an earlier voxel-fog version that took 15–120 ms per redraw.

What the downwash isolines show: before the fix, the rings at rotor height were about 0.5 m wide for every level, and modules beside each other at nearly the same height sat inside each other's 10 W ring: the singularity described below. After the fix the ovals start a few mm to 2.5 cm under the rotor plane and the force never exceeds the upper module's thrust.

The isolines use the simulators' own formulas and coefficients (upstream `_groundEffect` and `_drag`, and the configured downwash model from `modular_uav/physics/downwash.py`). Two simplifications apply to the drawing only: rotor heights equal the CoM height (as MuJoCo does), and the forces are taken in the world frame (PyBullet applies downwash and drag in the body frame).

## Findings: why the aero modes blow up (2026-11-04)

The runs used `PYB_GND_DRAG_DW` in PyBullet and `MJC_GND_DRAG_DW` in MuJoCo, with default settings and `downwash_model: original`. The numbers below come from recording the module states every step during those runs. `box/3x3/base.py` and `tetra/4/base.py` fail in both simulators: no snap, and the modules are thrown far off.

**Cause: the downwash model is singular when two modules are at the same height.** Both forks use the same model (Zhou, DSL), applied to module i from each module j above it:

    force = −DW_COEFF_1 (PROP_RADIUS / (4 Δz))² · exp(−½ (Δxy / β)²),   β = DW_COEFF_2 Δz + DW_COEFF_3,   for Δz > 0

With the CF2X coefficients (2267.18, 0.16, −0.11, R = 23.1 mm) the force is 0.0756 N / Δz² (Δz in m), so it goes to infinity as Δz → 0⁺. Flat layouts fly every module at the same commanded height, so their heights differ only by floating-point roundoff, and the Gaussian is still large across a 0.12–0.6 m neighbour spacing:

| Run | When | Height differences | Downwash on a module |
|---|---|---|---|
| PyBullet `box/3x3` | t = 3.0042 s, end of the rise (all modules at 0.99 m) | 3–6 nm | 1.6 × 10¹² W (edges), 3.2 × 10¹² W (centre) |
| MuJoCo `box/3x3` | t = 3.0000 s, same instant | ~1 fm (a few float64 ULPs) | 8.6 × 10²⁵ W (edges), 1.1 × 10²⁶ W (centre) |
| both, `tetra/4` | t = 0, all modules on the ground at 0.07 m | roundoff | 6.4 × 10⁴ W (PyBullet), 2.6 × 10⁵ W (MuJoCo) in the first step |

The simulators then break in different ways:

- **PyBullet:** in one step, the hit modules reach PyBullet's 100 m/s velocity clamp. In `box/3x3` the edge and centre modules are thrown 0.42 m down per step, then fly off up to 29 m sideways.
- **MuJoCo:** prints `Nan, Inf or huge value in QACC at DOF 8. The simulation is unstable. Time = 3.0042.` (DOF 8 is module 1's z) and resets the state to the start pose on the ground. The modules are then level again, so the same singularity keeps them pinned to the floor (about 1.7 × 10¹⁵ W from t = 4 s). In `tetra/4` MuJoCo does not reset, and the modules end up hundreds of km away.

Other problems with the model that the isolines make visible:

- **The plume narrows downward.** β = 0.16 Δz − 0.11 is negative for Δz < 0.69 m and zero at 0.69 m. The Gaussian uses β², so the wake is widest just under the rotor, pinches to nothing at 0.69 m below, and widens again further down. A real wake spreads out as it goes down.
- **It does not depend on the upper module's thrust.** A module with its motors off still pushes down on the modules below it.
- **No reaction force.** The force pushes on the lower module, but nothing takes thrust away from the upper one.
- **Welded modules still push on each other.** After the snap, an upper tetra layer pushes its own lower layer down. That is a net external force on the assembly, which the controller has to cancel.
- **The magnitude is plausible only at real separations.** At t = 1.40 s in PyBullet `tetra/4`, module 3 is 21 cm under module 0 and gets 3.2 W of downwash.

The ground effect is bounded (it is clipped at h = `GND_EFF_H_CLIP`, 3.8 cm) and stays below 0.44 W in every run. Drag stays below 0.1 W. Neither one causes the blow-up.

## Fix: regularised downwash (2026-11-04)

`downwash_model: regularised` (`modular_uav/physics/downwash.py`, used by the `_downwash` override in both backends) computes, for module i under module j (Δz > 0):

    force = −min(α, T_j) · exp(−½ (Δxy / β)²) · min(Δz / dw_dz_ramp, 1),   β = max(DW_COEFF_2 Δz + DW_COEFF_3, L)

- **Peak capped at the upper module's thrust T_j = KF Σ rpm²** (N; field `dw_thrust_cap`). The force is module j's jet hitting module i, and the jet's whole momentum flux equals T_j, so the centreline force cannot exceed it. The cap goes on the peak α, not on the product. A first version capped the product: at Δz = 6 cm, α is 71 W, so even 3 plume widths out (Δxy = 31 cm) a module felt 0.58 W, and in `tetra/4` the base modules dragged each other down in a chain (2 → 0 → 1). Capping α brings that to 0.01 W. It also makes the force depend on the upper module's thrust: a module with its motors off no longer pushes down.
- **Ramp over `dw_dz_ramp` = 5 cm** (field; 0 turns it off). A module within 5 cm of another's rotor plane sits beside its wake, not in it, so the force fades linearly to 0 at Δz = 0. This removes the roundoff blow-up between modules at equal height.
- **Plume width floored at the arm length L** (field `dw_min_width`, null = L) (39.7 mm on CF2X): a jet is at least as wide as the rotors that make it. The fitted β is negative for Δz < 0.69 m, outside its data, and its |β| of 11 cm at the rotor plane let the centre of the 3×3 feel about 0.5 W from each of its 8 neighbours once it dipped a few cm (2.9 W in all, more than its 2.25 W maximum thrust), so it sank 35 cm and never docked. With the floor the plume is 4 cm wide down to Δz = 0.94 m and then widens, so it no longer narrows downward.

The assembly controller also gained an integral term (see "Assembly controller"), because the remaining downwash is a steady force that a PD loop answers with a steady sag.

Still in the model: there is no reaction on the upper module (its thrust is unchanged, which is right for momentum), and welded modules still blow on each other, which is a real loss of lift for stacked structures. The 5 cm ramp, the thrust cap and the arm-length width floor are modelling choices, not fitted to data.
