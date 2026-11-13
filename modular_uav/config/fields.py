"""Every run parameter, in one registry.

Each Field is one key of a scenario YAML file and the CLI flag of the same name
(`meet_z: 1.2` in YAML is `--meet_z 1.2` on the command line). master.yaml is
generated from this list (python -m modular_uav.config.master), and a test checks
that the two agree. To add a parameter: add a Field here, use cfg.<name> in the
code, regenerate master.yaml.

Units: m, s, rad unless the name says otherwise (_deg, _cm, _hz). World frame: z up.
"""
from dataclasses import dataclass, field


@dataclass(frozen=True)
class Field:
    name: str
    default: object
    help: str
    group: str
    choices: tuple = ()
    type: type = None            # element type; inferred from the default when None
    nullable: bool = False       # null / none allowed (meaning given in help)
    cages: tuple = ()            # cages this field matters for (scenarios/<cage>/master.yaml); () = all
    is_list: bool = False        # a list even when the default is null
    extra: dict = field(default_factory=dict)

    @property
    def kind(self):
        """'bool', 'list' or 'scalar'."""
        if isinstance(self.default, bool) or self.type is bool:
            return "bool"
        return "list" if self.is_list or isinstance(self.default, (list, tuple)) else "scalar"

    @property
    def elem_type(self):
        if self.type is not None:
            return self.type
        v = self.default[0] if self.kind == "list" else self.default   # only reached without `type`
        return type(v)


GROUPS = {
    "run": "What to run and how to show it",
    "world": "Several scenarios in one simulation, side by side",
    "physics": "Simulator physics and the downwash model",
    "module": "The caged module (a CF2X inside a rigid rod cage)",
    "layout": "The structure the modules dock into",
    "approach": "Approach, snap (docking) and first hover",
    "failure": "Failure segment: chosen modules lose thrust mid-flight",
    "circle": "Circle segment: the assembly flies laps of a horizontal circle",
    "land": "Climb and landing",
    "controller": "Geometric SE(3) controller gains (per module before the snap, assembly after)",
    "checks": "Pass/fail thresholds of the self-checks (the run exits 0 only if all pass)",
    "hover": "task: hover (one module on the simulator's stock PID)",
    "plume": "Live aero isolines in the viewer (needs gui)",
    "logging": "Flight log and plots",
}

# Layouts each cage docks into; the first is the default of scenarios/<cage>/master.yaml.
# Flat shapes (rows ... hexagon) need a flat lattice, pyramid needs a 3-D one (tetra, dodeca).
FLAT = ("rows", "square", "triangle", "rhombus", "hexagon")
CAGE_LAYOUTS = {
    "box": ("square",) + FLAT[:1] + FLAT[2:],
    "tetra": ("pyramid",),
    "triangle": ("triangle",) + FLAT[:2] + FLAT[3:],
    "hex": ("hexagon",) + FLAT[:4],
    "dodeca": ("rhombus",) + FLAT[:3] + FLAT[4:] + ("pyramid",),
    "tiling": ("rings",),
}
TILINGS = ("3.4.6.4", "4.4.4.4", "3.3.3.3.3.3", "6.6.6", "3.6.3.6", "3.3.3.4.4", "3.3.4.3.4", "4.8.8",
           "4.6.12", "3.12.12")   # dock/tilings.py

FIELDS = [
    # ---------------------------------------------------------------- run
    Field("task", "dock", "dock: modules take off, dock into `layout` and fly the mission. "
          "hover: one module hovers on the simulator's stock PID (checks the cage model)",
          "run", ("dock", "hover")),
    Field("sim", "pybullet", "simulator backend: pybullet (gym-pybullet-drones), mujoco (MuJoCo-drones-gym), "
          "or both: run the scenario in each, then plot how far apart the two runs are over time "
          "(diff_<name>.png in output_folder)", "run", ("pybullet", "mujoco", "both")),
    Field("gui", False, "open the simulator viewer and run in real time", "run"),
    Field("freq", 240, "Hz, physics and control rate of the dock task (one physics step per control step)", "run"),
    Field("max_t", 900.0, "s of sim time, hard stop of a run", "run"),
    Field("print_every", 0.5, "s of sim time between status lines (0 = no status lines)", "run"),
    Field("record", False, "save video with the simulator's own recorder into output_folder (PyBullet: mp4, needs gui; MuJoCo: PNG frames)", "run"),
    Field("output_folder", "results", "folder for logs, plots and recordings", "run"),
    # ---------------------------------------------------------------- world
    Field("include", [], "scenario files to run together in this simulation, side by side (each keeps its own "
          "fields; this file's other fields and the flags override them all; run, physics, plume and logging "
          "fields come from here only). Empty: just this scenario", "world", type=str),
    Field("include_gap", 0.5, "m, clear space between the footprints of included scenarios (their staging, "
          "circles and assemblies), which are laid out on a square-ish grid, so they share the space without "
          "touching or feeling each other's downwash",
          "world"),
    # ---------------------------------------------------------------- physics
    Field("physics", "gnd_drag_dw", "aero effects on top of rotor thrust: base (none), gnd (ground effect), "
          "drag, dw (downwash) or gnd_drag_dw (all three). Maps to the simulator's Physics enum "
          "(PYB_* / MJC_*)", "physics", ("base", "gnd", "drag", "dw", "gnd_drag_dw")),
    Field("downwash_model", "regularised", "original: the simulator's own _downwash, untouched (singular when "
          "two modules are at equal height). regularised: ours, with the three bounds below "
          "(see docs/dock.md, 'Fix: regularised downwash')", "physics", ("original", "regularised")),
    Field("dw_thrust_cap", True, "regularised only: cap the downwash peak at the upper module's thrust", "physics"),
    Field("dw_dz_ramp", 0.05, "m, regularised only: downwash fades in linearly over this height below the "
          "upper rotor plane (0 = no ramp)", "physics"),
    Field("dw_min_width", None, "m, regularised only: floor on the plume width beta. null = the module's arm "
          "length L; 0 = no floor", "physics", type=float, nullable=True),
    Field("module_collisions", "undocked", "contact between modules: off (none), all (every pair; docked "
          "cages touch, so contact fights the welds) or undocked (every pair except those whose cages touch in "
          "the docked structure)", "physics", ("off", "all", "undocked")),
    # ---------------------------------------------------------------- module
    Field("cage", "box", "cage around each CF2X: box (docked face to face, or half a face between offset "
          "rows), tetra (regular tetrahedron, docked vertex to vertex), triangle (triangular prism, see "
          "triangle_connect), hex (hexagonal prism, docked face to face), dodeca (regular dodecahedron, docked as "
          "in the Dodecacopter: two shared vertices, coplanar faces), tiling (a mix of polygon prisms, see "
          "`tiling`) or none (bare CF2X, task: hover only)",
          "module", ("box", "tetra", "triangle", "hex", "dodeca", "tiling", "none")),
    Field("tiling", "3.4.6.4", "tiling cages: an edge-to-edge tiling of regular polygons, named by the polygons "
          "round each vertex. 3.4.6.4 triangles + squares + hexagons, 3.6.3.6 triangles + hexagons, 3.3.3.4.4 "
          "and 3.3.4.3.4 triangles + squares, 4.8.8 squares + octagons, 4.6.12 squares + hexagons + 12-gons, "
          "3.12.12 triangles + 12-gons; 4.4.4.4, 3.3.3.3.3.3, 6.6.6 one polygon. Every tile is a module, an "
          "n-gon prism docked face to face", "module", TILINGS, cages=("tiling",)),
    Field("tile_edge", 0.23, "m, tiling cages: polygon edge shared by every tile (0.23 fits a CF2X in a "
          "triangle)", "module", cages=("tiling",)),
    Field("rod_density", 0.0026, "kg/m, tiling cages: rod mass per metre (a 120 mm box cage of 3 g has 2.6 g/m)",
          "module", cages=("tiling",)),
    Field("drone_scale", "1", "drone size, as a Froude scale k of the CF2X: lengths x k, mass x k^3, thrust "
          "coefficient x k^4 (same thrust-to-weight). A number, or fit: each module type gets the largest "
          "drone that clears its cage by cage_clearance (so big tiles carry big drones)", "module", type=str),
    Field("cage_color", None, "cage RGBA, e.g. 0.9,0.9,0.9,1. null: one colour per module type (amber, teal, "
          "pink, ...), so mixed modules stand out", "module", type=float, nullable=True, is_list=True),
    Field("box_size", [0.12, 0.12, 0.05], "m, box cage x, y, z size (x and y are also the slot pitch)", "module",
          cages=("box",)),
    Field("tetra_edge", 0.31, "m, tetra cage edge (also the neighbour distance in a pyramid)", "module",
          cages=("tetra",)),
    Field("triangle_edge", 0.23, "m, triangle cage edge", "module", cages=("triangle",)),
    Field("triangle_connect", "edge", "edge: every module points the same way (one vertex on +y) and "
          "neighbours share a vertical cage edge, one edge length apart. face: modules alternate pointing +y "
          "and -y (turned 180 deg about z) and neighbours share a full face, edge / sqrt(3) apart",
          "module", ("edge", "face"), cages=("triangle",)),
    Field("hex_edge", 0.07, "m, hex cage edge (neighbour distance = sqrt(3) x edge, flat to flat)", "module",
          cages=("hex",)),
    Field("prism_height", 0.05, "m, height of the triangle and hex prisms", "module", cages=("triangle", "hex")),
    Field("dodeca_edge", 0.06, "m, dodeca cage edge (neighbour distance = sqrt(2) x golden ratio x edge)",
          "module", cages=("dodeca",)),
    Field("tilt", "none", "dodeca: none (every rotor axis vertical) or alternate: each module off the "
          "assembly's vertical axis turns onto a cage symmetry that tilts its rotor axis 41.8 deg onto a "
          "neighbouring cage vertex, picked so that the tangential tilt alternates in sign around the centre "
          "(the 6-DOF hexarotor of Garanger et al. 2026). Modules dock upright and are turned at the snap",
          "module", ("none", "alternate"), cages=("dodeca",)),
    Field("cage_mass", 0.003, "kg, whole cage, spread over the rods by length", "module"),
    Field("cage_rod_radius", 0.001, "m, rod radius (visual and collision)", "module"),
    Field("cage_clearance", 0.005, "m, the cage must clear every CF2 mesh vertex by at least this", "module"),
    # ---------------------------------------------------------------- layout
    Field("layout", "square", "shape the modules dock into, `size` sets how big. Flat: rows (the `rows` "
          "field), square (size x size), triangle (side size), rhombus (side size), hexagon (side size). "
          "3-D (tetra, dodeca): pyramid (tetrahedral stack, size layers). Tilings: rings (the centre tile and "
          "`size` rings of docked neighbours)", "layout",
          ("rows", "square", "triangle", "rhombus", "hexagon", "pyramid", "rings")),
    Field("size", 3, "layout size: side length in modules, or layers of a pyramid", "layout"),
    Field("rows", [2, 3, 2], "layout rows: modules per row, top (+y) row first, every row centred. Box: rows "
          "of equal parity line up (full faces), others sit half a cage off (half faces). Triangular "
          "lattices (hex, dodeca, triangle): rows always sit half a slot off; a row with the parity of the "
          "one above shifts half a slot to +x. Face-docked triangles: counts are triangles per row",
          "layout"),
    Field("fill", "solid", "which modules of the shape to keep: solid (all), hollow (those on the convex "
          "hull's boundary: a flat shape's outline, a 3-D shape's surface) or frame (those on the hull's edges: "
          "the outline, or a 3-D shape's edge skeleton). hexagon 2 + frame = ring of 6", "layout",
          ("solid", "hollow", "frame")),
    Field("fractal", 1, "levels of self-similar repetition of the (filled) shape: each level puts a copy at "
          "every module, scaled up just enough not to overlap. triangle 2 -> Sierpinski triangle (3^L), "
          "pyramid 2 -> Sierpinski tetrahedron (4^L), square 3 + frame -> Sierpinski carpet (8^L)", "layout"),
    # ---------------------------------------------------------------- approach
    Field("start_spacing", 0.5, "m, ground distance between neighbouring module centres at the start", "approach"),
    Field("z0", None, "m, start height of the module CoMs. null = resting on the cage bottom "
          "(box, triangle, hex 0.03; tetra 0.07; dodeca 0.09)", "approach", type=float, nullable=True),
    Field("stage_spacing", 0.01, "m, distance between neighbouring module centres at the staging points, where "
          "the modules hold before they close in to dock (the layout scaled up). Never less than stage_gap "
          "between the cages' bounding spheres, so the default is the tightest spread stage_gap allows",
          "approach"),
    Field("stage_gap", 0.05, "m, least clearance between neighbouring cages' bounding spheres at the staging "
          "points; raise it for a wider spread, lower it (even 0) for a tighter one", "approach"),
    Field("meet_z", 1.0, "m, docking height of the assembly CoM", "approach"),
    Field("stage_clearance", 0.3, "m, lowest cage height at the staging points", "approach"),
    Field("rise_t", 3.0, "s, climb from the ground to the staging points", "approach"),
    Field("close_t", 4.0, "s, contract from the staging points to the slots", "approach"),
    Field("snap_gap", 0.03, "m, every docked pair within this of its slot offset triggers the welds", "approach"),
    Field("snap_timeout", 5.0, "s after rise_t + close_t without a snap: the run stops and fails", "approach"),
    Field("hover_t", 1.0, "s, hover at meet_z after the snap", "approach"),
    # ---------------------------------------------------------------- failure
    Field("failure", False, "add the failure segment: climb to fail_z, hold, failure, recover, final hover",
          "failure"),
    Field("fail_mask", "edge", "failing modules, one 0/1 per module in map order (1,0,0 or [1, 0, 0] or 100), "
          "or edge = the -x edge of the bottom layer", "failure", type=str),
    Field("fail_z", 1.0, "m, climb or descend here before the failure", "failure"),
    Field("hold_t", 1.0, "s, hover at fail_z before the failure", "failure"),
    Field("outage", 0.5, "fraction of commanded thrust a failing module keeps (0 = motors off)", "failure"),
    Field("outage_t", 0.5, "s, failure duration", "failure"),
    Field("final_t", 3.0, "s, hover after recovery, before landing", "failure"),
    Field("recover_pos_tol", 0.05, "m, 'stable' after a failure: position error below this", "failure"),
    Field("recover_vel_tol", 0.05, "m/s, 'stable': speed below this", "failure"),
    Field("recover_tilt_deg", 3.0, "deg, 'stable': tilt below this", "failure"),
    Field("recover_rate_tol", 0.1, "rad/s, 'stable': body rate below this", "failure"),
    # ---------------------------------------------------------------- circle
    Field("circle", False, "add the circle segment after the hover (and after the failure segment)", "circle"),
    Field("radius", 1.0, "m, circle radius", "circle"),
    Field("laps", 2.0, "laps around the circle", "circle"),
    Field("lap_t", 10.0, "s, average time per lap (speed follows 1 - cos, peak = 2 x average)", "circle"),
    # ---------------------------------------------------------------- land
    Field("climb_rate", 1.0, "m/s, climb or descent to fail_z", "land"),
    Field("land_speed", 1.0, "m/s, descent speed above land_slow_below", "land"),
    Field("land_speed_low", 0.3, "m/s, descent speed below land_slow_below", "land"),
    Field("land_slow_below", 1.0, "m, height where the descent slows down", "land"),
    # ---------------------------------------------------------------- controller
    Field("pos_wn", 2.5, "rad/s, assembly position loop natural frequency", "controller"),
    Field("module_wn", 4.0, "rad/s, per-module position loop during the approach (stiffer: pushes into "
          "the downwash of modules above)", "controller"),
    Field("pos_zeta", 0.9, "position loop damping ratio", "controller"),
    Field("ki_ratio", 0.256, "integral gain KI = ki_ratio wn^3 (s^-3), removes the steady downwash sag. "
          "0 = PD only", "controller"),
    Field("carry_integral", True, "at the snap, start the assembly integral at the mean module integral "
          "(keeps the downwash already learnt)", "controller"),
    Field("att_wn_rp", 20.0, "rad/s, attitude loop natural frequency, roll and pitch", "controller"),
    Field("att_wn_yaw", 8.0, "rad/s, attitude loop natural frequency, yaw", "controller"),
    Field("att_zeta", 0.8, "attitude loop damping ratio", "controller"),
    Field("allocation", "capacity", "how the assembly controller shares thrust when rotors differ (mixed drone "
          "sizes, tilted rotors): capacity (each rotor in proportion to its maximum thrust, so all saturate "
          "together) or equal (least-norm thrusts). Identical upright modules keep the prioritised allocation",
          "controller", ("capacity", "equal")),
    Field("max_tilt_deg", 35.0, "deg, cap on the commanded tilt", "controller"),
    # ---------------------------------------------------------------- checks
    Field("check_hover_cm", 5.0, "cm, hover position error after the snap", "checks"),
    Field("check_slot_cm", 0.5, "cm, worst slot error (assembly stays rigid)", "checks"),
    Field("check_upset_deg", 2.0, "deg, the failure must tilt the assembly more than this ...", "checks"),
    Field("check_upset_drop", 0.05, "m, ... or drop it more than this", "checks"),
    Field("check_recover_margin", 0.1, "m, lowest CoM during recovery stays this far above the ground rest "
          "height", "checks"),
    Field("check_final_cm", 5.0, "cm, final hover error after recovery", "checks"),
    Field("check_circle_cm", 10.0, "cm, worst circle tracking error", "checks"),
    Field("check_land_cm", 2.5, "cm, landed CoM height above the ground rest height", "checks"),
    Field("check_land_tilt_deg", 5.0, "deg, landed tilt", "checks"),
    Field("check_land_speed", 0.05, "m/s, landed speed", "checks"),
    # ---------------------------------------------------------------- hover task
    Field("hover_target", [0.0, 0.0, 1.0], "m, hover task: target position", "hover"),
    Field("hover_duration", 10.0, "s, hover task: flight time", "hover"),
    Field("hover_ctrl_freq", 48, "Hz, hover task: stock PID rate (physics runs at freq)", "hover"),
    Field("hover_tol", 0.05, "m, hover task: final position error check", "hover"),
    # ---------------------------------------------------------------- plume
    Field("plume", "off", "draw live isolines of equal aero force: off, dw (downwash), gnd (ground effect), "
          "drag or all. Bare --plume means dw", "plume", ("off", "dw", "gnd", "drag", "all")),
    Field("plume_rate", 20.0, "Hz of sim time, isoline redraw rate", "plume"),
    Field("plume_levels", [0.02, 0.1, 0.5], "isoline levels, multiples of one module's weight W, weakest first",
          "plume"),
    # ---------------------------------------------------------------- logging
    Field("plot", False, "plot the flight log at the end (shared Logger, opens a window)", "logging"),
    Field("live_plot", False, "open a window that plots the run as it flies: height, horizontal error and tilt "
          "of each assembly, against their targets", "logging"),
    Field("live_plot_rate", 5.0, "Hz of sim time, live plot refresh rate", "logging"),
    Field("save_log", False, "save the flight log as .npy into output_folder", "logging"),
    Field("log_rate", 48, "Hz, flight log sample rate (used by plot and save_log)", "logging"),
]

BY_NAME = {f.name: f for f in FIELDS}
assert len(BY_NAME) == len(FIELDS), "duplicate field name"
assert all(f.group in GROUPS for f in FIELDS)
assert all(set(f.cages) <= set(CAGE_LAYOUTS) for f in FIELDS)
