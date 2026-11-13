"""Build the run config: field defaults < scenario YAML < command line.

    cfg = load(["--scenario", "scenarios/box/3x3/fail.yaml", "--sim", "mujoco"])

A scenario YAML is a flat mapping of field names (see fields.py / master.yaml).
Unknown keys and wrong types are errors, so a typo never silently falls back to a default.
"""
import argparse
import sys
from pathlib import Path
from types import SimpleNamespace

import yaml

from modular_uav.config import help as helpscreen
from modular_uav.config.fields import BY_NAME, CAGE_LAYOUTS, FIELDS, GROUPS
from modular_uav.utils.utils import str2bool

NULLS = ("null", "none", "~")


class ConfigError(ValueError):
    pass


def _convert(f, v, where):
    """Coerce one value (from YAML or a CLI string) to the field's type."""
    if v is None or (f.nullable and isinstance(v, str) and v.lower() in NULLS):   # cage: none is a choice, not null
        if f.nullable:
            return None
        raise ConfigError(f"{where}: {f.name} cannot be null")
    try:
        if f.kind == "bool":
            return str2bool(v)
        if f.kind == "list":
            # "3,3,3", "3 3 3", ["3,3,3"] (one quoted CLI word) and [3, 3, 3] all mean the same list
            items = [x for w in (v if isinstance(v, (list, tuple)) else [v])
                     for x in (str(w).replace(",", " ").split() if isinstance(w, str) else [w])]
            out = [f.elem_type(x) for x in items]
            if f.elem_type is int and any(float(x) != int(float(x)) for x in items):
                raise ValueError
            return out
        if f.name == "fail_mask" and isinstance(v, (list, tuple)):
            return ",".join(str(int(x)) for x in v)
        if f.elem_type is int:
            if float(v) != int(float(v)):
                raise ValueError
            return int(float(v))
        out = f.elem_type(v)
    except (ValueError, TypeError, argparse.ArgumentTypeError):
        raise ConfigError(f"{where}: {f.name} = {v!r} is not a valid {f.elem_type.__name__}"
                          + (" list" if f.kind == "list" else "")) from None
    if f.choices and out not in f.choices:
        raise ConfigError(f"{where}: {f.name} = {out!r}, choose from {', '.join(f.choices)}")
    return out


def read_yaml(path):
    """Scenario file -> {field: value}, checked against the registry."""
    path = Path(path)
    if not path.is_file():
        raise ConfigError(f"scenario file not found: {path}")
    data = yaml.safe_load(path.read_text()) or {}
    if not isinstance(data, dict):
        raise ConfigError(f"{path}: expected a mapping of field: value")
    unknown = sorted(set(data) - set(BY_NAME))
    if unknown:
        raise ConfigError(f"{path}: unknown field(s) {', '.join(unknown)} (see scenarios/master.yaml)")
    return {k: _convert(BY_NAME[k], v, str(path)) for k, v in data.items()}


def parser():
    p = argparse.ArgumentParser(
        prog="simulate.py", usage=helpscreen.USAGE, add_help=False,   # -h / --help: config/help.py
        description="Master script. Every field can come from a scenario YAML (--scenario) or a flag of the "
                    "same name; a flag wins over the file, the file wins over the default. "
                    "All fields with explanations: scenarios/master.yaml.",
        argument_default=argparse.SUPPRESS)
    p.add_argument("--scenario", metavar="YAML", help="scenario file, e.g. scenarios/box/3x3/fail.yaml")
    p.add_argument("--print_config", action="store_true", help="print the merged config as YAML and exit")
    groups = {g: p.add_argument_group(g, desc) for g, desc in GROUPS.items()}
    for f in FIELDS:
        h = f"{f.help} (default: {f.default})".replace("%", "%%")
        kw = dict(help=h, metavar=f.elem_type.__name__.upper())
        if f.kind == "bool":
            kw.update(nargs="?", const="true", metavar="BOOL")     # --gui  or  --gui false
        elif f.kind == "list":
            kw.update(nargs="+")
        elif f.choices:
            kw.update(metavar="{" + ",".join(f.choices) + "}")
        if f.name == "plume":
            kw.update(nargs="?", const="dw")
        groups[f.group].add_argument(f"--{f.name}", **kw)
    return p


def load(argv=None):
    """Merged, validated config as a namespace with one attribute per field."""
    if helpscreen.show(sys.argv[1:] if argv is None else argv):
        sys.exit(0)
    args = vars(parser().parse_args(argv))
    values = {f.name: f.default for f in FIELDS}
    try:
        source = {k: "default" for k in values}
        scenario = args.pop("scenario", None)
        if scenario:
            from_file = read_yaml(scenario)
            values.update(from_file)
            source.update({k: "scenario" for k in from_file})
        printing = args.pop("print_config", False)
        for k, v in args.items():
            values[k] = _convert(BY_NAME[k], v, "command line")
            source[k] = "cli"
        cfg = SimpleNamespace(**values, scenario=scenario)
        validate(cfg)
    except ConfigError as e:
        sys.exit(f"config error: {e}")
    cfg._source = source
    if printing:
        print(dump(cfg))
        sys.exit(0)
    return cfg


WORLD_GROUPS = ("run", "world", "physics", "plume", "logging")   # one value for the whole simulation


def derive(cfg, path):
    """Config of an included scenario: defaults < its file < the including file's own fields and the
    flags (`include` itself aside). Fields of WORLD_GROUPS always come from the including config."""
    values = {f.name: f.default for f in FIELDS}
    try:
        values.update(read_yaml(path))
        for f in FIELDS:
            if f.group in WORLD_GROUPS or cfg._source.get(f.name, "default") != "default":
                values[f.name] = getattr(cfg, f.name)
        values["include"] = []
        out = SimpleNamespace(**values, scenario=str(path))
        validate(out)
    except ConfigError as e:
        sys.exit(f"config error: {path}: {e}")
    out._source = {f.name: "derived" for f in FIELDS}
    return out


def dump(cfg):
    """Config as flat YAML, in registry order."""
    return yaml.safe_dump({f.name: getattr(cfg, f.name) for f in FIELDS}, sort_keys=False)


def _positive(text):
    try:
        return float(text) > 0
    except ValueError:
        return False


def validate(c):
    """Cross-field checks. Per-layout checks (fail_mask length) happen once the layout is built."""
    def need(ok, msg):
        if not ok:
            raise ConfigError(msg)
    if c.task == "dock":
        need(c.cage != "none", f"task: dock needs a cage ({', '.join(CAGE_LAYOUTS)})")
        need(c.layout in CAGE_LAYOUTS[c.cage],
             f"layout {c.layout} does not fit cage {c.cage}: "
             + "; ".join(f"{k} takes {' or '.join(v)}" for k, v in CAGE_LAYOUTS.items()))
    need(c.plume == "off" or c.gui, "plume draws in the viewer: set gui: true")
    # upstream f83fbdd: headless frames go through Image.fromarray on an int64 array, which current Pillow rejects
    need(not (c.record and c.sim == "pybullet" and not c.gui),
         "record on pybullet needs gui: true (PyBullet then writes an mp4; its headless PNG path crashes upstream)")
    need(len(c.box_size) == 3 and min(c.box_size) > 0, "box_size needs 3 positive values (x, y, z)")
    need(len(c.hover_target) == 3, "hover_target needs 3 values (x, y, z)")
    need(len(c.rows) >= 1 and min(c.rows) >= 1, "rows needs positive counts")
    need(c.size >= 1, "size must be 1 or more")
    need(c.tilt == "none" or c.cage == "dodeca", "tilt: alternate needs dodeca cages (cage symmetries)")
    need(c.fractal >= 1, "fractal is a number of levels, 1 or more (1 = the shape itself)")
    need(c.cage != "tiling" or c.fractal == 1, "fractal does not apply to tilings (they are not lattices)")
    need(c.drone_scale == "fit" or _positive(c.drone_scale), f"drone_scale is a number > 0 or fit, got {c.drone_scale!r}")
    need(c.task != "hover" or c.drone_scale == "1", "task: hover flies the stock PID, tuned for the CF2X: drone_scale 1")
    need(c.tile_edge > 0 and c.rod_density > 0 and c.include_gap >= 0, "tile_edge, rod_density > 0, include_gap >= 0")
    need(c.cage_color is None or (len(c.cage_color) == 4 and all(0 <= x <= 1 for x in c.cage_color)),
         "cage_color is r,g,b,a in 0..1, or null")
    for f in c.include:
        need(Path(f).is_file(), f"include: scenario file not found: {f}")
    need(c.live_plot_rate > 0, "live_plot_rate must be > 0")
    need(c.freq > 0 and c.hover_ctrl_freq > 0 and c.freq % c.hover_ctrl_freq == 0,
         "freq must be a positive multiple of hover_ctrl_freq")
    need(c.log_rate > 0 and c.freq % c.log_rate == 0, "freq must be a multiple of log_rate")
    need(0 <= c.outage <= 1, "outage is a fraction of thrust, 0 to 1")
    need(c.dw_dz_ramp >= 0 and (c.dw_min_width is None or c.dw_min_width >= 0), "dw_* values must be >= 0")
    need(all(x > 0 for x in (c.tetra_edge, c.triangle_edge, c.hex_edge, c.prism_height, c.dodeca_edge, c.cage_mass, c.cage_rod_radius, c.start_spacing, c.rise_t,
                             c.close_t, c.snap_gap, c.lap_t, c.climb_rate, c.land_speed, c.land_speed_low,
                             c.pos_wn, c.module_wn, c.att_wn_rp, c.att_wn_yaw, c.plume_rate)),
         "geometry, times, speeds, rates and gains must be > 0")
    need(list(c.plume_levels) == sorted(c.plume_levels) and min(c.plume_levels) > 0,
         "plume_levels must be positive and ascending")
