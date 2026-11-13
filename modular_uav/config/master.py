"""Render scenarios/master.yaml and one sub-master per cage, scenarios/<cage>/master.yaml,
from the field registry.

    python -m modular_uav.config.master          # rewrite all of them

It also writes docs/fields.html, the searchable field reference (config/help.py).

A sub-master is the short view of one cage: the main switches (SUB_SWITCHES) and the
module and layout fields that matter for that cage (Field.cages), with `cage` and `layout`
set to that cage and its first layout. Every other field keeps its master.yaml default. tests/test_config.py fails when any of
them and fields.py disagree.
"""
import textwrap
from pathlib import Path

import yaml

from modular_uav.config import help as helpscreen
from modular_uav.config.fields import CAGE_LAYOUTS, FIELDS, GROUPS

SCENARIOS = Path(__file__).resolve().parents[2] / "scenarios"
PATH = SCENARIOS / "master.yaml"
SUB_PATHS = {cage: SCENARIOS / cage / "master.yaml" for cage in CAGE_LAYOUTS}
SUB_GROUPS = ("module", "layout")                       # sub-masters list these groups in full ...
SUB_SWITCHES = ("task", "sim", "gui", "failure", "fail_mask", "circle")   # ... plus these fields
SUB_DEFAULTS = {"tiling": {"size": 2, "drone_scale": "fit"}}   # so every sub-master flies as is

HEADER = """\
# master.yaml: every field the master script understands, at its default value.
#
# What it is: the reference for all scenario files and command-line flags. Each key
#   here is also a flag of the same name: `meet_z: 1.2` in a file is `--meet_z 1.2`
#   on the command line. Precedence: flag > scenario file > default (this file).
# How to use it: copy the keys you want to change into a scenario file, or run it
#   as is (a 3 x 3 box grid docks, hovers and lands in PyBullet):
#     python simulate.py --scenario scenarios/master.yaml
# Generated from modular_uav/config/fields.py by `python -m modular_uav.config.master`.
#   Edit fields.py, not this file. Units: m, s, rad unless a name says _deg, _cm or _hz.
"""


SUB_HEADER = """\
# {cage}/master.yaml: the {cage} cage, its layouts and the main switches, at their defaults.
#
# What it is: the short view of scenarios/master.yaml for one cage. It lists the cage and
#   layout fields that matter for {cage} cages, plus the mission switches; `cage` and
#   `layout` are set to {cage} and {layout}. Every other field (physics, approach, failure
#   and circle details, controller gains, checks, ...) keeps the default and explanation
#   given in scenarios/master.yaml, and can be added here or set with a flag of the same name.
#   Precedence: flag > scenario file > default.
# How to use it: copy the keys you want to change into a scenario file, or run it as is
#   (the {layout} layout of {cage} cages docks, hovers and lands in PyBullet):
#     python simulate.py --scenario scenarios/{cage}/master.yaml
# Generated from modular_uav/config/fields.py by `python -m modular_uav.config.master`.
#   Edit fields.py, not this file. Units: m, s, rad unless a name says _deg, _cm or _hz.
"""


def fields(cage=None):
    """(field, value, choices) in registry order; cage: only that cage's fields, with its cage and layout."""
    for f in FIELDS:
        if cage is None:
            yield f, f.default, f.choices
        elif f.name in SUB_SWITCHES or (f.group in SUB_GROUPS and (not f.cages or cage in f.cages)):
            value = {"cage": cage, "layout": CAGE_LAYOUTS[cage][0], **SUB_DEFAULTS.get(cage, {})}.get(f.name, f.default)
            yield f, value, CAGE_LAYOUTS[cage] if f.name == "layout" else f.choices


def _value(v):
    return yaml.safe_dump({"x": v}, default_flow_style=True, width=1000)[4:-2].strip()


def render(cage=None):
    """master.yaml (cage None) or scenarios/<cage>/master.yaml, as text."""
    out = [HEADER if cage is None else SUB_HEADER.format(cage=cage, layout=CAGE_LAYOUTS[cage][0])]
    rows = list(fields(cage))
    for g, title in GROUPS.items():
        if not any(r[0].group == g for r in rows):
            continue
        out.append(f"\n# {'-' * 78}\n# {g}: {title}\n# {'-' * 78}\n")
        for f, value, choices in (r for r in rows if r[0].group == g):
            note = f.help + (f"  [choices: {', '.join(choices)}]" if choices else "")
            out += [f"# {line}\n" for line in textwrap.wrap(note, 88)]
            out.append(f"{f.name}: {_value(value)}\n")
    return "".join(out)


if __name__ == "__main__":
    for cage, path in [(None, PATH), *SUB_PATHS.items()]:
        path.parent.mkdir(exist_ok=True)
        path.write_text(render(cage))
        print(f"wrote {path}")
    helpscreen.HTML_PATH.write_text(helpscreen.html())
    print(f"wrote {helpscreen.HTML_PATH}")
