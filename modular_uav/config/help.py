"""Help screens of the master script, built from the field registry.

    python simulate.py -h               short: the common flags, then every field name by group
    python simulate.py --help all       every field: flag, type, default, choices, explanation
    python simulate.py --help WORD      fields whose name, group, explanation or choices contain WORD
                                        (a group name such as `layout` shows that whole group)

docs/fields.html is the same reference as a searchable page (written by `python -m modular_uav.config.master`).
"""
import json
import shutil
import sys
import textwrap
from pathlib import Path

from modular_uav.config.fields import CAGE_LAYOUTS, FIELDS, GROUPS

HTML_PATH = Path(__file__).resolve().parents[2] / "docs" / "fields.html"
COMMON = ("sim", "gui", "cage", "layout", "size", "rows", "fill", "fractal", "tiling", "drone_scale", "include",
          "failure", "circle", "live_plot", "plume", "plot")
USAGE = "python simulate.py [--scenario FILE] [--print_config] [--FIELD VALUE ...]"


def _style():
    tty = sys.stdout.isatty()
    return (lambda s: f"\033[1m{s}\033[0m" if tty else s), (lambda s: f"\033[2m{s}\033[0m" if tty else s)


def _value(v):
    if isinstance(v, bool):
        return "true" if v else "false"
    if v is None:
        return "null"
    if isinstance(v, (list, tuple)):
        return ",".join(str(x) for x in v)
    return str(v)


def _kind(f):
    if f.kind == "bool":
        return "[BOOL]"
    if f.choices:
        return "{" + "|".join(f.choices) + "}"
    return f.elem_type.__name__.upper() + ("[,...]" if f.kind == "list" else "")


def short():
    bold, dim = _style()
    out = [f"usage: {USAGE}", "",
           "Every field is a key in a scenario YAML and a flag of the same name: `size: 3` in a file is",
           "`--size 3`. Precedence: flag > scenario file > default. Lists: --rows 2,1,2 or --rows 2 1 2.",
           "", bold("Common"),
           f"  {'--scenario FILE':22} scenario YAML, e.g. scenarios/dodeca/hexa6dof/base.yaml"]
    by = {f.name: f for f in FIELDS}
    for name in COMMON:
        f = by[name]
        first = f.help.split(". ")[0].split(": ")[0] if not f.choices else " | ".join(f.choices)
        out.append(f"  {'--' + name:22} {textwrap.shorten(first, 58, placeholder='...')}  {dim('(' + _value(f.default) + ')')}")
    out += ["", bold("All fields, by group") + "  (python simulate.py --help GROUP explains one group)"]
    width = shutil.get_terminal_size((100, 24)).columns
    for g in GROUPS:
        names = " ".join(f.name for f in FIELDS if f.group == g)
        out += textwrap.wrap(names, width - 2, initial_indent=f"  {g:11}", subsequent_indent=" " * 13)
    out += ["", "More: --help all (every field explained), --help WORD (search), --print_config (merged",
            "config as YAML), docs/fields.html (searchable page), scenarios/master.yaml (all defaults)."]
    return "\n".join(out)


def full(fields=None):
    """Every field (or the given ones), grouped: flag and type, default and cages, then the explanation."""
    bold, dim = _style()
    width = min(shutil.get_terminal_size((100, 24)).columns, 110)
    fields = FIELDS if fields is None else fields
    out = []
    for g, title in GROUPS.items():
        fs = [f for f in fields if f.group == g]
        if not fs:
            continue
        out += ["", bold(f"{g}: {title}")]
        for f in fs:
            meta = f"default {_value(f.default)}" + (f", {'/'.join(f.cages)} only" if f.cages else "")
            out.append(f"  {bold('--' + f.name)} {_kind(f)}  {dim(meta)}")
            out += textwrap.wrap(f.help, width - 6, initial_indent="      ", subsequent_indent="      ")
    return "\n".join(out[1:])


def search(word):
    w = word.lower()
    if w in GROUPS:
        return full([f for f in FIELDS if f.group == w])
    hits = [f for f in FIELDS if w in " ".join([f.name, f.group, f.help, *f.choices, *f.cages]).lower()]
    if not hits:
        return f"No field matches {word!r}. Try --help all, or a group: {', '.join(GROUPS)}."
    return f"{len(hits)} field(s) match {word!r}\n\n" + full(hits)


def show(argv):
    """Handle -h / --help [all | WORD] in argv: print and return True, else return False."""
    for k, a in enumerate(argv):
        if a in ("-h", "--help"):
            topic = argv[k + 1] if k + 1 < len(argv) and not argv[k + 1].startswith("-") else None
            print(short() if topic is None else full() if topic == "all" else search(topic))
            return True
    return False


def html():
    """docs/fields.html: the registry as a searchable page (self-contained, data inline)."""
    data = dict(groups=GROUPS, cages=list(CAGE_LAYOUTS), layouts=CAGE_LAYOUTS, fields=[
        dict(name=f.name, group=f.group, help=f.help, default=_value(f.default), kind=_kind(f),
             choices=list(f.choices), cages=list(f.cages)) for f in FIELDS])
    return PAGE.replace("/*DATA*/", json.dumps(data, separators=(",", ":")).replace("</", "<\\/"))


PAGE = (Path(__file__).with_name("fields_page.html")).read_text()
