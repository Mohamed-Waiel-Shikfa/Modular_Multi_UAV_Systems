"""Config layer: master.yaml in sync with the registry, precedence, and input checks."""
from pathlib import Path

import pytest
import yaml

from modular_uav.config import load
from modular_uav.config.fields import FIELDS
from modular_uav.config.master import PATH, SUB_PATHS, render

ROOT = Path(__file__).resolve().parents[1]


def test_master_yaml_matches_registry():
    assert PATH.read_text() == render(), "run: python -m modular_uav.config.master"
    data = yaml.safe_load(PATH.read_text())
    assert list(data) == [f.name for f in FIELDS]
    assert all(data[f.name] == f.default for f in FIELDS)


@pytest.mark.parametrize("cage", sorted(SUB_PATHS))
def test_sub_master_yaml_matches_registry(cage):
    path = SUB_PATHS[cage]
    assert path.read_text() == render(cage), "run: python -m modular_uav.config.master"
    c = load(["--scenario", str(path)])
    assert c.cage == cage and all(f.name in yaml.safe_load(path.read_text()) for f in FIELDS
                                  if f.cages and cage in f.cages)


@pytest.mark.parametrize("path", sorted((ROOT / "scenarios").rglob("*.yaml")), ids=lambda p: str(p.relative_to(ROOT)))
def test_every_scenario_loads_and_has_a_header(path):
    assert path.read_text().startswith("#"), "every YAML starts with a short explainer"
    load(["--scenario", str(path)])


def test_precedence_cli_over_yaml_over_default(tmp_path):
    f = tmp_path / "s.yaml"
    f.write_text("meet_z: 1.5\nradius: 2.0\n")
    c = load(["--scenario", str(f), "--meet_z", "2.5"])
    assert (c.meet_z, c.radius, c.laps) == (2.5, 2.0, 2.0)
    assert (c._source["meet_z"], c._source["radius"], c._source["laps"]) == ("cli", "scenario", "default")


def test_bools_lists_and_nulls():
    c = load(["--gui", "--circle", "false", "--box_size", "0.13", "0.13", "0.06", "--z0", "null"])
    assert c.gui is True and c.circle is False and c.box_size == [0.13, 0.13, 0.06] and c.z0 is None
    c = load(["--plume", "--gui"])
    assert c.plume == "dw"


@pytest.mark.parametrize("argv", [["--sim", "gazebo"], ["--size", "2.5"], ["--layout", "pyramid"],
                                  ["--plume", "all"], ["--outage", "1.5"], ["--meet_z", "null"],
                                  ["--cage", "hex", "--layout", "pyramid"],
                                  ["--cage", "box", "--tilt", "alternate"], ["--rows", "2", "0"]])
def test_bad_input_exits(argv):
    with pytest.raises(SystemExit):
        load(argv)


def test_fields_html_matches_registry():
    from modular_uav.config import help as helpscreen
    assert helpscreen.HTML_PATH.read_text() == helpscreen.html(), "run: python -m modular_uav.config.master"


@pytest.mark.parametrize("argv, expect", [(["-h"], "Common"), (["--help", "all"], "--dw_dz_ramp"),
                                          (["--help", "tilt"], "field(s) match"), (["--help", "layout"], "--fractal")])
def test_help_screens(argv, expect, capsys):
    with pytest.raises(SystemExit) as e:
        load(argv)
    assert e.value.code == 0 and expect in capsys.readouterr().out


def test_lists_take_commas_or_spaces():
    """`--rows 2,1,2`, `--rows "2, 1, 2"` and `--rows 2 1 2` are the same list."""
    for argv in (["--rows", "2,1,2"], ["--rows", "2, 1, 2"], ["--rows", "2", "1", "2"], ["--rows", "2,1", "2"]):
        assert load(argv).rows == [2, 1, 2]


def test_unknown_yaml_key_exits(tmp_path):
    f = tmp_path / "s.yaml"
    f.write_text("meetz: 1.5\n")
    with pytest.raises(SystemExit):
        load(["--scenario", str(f)])


def test_included_scenarios_take_world_fields_from_the_includer(tmp_path):
    from modular_uav.config.loader import derive
    world = tmp_path / "w.yaml"
    world.write_text(f"include: [{ROOT}/scenarios/hex/7/base.yaml]\ncircle: true\nsim: mujoco\n")
    c = load(["--scenario", str(world), "--freq", "480"])
    d = derive(c, c.include[0])
    assert (d.cage, d.layout, d.circle, d.sim, d.freq, d.include) == ("hex", "hexagon", True, "mujoco", 480, [])
