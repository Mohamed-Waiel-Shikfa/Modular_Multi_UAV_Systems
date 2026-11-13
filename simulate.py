#!/usr/bin/env python3
"""Master script: every experiment runs through here.

  python simulate.py --scenario scenarios/box/3x3/fail.yaml
  python simulate.py --scenario scenarios/box/3x3/fail.yaml --sim mujoco --downwash_model original
  python simulate.py --cage dodeca --layout sierpinski --size 2 --circle --gui
  python simulate.py -h                        # short help; --help all, --help WORD, docs/fields.html

Precedence: command-line flag > scenario YAML > default. Field names are the same in YAML
and on the command line. Exits 0 only if every self-check of the run passed.
"""
import sys

from modular_uav.config import load
from modular_uav.tasks import run

if __name__ == "__main__":
    sys.exit(run(load(sys.argv[1:])))
