"""Self-checks of a dock run: every phase reached, and each did what it should.

log: what mission.run recorded per phase. Thresholds are the check_* fields.
"""


def evaluate(cfg, log, n, failing, rest_z):
    """Ordered {description: passed}. failing: the fail mask (None without a failure segment)."""
    c = cfg
    checks = {
        f"all {n} modules snapped within snap_gap ({100 * c.snap_gap:g} cm)": "snap" in log,
        f"assembly hovers (< {c.check_hover_cm:g} cm) and stays rigid (< {c.check_slot_cm:g} cm slot error)":
            "hover" in log and log["hover"]["err_cm"] < c.check_hover_cm and log["hover"]["slot_cm"] < c.check_slot_cm,
    }
    if failing is not None:
        if failing.any() and c.outage < 1:
            checks[f"failure measurably upsets the assembly (> {c.check_upset_deg:g} deg tilt or "
                   f"> {100 * c.check_upset_drop:g} cm drop)"] = \
                "failure" in log and (log["failure"]["max_tilt"] > c.check_upset_deg
                                      or log["failure"]["drop"] > c.check_upset_drop)
        checks["recovered to a stable hover without touching the ground"] = \
            "recover" in log and log["recover"]["z_min"] > rest_z + c.check_recover_margin
        checks[f"final hover holds (< {c.check_final_cm:g} cm)"] = \
            "final" in log and log["final"]["err_cm"] < c.check_final_cm
    if c.circle:
        checks[f"circle flown (worst tracking error < {c.check_circle_cm:g} cm)"] = \
            "circle" in log and log["circle"]["err_cm"] < c.check_circle_cm
    checks["landed upright and at rest"] = (
        "land" in log and log["land"]["z"] < rest_z + c.check_land_cm / 100
        and log["land"]["tilt"] < c.check_land_tilt_deg and log["land"]["speed"] < c.check_land_speed)
    return checks


def report(checks):
    """Print PASS/FAIL lines; exit code 0 if all passed."""
    print()
    for name, ok in checks.items():
        print(f"  [{'PASS' if ok else 'FAIL'}] {name}")
    return 0 if all(checks.values()) else 1
