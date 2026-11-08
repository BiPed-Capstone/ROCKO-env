#!/usr/bin/env python3
"""The +/-30 % robustness sweep of the sim gains: scenarios U, I, V on a changed model.

Stage 5c-2 (docs/journal/2026-11-03-stage5c2-sweep.md; sim-plant.md §4.3). Four of SimPlant's
estimates are moved by -30 % and +30 %, the sim gains (rocko_sim_controllers.yaml as it is) stay
fixed, and verify_sim.sh's scenarios judge each case with their committed thresholds, unchanged.

  swept                 SimPlant parameter        nominal     -30 %       +30 %
  m  body mass          body_mass_kg              2.0 kg      1.4         2.6
  h  centre-of-mass     com_height_m              0.20 m      0.14        0.26
     height
  k  motor gain         motor_stall_torque_Nm     0.774725    0.5423075   1.0071425   (scales K_u,
                                                  N m                                  K_w and b)
  d  IMU delay          imu_delay_ms              20 ms       14          26          (whole ms)

Cases: the nominal model; each parameter alone at -30 % and +30 % (8, "one at a time"); and the 16
corners, every parameter at one of its extremes. Each case runs U and I once and V three times
(V's overshoot verdict is noisy: 23.5-29.1 % in 6 runs of the nominal model, stage 5c-1).

  case verdict   FAIL      any run of U, I or V failed
                 MARGINAL  all passed, but V's largest overshoot was above MARGINAL_OVERSHOOT_PCT
                 PASS      all passed with V's overshoot at or below it
                 COULD NOT JUDGE / INCOMPLETE  a run could not be judged / is missing

Subcommands, from the repo root on the laptop:

  docker compose -f docker/compose.yml exec -T ros2 python3 /ros2_ws/src/ROCKO-env/sim/parameter_sweep.py run
      inside the dev container (it calls verify_sim.sh; build first). ~25 cases x 5 launches, about
      an hour. Appends each run to <stem>.csv as it finishes, so a stopped sweep resumes where it
      stopped; then writes <stem>.md.
  python3 -I sim/parameter_sweep.py report
      laptop or container, seconds: <stem>.md again from <stem>.csv. It also runs the linear model
      (cascade_linearisation.py, same perturbations) for each case and says where it disagrees.

A result here is a result about a MODEL WHOSE PARAMETERS ARE ESTIMATES, with encoder_sign = -1
ASSUMED (proposal 0006). "Passes" means "balances in simulation under an estimated model", nothing
about the robot.
"""

import argparse
import csv
import itertools
import json
import os
import re
import subprocess
import sys
import tempfile
import time
from datetime import date

HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.dirname(HERE)
sys.path.insert(0, HERE)
import cascade_linearisation as lin  # noqa: E402  (standard library only; the linear model)

VERIFY_SIM = os.path.join(REPO, "verify_sim.sh")
DEFAULT_STEM = os.path.join(REPO, "sim", "results", "stage5c2-sweep")

# Today's values: the xacro defaults (sim/check_sim_config.py keeps the launch file's equal).
NOMINAL = {"body_mass_kg": 2.0, "com_height_m": 0.20, "motor_stall_torque_Nm": 0.774725, "imu_delay_ms": 20}
LETTER = {"body_mass_kg": "m", "com_height_m": "h", "motor_stall_torque_Nm": "k", "imu_delay_ms": "d"}
UNIT = {"body_mass_kg": "kg", "com_height_m": "m", "motor_stall_torque_Nm": "N m", "imu_delay_ms": "ms"}
SPREAD = 0.30  # +/-30 %, sim-plant.md §4.3
V_REPEATS = 3
# [%] V passes at <= V_OVERSHOOT_PCT (30 %). A case whose largest V overshoot passed but was above
# this is reported MARGINAL rather than PASS. Chosen with the sweep (stage 5c-2) as 3 points under
# the limit; no derivation was written down. For scale: V's old metric spread over 23.5-29.1 % in 6
# runs of one model (stage 5c-1), so 3 points is about half that run-to-run spread.
MARGINAL_OVERSHOOT_PCT = 27.0
# How verify_sim.sh launches each scenario, COPIED BY HAND from its scenario functions for the
# report's record; nothing checks it. If verify_sim.sh changes a scenario's launch, change it here.
CODE_SCENARIO_ARGS = {
    "U": "teleop:=false initial_pitch_deg:=2.0 encoder_sign:=-1",
    "I": "teleop:=false encoder_sign:=-1 impulse_time_s:=2.0 impulse_angular_velocity_deg_per_s:=100",
    "V": "teleop:=true encoder_sign:=-1 (fake gamepad: 0, then 0.1 m/s from 1 s)",
}
THRESHOLD_NAMES = ["U_SETTLE_FROM_S", "U_SETTLED_DEG", "U_MAX_SPEED_M_S", "I_SETTLE_S", "I_SETTLED_DEG",
                   "V_RISE_FRACTION", "V_RISE_S", "V_OVERSHOOT_PCT", "V_PITCH_DEG", "TIPOVER_DEG",
                   "SCENARIO_ENCODER_SIGN"]
COLUMNS = ["case", "kind", "body_mass_kg", "com_height_m", "motor_stall_torque_Nm", "imu_delay_ms",
           "scenario", "repeat", "verdict", "fell_at_s", "u_late_pitch_deg", "u_late_speed_m_s",
           "i_settle_s", "i_peak_pitch_deg", "v_rise_s", "v_overshoot_pct", "v_peak_pitch_deg",
           "reason", "commit", "verify_exit", "log"]


# ---- The cases -----------------------------------------------------------------------------------
def moved(name, sign):
    """The parameter at nominal x (1 + sign x 30 %). The IMU delay is a whole number of ms in
    SimPlant (imu_delay_ms is an int); 20 ms +/- 30 % is exactly 14 and 26 ms, nothing rounded."""
    value = NOMINAL[name] * (1 + sign * SPREAD)
    return int(round(value)) if name == "imu_delay_ms" else round(value, 7)


def case_id(params):
    """'nominal', or one sign per moved parameter: 'm-' (mass -30 %), 'm+h-k+d-' (a corner)."""
    parts = [LETTER[n] + ("-" if params[n] < NOMINAL[n] else "+") for n in NOMINAL if params[n] != NOMINAL[n]]
    return "".join(parts) or "nominal"


def all_cases(grid):
    cases = []
    if grid in ("oat", "all"):
        cases.append(("nominal", dict(NOMINAL)))
        for name in NOMINAL:
            for sign in (-1, +1):
                cases.append(("one at a time", dict(NOMINAL, **{name: moved(name, sign)})))
    if grid in ("corners", "all"):
        # Every combination of +30 % and -30 %, "+" first and the last parameter changing fastest:
        # m+h+k+d+, m+h+k+d-, m+h+k-d+, ... m-h-k-d-.
        for signs in itertools.product((+1, -1), repeat=len(NOMINAL)):
            cases.append(("corner", {n: moved(n, s) for n, s in zip(NOMINAL, signs)}))
    return [(case_id(p), kind, p) for kind, p in cases]


def overrides_of(params):
    return " ".join(f"{n}:={v}" for n, v in params.items())


# ---- Running verify_sim.sh (inside the container) --------------------------------------------------
def commit_of():
    try:
        sha = subprocess.run(["git", "-C", REPO, "rev-parse", "--short", "HEAD"], capture_output=True,
                             text=True, check=True).stdout.strip()
        dirty = subprocess.run(["git", "-C", REPO, "status", "--porcelain"], capture_output=True,
                               text=True, check=True).stdout.strip()
        return sha + ("+dirty" if dirty else "")
    except (OSError, subprocess.CalledProcessError):
        return "unknown"


def read_rows(csv_path):
    if not os.path.exists(csv_path):
        return []
    with open(csv_path, newline="", encoding="utf-8") as f:
        return list(csv.DictReader(f))


def number(metrics, key):
    value = metrics.get(key)
    return "" if value is None else f"{value:.4g}" if isinstance(value, float) else str(value)


def row_of(case, kind, params, scenario, repeat, verdict, metrics, reason, commit, status, log):
    row = {"case": case, "kind": kind, **{n: params[n] for n in NOMINAL}, "scenario": scenario,
           "repeat": repeat, "verdict": verdict, "fell_at_s": number(metrics, "fell_at_s"),
           "reason": reason, "commit": commit, "verify_exit": status, "log": log}
    if scenario == "U":
        row.update(u_late_pitch_deg=number(metrics, "max_abs_pitch_after_settle_from_deg"),
                   u_late_speed_m_s=number(metrics, "max_abs_speed_after_settle_from_m_s"))
    elif scenario == "I":
        row.update(i_settle_s=number(metrics, "settle_time_after_impulse_s"),
                   i_peak_pitch_deg=number(metrics, "peak_abs_pitch_after_impulse_deg"))
    else:
        row.update(v_rise_s=number(metrics, "rise_time_s"), v_overshoot_pct=number(metrics, "overshoot_pct"),
                   v_peak_pitch_deg=number(metrics, "peak_abs_pitch_deg"))
    return row


def run_verify(scenarios, params, log_path):
    """One `verify_sim.sh scenarios <names>` with the overrides. {name: (verdict, metrics)}, exit."""
    with tempfile.TemporaryDirectory() as metrics_dir, open(log_path, "w", encoding="utf-8") as log:
        env = dict(os.environ, SIM_PLANT_OVERRIDES=overrides_of(params), SCENARIO_METRICS_DIR=metrics_dir)
        try:
            status = subprocess.run([VERIFY_SIM, "scenarios", *scenarios], stdout=log,
                                    stderr=subprocess.STDOUT, env=env, timeout=900).returncode
        except subprocess.TimeoutExpired:
            status = "timeout"
        with open(log_path, encoding="utf-8") as f:
            text = f.read()
        results = {}
        for name in scenarios:
            # verify_sim.sh's own verdict line, which also counts the wiring and the release.
            found = re.search(rf"^\s*SCENARIO {name}: (PASS|FAIL|COULD NOT JUDGE)", text, re.MULTILINE)
            path = os.path.join(metrics_dir, f"{name}.json")
            metrics = {}
            if os.path.exists(path):
                with open(path, encoding="utf-8") as f:
                    metrics = json.load(f)
            verdict = found.group(1) if found else "COULD NOT JUDGE"
            reason = metrics.get("reason") or f"no verdict line (verify_sim.sh exit {status})"
            results[name] = (verdict, metrics, reason)
    return results, status


def run(args):
    stem = args.stem
    os.makedirs(os.path.dirname(stem), exist_ok=True)
    os.makedirs(args.log_dir, exist_ok=True)
    csv_path = stem + ".csv"
    done = {(r["case"], r["scenario"], r["repeat"]) for r in read_rows(csv_path)}
    commit = commit_of()
    cases = [c for c in all_cases(args.grid) if not args.only or c[0] in args.only.split(",")]
    print(f"parameter_sweep: {len(cases)} cases, U and I once, V x{args.v_repeats}; commit {commit}; "
          f"results -> {csv_path}; logs -> {args.log_dir}", flush=True)
    new_file = not os.path.exists(csv_path)
    with open(csv_path, "a", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=COLUMNS)
        if new_file:
            writer.writeheader()
        for case_index, (case, kind, params) in enumerate(cases, 1):
            # First U, I and the first V in one verify_sim.sh call, then each further V on its own.
            calls = [[("U", 1), ("I", 1), ("V", 1)]] + [[("V", k)] for k in range(2, args.v_repeats + 1)]
            for call in calls:
                todo = [(s, k) for s, k in call if (case, s, str(k)) not in done]
                if not todo:
                    continue
                names = [s for s, _ in todo]
                log = os.path.join(args.log_dir, f"{case}_{'-'.join(names)}_{todo[0][1]}.log")
                started = time.monotonic()
                results, status = run_verify(names, params, log)
                for s, k in todo:
                    verdict, metrics, reason = results[s]
                    writer.writerow(row_of(case, kind, params, s, k, verdict, metrics, reason, commit, status, log))
                    print(f"  [{case_index}/{len(cases)}] {case:<9} {s}#{k}: {verdict}: {reason}", flush=True)
                f.flush()
                print(f"      ({time.monotonic() - started:.0f} s, verify_sim.sh exit {status})", flush=True)
    report(args)


# ---- The report ----------------------------------------------------------------------------------
def thresholds():
    """THRESHOLD_NAMES as written in verify_sim.sh (text), read the same way the linear model reads them."""
    return lin.verify_sim_values(THRESHOLD_NAMES)


def linear_prediction(gains, params):
    """The linear model (cascade_linearisation.py) for this case: largest |z| and the screen of U, I, V,
    at the case's IMU delay, with SimPlant's write quirks."""
    lin.set_plant(params["body_mass_kg"], params["com_height_m"], params["motor_stall_torque_Nm"])
    try:
        delay = int(params["imu_delay_ms"])
        rad = lin.radius(gains, delay_ms=delay)
        screen = {s: lin.simulate(gains, s, delay_ms=delay) for s in ("U", "I", "V")}
    finally:
        lin.reset_plant()
    return rad, screen


def as_float(text):
    return float(text) if text not in ("", None, "None") else None


def failed_on(runs):
    """What failed, e.g. 'V overshoot 1/3' or 'U fell': V's overshoot alone is the verdict the
    verifier's own noise moves (stage 5c-1), so it is told apart from a fall or a slow recovery."""
    causes = []
    for s in ("U", "I", "V"):
        rs = [r for r in runs if r["scenario"] == s]
        bad = [r for r in rs if r["verdict"] == "FAIL"]
        if not bad:
            continue
        if any(r["fell_at_s"] for r in bad):
            what = "fell"
        elif s == "V" and all(r["reason"].startswith("overshoot") for r in bad):
            what = "overshoot"
        else:
            what = "limit"
        causes.append(f"{s} {what} {len(bad)}/{len(rs)}")
    return ", ".join(causes)


def case_verdict(runs, v_repeats):
    verdicts = [r["verdict"] for r in runs]
    if "FAIL" in verdicts:
        return f"FAIL ({failed_on(runs)})"
    if "COULD NOT JUDGE" in verdicts:
        return "COULD NOT JUDGE"
    if sorted((r["scenario"], r["repeat"]) for r in runs) != sorted(
            [("I", "1"), ("U", "1")] + [("V", str(k)) for k in range(1, v_repeats + 1)]):
        return "INCOMPLETE"
    overshoots = [as_float(r["v_overshoot_pct"]) for r in runs if r["scenario"] == "V"]
    return "MARGINAL" if max(overshoots) > MARGINAL_OVERSHOOT_PCT else "PASS"


def cell(runs, scenario):
    """The key metric(s) of a scenario's runs, for the table."""
    rs = [r for r in runs if r["scenario"] == scenario]
    if not rs:
        return "not run"
    fails = [r for r in rs if r["verdict"] != "PASS"]
    if scenario == "U":
        r = rs[0]
        text = f"{r['u_late_pitch_deg'] or '-'} deg, {r['u_late_speed_m_s'] or '-'} m/s"
    elif scenario == "I":
        r = rs[0]
        text = f"{r['i_settle_s'] or '-'} s (peak {r['i_peak_pitch_deg'] or '-'} deg)"
    else:
        over = [as_float(r["v_overshoot_pct"]) for r in rs if as_float(r["v_overshoot_pct"]) is not None]
        rise = [as_float(r["v_rise_s"]) for r in rs if as_float(r["v_rise_s"]) is not None]
        text = (f"{min(over):.1f}-{max(over):.1f} %" if over else "-") + (
            f", rise {min(rise):.2f}-{max(rise):.2f} s" if rise else "") + f" ({len(rs)} runs)"
    if any(r["fell_at_s"] for r in rs):
        text = "fell; " + text
    return ("**" + "/".join(r["verdict"] for r in fails) + "** " if fails else "") + text


def linear_cell(rad, screen):
    if rad >= 1.0:
        return f"UNSTABLE (\\|z\\| {rad:.4f})"
    parts = []
    for s in ("U", "I", "V"):
        res = screen[s]
        if res["fell_at_s"] is not None:
            parts.append(f"{s} fell")
        elif s == "V":
            parts.append(f"V {res['overshoot_pct']:.1f} %" + ("" if res["pass"] else " FAIL"))
        else:
            parts.append(s + (" pass" if res["pass"] else " FAIL"))
    return f"\\|z\\| {rad:.4f}; " + ", ".join(parts)


def agreement(runs, rad, screen):
    """Where the linear screen and the nonlinear sim disagree, per scenario (V: any repeat)."""
    diffs = []
    for s in ("U", "I", "V"):
        rs = [r for r in runs if r["scenario"] == s]
        if not rs or any(r["verdict"] == "COULD NOT JUDGE" for r in rs):
            continue
        sim_pass = all(r["verdict"] == "PASS" for r in rs)
        lin_pass = rad < 1.0 and screen[s]["pass"]
        if sim_pass != lin_pass:
            diffs.append(f"{s}: sim {'pass' if sim_pass else 'FAIL'}, linear {'pass' if lin_pass else 'FAIL'}")
    return "; ".join(diffs) or "agree"


def header_lines(stem, rows, gains):
    """The report's title, warnings, reproducibility block, legend and table header."""
    th = thresholds()
    commits = sorted({r["commit"] for r in rows})
    legend = ", ".join(f"{LETTER[n]} = {n} ({NOMINAL[n]} {UNIT[n]})" for n in NOMINAL)
    return [f"# Robustness sweep of the sim gains (+/-30 %): results",
           "",
           "Written by `sim/parameter_sweep.py` from `" + os.path.relpath(stem + ".csv", REPO) + "`; do not "
           "edit by hand. Explained in [the stage 5c-2 journal](../../docs/journal/2026-11-03-stage5c2-sweep.md).",
           "",
           "**A MODEL WITH ESTIMATED, UNMEASURED PARAMETERS** ([sim-plant.md §4](../../docs/architecture/sim-plant.md)), "
           "with `encoder_sign:=-1` **ASSUMED** (proposal 0006). PASS means \"balances in simulation under an "
           "estimated model\"; nothing here is about the robot.",
           "",
           "```",
           f"Report written: {date.today().isoformat()}",
           f"Commit(s):      {', '.join(commits)} (the commit the runs were made on; +dirty = uncommitted changes)",
           f"Command:        verify_sim.sh scenarios U I V, then verify_sim.sh scenarios V x{V_REPEATS - 1}, per case,",
           "                with SIM_PLANT_OVERRIDES=\"body_mass_kg:=.. com_height_m:=.. motor_stall_torque_Nm:=.. imu_delay_ms:=..\"",
           *[f"Launch ({s}):     rocko_sim.launch.py {a} + the case's four overrides" for s, a in CODE_SCENARIO_ARGS.items()],
           f"Config:         rocko_sim_controllers.yaml: pitch p {gains['pitch'][0]} i {gains['pitch'][1]} d {gains['pitch'][2]} "
           f"(D on pitch_rate: {gains['gyro_d']}), velocity p {gains['velocity'][0]} i {gains['velocity'][1]} d {gains['velocity'][2]}",
           "Thresholds:     " + ", ".join(f"{n}={v}" for n, v in th.items()) + " (verify_sim.sh, unchanged)",
           f"Case verdict:   FAIL if any run failed; MARGINAL if all passed but V's overshoot > {MARGINAL_OVERSHOOT_PCT:g} % in a run",
           "Battery:        n/a (sim; SimPlant's battery_voltage_V 12.0)",
           "Surface:        n/a (sim; no floor model)",
           "Hardware:       SimPlant (simulated), not the mock, not the robot",
           "Logs:           verify_sim.sh output per call, inside the dev container (column `log` of the CSV)",
           "```",
           "",
           legend + "; `-` = -30 %, `+` = +30 %. U: largest |pitch| and |v| from 5 s on. "
           "I: settle time after the push (peak |pitch|). V: overshoot min-max over the repeats, rise to 90 %. "
           "Linear model: `cascade_linearisation.py` with the same four changes (largest \\|z\\| per 10 ms "
           "cycle, < 1 = stable; time screen with SimPlant's write quirks).",
           "",
           "| Case | Kind | m kg | h m | k N m | d ms | U | I | V | **Verdict** | Linear model | Linear vs sim |",
           "| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |"]


def table_row(case, runs, gains):
    """One case's line of the table, and one line for each of its runs that did not pass."""
    first = runs[0]
    params = {n: float(first[n]) for n in NOMINAL}
    rad, screen = linear_prediction(gains, params)
    verdict = case_verdict(runs, V_REPEATS)
    line = (f"| `{case}` | {first['kind']} | {first['body_mass_kg']} | {first['com_height_m']} | "
            f"{first['motor_stall_torque_Nm']} | {first['imu_delay_ms']} | {cell(runs, 'U')} | "
            f"{cell(runs, 'I')} | {cell(runs, 'V')} | **{verdict}** | {linear_cell(rad, screen)} | "
            f"{agreement(runs, rad, screen)} |")
    failing = [f"- `{case}` {r['scenario']} run {r['repeat']}: {r['verdict']}: {r['reason']}"
               for r in runs if r["verdict"] != "PASS"]
    return line, failing


def report(args):
    stem = args.stem
    rows = read_rows(stem + ".csv")
    if not rows:
        print(f"parameter_sweep: no results in {stem}.csv; nothing to report")
        return 2
    gains = lin.read_yaml(lin.SIM_YAML)
    out = header_lines(stem, rows, gains)
    order = [c[0] for c in all_cases("all")]
    by_case = {}
    for r in rows:
        by_case.setdefault(r["case"], []).append(r)
    failing = []
    for case in sorted(by_case, key=lambda c: order.index(c) if c in order else len(order)):
        line, case_failing = table_row(case, by_case[case], gains)
        out.append(line)
        failing += case_failing
    counts = {}
    overshoot_only = 0
    for case, runs in by_case.items():
        v = case_verdict(runs, V_REPEATS)
        counts[v.split(" (")[0]] = counts.get(v.split(" (")[0], 0) + 1
        causes = failed_on(runs).split(", ")
        overshoot_only += v.startswith("FAIL") and all(c.startswith("V overshoot") for c in causes)
    out += ["", "**Cases by verdict:** " + ", ".join(f"{v} {n}" for v, n in sorted(counts.items()))
            + f". Of the FAIL cases, {overshoot_only} failed only on V's overshoot (no fall, U and I passed).", "",
            "## Runs that did not pass", ""] + (failing or ["None."]) + [""]
    with open(stem + ".md", "w", encoding="utf-8") as f:
        f.write("\n".join(out))
    print(f"parameter_sweep: wrote {stem}.md ({len(by_case)} cases, {len(rows)} runs): "
          + ", ".join(f"{v} {n}" for v, n in sorted(counts.items())))
    return 0


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    sub = parser.add_subparsers(dest="command", required=True)
    for name in ("run", "report"):
        p = sub.add_parser(name)
        p.add_argument("--stem", default=DEFAULT_STEM, help="results file name without .csv/.md")
        if name == "run":
            p.add_argument("--grid", choices=["oat", "corners", "all"], default="all",
                           help="oat: nominal + one at a time (9 cases); corners: the 16; all: both")
            p.add_argument("--only", help="comma-separated case names, e.g. nominal,m-")
            p.add_argument("--v-repeats", type=int, default=V_REPEATS)
            p.add_argument("--log-dir", default="/tmp/parameter_sweep_logs",
                           help="verify_sim.sh output, one file per call (not committed)")
    args = parser.parse_args()
    return run(args) if args.command == "run" else report(args)


if __name__ == "__main__":
    sys.exit(main() or 0)
