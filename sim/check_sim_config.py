#!/usr/bin/env python3
"""Tier 0 drift guard for the simulated robot's configuration. No ROS needed, only PyYAML.

verify_urdf.sh runs it. It fails when two files that must agree have drifted apart:

  1. WIRING. rocko_sim_controllers.yaml must equal rocko_controllers.yaml in everything except the
     `gains` maps (controller names, types, dof_names, command and state interfaces, update_rate).
     Sim gains may differ; the wiring may not, or the sim would test a different cascade than the
     robot runs. Values are compared with their type: 100 and 100.0 differ. The only exceptions are
     the entries of INTENTIONAL_WIRING_DIFFERENCES, each allowed only with exactly the two values
     it names; an entry whose values no longer match either file fails, so it cannot go stale.
  2. GAINS STRUCTURE. A `gains` map also holds wiring and behaviour switches, so in each file the
     keys of a controller's `gains` must be exactly its `dof_names` (a joint with no entry silently
     gets p = i = d = 0), and every key under `gains` except the tunable numbers (TUNABLE_GAINS) must
     be equal in the two files (an added `angle_wraparound: true` is a different controller).
  3. DEFAULTS. A simulator setting's default is written in up to three places:
       SimPlantParams in SimPlantModel.hpp  (used by the unit test),
       <xacro:arg> in rocko.urdf.xacro      (used by the plugin: the real source of truth),
       SIM_ARGUMENTS in rocko_sim.launch.py (the settings you can set on the command line).
     Every xacro default must equal the header's, and every launch default must equal the xacro's.
     The names of the <param> tags in rocko.ros2_control.xacro must be exactly the header's fields,
     and each tag's text must be exactly `$(arg <its own name>)`, so a value cannot be wired to
     the wrong arg or hard-coded. A setting that is an `int` in the header must be written as an
     integer literal in the xacro and the launch file.
  4. IMU FAULT LIMIT. SimImuOutput copies kMaxConsecutiveImuFailures from ICM20948.hpp (a private
     member of a class that needs ROS headers, so the sim cannot use it directly), and relies on the
     tracker's default window limit, as ICM20948.hpp does.

Usage:  check_sim_config.py [--root DIR]     (DIR defaults to the repo root, above sim/)
Exit status: 0 = all agree   1 = a difference (printed)   2 = could not run (a file could not be
read or parsed, or PyYAML is missing)
"""

import argparse
import ast
import math
import os
import re
import sys
import xml.etree.ElementTree as ET

try:
    import yaml
except ImportError:
    print("ERROR: PyYAML is not installed (python3 -m pip install pyyaml); nothing was checked.")
    sys.exit(2)

XACRO_NS = "{http://www.ros.org/wiki/xacro}"
# The xacro args that are not simulator settings.
NON_SIM_XACRO_ARGS = {"prefix", "use_mock_hardware", "use_sim_plant"}
# The numbers of a `gains` entry that a person tunes. The sim file may change these; every other key
# under `gains` (angle_wraparound, antiwindup, ...) changes how the controller behaves and must match.
TUNABLE_GAINS = {"p", "i", "d", "i_clamp_max", "i_clamp_min", "u_clamp_max", "u_clamp_min",
                 "feedforward_gain"}

CONFIG = "rocko_env/rocko_env/bringup/config"
HEADER = "rocko_env/rocko_env/hardware/sim/include/rocko_env/SimPlantModel.hpp"
URDF_XACRO = "rocko_env/rocko_env/description/urdf/rocko.urdf.xacro"
CONTROL_XACRO = "rocko_env/rocko_env/description/ros2_control/rocko.ros2_control.xacro"
LAUNCH = "rocko_env/rocko_env/bringup/launch/rocko_sim.launch.py"
ICM_HEADER = "rocko_env/rocko_env/hardware/sensors/icm20948/include/rocko_env/ICM20948.hpp"


def read(root, relative):
    with open(os.path.join(root, relative)) as f:
        return f.read()


# ---- 1. wiring -----------------------------------------------------------------------------------
def without_gains(node):
    """The same data with every `gains` key removed, at any depth."""
    if isinstance(node, dict):
        return {k: without_gains(v) for k, v in node.items() if k != "gains"}
    return node


def flatten(node, path=""):
    """{"a/b/c": value} for every leaf, so a difference can be printed as one line."""
    if isinstance(node, dict):
        leaves = {}
        for key, value in node.items():
            leaves.update(flatten(value, f"{path}/{key}"))
        return leaves
    return {path: node}


def load_controllers(root, file_name):
    return yaml.safe_load(read(root, f"{CONFIG}/{file_name}"))


def differences(hardware, sim, what):
    """One problem line for every key whose value differs between two flattened maps, or is in one
    only. repr() compares the type too: 100 and 100.0 are different YAML (an int and a float)."""
    problems = []
    for key in sorted(set(hardware) | set(sim)):
        h = repr(hardware.get(key, "<absent>"))
        s = repr(sim.get(key, "<absent>"))
        if h != s:
            problems.append(f"{what} differs at {key}: rocko_controllers.yaml has {h}, "
                            f"rocko_sim_controllers.yaml has {s}")
    return problems


# Wiring the sim may have on purpose and the robot does not (yet), as
# {flattened key: (value in rocko_controllers.yaml, value in rocko_sim_controllers.yaml)}.
# Keep this list as short as possible: every entry means the sim tests a cascade the robot does not run.
INTENTIONAL_WIRING_DIFFERENCES = {
    # H-6, sim only; real YAML change is proposal-gated. The pitch PID takes its D term from the
    # gyro's pitch_rate state instead of differentiating the delayed pitch angle.
    "/pitch_pid_controller/ros__parameters/reference_and_state_interfaces":
        (["pitch"], ["pitch", "pitch_rate"]),
}


def check_wiring(root):
    hardware = flatten(without_gains(load_controllers(root, "rocko_controllers.yaml")))
    sim = flatten(without_gains(load_controllers(root, "rocko_sim_controllers.yaml")))
    problems = []
    for key, (hardware_value, sim_value) in INTENTIONAL_WIRING_DIFFERENCES.items():
        found = (hardware.get(key, "<absent>"), sim.get(key, "<absent>"))
        if repr(found) == repr((hardware_value, sim_value)):
            del hardware[key], sim[key]  # exactly the allowed difference: do not compare it
        else:
            problems.append(
                f"allowed wiring difference at {key} no longer matches: expected "
                f"rocko_controllers.yaml {hardware_value!r} and rocko_sim_controllers.yaml "
                f"{sim_value!r}, found {found[0]!r} and {found[1]!r}. If the change was made on "
                "purpose (e.g. the proposal was applied), update or remove the entry in "
                "INTENTIONAL_WIRING_DIFFERENCES")
    problems += differences(hardware, sim, "wiring")
    return (problems,
            f"{len(hardware)} wiring values equal, with their types, plus "
            f"{len(INTENTIONAL_WIRING_DIFFERENCES)} allowed difference(s) "
            f"(INTENTIONAL_WIRING_DIFFERENCES; gains maps: next check)")


# ---- 2. gains structure --------------------------------------------------------------------------
def gains_of(controllers):
    """{controller: {dof: {key: value}}} for every controller that has dof_names or a gains map."""
    found = {}
    for name, body in controllers.items():
        params = body.get("ros__parameters", {}) if isinstance(body, dict) else {}
        if "dof_names" in params or "gains" in params:
            found[name] = {"dof_names": params.get("dof_names", []), "gains": params.get("gains")}
    return found


def check_gains(root):
    problems = []
    tunable_free = {}
    for file_name in ("rocko_controllers.yaml", "rocko_sim_controllers.yaml"):
        side = {}
        for controller, entry in gains_of(load_controllers(root, file_name)).items():
            gains = entry["gains"]
            if not isinstance(gains, dict):
                problems.append(f"{file_name}: {controller} has dof_names but no `gains` map")
                continue
            if set(gains) != set(entry["dof_names"]):
                problems.append(
                    f"{file_name}: {controller} `gains` keys {sorted(gains)} are not its dof_names "
                    f"{sorted(entry['dof_names'])} (a joint with no entry gets p = i = d = 0)")
            for dof, numbers in gains.items():
                if not isinstance(numbers, dict):
                    problems.append(f"{file_name}: {controller} gains for {dof} is not a map")
                    continue
                for key, value in numbers.items():
                    if key not in TUNABLE_GAINS:
                        side[f"/{controller}/gains/{dof}/{key}"] = value
        tunable_free[file_name] = side
    problems += differences(tunable_free["rocko_controllers.yaml"],
                            tunable_free["rocko_sim_controllers.yaml"], "gains setting")
    return problems, ("each controller's `gains` keys are its dof_names in both files; every gains "
                      "key except the tunable numbers is equal")


# ---- 3. defaults ---------------------------------------------------------------------------------
def header_defaults(root):
    """{field name: (type, default text)} for the members of struct SimPlantParams."""
    text = read(root, HEADER)
    body = text[text.index("struct SimPlantParams"):]
    body = body[:body.index("\n};")]
    return {name: (kind, value.strip())
            for kind, name, value in re.findall(r"^\s*(double|int|bool)\s+(\w+)\s*=\s*([^;]+);",
                                                body, flags=re.MULTILINE)}


# The one non-literal default in the header (motor_no_load_speed_rad_s, 1150 RPM in rad/s). Any other
# expression is not understood on purpose: float() rejects it and the check says so.
KNOWN_EXPRESSIONS = {"1150.0 / 60.0 * 2.0 * kPi": 1150.0 / 60.0 * 2.0 * math.pi}


def as_number(text):
    """A number from '60.0', '-1', '+1.0' or a text in KNOWN_EXPRESSIONS. ValueError otherwise."""
    text = text.strip()
    if text in KNOWN_EXPRESSIONS:
        return KNOWN_EXPRESSIONS[text]
    return float(text)


def same(kind, a, b):
    """Do two default texts mean the same value for a setting of this C++ type?"""
    if kind == "bool":
        return a.strip() == b.strip()
    return math.isclose(as_number(a), as_number(b), rel_tol=1e-12, abs_tol=0.0)


def xacro_defaults(root):
    tree = ET.parse(os.path.join(root, URDF_XACRO))
    return {a.get("name"): a.get("default") for a in tree.getroot().iter(f"{XACRO_NS}arg")
            if a.get("name") not in NON_SIM_XACRO_ARGS}


def launch_defaults(root):
    for node in ast.parse(read(root, LAUNCH)).body:
        if isinstance(node, ast.Assign) and node.targets[0].id == "SIM_ARGUMENTS":
            return {name: default for name, (default, _) in ast.literal_eval(node.value).items()}
    raise ValueError("SIM_ARGUMENTS not found")


def control_xacro_params(root):
    """{name: text} for the <param> tags in the SimPlant block of rocko.ros2_control.xacro."""
    text = read(root, CONTROL_XACRO)
    block = text[text.index('<ros2_control name="SimRocko"'):]
    block = block[:block.index("</hardware>")]
    return dict(re.findall(r'<param name="(\w+)">([^<]*)</param>', block))


INTEGER_LITERAL = r"[+-]?\d+"


def check_defaults(root):
    header = header_defaults(root)
    xacro = xacro_defaults(root)
    launch = launch_defaults(root)
    params = control_xacro_params(root)
    problems = []

    for name in sorted(set(header) - set(xacro)):
        problems.append(f"SimPlantParams has '{name}' but rocko.urdf.xacro has no such arg")
    for name in sorted(set(xacro) - set(header)):
        problems.append(f"rocko.urdf.xacro has arg '{name}' but SimPlantParams has no such member")
    for name in sorted(set(header) - set(params)):
        problems.append(f"rocko.ros2_control.xacro gives SimPlant no <param name=\"{name}\">")
    for name in sorted(set(params) - set(header)):
        problems.append(f"rocko.ros2_control.xacro gives SimPlant <param name=\"{name}\">, "
                        "which SimPlantParams does not have")
    for name, text in sorted(params.items()):
        if text != f"$(arg {name})":
            problems.append(f"rocko.ros2_control.xacro: <param name=\"{name}\"> is {text!r}, "
                            f"not '$(arg {name})'")
    for name, (kind, header_default) in sorted(header.items()):
        if name not in xacro:
            continue
        try:
            if not same(kind, header_default, xacro[name]):
                problems.append(f"default of '{name}': SimPlantModel.hpp says {header_default}, "
                                f"rocko.urdf.xacro says {xacro[name]}")
        except ValueError as error:
            problems.append(f"default of '{name}': cannot compare SimPlantModel.hpp "
                            f"({header_default}) with rocko.urdf.xacro ({xacro[name]}): {error}")
        if kind == "int" and not re.fullmatch(INTEGER_LITERAL, xacro[name]):
            problems.append(f"default of '{name}' in rocko.urdf.xacro is {xacro[name]!r}, but the "
                            "setting is an int in SimPlantModel.hpp: write an integer literal")
    for name, launch_default in sorted(launch.items()):
        if name not in xacro:
            problems.append(f"rocko_sim.launch.py SIM_ARGUMENTS has '{name}', which "
                            "rocko.urdf.xacro does not have")
            continue
        if name not in header:
            continue  # already reported above: the xacro has an arg SimPlantParams does not
        kind = header[name][0]
        try:
            if not same(kind, xacro[name], launch_default):
                problems.append(f"default of '{name}': rocko.urdf.xacro says {xacro[name]}, "
                                f"rocko_sim.launch.py says {launch_default}")
        except ValueError as error:
            problems.append(f"default of '{name}': cannot compare rocko.urdf.xacro "
                            f"({xacro[name]}) with rocko_sim.launch.py ({launch_default}): {error}")
        if kind == "int" and not re.fullmatch(INTEGER_LITERAL, launch_default):
            problems.append(f"default of '{name}' in rocko_sim.launch.py is {launch_default!r}, but "
                            "the setting is an int in SimPlantModel.hpp: write an integer literal")
    return problems, (f"{len(header)} simulator settings: header, xacro and ros2_control params "
                      f"agree; {len(launch)} launch-file defaults match the xacro")


# ---- 4. IMU fault limit --------------------------------------------------------------------------
def check_imu_limit(root):
    icm = read(root, ICM_HEADER)
    sim = read(root, HEADER)
    problems = []
    icm_limit = re.search(r"kMaxConsecutiveImuFailures\s*=\s*(\d+)", icm)
    sim_limit = re.search(r"kMaxConsecutiveFailures\s*=\s*(\d+)", sim)
    if not icm_limit or not sim_limit:
        problems.append("could not find the consecutive-failure limit in ICM20948.hpp or SimPlantModel.hpp")
    elif icm_limit.group(1) != sim_limit.group(1):
        problems.append(f"consecutive IMU failures that trip the fault: ICM20948.hpp says "
                        f"{icm_limit.group(1)}, SimPlantModel.hpp says {sim_limit.group(1)}")
    if not re.search(r"kMaxWindowImuFailures\s*=\s*ImuFaultTracker::kMaxWindowFailures", icm):
        problems.append("ICM20948.hpp no longer uses the tracker's default window limit, which "
                        "SimImuOutput relies on")
    return problems, "IMU fault limit equal in ICM20948.hpp and SimPlantModel.hpp"


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("--root", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."),
                        help="repo root (a copy of the files is enough)")
    args = parser.parse_args()

    failed = False
    for check in (check_wiring, check_gains, check_defaults, check_imu_limit):
        try:
            problems, summary = check(args.root)
        except (OSError, ValueError, KeyError, yaml.YAMLError, ET.ParseError) as error:
            print(f"ERROR {check.__name__}: {type(error).__name__}: {error}")
            return 2
        if problems:
            failed = True
            for problem in problems:
                print(f"FAIL  {problem}")
        else:
            print(f"ok    {summary}")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
