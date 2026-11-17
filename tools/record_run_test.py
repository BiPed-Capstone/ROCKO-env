#!/usr/bin/env python3
"""Host test for the parts of RecordRun.py that need no ROS (rocko_env/rocko_env/recording/RecordRun.py).

Run from the repo root:  python3 -I tools/record_run_test.py

Checks: a label becomes a safe folder name; run.yaml is valid YAML-as-JSON lines with every field
and "not recorded" for the ones nobody filled in; the commit is found for this checkout; and the
program refuses to record, without starting anything, when the disk has less free than asked.
What it cannot show: that a bag is recorded (verify_tools.sh, in the simulation) or anything on the Pi.
"""
import importlib.util
import json
import pathlib
import subprocess
import sys
import tempfile
import time
import types

REPO = pathlib.Path(__file__).resolve().parent.parent
SCRIPT = REPO / "rocko_env" / "rocko_env" / "recording" / "RecordRun.py"
spec = importlib.util.spec_from_file_location("RecordRun", SCRIPT)
rr = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rr)  # main() is behind `if __name__ == "__main__"`, so nothing runs


def check(condition, what):
    if not condition:
        raise AssertionError(what)


def test_label_becomes_a_safe_folder_name():
    check(rr.safe_label("pitch step #2") == "pitch_step__2", "spaces and symbols become _")
    check(rr.safe_label("../../etc") == "______etc", "a label cannot climb out of the log folder")
    check(rr.safe_label("") == "run" and rr.safe_label("   ") == "run", "an empty label is 'run'")
    check(len(rr.safe_label("x" * 500)) == 60, "a long label is cut")


def test_run_yaml_has_every_field_and_marks_the_empty_ones():
    args = types.SimpleNamespace(label="t", launch="rocko.launch.py", hardware="real", config="",
                                 battery_v="12.1", surface="", notes="")
    fields = rr.run_metadata(args, time.localtime())
    with tempfile.TemporaryDirectory() as folder:
        path = pathlib.Path(folder) / "run.yaml"
        rr.write_run_yaml(path, fields)
        lines = [line for line in path.read_text().splitlines() if not line.startswith("#")]
    values = {line.split(": ", 1)[0]: json.loads(line.split(": ", 1)[1])
              for line in lines if ": " in line and not line.startswith("  ")}
    check(values["battery_v"] == "12.1" and values["hardware"] == "real", "given fields are written as given")
    for empty in ("surface", "notes", "config", "config_sha256"):
        check(values[empty] == rr.NOT_RECORDED, f"{empty} was not given, so it must say so")
    check(values["commit"] not in ("", "unknown"), "the commit of this checkout must be found")
    check(values["tree"] in ("clean", "dirty"), "the tree is clean or dirty")
    topics = [json.loads(line[4:]) for line in lines if line.startswith("  - ")]
    check(topics == rr.RECORDED_TOPICS, "run.yaml lists exactly the recorded topics")
    check(len(set(topics)) == len(topics), "no topic is listed twice")


def test_refuses_to_record_when_the_disk_is_nearly_full():
    with tempfile.TemporaryDirectory() as folder:
        # Ask for more free space than any disk has: the program must stop before it starts ros2.
        result = subprocess.run(
            [sys.executable, "-I", str(SCRIPT), "--log-root", folder, "--min-free-mb", "1e15"],
            capture_output=True, text=True, timeout=20)
        check(result.returncode == 1, f"must exit 1, got {result.returncode}")
        check("NOT recording" in result.stderr, "must say why on stderr")
        check(list(pathlib.Path(folder).iterdir()) == [], "must not make a run folder")


def main():
    tests = [(name, fn) for name, fn in sorted(globals().items()) if name.startswith("test_") and callable(fn)]
    failed = 0
    for name, fn in tests:
        try:
            fn()
            print(f"PASS  {name}")
        except Exception as error:  # noqa: BLE001 - report every failure, not just the first
            failed += 1
            print(f"FAIL  {name}: {error!r}")
    print(f"{len(tests) - failed}/{len(tests)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    sys.exit(main())
