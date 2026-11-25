#!/usr/bin/env python3
"""Host test for the arithmetic of tools/loop_timing.py (no ROS). Run: python3 -I tools/loop_timing_test.py"""

import importlib.util
import math
import pathlib

spec = importlib.util.spec_from_file_location(
    "loop_timing", pathlib.Path(__file__).resolve().parent / "loop_timing.py")
lt = importlib.util.module_from_spec(spec)
spec.loader.exec_module(lt)


def check(condition, what):
    if not condition:
        raise AssertionError(what)


def test_summarise_nearest_rank():
    n, median, p99, biggest = lt.summarise(list(range(1, 101)))  # 1..100
    check((n, median, p99, biggest) == (100, 50, 99, 100), f"got {(n, median, p99, biggest)}")


def test_summarise_drops_nan_and_handles_empty():
    check(lt.summarise([math.nan, 3.0]) == (1, 3.0, 3.0, 3.0), "NaN must be dropped")
    check(lt.summarise([]) == (0, None, None, None), "empty input")
    check(lt.summarise([math.nan]) == (0, None, None, None), "all NaN")


def test_cycle_series_converts_rate_to_period():
    names = ["controller_manager.total_time",
             "Gyro.stats/read_cycle/periodicity/current_value",
             "joint_state_broadcaster.stats/periodicity/current_value",
             "Gyro.stats/read_cycle/execution_time/current_value",
             "Gyro.stats/read_cycle/periodicity/max"]  # not a per-cycle value: ignored
    s = lt.cycle_series(names, [[80.0, 100.0, 50.0, 1500.0, 101.0], [90.0, 0.0, 100.0, 1600.0, 101.0]])
    check(s["controller_manager.total_time [us]"] == [80.0, 90.0], "total time kept as is")
    check(s["Gyro read period [ms]"][0] == 10.0 and math.isnan(s["Gyro read period [ms]"][1]),
          "100 Hz is 10 ms; a rate of 0 is NaN, not a division by zero")
    check(s["joint_state_broadcaster period [ms]"] == [20.0, 10.0], "controller period")
    check(s["Gyro read exec [us]"] == [1500.0, 1600.0], "execution time")
    check(len(s) == 4, f"the max entry must be ignored; got {sorted(s)}")


def test_owner_of_read_write_and_controller():
    check(lt.owner_of("LeftDriveMotor.stats/write_cycle/periodicity/current_value") == "LeftDriveMotor write", "write")
    check(lt.owner_of("Gyro.stats/read_cycle/execution_time/current_value") == "Gyro read", "read")
    check(lt.owner_of("joint_state_broadcaster.stats/periodicity/current_value") == "joint_state_broadcaster",
          "a controller has no read or write")


def main():
    tests = [(n, f) for n, f in sorted(globals().items()) if n.startswith("test_")]
    failed = 0
    for name, fn in tests:
        try:
            fn()
            print(f"PASS  {name}")
        except Exception as error:
            failed += 1
            print(f"FAIL  {name}: {error!r}")
    print(f"{len(tests) - failed}/{len(tests)} passed")
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
