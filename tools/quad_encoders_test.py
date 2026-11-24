#!/usr/bin/env python3
"""Host test for the encoder node's two services (QuadEncoders.py; finding U-2). No ROS, no
encoders, no Raspberry Pi needed.

Run from the repo root:  python3 -I tools/quad_encoders_test.py

It loads QuadEncoders.py with small stand-ins for rclpy, the Encoder module and the service type,
gives it fake encoders (a count you set) and a fake clock (a time you set), and checks:
  * the node offers left_encoder_data and right_encoder_data and NO encoder_data;
  * each wheel's position and velocity come from ITS OWN encoder (the left reply never carries the
    right wheel's counts, and the other way round);
  * the right encoder's two pins are swapped on purpose (Encoder(b, a)); the left's are not;
  * the velocity divides by the MEASURED time between that wheel's calls: the same counts over
    0.02 s give half the velocity of 0.01 s, at the filter's steady state;
  * a call to the other wheel in between changes nothing (the old shared service called twice per
    cycle and read about half the true speed);
  * the first call, and a call with no time elapsed, give a finite velocity (no division by zero).
What it cannot show: the real encoder's counts, the 100 Hz timing on a Pi, or the robot. Tier 0 only.
"""
import importlib.util
import math
import pathlib
import sys
import types
from types import SimpleNamespace

REPO = pathlib.Path(__file__).resolve().parent.parent
NODE = REPO / "rocko_env" / "rocko_env" / "hardware" / "sensors" / "quadEncoder" / "QuadEncoders.py"


class FakeNode:
    def __init__(self, name):
        self.services = {}

    def declare_parameter(self, name, default):
        pass

    def get_parameter(self, name):
        pins = {"left_a_pin": 1, "left_b_pin": 2, "right_a_pin": 3, "right_b_pin": 4}
        value = SimpleNamespace(integer_value=pins[name])
        return SimpleNamespace(get_parameter_value=lambda: value)

    def create_service(self, srv_type, name, callback):
        self.services[name] = callback
        return name


class FakeEncoder:
    def __init__(self, pin_a, pin_b):
        self.pins = (pin_a, pin_b)
        self.counts = 0

    def read(self):
        return self.counts


def install_stubs():
    def module(name, **attrs):
        mod = types.ModuleType(name)
        mod.__dict__.update(attrs)
        sys.modules[name] = mod

    module("rclpy")
    module("rclpy.node", Node=FakeNode)
    module("Encoder", Encoder=FakeEncoder)
    module("rocko_interfaces")
    module("rocko_interfaces.srv", QuadEncoderData=None)


install_stubs()
spec = importlib.util.spec_from_file_location("QuadEncoders", NODE)
quad = importlib.util.module_from_spec(spec)
spec.loader.exec_module(quad)

CLOCK = [0.0]  # [s] what time.monotonic() returns in the node
quad.time = SimpleNamespace(monotonic=lambda: CLOCK[0])
COUNTS_PER_M = quad.COUNTS_PER_METER


def new_node():
    CLOCK[0] = 100.0  # an arbitrary start: monotonic time has no zero
    return quad.QuadEncoders()


def ask(node, wheel, counts, at_s):
    """Set the wheel's encoder to `counts` and the clock to at_s, call that wheel's service."""
    CLOCK[0] = at_s
    encoder = node.left_enc if wheel == "left" else node.right_enc
    encoder.counts = counts
    response = SimpleNamespace(position=None, velocity=None)
    return node.services[f"{wheel}_encoder_data"](None, response)


def settle(node, wheel, counts_per_cycle, period_s, cycles=400, skip_other=None):
    """Run `cycles` calls on one wheel at a constant speed; return the last reply."""
    t = CLOCK[0]
    counts = 0
    reply = ask(node, wheel, 0, t)  # the first call sets the reference
    for _ in range(cycles):
        t += period_s
        counts += counts_per_cycle
        if skip_other:  # the other wheel is asked in the same cycle, as the real robot does
            ask(node, skip_other, 0, t)
        reply = ask(node, wheel, counts, t)
    return reply


def check(condition, what):
    if not condition:
        raise AssertionError(what)


def test_two_services_and_no_old_one():
    node = new_node()
    check(sorted(node.services) == ["left_encoder_data", "right_encoder_data"],
          f"the node must offer exactly left_encoder_data and right_encoder_data; it offers {sorted(node.services)}")


def test_right_encoder_pins_are_swapped():
    node = new_node()
    check(node.left_enc.pins == (1, 2), f"left must use (a, b) = (1, 2); got {node.left_enc.pins}")
    check(node.right_enc.pins == (4, 3), f"right must use (b, a) = (4, 3); got {node.right_enc.pins}")


def test_each_wheel_reports_its_own_position():
    node = new_node()
    left = ask(node, "left", 1000, 100.00)
    right = ask(node, "right", -400, 100.00)
    check(math.isclose(left.position, 1000 / COUNTS_PER_M), f"left position wrong: {left.position}")
    check(math.isclose(right.position, -400 / COUNTS_PER_M),
          f"right position must come from the RIGHT encoder; got {right.position}")


def test_each_wheel_reports_its_own_velocity():
    node = new_node()
    left = settle(node, "left", counts_per_cycle=10, period_s=0.01)
    check(left.velocity > 0.0, "the left wheel moved forward, so its velocity must be positive")
    right = ask(node, "right", 0, CLOCK[0] + 0.01)  # right never moved
    check(right.velocity == 0.0, f"the right wheel never moved; its velocity is {right.velocity}")


def test_steady_speed_is_read_at_full_value():
    # 10 counts every 10 ms is a constant speed v. After many time constants the filter equals v,
    # and a call to the other wheel in the same cycle must not change that (finding U-2: it used to
    # halve it).
    expected_m_s = 10 / COUNTS_PER_M / 0.01
    for other in (None, "right"):
        node = new_node()
        reply = settle(node, "left", 10, 0.01, skip_other=other)
        check(math.isclose(reply.velocity, expected_m_s, rel_tol=1e-3),
              f"other wheel asked in between = {other}: read {reply.velocity} m/s, true speed {expected_m_s} m/s")


def test_velocity_uses_the_measured_time():
    # The same speed read at 50 Hz and at 100 Hz gives the same answer (counts scale with time). The
    # same COUNTS in twice the time are half the speed.
    fast = settle(new_node(), "left", 10, 0.01).velocity
    slow_same_speed = settle(new_node(), "left", 20, 0.02).velocity
    check(math.isclose(fast, slow_same_speed, rel_tol=1e-3),
          f"one speed must read the same at 10 ms and 20 ms cycles; got {fast} and {slow_same_speed}")
    slow_same_counts = settle(new_node(), "left", 10, 0.02).velocity
    check(math.isclose(slow_same_counts, fast / 2, rel_tol=1e-3),
          f"10 counts per 0.02 s must read half of 10 counts per 0.01 s; got {slow_same_counts} vs {fast}")


def test_first_call_is_safe():
    node = new_node()
    left = ask(node, "left", 5000, 100.0)  # encoder already has counts when the first call arrives
    check(left.velocity == 0.0, f"no earlier sample, so the first velocity must be 0.0; got {left.velocity}")
    check(math.isclose(left.position, 5000 / COUNTS_PER_M), "the first call must still report the position")


def test_no_time_elapsed_is_safe():
    node = new_node()
    ask(node, "left", 0, 100.00)
    before = ask(node, "left", 100, 100.01).velocity
    same_time = ask(node, "left", 200, 100.01)  # the clock did not advance
    check(math.isfinite(same_time.velocity) and same_time.velocity == before,
          f"dt = 0 must hold the last velocity; got {same_time.velocity} after {before}")
    backwards = ask(node, "left", 300, 100.00)  # the clock went back (it cannot, but never divide by it)
    check(math.isfinite(backwards.velocity) and backwards.velocity == before,
          f"dt < 0 must hold the last velocity; got {backwards.velocity}")
    # The reference sample was kept, so the next good call measures all the motion since 100.01.
    after = ask(node, "left", 300, 100.02).velocity
    check(after > before, "the motion during the skipped calls must show up in the next good call")


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
