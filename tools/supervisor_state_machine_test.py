#!/usr/bin/env python3
"""Host test for the supervisor's rules. No ROS, no robot.

Run from the repo root:  python3 -I tools/supervisor_state_machine_test.py

Checks rocko_env/supervisor/state_machine.py against the numbered rules in
docs/architecture/robot-supervisor.md (boot, enable, mode change, disable, auto-disable, refusal),
both gamepad combos, auto-disable on a silent controller, and the fault path; and modes.py.
What it cannot show: Supervisor.py (processes, controller_manager), ROS timing, the robot. Not a tier.
"""
import pathlib
import sys

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "rocko_env"))
from rocko_env.supervisor import modes  # noqa: E402
from rocko_env.supervisor import state_machine  # noqa: E402
from rocko_env.supervisor.state_machine import (  # noqa: E402
    CM_UNANSWERED_FAULT_S, CM_UNANSWERED_KILL_S, DEACTIVATE_CONFIRM_S, StateMachine)
from rocko_env.teleop import gamepad  # noqa: E402

LB_RB = {"lb", "rb"}
LT_RT_RB = {"lt", "rt", "rb"}


def check(condition, what):
    if not condition:
        raise AssertionError(what)


def ready_machine(mode="tank"):
    """A machine that has booted into `mode` and is ready (disabled)."""
    m = StateMachine(mode)
    m.take_actions()
    m.stack_ready()
    return m


def test_rule1_boot():
    m = StateMachine()
    check((m.mode, m.enabled, m.phase, m.last_source) == ("tank", False, "starting", "boot"), "boot state")
    check(m.take_actions() == [("START_STACK", "tank")], "boot starts tank's stack")
    check(m.take_actions() == [], "actions are drained")
    m.stack_ready()
    check(m.phase == "ready" and not m.enabled, "ready, still disabled")


def test_rule2_enable_needs_ready():
    m = StateMachine()
    ok, msg = m.request_enable(True, "dashboard")
    check(not ok and not m.enabled and "starting" in msg, "refused while starting")
    m = ready_machine()
    ok, _ = m.request_enable(True, "dashboard")
    check(ok and m.enabled and m.last_source == "dashboard", "enabled when ready")
    check(m.take_actions() == [("ACTIVATE",)], "tank enable activates, no re-arm")


def test_rule2_balance_pitch():
    m = ready_machine("balance")
    for pitch, band, why in [(None, 5.0, "unknown pitch"), (1.0, None, "unknown band"),
                             (float("nan"), 5.0, "nan pitch"), (6.0, 5.0, "outside band"),
                             (-6.0, 5.0, "outside band, negative")]:
        ok, msg = m.request_enable(True, "gamepad", pitch, band)
        check(not ok and not m.enabled and msg, f"refused: {why}")
    check(m.take_actions() == [], "refusals queue nothing")
    ok, _ = m.request_enable(True, "gamepad", -4.9, 5.0)
    check(ok and m.enabled, "enabled inside the band")
    check(m.take_actions() == [("ACTIVATE",), ("PUBLISH_REARM",)], "balance enable activates then re-arms")


def test_rule3_mode_change():
    m = ready_machine()
    m.request_enable(True, "dashboard")
    ok, msg = m.request_mode("balance", "dashboard")
    check(not ok and m.mode == "tank" and m.phase == "ready" and "disable" in msg, "refused while enabled")
    m.request_enable(False, "dashboard")
    m.take_actions()
    ok, _ = m.request_mode("balance", "dashboard")
    check(ok and m.phase == "stopping" and m.requested_mode == "balance" and m.mode == "tank", "stopping")
    check(m.take_actions() == [("STOP_STACK",)], "stop the old stack")
    ok, _ = m.request_mode("tank", "dashboard")
    check(not ok, "no second change while stopping")
    m.stack_stopped()
    check(m.mode == "balance" and m.phase == "starting" and m.requested_mode == "balance", "starting")
    check(m.take_actions() == [("START_STACK", "balance")], "start the new stack")
    ok, _ = m.request_mode("tank", "dashboard")
    check(not ok, "no change while starting")
    m.stack_ready()
    check(m.phase == "ready" and m.requested_mode == "" and m.message == "", "ready, request cleared")
    ok, msg = m.request_mode("balance", "dashboard")
    check(ok and m.phase == "ready" and m.take_actions() == [], "same mode when ready is a no-op")
    ok, _ = m.request_mode("hover", "dashboard")
    check(not ok, "unknown mode refused")


def test_rule4_disable_always():
    m = ready_machine()
    ok, _ = m.request_enable(False, "dashboard")
    check(ok and m.take_actions() == [], "disable when disabled is fine and does nothing")
    for phase in ("starting", "stopping", "fault"):
        m = ready_machine()
        m.request_enable(True, "dashboard")
        m.take_actions()
        m.phase = phase   # reach the odd phase directly: disable must not care how we got there
        ok, _ = m.request_enable(False, "gamepad")
        check(ok and not m.enabled and m.take_actions() == [("DEACTIVATE",)], f"disable while {phase}")


def test_rule5_joy_silence():
    m = ready_machine()
    t = 100.0
    m.joy_message(set(), t)
    check(m.joy_message(LB_RB, t + 0.05) == (True, "enabled") and m.enabled, "LB+RB enables")
    m.take_actions()
    m.joy_message(set(), t + 0.10)       # still heard while enabled
    check(not m.tick(t + 0.10 + gamepad.JOY_TIMEOUT_S - 0.01), "not yet silent")
    check(m.tick(t + 0.10 + gamepad.JOY_TIMEOUT_S + 0.01), "silent too long: auto-disable")
    check(not m.enabled and m.last_source == "supervisor" and m.message, "source supervisor, reason given")
    check(m.take_actions() == [("DEACTIVATE",)], "deactivates")
    check(m.phase == "ready", "silence alone is not a fault")


def test_rule5_no_auto_disable_if_never_heard_enabled():
    m = ready_machine()
    m.joy_message(set(), 10.0)                 # controller heard while disabled...
    m.request_enable(True, "dashboard")        # ...then enabled from the dashboard
    check(not m.tick(10.0 + 5.0) and m.enabled, "heard only while disabled: no auto-disable")
    m.joy_message(set(), 20.0)                 # now heard while enabled
    check(m.tick(21.0) and not m.enabled, "heard while enabled, then silent: auto-disable")
    m = ready_machine()
    m.request_enable(True, "dashboard")
    check(not m.tick(1000.0) and m.enabled, "never heard at all: no auto-disable")


def test_rule5_fault():
    m = ready_machine()
    m.request_enable(True, "dashboard")
    m.take_actions()
    m.stack_failed("controller diffbot_base_controller is missing")
    check(not m.enabled and m.phase == "fault" and m.last_source == "supervisor", "fault disables")
    check("missing" in m.message, "reason kept")
    check(m.take_actions() == [("DEACTIVATE",)], "deactivate on fault")
    ok, _ = m.request_enable(True, "dashboard")
    check(not ok and not m.enabled, "cannot enable in fault")
    ok, _ = m.request_mode("tank", "dashboard")
    check(ok and m.phase == "stopping", "fault is left by a mode change (restart), even the same mode")
    m.take_actions()
    m.stack_stopped()
    m.stack_ready()
    check(m.phase == "ready" and m.message == "", "recovered")


def test_start_timeout_and_kill_fault():
    m = StateMachine()
    m.take_actions()
    m.stack_failed("stack did not become ready in 60 s")
    check(m.phase == "fault" and m.take_actions() == [], "start failure: fault, nothing to deactivate")
    m = ready_machine()
    m.request_mode("balance", "dashboard")
    m.stack_failed("stack did not stop in 15 s and was killed; check the motors (H-7)")
    check(m.phase == "fault" and m.requested_mode == "", "kill during a change: fault")


def test_rule6_refusal_changes_nothing():
    m = ready_machine()
    before = m.snapshot()
    m.request_mode("nope", "gamepad")
    after = m.snapshot()
    check({k: v for k, v in after.items() if k != "message"} == {k: v for k, v in before.items() if k != "message"},
          "only the message changes")
    check(after["message"] != "", "the reason is in the message")


def test_combo_enable_toggle():
    m = ready_machine()
    m.joy_message(set(), 0.0)
    m.joy_message(LB_RB, 0.05)
    check(m.enabled, "LB+RB on")
    m.joy_message(LB_RB, 0.10)   # still held: no second toggle
    check(m.enabled, "held does not re-fire")
    m.joy_message(set(), 0.15)
    m.joy_message(LB_RB, 0.20)
    check(not m.enabled and m.last_source == "gamepad", "LB+RB again turns it off")


def test_combo_mode_toggle():
    m = ready_machine()
    m.joy_message(set(), 0.0)
    r = m.joy_message(LT_RT_RB, 0.05)
    check(r[0] and m.phase == "stopping" and m.requested_mode == "balance", "LT+RT+RB changes mode")
    m = ready_machine()
    m.joy_message(set(), 0.0)
    m.joy_message(LB_RB, 0.05)
    m.take_actions()
    m.joy_message(set(), 0.10)
    r = m.joy_message(LT_RT_RB, 0.15)
    check(r is not None and not r[0] and m.mode == "tank" and m.enabled, "mode combo refused while enabled")
    m = ready_machine()
    m.joy_message({"lt", "rt", "rb", "lb"}, 0.0)   # all held: neither combo (exact match)
    m.joy_message({"lt", "rt", "rb", "lb"}, 0.05)
    check(not m.enabled and m.phase == "ready", "four held matches neither combo")


def test_combo_held_at_first_contact_does_not_fire():
    m = ready_machine()
    m.joy_message(LB_RB, 0.0)
    m.joy_message(LB_RB, 0.05)
    check(not m.enabled, "combo already held when first heard must be released first")


def test_balance_enable_via_combo_uses_pitch():
    m = ready_machine("balance")
    m.joy_message(set(), 0.0)
    r = m.joy_message(LB_RB, 0.05, pitch_deg=None, rearm_max_pitch_deg=5.0)
    check(r is not None and not r[0] and not m.enabled, "unknown pitch refuses the combo")


def test_modes_table():
    check(modes.mode_names() == ["tank", "balance"], "two modes")
    check(modes.launch_file("tank") == "rocko_tank_mode.launch.py", "tank launch")
    check(modes.launch_file("balance") == "rocko.launch.py", "balance launch")
    check(modes.motion_controllers("tank") == ["diffbot_base_controller"], "tank motion")
    check(len(modes.motion_controllers("balance")) == 4, "balance has four cascade controllers")
    # No IMU node since the C-1 fix (2026-11-24): the IMU plugin reads the sensor inside controller_manager.
    check("icm20948_node" not in modes.expected_nodes("balance", False), "real: no IMU node any more")
    check("node" in modes.expected_nodes("balance", False), "real: encoder node expected")
    check("node" not in modes.expected_nodes("tank", True), "mock: encoder node not expected")


def enabled_balance():
    m = ready_machine("balance")
    ok, _ = m.request_enable(True, "dashboard", 0.0, 10.0)
    check(ok, "balance enabled inside the band")
    m.take_actions()
    return m


def test_rule5_balance_never_dropped_on_silence():
    # C-3: in balance, disabling drops the robot; lost input must give zero velocity (the teleop does), not a disable.
    m = enabled_balance()
    m.joy_message(set(), 1.0)
    check(not m.tick(1.0 + gamepad.JOY_TIMEOUT_S + 0.01), "balance: silence does not disable")
    check(m.enabled and m.take_actions() == [] and "still enabled" in m.message, "still enabled, nothing queued, says why")


def test_rule5_balance_lost_node_keeps_enabled():
    m = enabled_balance()
    m.stack_failed("missing: balance_teleop", controllers_intact=True)
    check(m.enabled and m.phase == "fault" and m.take_actions() == [], "balance, helper node lost: fault, still enabled")
    check("STILL ENABLED" in m.message, "the dashboard says so")
    ok, _ = m.request_mode("tank", "dashboard")
    check(not ok, "no mode change while still enabled")
    ok, _ = m.request_enable(False, "dashboard")
    check(ok and not m.enabled and m.take_actions() == [("DEACTIVATE",)], "a person can still disable")
    m = enabled_balance()
    m.stack_failed("missing: pitch_pid_controller", controllers_intact=False)
    check(not m.enabled and m.take_actions() == [("DEACTIVATE",)], "balance, a cascade controller lost: disables")
    m = ready_machine("tank")
    m.request_enable(True, "dashboard")
    m.take_actions()
    m.stack_failed("missing: tank_teleop", controllers_intact=True)
    check(not m.enabled, "tank, helper node lost: disables")


# --- controller_manager silence, verified disable, faults while enabled -------------------------------
TANK_STATES_OFF = {"joint_state_broadcaster": "active", "diffbot_base_controller": "inactive"}
CASCADE = ["left_add_feedforward_controller", "right_add_feedforward_controller",
           "pitch_pid_controller", "velocity_pid_controller"]


def balance_states(motion):
    return {"joint_state_broadcaster": "active", **{n: motion for n in CASCADE}}


def enabled_tank():
    m = ready_machine("tank")
    m.request_enable(True, "dashboard")
    m.take_actions()
    return m


def test_unanswered_is_unknown_not_missing():
    # B1: a slow controller_manager must not read as "every controller missing" (that disables the cascade).
    for m in (enabled_tank(), enabled_balance()):
        m.controller_manager_unanswered(CM_UNANSWERED_FAULT_S - 0.1)
        check(m.phase == "ready" and m.enabled and m.take_actions() == [], "short silence changes nothing")
    # a POSITIVE answer that lacks an expected controller keeps the old rule: missing -> fault, disabled
    m = enabled_balance()
    m.observe_stack(balance_states("active") | {"pitch_pid_controller": "x"}, [], False)
    check(m.phase == "ready" and m.enabled, "all listed: fine")
    states = balance_states("active")
    del states["pitch_pid_controller"]
    m.observe_stack(states, [], False)
    check(m.phase == "fault" and not m.enabled and m.take_actions() == [("DEACTIVATE",)],
          "a listed answer without a cascade controller: fault and disable")
    check("pitch_pid_controller" in m.message, "names the missing controller")


def test_tank_silence_faults_then_kills():
    m = enabled_tank()
    m.controller_manager_unanswered(CM_UNANSWERED_FAULT_S)
    check(m.phase == "fault" and not m.enabled and "controller_manager" in m.message, "tank: fault and disabled")
    check(m.take_actions() == [("DEACTIVATE",)], "tank: deactivate is tried")
    m.controller_manager_unanswered(CM_UNANSWERED_FAULT_S + CM_UNANSWERED_KILL_S - 0.1)
    check(m.take_actions() == [], "not killed before the kill delay")
    m.controller_manager_unanswered(CM_UNANSWERED_FAULT_S + CM_UNANSWERED_KILL_S)
    check(m.take_actions() == [("KILL_STACK",)] and "PWM" in m.message, "killed after the delay, message names the stuck PWM")
    m.controller_manager_unanswered(100.0)
    check(m.take_actions() == [], "kill is queued once")
    ok, _ = m.request_mode("tank", "dashboard")
    check(ok and m.phase == "stopping", "recoverable by choosing the mode again")
    m = ready_machine("tank")
    m.controller_manager_unanswered(CM_UNANSWERED_FAULT_S)
    check(m.phase == "fault" and m.take_actions() == [], "disabled tank also faults, nothing to deactivate")


def test_balance_silence_stays_enabled_and_is_never_killed():
    m = enabled_balance()
    m.controller_manager_unanswered(CM_UNANSWERED_FAULT_S)
    check(m.phase == "fault" and m.enabled and m.take_actions() == [], "balance: fault, STILL enabled, no action")
    check("still enabled" in m.message and "not answering" in m.message, "message says why it stays enabled")
    m.controller_manager_unanswered(10 * (CM_UNANSWERED_FAULT_S + CM_UNANSWERED_KILL_S))
    check(m.enabled and m.take_actions() == [], "balance: never killed while enabled")
    ok, _ = m.request_enable(False, "dashboard")
    check(ok and not m.enabled and m.take_actions() == [("DEACTIVATE",)], "a person can disable it")
    m.tick(1000.0)
    m.tick(1000.0 + DEACTIVATE_CONFIRM_S + 0.1)
    check(m.take_actions() == [("KILL_STACK",)], "once disabled, an unconfirmed disable kills the stack")


def test_verified_disable_retries_confirms_and_times_out():
    m = enabled_tank()
    m.request_enable(False, "dashboard")
    check(m.take_actions() == [("DEACTIVATE",)], "disable asks")
    m.tick(10.0)   # the confirm clock starts at the first tick after the disable
    m.controller_states_reply({**TANK_STATES_OFF, "diffbot_base_controller": "active"}, m.motion_epoch - 1)
    check(m.take_actions() == [], "an answer to a question asked before the disable proves nothing")
    m.controller_states_reply({**TANK_STATES_OFF, "diffbot_base_controller": "active"}, m.motion_epoch)
    check(m.take_actions() == [("DEACTIVATE",)], "still active: asks again")
    m.controller_states_reply(TANK_STATES_OFF, m.motion_epoch)
    m.tick(10.0 + 10 * DEACTIVATE_CONFIRM_S)
    check(m.take_actions() == [] and m.phase == "ready", "confirmed inactive: no timeout, no kill")
    # never confirmed
    m = enabled_tank()
    m.request_enable(False, "dashboard")
    m.take_actions()
    m.tick(10.0)
    m.tick(10.0 + DEACTIVATE_CONFIRM_S - 0.1)
    check(m.take_actions() == [], "not before the confirm time")
    m.tick(10.0 + DEACTIVATE_CONFIRM_S + 0.1)
    check(m.take_actions() == [("KILL_STACK",)] and m.phase == "fault" and not m.enabled, "unconfirmed: fault and kill")
    # a motion controller found active while disabled in READY takes the same path
    m = ready_machine("tank")
    m.controller_states_reply({**TANK_STATES_OFF, "diffbot_base_controller": "active"}, m.motion_epoch)
    check(m.take_actions() == [("DEACTIVATE",)], "active while disabled: deactivate")
    m.tick(0.0)
    m.tick(DEACTIVATE_CONFIRM_S + 0.1)
    check(m.take_actions() == [("KILL_STACK",)], "and it is killed if that never works")
    # row 4.14: a controller whose on_deactivate returned an error (the zero that stops its wheel was not
    # written) lands in "unconfigured", not "inactive". That is not a confirmed disable: kill after the timer.
    m = enabled_balance()
    m.request_enable(False, "dashboard")
    m.take_actions()
    m.tick(20.0)
    failed = {n: "inactive" for n in modes.motion_controllers("balance")}
    failed["left_add_feedforward_controller"] = "unconfigured"
    m.controller_states_reply({"joint_state_broadcaster": "active", **failed}, m.motion_epoch)
    check(m.take_actions() == [("KILL_STACK",)] and m.phase == "fault",
          "a controller left unconfigured by a failed deactivate: killed at once")
    # but NOT while the stack starts: the --inactive spawner leaves controllers unconfigured for a while
    m = StateMachine()
    m.take_actions()   # drain the boot START_STACK
    check(m.phase == "starting", "a new machine is starting")
    m.controller_states_reply({n: "unconfigured" for n in modes.motion_controllers(m.mode)}, m.motion_epoch)
    check(m.take_actions() == [], "starting: unconfigured controllers are normal, no action")
    # disable, then enable again before any answer: the wait for proof must not kill the enabled robot
    m = enabled_tank()
    m.request_enable(False, "dashboard")
    m.take_actions()
    m.tick(1.0)
    m.request_enable(True, "dashboard")
    m.take_actions()
    m.tick(1.0 + 10 * DEACTIVATE_CONFIRM_S)
    check(m.take_actions() == [] and m.enabled and m.phase == "ready", "re-enabled: not killed by the old disable's timer")
    # the stack going away ends the wait
    m = enabled_tank()
    m.request_enable(False, "dashboard")
    m.stack_gone()
    m.tick(0.0)
    m.tick(100.0)
    check(m.take_actions() == [("DEACTIVATE",)], "nothing left to confirm once the stack is gone (only the first ask)")


def test_fault_while_enabled_still_judges_cascade_loss():
    m = enabled_balance()
    m.observe_stack(balance_states("active"), ["balance_teleop"], False)
    check(m.phase == "fault" and m.enabled and "STILL ENABLED" in m.message, "helper node lost: fault, still enabled")
    m.observe_stack(balance_states("active"), ["balance_teleop"], False)
    check(m.enabled and m.take_actions() == [], "same fault again changes nothing")
    states = balance_states("active")
    del states["velocity_pid_controller"]
    m.observe_stack(states, ["balance_teleop"], False)
    check(not m.enabled and m.take_actions() == [("DEACTIVATE",)], "a cascade controller lost later: disabled")


def test_messages_spell_out_finding_ids():
    import re
    m = enabled_balance()
    m.controller_manager_unanswered(CM_UNANSWERED_FAULT_S)
    texts = [m.message]
    m = enabled_balance()
    m.stack_failed("missing: balance_teleop", controllers_intact=True)
    texts.append(m.message)
    m = enabled_balance()
    m.joy_message(set(), 1.0)
    m.tick(1.0 + gamepad.JOY_TIMEOUT_S + 0.01)
    texts.append(m.message)
    check(all(re.search(r"\b[A-Z]-\d\b", t) is None for t in texts), f"no bare finding IDs in {texts}")


def main():
    tests = [(n, f) for n, f in sorted(globals().items()) if n.startswith("test_") and callable(f)]
    for name, fn in tests:
        fn()
        print(f"ok   {name}")
    print(f"PASS {len(tests)} tests")


if __name__ == "__main__":
    main()
