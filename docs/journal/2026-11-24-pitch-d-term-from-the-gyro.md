# Prepare the gyro D term for the robot: handle finding L-8 and propose the YAML change (handoff item 5)

- **Date / commit:** 2026-11-24, built on `a27e9a3` (this entry is added by the commit that makes the change)
- **Branch:** `ai-forward`
- **Environment:** laptop (macOS) + Docker dev container (Jazzy); nothing run on the robot

## Goal and success criteria

Item 5 of the [next-steps handoff](../next-steps.md): make `pitch_pid_controller` take its D term from the gyro's
`pitch_rate` on the robot (finding H-6), with the L-8 caveat handled. The YAML change is the owner's to approve, so the
agent's part is everything around it.

Done means:

1. Both ways L-8 can bite are dealt with or bounded, in code where they can be:
   - a NaN rate reference: shown impossible since C-1 (stated, with where);
   - a constant `d x rate at enable` offset: the supervisor refuses a balance enable unless the pitch rate is known and
     within a small band. A host test covers the rule and goes red without it.
2. A proposal with the exact YAML edit, why, the evidence and its tiers, and the robot test.
3. The ladder stays green (Tier 2 `verify_supervisor.sh` still enables balance on the mock, whose `pitch_rate` is 0).

## Plan

- `supervisor/state_machine.py`: `ENABLE_MAX_PITCH_RATE_DEG_S` (PROPOSED; value in the next-steps proposed-values table) and the refusal in
  `request_enable` (and `joy_message`, which calls it). `supervisor/Supervisor.py`: read `pitch_rate` from
  `/dynamic_joint_states` next to `pitch`, counted as unknown when older than 0.2 s (review round 1).
- `tools/supervisor_state_machine_test.py`: the new cases, and the existing balance enables given a rate.
- [Proposal 0016](../proposals/0016-pitch-d-term-from-the-gyro.md): `reference_and_state_interfaces: ["pitch", "pitch_rate"]`
  with no gain change.
- Docs: the supervisor's design page, the controller guide, L-8's status, proposal 0008, next-steps.
- Risk: the rule refuses an enable while the robot moves, which it should; if the limit is too tight to enable by hand,
  the refusal says so and the constant is raised.

## What changed

- `state_machine.py`: the hold-still rule (balance only; tank ignores the rate). `Supervisor.py`: `pitch_rate` tracked
  and passed with `pitch`.
- `tools/supervisor_state_machine_test.py`: two new tests (refusals for unknown, NaN, infinite and too-large rates in
  both directions, acceptance at the limit; tank ignores it) and the combo test extended.
- `docs/proposals/0016-pitch-d-term-from-the-gyro.md` (new) and its index row.
- Docs: `architecture/robot-supervisor.md` rule 2, `guides/drive-with-usb-controller.md`, L-8's status in the findings,
  proposal 0008 item 3, next-steps item 5, rows 1.2 and 1.13, and the proposed-values table.

## Verification

| Tier | Command | Result |
| --- | --- | --- |
| host | `python3 -I tools/supervisor_state_machine_test.py` | all pass; **red** (exit 1, "a moving robot refuses the combo") with the rate refusal disabled in a copy |
| 0 to 2 | `tools/run_ladder.sh --no-sim` | all PASS, run twice (before and after review fixes): Tier 0, Tier 1 with 0 compiler warnings, Tier 2 mock both modes and `verify_supervisor.sh` (which enables balance on the mock: its `pitch_rate` is a constant 0, so it cannot tell the rule from no rule), every host test, hook tests, doc links |
| 3 | not run | nothing the simulation runs changed (the sim launch has no supervisor) |
| 4 | not run | the rule is not on the robot until the next pull; the YAML change is proposal 0016 |

## Review rounds

| Round | Reviewer | Findings | Fixed / declined (why) |
| --- | --- | --- | --- |
| 1 | ros2-control-reviewer | no wiring errors; Medium: the reference is re-taken at each enable only under the supervisor (a standalone launch activates at spawn); Medium: `d` keeps sign and units but the D term stops reacting to changes of the target lean; Low: the rate checked could be up to 1 s old; Low: test step 2 cannot print while disabled | all fixed: the proposal states the supervised-only limit and the behaviour change; the rate now counts as unknown after 0.2 s (`PITCH_RATE_MAX_AGE_S`); steps 2 and 3 merged |
| 1 | explainability-reviewer | the constant's comment pointed at a `pitch_rate` the real YAML does not have yet; why the rule is active now; the sign convention; no worked example of the offset; the refusal text read like a direction | all fixed; returning a named tuple from `balance_inputs` declined (three values, one caller pattern, documented in its docstring) |
| 1 | docs-reviewer | stale "left: L-8" in next-steps, the 3.0 copied into four docs, step 2 unworkable, an unbacked Tier 3 claim, the gate label, the emptied sim-config dict not mentioned, minor | all fixed: other docs name the constant and point at the next-steps table; the Tier 3 claim links the journal that ran it |

## Not verified — needs a human

The owner decides proposal 0016; its test procedure is there. Whether the limit can be met by a person holding the robot is
checked the first time balance is enabled after the next pull (the refusal message gives the rate seen).
