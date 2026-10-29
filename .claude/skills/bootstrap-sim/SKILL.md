---
name: bootstrap-sim
description: Build ROCKO's missing verification tiers — the physics simulation (ADR 0003) and the checks around it. Use when asked to "set up simulation", "add Gazebo", "make the sim", "run the robot in sim", when a task needs Tier 3 and it does not exist yet, or when picking what to build to make the agent loop stronger.
---

# Building the simulator the agent loop needs

The agent loop in [CLAUDE.md](../../../CLAUDE.md) is only as good as what it can run to check its
own work. Without Tier 3 it stops at Tier 2, a *wiring* check with no physics. Everything about
balance — gains, filters, the cascade — needs Tier 3. This skill is the ordered path there. It is the
highest-leverage work in the repo: every step makes every later task checkable.

Status of each tier lives in [`docs/architecture/verification-ladder.md`](../../../docs/architecture/verification-ladder.md).
Update it when a step lands. Work top to bottom; each step is its own goal with its own journal entry.

## The backlog

| # | Step | Done when | Who |
| --- | --- | --- | --- |
| 1 | `use_mock_hardware` selects a real mock | `verify_mock.sh` and `verify_mock.sh tank` pass; every launch file defaults to `false` | Status: [ADR 0003 "Progress"](../../../docs/decisions/0003-simulation-target.md) (follow-on 1) |
| 2 | `rocko_env` builds off the Pi | `build_ros.sh` in the container builds both packages; the Pi default still requires WiringPi | Status: [ADR 0003 "Progress"](../../../docs/decisions/0003-simulation-target.md) (follow-on 2) |
| 3 | Compile-check the motor plugin off-Pi | A clearly named, **compile-only** check builds `Motor12VoltQuadEncoder.cpp` against a stub `wiringPi.h`/`softPwm.h` with `-fsyntax-only`. It must be impossible to link or run, and the docs must say it proves syntax and types only | Agent |
| 4 | Measured inertial properties | `rocko_description.urdf.xacro` has mass, centre of mass and inertia per link, each with a comment naming where the number came from | **Human measures**, agent writes the procedure and wires the numbers in |
| 5 | **Decide which Tier 3 plant** (see below) | A human has picked in a proposal; ADR 0003 is amended or superseded to match | **Human decides**; agent writes the proposal |
| 6 | The plant exists and the same controllers run against it | `rocko_sim.launch.py` starts it; the same `rocko_controllers.yaml` loads; `ros2 control list_controllers` shows all active; pitch now **changes** when the wheels move | Agent |
| 7 | Scenarios with pass/fail | A scenario runner executes: hold upright 10 s, recover from an impulse, follow a velocity step. Each scenario writes a metrics file and exits non-zero on failure | Agent |
| 8 | Sim-to-real gap, written down | `docs/architecture/sim-to-real-gap.md` lists what the model omits (backlash, PWM aliasing, battery sag, IMU latency) and what each omission does to a gain | Agent |
| 9 | Let the loop run it | `.claude/settings.json` allows the sim launch command; `verification-ladder.md` marks Tier 3 as built; `tune-gains` is now actionable | Agent writes a **proposal** for the settings change (it edits a guardrail, `CLAUDE.md` §3); a human approves it |

Order matters. Step 4 is first of the gating human steps because a sim with invented inertia is
*confidently wrong* — worse for a student than no sim (ADR 0003). If you reach step 4 and the numbers
have not been measured, do the steps that do not depend on them, then stop and hand the human the
measurement procedure rather than guessing. If you must use placeholder estimates to keep moving,
label every one `ESTIMATE, NOT MEASURED` in the xacro and in the docs.

## Which Tier 3 plant? Two documents disagree

[ADR 0003](../../../docs/decisions/0003-simulation-target.md) (`Proposed`) chooses Gazebo with
`gz_ros2_control`. Finding **M-4** in
[`balance-loop-findings.md`](../../../docs/claude-analysis/balance-loop-findings.md) recommends a
`rocko_env/SimPlant` hardware plugin instead: roughly 150–200 lines of C++ that integrate
inverted-pendulum (cart-pole) dynamics and export the same `pitch` / `velocity` / `position` state
interfaces as the real hardware. They are not exclusive, but you build one first:

| | `SimPlant` (M-4) | Gazebo + `gz_ros2_control` (ADR 0003) |
| --- | --- | --- |
| Size | One readable file a student can follow in a sitting | A large dependency and a world, mesh and contact setup |
| Models | Cart-pole dynamics with your own friction and delay terms | Full rigid-body physics, real wheel contact |
| Exercises | The entire real control stack through the hardware interface | The same, plus contact and geometry |
| Cost to the image | None | Gazebo and its bridge |
| Risk | You only get the physics you wrote down, and you must keep the model honest | Heavy, and still needs measured inertia |
| Fits "simple and explainable" | Yes | Less so |

My recommendation is `SimPlant` first: it is the smaller thing, it makes the cascade checkable
sooner, and Gazebo can be added later if contact fidelity turns out to matter. That is a project
decision, not mine alone, so step 5 is a proposal. Steps 1–4 and 8 do not depend on it; do them first.

## Rules for every step

- **Honesty over progress.** Do not mark a tier built until you have run it and watched it pass *and*
  watched it fail on a deliberately broken input. A verifier that cannot fail is the C-2 mistake: a
  sensor that reports "upright" no matter what.
- **Same controllers, different hardware.** The point of `ros2_control`'s split is that the sim
  replaces only the hardware layer. If you find yourself editing a controller to make the sim work,
  stop: the sim is wrong, or the controller has a hidden dependency worth documenting.
- **Do not claim transfer.** Gains found in the sim are candidates, not results. Say so in every
  doc you write.
- **Keep it simple.** One world, one robot, one launch file per mode. No plugin framework. A student
  should be able to read the whole sim setup in one sitting.
- **Document as you go.** Each step updates the verification ladder, the README's laptop section,
  and ADR 0003's follow-on list in the same commit. ADR 0003 stays `Proposed` until the simulator
  actually runs, then becomes `Accepted` in the commit that makes it true.

## Hand-off to a human at step 4

Write `docs/guides/measure-inertia.md` before asking. It must say: which tools (a kitchen scale, a
ruler, a string), what to weigh and measure on ROCKO (each link's mass; wheel radius; the robot's
centre of mass found by balancing it on an edge; the pendulum period for inertia), how to turn
the measurements into numbers for the xacro, and where to record them with the date and who
measured. Then raise it as a proposal ([`docs/proposals/`](../../../docs/proposals/README.md)) —
it is a robot-in-hand step the agent cannot do.
