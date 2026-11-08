# ADR 0003 — Simulate with `gz_ros2_control` for off-robot development

- **Status:** Proposed — **the decision itself (a Gazebo physics simulation) is not implemented.**
  Follow-on steps 1 and 2 are implemented on the `ai-forward` branch only, and a different plant,
  `SimPlant`, was built first (2026-10-31); see "Progress" below. The ADR **stays `Proposed`**: it
  names Gazebo, Gazebo does not exist here, and the proposal to amend it
  ([proposal 0005](../proposals/0005-amend-adr-0003-simulpant-first.md)) is Open, awaiting a human.
- **Date:** 2026-09-20 (progress added 2026-10-27 and 2026-10-31)
- **Deciders:** ROCKO capstone team

> Read the status line. This ADR records an intention. There is **no Gazebo simulation** in this
> repository. On `ai-forward` there is a *mock* that checks controller wiring (no physics, not a
> simulation) and, since 2026-10-31, `SimPlant`: a small physics **model** with estimated, unmeasured
> parameters and only a few checks. It is not a finished Tier 3. No page in `docs/` should describe
> any of this as evidence about the robot. See the
> [verification ladder](../architecture/verification-ladder.md) for what exists.

## Progress (on `ai-forward`; not on `main`)

| Follow-on | State |
| --- | --- |
| 1. `use_mock_hardware` selects a mock plugin | **Done.** Selects `mock_components/GenericSystem`. Verified by `verify_mock.sh`. |
| 2. `rocko_env` configures and builds off-Pi | **Done**, with a deliberate cost: `-DROCKO_SKIP_WIRINGPI=ON` leaves out the motor plugin, so `Motor12VoltQuadEncoder.cpp` is **not compiled** off a Pi. The Pi default is unchanged and still requires WiringPi. Since 2026-10-31 a separate compile-only check type-checks that file against stub headers (syntax and types only; see [the ladder](../architecture/verification-ladder.md)). |
| 3. Measured inertial properties | Not done. Needs a person with the robot. `SimPlant` uses estimates, labelled `ESTIMATE, NOT MEASURED`, in their place. |
| 4. `gz_ros2_control` and a world | Not done. `SimPlant` (a hardware plugin with its own dynamics, no Gazebo) was built instead, first, on 2026-10-31: see [sim-plant.md](../architecture/sim-plant.md). Status of the tier: [the ladder](../architecture/verification-ladder.md) (built on 2026-11-07 with limits: a model with estimated parameters and an assumed encoder sign, which says nothing about whether the robot balances). |
| 5. Document the sim-to-real gap | Not done as its own page. [sim-plant.md §6](../architecture/sim-plant.md) lists what `SimPlant` leaves out. |

**An open disagreement.** Finding **M-4** in
[`balance-loop-findings.md`](../claude-analysis/balance-loop-findings.md) recommends a `SimPlant`
hardware plugin (about 200 lines of C++ integrating cart-pole dynamics) over a full Gazebo setup.
This ADR chose Gazebo before that audit. For the 2026-10-31 run the person decided `SimPlant` first
and it was built; Gazebo is not ruled out. The amendment to this ADR that records that is
[proposal 0005](../proposals/0005-amend-adr-0003-simulpant-first.md) (**Open**, not decided), so until a
human accepts it this ADR and the code disagree. The
`bootstrap-sim` skill lays out the trade.

## Context

As of 2026-09-20, when this ADR was written (before the `ai-forward` work in "Progress" above), two
facts made the situation expensive:

1. **`rocko_env` only built on a Raspberry Pi.** The motor plugin links against
   [WiringPi](https://github.com/WiringPi/WiringPi), which needs real Pi GPIO. Off-Pi — including in
   the Docker dev container — only `rocko_interfaces` built. This is a hardware constraint, not an
   OS one, so Docker cannot route around it. (On `ai-forward` the rest of `rocko_env` now builds
   off-Pi with the motor plugin left out; the constraint on that one file remains.)
2. **Balance tuning requires the physical robot.** Every gain experiment needs a charged battery, a
   floor, and somebody to catch the robot. That throttles iteration to a few students in one room,
   and a fall can damage hardware. This is still true.

There was a `use_mock_hardware` argument threaded through the launch files and the xacro macro
(`rocko.ros2_control.xacro:4` at the time), but it was **accepted and then ignored** — no
`mock_components/GenericSystem` existed anywhere in the repo. It offered no protection against
driving real motors ([issue #10](https://github.com/BiPed-Capstone/ROCKO-env/issues/10)). On
`ai-forward` it now selects a mock.

## Decision

Adopt **`gz_ros2_control`** (Gazebo + `ros2_control`) as ROCKO's simulation target, so `rocko_env`
builds off-robot (less the WiringPi motor plugin, which needs a Pi) and the balance cascade runs
off-robot against simulated physics.

Gazebo, rather than a non-physical mock, because the goal is not only "does it compile" but "can the
cascade be tuned without a robot." An inverted pendulum needs real dynamics — mass, inertia, gravity,
wheel contact — to be tuned against at all.

The ros2_control hardware/controller split is what makes this possible without touching the
controllers: swapping the hardware plugin in the xacro replaces the device layer and leaves the
cascade untouched. That is the payoff for the layering in [ADR 0002](0002-cascaded-balance-control.md).

## Alternatives considered

- **`mock_components/GenericSystem` only.** Much cheaper — it would make `rocko_env` build and its
  controllers load off-Pi, which alone would unblock CI and catch wiring errors. But it has no
  physics: commanded state is echoed straight back, so a balance controller always "succeeds" and
  nothing about tuning can be learned. **Reasonable as a first step toward this ADR, and strictly
  better than today, but not sufficient on its own.**
- **Stub WiringPi (a no-op `libwiringPi` for laptops).** Fixes linking only. Same no-physics problem,
  plus it makes an off-Pi build look more capable than it is — a trap in a teaching repo.
- **CMake guards around Pi-only sources.** Needed regardless, but a build fix rather than a
  simulation strategy.
- **Stay Pi-only.** The status quo. Costs iteration speed, blocks CI on the package that matters, and
  means every student needs physical access to one robot.

## Consequences

- **Good:** balance tuning becomes possible on a laptop; the package builds off-Pi (everything but the WiringPi motor plugin, which stays uncompiled there); CI can
  compile and load `rocko_env`; a student can break things without breaking hardware.
- **Bad:** the URDF needs **real inertial properties** — masses, inertia tensors, wheel friction — and
  `rocko_description.urdf.xacro` does not have trustworthy ones today. Sim gains will not transfer
  directly to the robot, and a sim that is *confidently wrong* is worse for a student than no sim, so
  the fidelity gap has to be documented alongside it. Gazebo is also a substantial new dependency for
  the Docker image.
- **Follow-on work, in order:**
  1. Make `use_mock_hardware` actually select a mock hardware plugin — closes issue #10 and stops the
     argument lying about what it does.
  2. Guard the WiringPi-dependent sources in CMake so `rocko_env` configures off-Pi.
  3. Measure and add inertial properties to the URDF.
  4. Add `gz_ros2_control` and a Gazebo world; add Gazebo to the Docker image.
  5. Document the sim-to-real gap honestly, in `docs/architecture/`.

## References

- `rocko_env/rocko_env/description/ros2_control/rocko.ros2_control.xacro` — where the mock now
  lives (the parameter was ignored at line 4 when this ADR was written)
- `rocko_env/CMakeLists.txt`, the `ROCKO_SKIP_WIRINGPI` block — replaced the unconditional WiringPi
  link that was at lines 75-80
- [Root README, "Working on a laptop instead of the robot"](../../README.md)
- [issue #10 — `use_mock_hardware` does nothing](https://github.com/BiPed-Capstone/ROCKO-env/issues/10)
- [gz_ros2_control docs](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html)
