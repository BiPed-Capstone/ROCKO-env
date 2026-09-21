# ADR 0003 — Simulate with `gz_ros2_control` for off-robot development

- **Status:** Proposed — **nothing in this ADR is implemented yet**
- **Date:** 2026-09-20
- **Deciders:** ROCKO capstone team

> Read the status line. This ADR records an intention. Today there is **no simulation** in this
> repository, and no page in `docs/` should describe one as working until this ADR is `Accepted`.

## Context

Two facts make the current situation expensive:

1. **`rocko_env` only builds on a Raspberry Pi.** The motor plugin links against
   [WiringPi](https://github.com/WiringPi/WiringPi), which needs real Pi GPIO. Off-Pi — including in
   the Docker dev container — only `rocko_interfaces` builds. This is a hardware constraint, not an
   OS one, so Docker cannot route around it.
2. **Balance tuning requires the physical robot.** Every gain experiment needs a charged battery, a
   floor, and somebody to catch the robot. That throttles iteration to a few students in one room,
   and a fall can damage hardware.

There is a `use_mock_hardware` argument threaded through the launch files and the xacro macro
(`rocko_env/rocko_env/description/ros2_control/rocko.ros2_control.xacro:4`), but it is **accepted and
then ignored** — no `mock_components/GenericSystem` exists anywhere in the repo. It offers no
protection against driving real motors
([issue #10](https://github.com/BiPed-Capstone/ROCKO-env/issues/10)).

## Decision

Adopt **`gz_ros2_control`** (Gazebo + `ros2_control`) as ROCKO's simulation target, so the full
`rocko_env` package builds and the balance cascade runs off-robot against simulated physics.

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

- **Good:** balance tuning becomes possible on a laptop; the full package builds off-Pi; CI can
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

- `rocko_env/rocko_env/description/ros2_control/rocko.ros2_control.xacro:4` — the ignored parameter
- `rocko_env/CMakeLists.txt:75-80` — the unconditional WiringPi link
- [Root README, "Working on a laptop instead of the robot"](../../README.md)
- [issue #10 — `use_mock_hardware` does nothing](https://github.com/BiPed-Capstone/ROCKO-env/issues/10)
- [gz_ros2_control docs](https://control.ros.org/jazzy/doc/gz_ros2_control/doc/index.html)
