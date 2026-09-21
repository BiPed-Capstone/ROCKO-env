# ADR 0001 — ROS 2 Jazzy and ros2_control 4.x are pinned

- **Status:** Accepted
- **Date:** 2026-09-20 (recorded; the constraint predates this record)
- **Deciders:** ROCKO capstone team

## Context

ROS 2 releases annually and `ros2_control` moves with it. Across distros the controller
configuration format and the hardware/controller C++ APIs both change in ways that are not
backward compatible.

ROCKO depends on `pid_controller/PidController` configured with **chained reference interfaces** —
the `dof_names: [pitch_pid_controller/left_wheel_joint]` style wiring in
`rocko_env/rocko_env/bringup/config/rocko_controllers.yaml`. Older `ros2_control` does not
understand that configuration format.

This is an educational platform used by students on their own machines. "It works on mine" is a
failure mode with a real cost here: a student who installs the wrong distro loses days to errors
that look like their own mistake.

## Decision

The supported environment is pinned, and the build enforces it:

| Component | Version |
| --- | --- |
| ROS 2 | Jazzy Jalisco |
| Ubuntu | 24.04 (Noble) |
| `ros2_control` | ≥ 4.0.0 |
| Python | 3.12 (ships with 24.04) |
| C++ | C++17 |
| CMake | ≥ 3.16 |

`rocko_env/CMakeLists.txt:25-29` makes this a hard build failure rather than a runtime surprise:

```cmake
find_package(controller_manager 4.0.0)
if(NOT controller_manager_FOUND)
  message(FATAL_ERROR "ros2_control version 4.0.0 or higher is required. ")
endif()
```

Humble ships `ros2_control` 2.x and Iron ships 3.x, so **both fail this check by design**.

**These versions do not change without a deliberate team decision.** A version bump is a project
decision with a migration plan, not a side effect of another change.

## Alternatives considered

- **Support multiple distros.** Rejected: the controller YAML format differs between them, so it
  would mean maintaining parallel configs for the most subtle part of the system, in a repo whose
  main purpose is to be understandable.
- **Warn instead of hard-failing.** Rejected: the failure would surface later as a controller that
  loads and misbehaves, which is far harder for a student to diagnose than a message naming the
  problem.
- **Track ROS 2 Rolling.** Rejected: a moving target breaks reproducibility, which is a stated
  project value.

## Consequences

- **Good:** one supported environment. A build error names the real problem. Results reproduce
  across machines.
- **Bad:** macOS and Windows have no usable Jazzy install, so laptop work needs the Docker container
  in [`docker/`](../../docker/). Jazzy's support window is finite and a migration will eventually be
  required.
- **Follow-on work:** Python libraries are **not** pinned — there is no `requirements.txt`, so `pip`
  takes the latest of each ([issue #14](https://github.com/BiPed-Capstone/ROCKO-env/issues/14)).
  That is a real gap in an otherwise locked toolchain.

## References

- `rocko_env/CMakeLists.txt:24-29` — the enforced check
- `rocko_env/package.xml`, `rocko_interfaces/package.xml` — declared dependencies
- `docker/Dockerfile:11` — `FROM ros:jazzy`
- [Root README, "Which ROS 2 version you need"](../../README.md)
