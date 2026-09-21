# ROCKO-env — working agreement for AI agents

ROCKO is a two-wheeled self-balancing robot and an **educational capstone platform**. Getting the
robot upright matters. A student understanding *why* it stays upright matters more.

Read this whole file before your first tool call.

---

## 1. Primary value: teach, don't just do

An agent that silently produces a correct patch has failed the main goal of this repo.

**Plan first, implement second — always in that order.**

For any task that changes behavior (code, gains, launch files, URDF, build config):

1. **Investigate and report.** Say what you read and what you found, citing `file.py:42`.
2. **Explain the robotics.** Name the concept in play — cascaded control, sensor-fusion delay,
   plant gain, real-time determinism, quantization — and explain it to someone who has had one
   controls course. Point at `docs/` or <https://control.ros.org/jazzy/> for depth.
3. **State the tradeoff.** What improves, what regresses, what you are unsure about.
4. **Ask your open questions — genuinely.** Do not pose a question and then answer it yourself so
   you can keep going. If a behavior change raises no questions for you, you have not thought
   hard enough about it.
5. **Wait for a go-ahead.** Then implement the *whole* thing and verify it (§5).

One gate per task, not per edit. Once the human approves the plan, finish the job — don't
re-litigate every line.

**No gate needed** for: reading and searching, read-only build/inspection commands, typo and
comment fixes, and writing documentation that was asked for.

### Never change these without an explicit human decision

| Thing | Why | Where |
| --- | --- | --- |
| Any **version pin** | Jazzy + ros2_control 4.x are locked, deliberately | [ADR 0001](docs/decisions/0001-version-lock.md) |
| **PID gains** | Gains are experimental results, not code | `rocko_env/rocko_env/bringup/config/*.yaml` |
| The **100 Hz critical path** | Real-time correctness, robot falls over | §4 |
| A **safety behavior** | Tipover cutoff, teleop watchdog, command clamps | [findings](docs/claude-analysis/balance-loop-findings.md) |

### Teach through the work, not around it

Prefer explaining a fix the human then makes, over making it for them, when the change *is* the
learning objective — control theory, filter design, loop timing. Boilerplate (CMake wiring,
plugin registration XML, docs) you should just write.

---

## 2. What the robot is

Read [`README.md`](README.md) first — it has the signal-path diagram, the folder map, and build
instructions. In one paragraph:

A gamepad on a laptop sends velocity over TCP to `Joystick.py` on the Pi. `DiffDriveController.py`
turns that into per-wheel references. Then a **cascade**: `velocity_pid_controller` compares
commanded vs. measured wheel velocity and outputs a *target lean angle*;
`pitch_pid_controller` compares that target against measured pitch and outputs *wheel velocity*;
`left/right_add_feedforward_controller` adds a feedforward term and drives the
`Motor12VoltQuadEncoder` hardware plugin. To go faster the robot must first lean — that inversion
is why the loops are nested in that order. See
[docs/architecture/control-cascade.md](docs/architecture/control-cascade.md).

Two modes: **balancing** (`rocko.launch.py`, the real thing) and **tank** (`rocko_tank_mode.launch.py`,
balancing disabled, plain diff drive).

---

## 3. Documentation is a deliverable, not a chore

This repo is judged on how well it is documented and how easily a stranger can reproduce a result.

- **All docs live in [`docs/`](docs/).** `README.md` stays the front door and links into `docs/`.
  Three homes: [`docs/architecture/`](docs/architecture/) (how it works and why),
  [`docs/guides/`](docs/guides/) (do this, get that), [`docs/decisions/`](docs/decisions/) (ADRs).
  `docs/claude-analysis/` holds AI-produced audits — keep new audits there.
- **A behavior change is not done until its docs change with it, in the same commit.**
- Write for a capstone student in week one. Define jargon on first use. Never write "simply".
- State what is *not* true. The README's honesty about `use_mock_hardware` being dead is the
  standard to match — do not quietly document an aspiration as a fact.
- Every procedure must be **copy-pasteable and complete**: which machine to run it on, which
  shell, what to source first, what success looks like.

Use the `write-docs` skill for conventions and the ADR template.

---

## 4. The 100 Hz critical path — performance rules

`controller_manager` runs at `update_rate: 100` (`rocko_controllers.yaml`). That is a **10 ms
budget** for every `read()` → `update()` → `write()` cycle, on a Raspberry Pi. Anything that
overruns shows up as jitter in the balance loop, which reads as bad gains and wastes tuning days.

**Critical path = these files.** Touching them means §5's review loop is mandatory.

- `rocko_env/rocko_env/hardware/motors/Motor12VoltQuadEncoder.cpp` + `wheel.hpp`
- `rocko_env/rocko_env/hardware/sensors/icm20948/ICM20948.cpp`
- `rocko_env/rocko_env/hardware/sensors/quadEncoder/QuadEncoder.cpp`
- `rocko_env/rocko_env/controllers/add_feedforward/AddFeedforwardController.cpp`
- `rocko_env/rocko_env/bringup/config/*.yaml` (update_rate, gains, interface wiring)

**In `on_activate`/`read`/`update`/`write`, never:**

- Block. No service calls, no `spin_until_future_complete`, no unbounded waits, no sleeps.
  Device I/O belongs *in* the hardware plugin, in C++, in-process — that is the entire reason
  `ros2_control` has hardware plugins. A Python node reached by DDS is not a driver.
- Allocate. No `new`, no container resize, no `std::string` construction, no `std::function`
  capture-by-value. Pre-allocate in `on_init`/`on_configure`.
- Log unconditionally. `RCLCPP_INFO` in the loop is a syscall and a lock; use the `_THROTTLE`
  variants or a `realtime_tools::RealtimePublisher` for telemetry.
- Touch the filesystem, or take a lock a non-RT thread also takes. Cross the RT boundary with
  `realtime_tools::RealtimeBuffer` / `RealtimeBox`.
- **Report a safe-looking value on sensor failure.** A failed IMU read must never surface as
  pitch = 0.0 ("upright"). Return `return_type::ERROR`, or hold the last value *and* raise a
  fault flag that a supervisor acts on. Silent fallbacks in a balance loop are how robots break.

**Also true of this robot specifically:**

- Sample the IMU **once per cycle as one transaction.** Three separate I²C reads at three instants
  produce an attitude that never existed.
- Use the **gyro's angular rate** for a derivative term. Differentiating a fused, filtered angle
  gives you a delayed, noisy D.
- Clamp before writing PWM. Every command into hardware gets range-checked at the boundary.
- Software PWM must run well above the control rate. Equal periods alias.

Before proposing a performance change, measure: `ros2 topic hz`, the `controller_manager` update
statistics, or a scoped timer. **Do not optimize on intuition — this is a teaching repo, so show
the number that justified the change.**

---

## 5. If you implement: build, then have subagents review, then iterate

When the human has approved a plan and you are writing the code, you must run this loop. Using
the `Agent` tool for this is expected here.

1. **Implement** the approved plan.
2. **Build every target that can build it** — see the `build-verify` skill. A change is not done
   while any reachable target is red.
3. **Review in parallel** with subagents (one `Agent` call, several agents):
   - `realtime-auditor` — mandatory if any §4 file changed.
   - `ros2-control-reviewer` — any ROS/controller/URDF/launch change.
   - `docs-reviewer` — any docs change, or any behavior change that should have had one.
4. **Fix every confirmed finding**, then go back to step 2.
5. **Iterate until a full pass comes back clean.** Cap at 3 rounds. If round 3 is still dirty,
   stop and hand the human the remaining findings — do not keep grinding silently.
6. **Report honestly.** Say which targets you actually built, which findings you fixed, and which
   you deliberately did not. If tests or builds failed, show the output.

### The honesty rule that matters most here

**You cannot verify balancing.** You have no robot, no IMU, no floor. "It compiles" and "the
controllers load" are the strongest claims available to you. Never write that balance performance
is improved, that a gain is better, or that a fix works on hardware. Say exactly what you checked
and hand the human a written test procedure for the rest.

---

## 6. Build targets

Three targets, and they are not equivalent. Details and commands in the `build-verify` skill.

| Target | What builds | Status |
| --- | --- | --- |
| **Docker dev container** (`docker/`) | `rocko_interfaces` fully; `rocko_env` does **not** link — no WiringPi off-Pi | Works today |
| **Raspberry Pi** (Ubuntu 24.04 + Jazzy + WiringPi) | Everything. The only place the real robot builds | Works today |
| **Simulation** (`gz_ros2_control`) | Intended: full `rocko_env` off-Pi, with physics, so the cascade can be tuned without a robot | **Not built yet** — [ADR 0003](docs/decisions/0003-simulation-target.md) |

Never claim a simulation build or run until `gz_ros2_control` actually exists in this repo. Today
`use_mock_hardware` is a xacro parameter that is accepted and then ignored
(`rocko.ros2_control.xacro:4`) — there is no `mock_components/GenericSystem` anywhere.

Always `source /opt/ros/jazzy/setup.bash` first, build from the workspace root (not `src/`), and
use `--symlink-install`.

---

## 7. Reproducibility

Anything that produced a number — a gain set, a calibration, a latency measurement — is only
useful if someone else can get the same number.

Record, with the result: the **commit SHA**, the **launch file and config** used, **battery
voltage**, the **surface**, and **where the log went**. See
[docs/guides/reproducing-results.md](docs/guides/reproducing-results.md). If you change a gain or
report a measurement and cannot fill those in, say so out loud instead of omitting it.

---

## 8. Conventions

- **C++17**, ROS 2 Jazzy, `ament_cmake`. Hardware plugins and controllers are C++; offline tools,
  calibration, and telemetry may be Python.
- Match surrounding style. Existing C++ uses `snake_case` members with a trailing `_`; classes and
  files are `PascalCase`; Python nodes are `PascalCase.py` because they are installed as executables.
- New plugins start from `examples/example_hardware/` and must be registered in
  `rocko_env_hardware.xml` or `rocko_env_controllers.xml`, or ROS will not find them. Use the
  `add-hardware` skill.
- Don't commit `build/`, `install/`, or `log/` — already gitignored, but they exist in the tree.
- The IMU directory and classes are named `ICM20948`; the part actually fitted is a **BNO055**.
  Don't "fix" this casually — it is a real rename touching plugin XML, URDF, and launch files.

---

## 9. Skills in this repo

| Skill | Use it when |
| --- | --- |
| `build-verify` | Building, or checking a change compiles across targets |
| `control-loop-change` | Touching any §4 critical-path file |
| `tune-gains` | Changing or reasoning about PID gains |
| `add-hardware` | Adding a sensor or actuator plugin |
| `write-docs` | Writing or updating anything in `docs/` |
