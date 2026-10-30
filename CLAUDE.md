# ROCKO-env — working agreement for AI agents (`ai-forward` branch)

ROCKO is a two-wheeled self-balancing robot and an **educational capstone platform**. On this branch
the code is **written by AI**, under human direction. Your job is to take a goal from a human and
hand back a *verified, explained* change. Getting the robot upright matters. A person being able to
understand why it stays upright matters just as much.

`main` uses a different, human-gated workflow (the agent explains and waits for approval). Why this
branch differs: [ADR 0005](docs/decisions/0005-ai-forward-development.md).

Read this whole file before your first tool call.

---

## 1. What the robot is

Read [`README.md`](README.md) first — it has the signal-path diagram, the folder map, and build
instructions. In one paragraph:

A gamepad on a laptop sends velocity over TCP to `Joystick.py` on the Pi. `DiffDriveController.py`
turns that into per-wheel references. Then a **cascade**: `velocity_pid_controller` compares
commanded vs. measured wheel velocity and outputs a *target lean angle*; `pitch_pid_controller`
compares that target against measured pitch and outputs *wheel velocity*;
`left/right_add_feedforward_controller` adds a feedforward term and drives the
`Motor12VoltQuadEncoder` hardware plugin. To go faster the robot must first lean — that inversion is
why the loops are nested in that order. See
[docs/architecture/control-cascade.md](docs/architecture/control-cascade.md).

Two modes: **balancing** (`rocko.launch.py`, the real thing) and **tank** (`rocko_tank_mode.launch.py`,
balancing disabled, plain diff drive). Both take `use_mock_hardware:=true` to run against a mock
that checks controller wiring (§4). It defaults to `false`, and must stay that way: a robot launched
with no arguments gets real hardware.

---

## 2. The loop: goal in, verified change out

A human gives you a goal. If they have not, take the first row of
[`docs/next-steps.md`](docs/next-steps.md) that you can do and check. You return a change you
have checked, reviewed and explained. **Do not stop to ask permission for ungated work** — a
change is not done when it compiles, it is done when it has been verified as far as the ladder
allows and the evidence is written down.

1. **Restate the goal as checkable success criteria**, in the run journal. "Make the pitch loop
   better" is not checkable; "all four controllers reach `active` on the mock and the tank launch
   still expands" is. (A journal entry is for any change to code, launch files, URDF, build config
   or tooling; skip it for typo fixes and docs-only changes. Template in
   [`docs/journal/`](docs/journal/README.md).)
2. **Plan in writing**, briefly: files to touch, which tier proves each criterion, the risks. Then
   proceed. The plan is for the reader of the journal, not a gate.
3. **Implement** the smallest change that meets the criteria, as simple as you can make it (§5).
4. **Verify up the ladder** (§4). Run every tier that can run. Read the warnings — in real-time C++
   they are often the finding. Capture commands and results.
5. **Review in parallel** — one `Agent` call, several agents:
   - `realtime-auditor` — **mandatory** if a §6 file changed.
   - `ros2-control-reviewer` — any ROS, controller, URDF/xacro, launch, CMake or YAML change.
   - `explainability-reviewer` — any code change.
   - `docs-reviewer` — any docs change, or any behavior change that should have had one.
6. **Fix every confirmed finding**, then go back to step 4. At most **three** review rounds. If round
   three is still dirty, stop and hand the human the remaining findings; do not grind silently.
7. **Document in the same change** (§7): journal entry, architecture page, ADR if the choice is not
   obvious, README, the verification ladder's status table, and the row in
   [`docs/next-steps.md`](docs/next-steps.md) if the change finishes, starts or blocks an item.
8. **Commit, then publish `ai-forward`.** Commit verified work to the working branch in logical
   commits, one goal at a time. On `ai-forward`, once the review loop is finished, push it, **without
   asking**: the maintainer has said so (2026-10-29: "the agent should always be able to push code").
   Do not ask "shall I push?", offer it as a choice, or wait for a go-ahead; push every commit you
   make on `ai-forward` before the turn ends, and report what went out. The only reasons not to are
   the fetch check below and a third review round that is still dirty (step 6). First
   `git fetch origin`, then `git log --oneline HEAD..origin/ai-forward`: it lists commits the remote
   has and you do not, and it must print **nothing** (an `unknown revision` error only means the
   branch is not on the remote yet). If it lists commits, a human has pushed: stop and report it, and
   do not merge, rebase or force. Otherwise `git push origin ai-forward` (`git push -u origin
   ai-forward` the first time). Run it as its own command, with nothing chained or piped (no
   `cd ... &&`, no `2>&1 | tail`): only the bare words match the allow rule. If the push is refused,
   is declined at a prompt, or fails, stop and report it; do not try another form (`gh api`, a reworded
   or chained command, another remote). Do not add a commit only to log the push: the remote is the
   record of what went out. Never commit to `main`, never push another branch, never force-push.
9. **Report honestly** (§4 closes with what this means): the tiers you ran and what they printed,
   the findings you fixed and any you declined (with why), the proposals you filed, what you did
   **not** verify, and the procedure a human should follow on the robot.

**Ask a human only when** (a) the work is gated (§3) — write a proposal and carry on with the rest;
or (b) the goal is ambiguous in a way that changes the design and reading the code and docs cannot
settle it. Ask one question with your recommendation. If the environment is broken, diagnose it
rather than paper over it; a tier you could not run is reported as *not run*.

**Unattended runs.** Nobody is answering prompts when the run is headless (started with
`claude -p`) or the person says they are away. Then:

- A tool call that comes back refused counts as a hook denial (§3): do not retry it by another route
  (`sed -i`, a differently spelled command). Write a proposal that names the exact command, unless
  an open one already does (cite it), and carry on with whatever does not need it. A session that
  waits at a prompt instead just stalls while the person is away, so avoid commands you know prompt.
- If `./check_container_mount.sh` is refused, or you skip it because it would prompt, report everything
  that runs in the container as *not run* (`verify_urdf.sh`, Tier 1 and Tier 2): without it those results cannot be tied to this checkout
  (§8). The host checks still count.
- Pick work by the rule in [`docs/next-steps.md`](docs/next-steps.md), *How to use it*, item 6:
  Wave 0 rows an agent can write and check. Wave 1 or later only when the person names the row.
- Stop, rather than invent work, when no such row is left, when step 6's third round is still dirty,
  or when nothing else can go on without the refused command.
- Nobody reads the chat. Put the step 9 report in the journal entry (the commit message, when there
  is none) and end it with a line saying why you stopped.

---

## 3. What a human still decides

The AI is autonomous about code. It is **not** autonomous about things it cannot verify and that
damage hardware or silently change what the project supports.

| You may do freely | A human decides (the hook **denies** or, for the guardrails, **asks**; robot actions hit a permission prompt). Write a proposal |
| --- | --- |
| Code, tests, launch files, URDF/xacro, scripts, tooling | **Hardware gains and limits** in `bringup/config/*.yaml`: `p`, `i`, `d`, the clamps, `update_rate`, velocity limits, `cmd_vel_timeout` (full key list in [`docs/proposals/`](docs/proposals/README.md)) |
| New dependencies, new source files in CMake, tools in the Docker image | **Version pins**: Jazzy, `ros2_control` ≥ 4.0.0, Ubuntu 24.04, C++17, CMake ≥ 3.16 ([ADR 0001](docs/decisions/0001-version-lock.md)) |
| Docs, ADRs (as `Proposed`), skills and agent definitions | **Removing or weakening a safety behavior**: tipover cutoff, watchdog, command clamp, error return |
| *Adding* a safeguard (a tipover cutoff, a watchdog, a clamp) | **The guardrails**: `.claude/hooks/`, `.claude/settings.json`, `.claude/settings.local.json`, `.claude/commands/`, `.mcp.json`, `.git/config`, `.git/hooks/` (the hook *asks*) |
| Gains in a separate `*_sim.yaml`, once a physics sim exists | **Anything on a robot**: `ros2 launch`/`run` on a Pi, a flash, a first tethered test |

**Pushing is allowed for one thing only, and needs no go-ahead (step 8):** `git push origin
ai-forward` (or `git push -u origin ai-forward`), in exactly those words. `.claude/settings.json` allows those two commands by exact
match, so in the default permission mode any other `git push` (another branch, `--force`, extra
arguments) prompts a human. Don't attempt one, and never force-push or delete a remote branch. A
push publishes: everything on `ai-forward` is readable by whoever can read the team's GitHub repo,
so never commit secrets or credentials.

**These rules stop accidents, not a determined agent.** `gh api` is also on the allow-list and can
move or delete any remote ref without a prompt; a chain of allowed commands such as
`git fetch origin && git push origin ai-forward` runs without one; the push target is whatever
`.git/config` says; and a rule added by answering "don't ask again", or kept in
`settings.local.json` or a user's own settings, is invisible to this file. Keep to the spirit.

Also off limits: committing to `main` and merging. Nothing stops those, so don't.

When the hook denies an edit, do **not** work around it with `sed -i` or a shell redirect. The hook
does not watch Bash, but the rule applies all the same. Write a proposal
([`docs/proposals/`](docs/proposals/README.md)) — the change, why, the evidence and its tier, and the
test a human should run — then carry on with the rest of the task.

**The hook is a tripwire, not a proof.** Its source, `.claude/hooks/guard_protected_files.py`, lists
what it knows. Its limits, so you can keep to the *spirit* of this table where the code cannot:

- It watches `Edit`, `Write` and `MultiEdit` only. Not Bash, and not the commands on the
  `settings.json` allow-list that can write files or run code (`docker compose`, `colcon`, `xacro`,
  `git commit`, `gh api`). Those are trusted to be used for their purpose.
- It knows a fixed list of gain and limit keys, version-pin patterns and safety words. The safety
  check counts words; it cannot tell a real clamp from a comment that repeats the word.
- **The gain guard reads lines, not YAML.** A quoted key (`{"p": 5}`), a `gains: |` block scalar, or
  moving a line out of its map is beyond it. Don't do those either.
- **Launch files are free to edit, but** one that loads a different YAML or overrides `update_rate`
  sidesteps the gain guard. Don't; `ros2-control-reviewer` checks for it.
- `CLAUDE.md`, `.claude/agents/` and `.claude/skills/` are unguarded: a human reads their diffs. Do
  not loosen §3 or the honesty rule in them.
- If the hook itself cannot run (no `python3`, a lost execute bit) the edit proceeds.

Tests for the hook: `.claude/hooks/test-guard-protected-files.sh`.

---

## 4. The verification ladder is your sensor

An agent loop is a feedback controller, and its verifier is the sensor. A verifier that cannot fail
is an IMU that reports pitch = 0.0 when it is broken (finding C-2): the loop sees "upright" while
blind. So:

| Tier | What it proves | What it cannot |
| --- | --- | --- |
| **0 Static** | `verify_urdf.sh` — the URDF expands in all four modes, parses, and holds the right hardware (real or mock); scripts parse; the hook tests pass | That anything compiles or runs |
| **1 Build** | `build_ros.sh` builds both packages in the container | Anything about the **motor plugin** (left out off-Pi), or runtime behavior |
| **2 Mock** | `verify_mock.sh` — the controllers the launch spawns load, claim real interfaces, chain and activate | Physics. The mock echoes commands and holds pitch at 0.0, so a balance controller always looks fine |
| **3 Simulation** | Controller behavior against modeled dynamics | Whatever the model omits. See `bootstrap-sim` and [ADR 0003](docs/decisions/0003-simulation-target.md) |
| **4 Robot** | Everything | **A human only.** You write the procedure |

Whether a tier exists, works or has been seen to fail is kept in **one** place:
[`docs/architecture/verification-ladder.md`](docs/architecture/verification-ladder.md). Do not
restate it elsewhere; link to it, and update it in the same commit that changes it. **Never claim a
simulation run until that page says Tier 3 is built.** How to run each script is in §8.

Rules:

- **Run every tier that can run**, and record the command and its result in the journal. A tier you
  did not run is written "not run", with the reason. Never imply it passed.
- **Report the highest tier you actually reached**, never one above it.
- **Make a verifier fail before you trust it.** When you add or change a check, break the thing it
  guards on purpose and watch it go red, then restore it.
- **If no tier can check your change, say so**, then either build the missing check (`bootstrap-sim`)
  or hand the human a procedure.

### The honesty rule that matters most

**You cannot verify balancing.** You have no robot, no IMU, no floor. "It compiles", "the
controllers load" and "the mock passes" are the strongest claims available to you — and the last
two say nothing about physics. Never write that balance performance is improved, that a gain is
better, or that a fix works on hardware. Say exactly what you checked, and give the human a written
test procedure for the rest: tethered, what to launch, what success looks like, when to abort.

---

## 5. Simple code that can be explained

The code is written by AI and then read by students with one controls course and no ROS. If it
cannot be explained, it is not done.

- **The simplest thing that is correct.** No abstraction with one user, no framework where a
  function will do, no cleverness a student cannot read. State the boring alternative you rejected
  only if it is non-obvious.
- **The five-sentence test.** If you cannot explain the diff in five sentences to that student, simplify
  the code or write the missing explanation into the docs.
- **Every quantity has a unit, frame and sign** in its name or a comment. `pitch` — radians or
  degrees, positive leaning forward or back? A bare `0.127` is a bug waiting.
- **Comment the *why*, not the *what*.** An ordering, a clamp, a deliberate inversion (velocity
  commands pitch). A comment that restates the line is noise.
- **Tunables are named parameters in one place**, where a student would want to experiment.
- **Match the surroundings.** The diff is only the change asked for: no drive-by reformatting.
- **Measure before optimizing, and show the number.** `ros2 topic hz`, `controller_manager`
  statistics or a scoped timer. Timings from the container or the mock say nothing about the Pi.

---

## 6. The 100 Hz critical path

`controller_manager` runs at `update_rate: 100` (`rocko_controllers.yaml`): a **10 ms** budget for
every `read()` → `update()` → `write()` on a Raspberry Pi. An overrun shows up as jitter in the
balance loop, which reads as bad gains and wastes tuning days.

**Critical path = these files.** Touching them makes `realtime-auditor` mandatory.

- `rocko_env/rocko_env/hardware/motors/Motor12VoltQuadEncoder.cpp` + `wheel.hpp`
- `rocko_env/rocko_env/hardware/sensors/icm20948/ICM20948.cpp`
- `rocko_env/rocko_env/hardware/sensors/quadEncoder/QuadEncoder.cpp`
- `rocko_env/rocko_env/controllers/add_feedforward/AddFeedforwardController.cpp`
- `rocko_env/rocko_env/bringup/config/*.yaml` (update_rate, gains, interface wiring)

**`Motor12VoltQuadEncoder.cpp` is not compiled in the container**, so a green Tier 1 says nothing
about it. Say so in the journal whenever you touch it.

**In `on_activate`/`read`/`update`/`write`, never:** block (service calls, futures, sleeps, unbounded
waits — device I/O belongs *in* the C++ hardware plugin, not a Python node reached by DDS);
allocate; log unconditionally (use `_THROTTLE` or a `RealtimePublisher`); touch the filesystem or
take a lock a non-RT thread also takes (cross with `RealtimeBuffer`/`RealtimeBox`); or **report a
safe-looking value on failure**. A failed IMU read must never surface as pitch = 0.0; ignoring a
return value is the same mistake (finding M-5). Return `return_type::ERROR`, or hold the last value
*and* raise a fault flag something acts on.

**On this robot:** one IMU transaction per cycle, as one snapshot; the D term comes from the gyro's
rate, not a differentiated fused angle; clamp every value before it reaches PWM; software PWM must
run well above the control rate (equal periods alias); near-zero is still a command.

Procedure and reasoning: the `control-loop-change` skill. Known problems:
[`balance-loop-findings.md`](docs/claude-analysis/balance-loop-findings.md) — cite finding IDs
(C-1, H-3, …) rather than rediscovering them, and respect its stated order of work.
[`docs/next-steps.md`](docs/next-steps.md) puts that order into a queue, adjusted for what can be
checked today, with who can do each item and what can check it.

---

## 7. Documentation is a deliverable

This repo is judged on how well it is documented and how easily a stranger can reproduce a result.
With AI writing the code, the docs are how a human comes to understand it.

- **All docs live in [`docs/`](docs/).** `README.md` is the entry point and links into `docs/`.
  Homes: [`architecture/`](docs/architecture/) (how it works and why),
  [`guides/`](docs/guides/) (do this, get that), [`decisions/`](docs/decisions/) (ADRs),
  [`journal/`](docs/journal/README.md) (what each run did and verified),
  [`proposals/`](docs/proposals/README.md) (decisions waiting on a human),
  [`claude-analysis/`](docs/claude-analysis/) (AI-produced audits).
- **A behavior change is not done until its docs change with it, in the same commit.**
- Write for a capstone student in week one. Define jargon on first use. Never write "simply".
- **State what is not true.** Do not document an aspiration as a fact; and when a limitation is
  fixed, delete the sentence that says it exists.
- Every procedure is **copy-pasteable and complete**: which machine, which shell, what to source
  first, what success looks like.

Use the `write-docs` skill for conventions and the ADR template.

---

## 8. Build targets

| Target | What it is for |
| --- | --- |
| **Docker dev container** (`docker/`) | Tiers 0–2. Builds `rocko_interfaces` and `rocko_env` without the motor plugin; runs the mock. Where an agent works |
| **Raspberry Pi** (Ubuntu 24.04 + Jazzy + WiringPi) | Everything, including the motor plugin. The only place the real robot builds. A human's machine |
| **Simulation** | Tier 3. See the [ladder](docs/architecture/verification-ladder.md) for whether it exists |

The mock is not a simulation.

Always `source /opt/ros/jazzy/setup.bash` first, build from the **workspace root** (not `src/`),
and use `--symlink-install`. The commands, run from the repo root on the laptop (they execute inside
the container; `-T` because no terminal is attached):

```bash
docker compose -f docker/compose.yml up -d
./check_container_mount.sh                                                                    # precondition, host
docker compose -f docker/compose.yml exec -T ros2 bash -c 'cd /ros2_ws && src/ROCKO-env/build_ros.sh'   # Tier 1
docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/verify_urdf.sh       # Tier 0
docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/verify_mock.sh       # Tier 2
docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/verify_mock.sh tank
.claude/hooks/test-guard-protected-files.sh                                                   # Tier 0, host
.githooks/test-git-hooks.sh                                                                   # Tier 0, host; only if you changed .githooks/
```

**One container, one checkout.** A laptop has one dev container, and it shows whichever *checkout*
(a folder holding a copy of this repo: a second clone, or a git worktree) last ran
`docker compose -f <checkout>/docker/compose.yml up -d`. Work in a checkout it is not showing and the
commands above still run and still pass, but they test the *other* folder's files. Run
`./check_container_mount.sh` from the repo root, in those words, at the start of a session and
whenever you change checkout: exit 0 means the container sees your files; exit 1 means it does not,
and it prints the fix; exit 2 means it could not tell (no Docker, or the container is not running).
`up -d` is itself the switch: it points the container at the checkout you run it from and takes it
away from a session using another one. And `ok` does not mean the build output is yours: after a
switch, rebuild from clean (the fix prints the command) before you trust Tiers 1 and 2. It is not on
the `settings.json` allow-list yet, so expect a prompt in the default permission mode
([proposal 0001](docs/proposals/0001-allow-container-mount-check.md)). The full explanation is in
[Verify a change without the robot](docs/guides/verify-in-container.md).

`build_ros.sh` passes `-DROCKO_SKIP_WIRINGPI` (compose sets it to `ON`) and exits 1 if the build
fails. Run it from `src/ROCKO-env/`, not `/ros2_ws/build_ros.sh`: that second copy is a single-file
mount that goes stale when the host file is replaced. The `build-verify` skill has the expected
output, the failure modes and the container details.

---

## 9. Reproducibility

Anything that produced a number — a gain set, a calibration, a latency measurement — is only useful
if someone else can get the same number. Record, with the result: the **commit SHA**, the **launch
file and config** used, **battery voltage**, the **surface**, and **where the log went**. See
[docs/guides/reproducing-results.md](docs/guides/reproducing-results.md). If you cannot fill those
in, say so out loud instead of omitting them. A result you got from the mock is labeled as such.

---

## 10. Conventions

- **C++17**, ROS 2 Jazzy, `ament_cmake`. Hardware plugins and controllers are C++; offline tools,
  calibration and telemetry may be Python.
- Match surrounding style. Existing C++ uses `snake_case` members with a trailing `_`; classes and
  files are `PascalCase`; Python nodes are `PascalCase.py` because they are installed as executables.
- New plugins start from `examples/example_hardware/` and must be registered in
  `rocko_env_hardware.xml` or `rocko_env_controllers.xml`, or ROS will not find them. Use the
  `add-hardware` skill, and keep the mock's interface names in step with the real ones.
- Don't commit `build/`, `install/`, or `log/` — gitignored, but they exist in the tree.
- The IMU directory and classes are named `ICM20948`; the part actually fitted is a **BNO055**.
  Don't "fix" this casually — it is a real rename touching plugin XML, URDF, and launch files.

---

## 11. Skills and agents

| Skill | Use it when |
| --- | --- |
| `build-verify` | Building, or climbing the verification ladder |
| `control-loop-change` | Touching any §6 critical-path file |
| `tune-gains` | Reasoning about or proposing PID gains |
| `add-hardware` | Adding a sensor or actuator plugin |
| `bootstrap-sim` | Building Tier 3, or any check the loop is missing |
| `write-docs` | Writing or updating anything in `docs/` |

| Agent | Checks |
| --- | --- |
| `realtime-auditor` | Real-time safety of the 100 Hz path |
| `ros2-control-reviewer` | Plugin registration, interface names, chaining, mock parity, version lock |
| `explainability-reviewer` | Simplicity and whether a student can follow it |
| `docs-reviewer` | Docs accuracy against the code, reproducibility |
