# Getting started: your first hour with ROCKO

**Goal:** go from a fresh clone to a laptop that builds the robot's code, runs every off-robot check, and runs
the simulated robot. After that you will know where to read and how to pick something to work on.

**Machine:** your laptop (macOS, or Windows inside WSL2). You do **not** need the robot for any of this. Every
command is run from the **repo root on your laptop**.

**What you need first:** `git`, [Docker Desktop](https://www.docker.com/products/docker-desktop/) running, and
`python3` and `g++` on the laptop (the host tests use them; macOS gets both from `xcode-select --install`), and a
network connection for the first build (it downloads packages).

## 1. Clone and start the dev container

The robot runs ROS 2 Jazzy on Ubuntu 24.04, which a laptop does not have, so the work happens inside a Docker
container that does. It sees your checkout's files directly: edit on the laptop, build in the container.

```bash
git clone https://github.com/BiPed-Capstone/ROCKO-env
cd ROCKO-env
git checkout ai-forward
docker compose -f docker/compose.yml up -d
./check_container_mount.sh
```

`ai-forward` is the branch where the work happens (`main` follows a slower, human-gated process:
[ADR 0005](../decisions/0005-ai-forward-development.md)).

Success: `up -d` ends with `Container rocko-env-dev  Started` (the first run downloads and builds an image,
several minutes), and the check prints `ok    the container sees this checkout: <your path>`. If it prints
anything else, it also prints the fix; the background is in
[Verify a change without the robot](verify-in-container.md#check-the-container-is-looking-at-your-files). A *worktree*,
which that page mentions, is a second working folder of the same repo.

## 2. Build and run every check

```bash
tools/run_ladder.sh
```

This builds the code in the container and runs the whole **verification ladder**: the checks that need no robot,
from "does the robot description expand" up to "does the controller hold a simulated robot upright". It prints
one line per check and ends with `RESULT: PASS`. The first run takes longest; the script's header gives the usual
times, and `tools/run_ladder.sh --no-sim` skips the slowest part. One file is left out of the build: the motor
plugin, which needs the Pi's GPIO library; a separate step only type-checks it.

Read [the verification ladder](../architecture/verification-ladder.md) next to it: it says what each line proves
and, as important, what it cannot. None of these checks can tell you whether the real robot balances.

If a line says `FAIL`, it prints the last lines of that check's output. The most common first-time cause is a
build left over from another checkout: `tools/run_ladder.sh --clean` rebuilds from nothing.

## 3. Run the simulated robot (optional)

The simulator is a physics model of ROCKO with estimated numbers (the [model's page](../architecture/sim-plant.md)
says which). It needs step 2's build. There is no picture: you watch its numbers. Two ROS words first: a *topic* is a
named stream of messages, and a *controller* is code the control loop runs every cycle (the [glossary](../glossary.md)
has both). Start a shell inside the container:

```bash
docker compose -f docker/compose.yml exec ros2 bash
```

then, inside the container:

```bash
source /opt/ros/jazzy/setup.bash
source /ros2_ws/install/setup.bash
ros2 launch rocko_env rocko_sim.launch.py
```

Success: the log scrolls for a few seconds and then quiets, with no `[ERROR]` lines.

In a second laptop terminal, open another container shell the same way and source the same two files. Then:

```bash
ros2 control list_controllers
```

Success: five controllers, each `active`: `velocity_pid_controller`, `pitch_pid_controller`,
`left_add_feedforward_controller`, `right_add_feedforward_controller` and `joint_state_broadcaster`. That is the
cascade from the [README](../../README.md#how-the-pieces-connect), running against the model.

```bash
ros2 topic echo /dynamic_joint_states
```

Success: a stream of messages, each listing `pitch`, `pitch_rate` and `velocity` for both wheels. `Ctrl-C` stops the
echo; then `Ctrl-C` in the first terminal stops the simulator, and `exit` leaves a container shell.

## 4. Read, in this order

1. The [root README](../../README.md): what the robot is, and the diagram of how a stick push becomes wheel speed.
2. [The control cascade](../architecture/control-cascade.md): why "go faster" means "lean further", and why the
   loops are nested the way they are.
3. [The glossary](../glossary.md), whenever a word is new.
4. [The balance-loop findings](../claude-analysis/balance-loop-findings.md): what is known to be wrong, by ID
   (C-1, H-3, ...), and what has been fixed.
5. The handoff box at the top of [next-steps.md](../next-steps.md): where the project is today.

## 5. Pick something to work on

[`next-steps.md`](../next-steps.md) is the work queue. Each row says who can do it (you, an AI agent, or only
someone at the robot) and what can check it. Rows marked for the robot need the robot and its owner; the others
can be done from a laptop.

Before changing anything, read the rules that apply to everyone, person or agent, in [`CLAUDE.md`](../../CLAUDE.md)
§3 (what a human decides: PID gains, version pins, safety behavior) and §5 (simple code a student can read). A
change is done when `tools/run_ladder.sh` passes and its docs changed in the same commit.

## 6. Working with Claude Code

If you have [Claude Code](https://claude.com/claude-code) installed and signed in, open it in the repo root. It loads [`CLAUDE.md`](../../CLAUDE.md) by itself; you do not need to paste
anything. Then give it a goal in one sentence: "take the next row of next-steps that you can do", or "fix finding
M-2". It plans, changes the code, runs the ladder, has reviewer agents check the change, writes a
[journal entry](../journal/README.md) saying what it checked, commits and pushes `ai-forward`.

What it will not do: change PID gains, version pins or a safety behavior (it writes a
[proposal](../proposals/README.md) for you to decide), run anything on the robot, or claim balancing works.

## Next

- Running and driving the real robot: [Run ROCKO on the Pi](run-on-the-pi.md) and
  [Drive ROCKO with a USB controller](drive-with-usb-controller.md).
- The full laptop walk-through, step by step with expected output: [Verify a change without the robot](verify-in-container.md).

## What commonly goes wrong

| You see | Why | Do |
| --- | --- | --- |
| `Cannot connect to the Docker daemon` | Docker Desktop is not running | Start it, wait for it to say running, retry |
| `check_container_mount.sh` prints a fix instead of `ok` | The container shows another checkout (a second clone or a worktree) | Run the `up -d` line it prints, then `tools/run_ladder.sh --clean` |
| `ros2: command not found` inside the container | The terminal is new and not sourced | The two `source` lines in step 3, in every new container shell |
| A host test fails with `g++: command not found` | No compiler on the laptop | `xcode-select --install` (macOS) or `sudo apt install g++` (WSL2) |
