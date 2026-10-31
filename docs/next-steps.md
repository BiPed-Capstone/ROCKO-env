# Next steps

**This is the work queue.** When you have no specific goal, take the first row below that you can do
and check. It owns three things no other page does: the **order**, **who** can do each item, and the
**status**. It deliberately does not repeat what a flaw *is* or how to fix it; each row links to the
page that does, so that fact has one home.

Checked against the code on 2026-10-28 (commit `059e602`). Two GitHub issues that were already fixed
(#15, #17) have been closed; others are only partly met, or fixed on this branch only. The
[last section](#github-issues-against-the-code) says which.

## How to use it

1. Pick the first row you can do (see *Who* and *Best check today*). Read its source in full before
   you start.
2. Follow the loop in [`CLAUDE.md`](../CLAUDE.md). Say in the journal which tier can check the
   change; anything only a robot can confirm goes in the hand-off procedure.
3. In the **same commit**, update the row's status here and mark the item fixed in its source (the
   finding, the issue note, the backlog row).
4. Found a new flaw? Add it to the [findings](claude-analysis/balance-loop-findings.md) and add a row
   here.
5. Gated change (gains, version pins, weakening a safety behavior, the guardrails)? Write a
   [proposal](proposals/README.md) and put its number in the row.
6. **Nobody watching** (a headless run, started with `claude -p`, or the person said they are away)?
   Take only rows in [Wave 0](#wave-0--do-now-nothing-blocks-these) that are *open* or *partly done*,
   whose *Who* starts with `agent`, and whose *Best check today* names a tier (not `—` or `none`): nothing could check
   the others, and "done" without a check is what `CLAUDE.md` §4 forbids. Today that is 0.1 to 0.6
   and 0.9. Work them in order. Wave 1 and later only when the person names the row. For an
   `agent + robot` row, write the code, run the checks, write the robot procedure into the journal
   entry, and set the status to *done, awaiting robot*. If what is left belongs to another row (0.1's
   end state needs 1.1), say so in the row; the status is still *done, awaiting robot*. Then go on to
   the next row.

## Reading the table

- **Who.** *agent*: an AI can write it and check it as far as the
  [verification ladder](architecture/verification-ladder.md) allows. *agent + robot*: an AI writes
  it, but only a person on the Pi can confirm it. *human*: needs a measurement or a decision.
- **Status.** A row's status starts with one of these, so a loop can tell where it stands: *open*
  (not started), *partly done* (some landed; the row says what is left), *done, awaiting robot* (the
  code landed and was checked as far as the ladder allows; a person has to run the procedure in the
  journal entry), *blocked on human* (agent work that waits on a decision or a measurement; the row says which),
  *done*. Change the word in the same commit that changes the row's state.
- **Best check today** is the highest tier that can say anything about the change, per the ladder.
  Until row 0.2 lands, the motor plugin is not compiled off a Pi at all. Nothing here is checked
  against dynamics until Tier 3 exists (rows 2.1 to 2.3), and only the robot (Tier 4) shows whether
  anything balances.
- Files on the 100 Hz path (`CLAUDE.md` §6) make `realtime-auditor` mandatory.

## Wave 0 — do now, nothing blocks these

Ordered by what can be done and checked now, **not** by risk, so it runs ahead of the audit's steps
2 to 5, which need a Pi or row 0.2. The audit's own order is not changed: C-2 (its step 1) is still
first here, and Wave 1 keeps its steps 2 to 5 in sequence.

| # | Item | Who | Best check today | Status and what to know |
| --- | --- | --- | --- | --- |
| 0.1 | **C-2**: a failed IMU read must not report pitch 0.0 (audit step 1) | agent + robot | T1 (service, node and plugin compile) | open. More than one file: today the `Icm20948Data.srv` response has no failure field, so the `.srv`, the node (`ICM20948.py`) and the plugin (`ICM20948.cpp`) all change. The mock replaces this plugin, so Tier 2 cannot see it. End state: [C-2](claude-analysis/balance-loop-findings.md), reached with 1.1 |
| 0.2 | **Compile-only check for the motor plugin** ([`bootstrap-sim`](../.claude/skills/bootstrap-sim/SKILL.md) step 3) | agent | T1 (new check) | open. **Unlocks 1.3 and 1.4.** Six findings cite this file (C-1, H-1, H-2, M-1, M-2, L-1) and a seventh, M-3, is about its `pwmVal`; none can be type-checked off a Pi until it exists |
| 0.3 | **M-5**: unchecked `set_value()` result | agent | T1 compile, T2 loads. The failure path cannot be provoked on the mock | open. [M-5](claude-analysis/balance-loop-findings.md). Critical path |
| 0.4 | **L-3**: deprecated include; invalid escapes in `RateLimiter.py` | agent | T1 (warnings gone) | open. [L-3](claude-analysis/balance-loop-findings.md). The include is in the controller header, so the auditor is mandatory |
| 0.5 | **L-4**: RViz layouts are never installed | agent | T1 (`install/.../share/rocko_env/rviz/` exists). That RViz opens with them needs a display, so a person | open. [L-4](claude-analysis/balance-loop-findings.md) |
| 0.6 | [#11](https://github.com/BiPed-Capstone/ROCKO-env/issues/11): `view_robot.launch.py` default argument | agent | T1, plus a manual look that the default now resolves to an installed file (needs 0.5) | open, confirmed: the default is still `"description"`. The README's workaround goes away with the fix |
| 0.7 | [#16](https://github.com/BiPed-Capstone/ROCKO-env/issues/16): the wheel radius disagrees | human measures, agent finishes | T2 | blocked on human. There are **four** values, not three: `0.06` m (the Python nodes), `0.127` m (the tank config) and `0.015` m (`wheel_radius` in `rocko_description.urdf.xacro`, which a Tier 3 model would use). An agent cannot know which is right. Measure once (the same measurement serves 2.2), then keep a single source |
| 0.8 | [#14](https://github.com/BiPed-Capstone/ROCKO-env/issues/14): unpinned Python dependencies | agent + robot | — | blocked on human: no requirements file exists. The versions must come from a working Pi, otherwise the pins are invented |
| 0.9 | [#13](https://github.com/BiPed-Capstone/ROCKO-env/issues/13): no tests in the packages | agent | T0, T2 | partly done on `ai-forward` by `verify_urdf.sh` and `verify_mock.sh`, which are scripts, not `colcon test`. Still missing: unit tests for `RateLimiter.py`, and running the checks under `colcon test` or CI |

## Wave 1 — the control-loop fixes, in the audit's order

The audit's steps 2 to 5, from its [order of work](claude-analysis/balance-loop-findings.md). All of
it is on the critical path. The audit asks that steps 1 to 5 be shown to reduce the spread of loop
period and feedback age *before* any tuning time is spent.

| # | Item | Who | Best check today | Status and what to know |
| --- | --- | --- | --- | --- |
| 1.1 | **C-1 (IMU)**, with the **UART migration** and **H-3**; the IMUPLUS mode and quaternion read in [`bno055-improvements.md`](claude-analysis/bno055-improvements.md) §§ 3–4 fold in | agent + robot | T1 compile | open. Needs a wiring change and a Pi. That doc also covers the hardcoded pitch offset (`7.6` in `ICM20948.py`) |
| 1.2 | **H-6**: gyro rate for the D term | agent + robot | T1; T2 only once the mock block also exports the new interface | open. The axis and sign must be established on the robot: read the finding first. Changing a gain is gated |
| 1.3 | **C-1 (encoders)** | agent + robot | none until **0.2** (the encoder read is in the motor plugin) | open |
| 1.4 | **H-2, M-1, M-3**, and **L-1** (same file) | agent + robot | none until **0.2** | open. Confirming any of it needs the robot |
| 1.5 | **M-2**: software PWM period | human | — | open. Moving to hardware PWM means moving a wire. Obviated by a motor-driver change; see below |
| 1.6 | **L-2**: rename `ICM20948` to the BNO055 | agent | T0, T1. A plugin-name mismatch between the plugin XML and the xacro would show only on a Pi | open. Touches the plugin XML, the URDF and the launch files, and `verify_urdf.sh` expects the current names. Do it while 1.1 has the plugin open |

## Wave 2 — simulation, then the safeguards that should be built against it

| # | Item | Who | Best check today | Status and what to know |
| --- | --- | --- | --- | --- |
| 2.1 | **Choose the Tier 3 plant**: Gazebo ([ADR 0003](decisions/0003-simulation-target.md)) or `SimPlant` ([M-4](claude-analysis/balance-loop-findings.md)) | human decides | — | open. The agent writes the proposal; `bootstrap-sim` has the comparison and a recommendation |
| 2.2 | **Measure the robot** (`bootstrap-sim` step 4) | human | — | open. The agent writes `docs/guides/measure-inertia.md` first. Share the wheel measurement with 0.7 |
| 2.3 | **Build the plant and the scenarios** (`bootstrap-sim` steps 6–7) and **the sim-to-real gap page** (step 8) | agent | T3, once built | open. Steps 6–7 are blocked by 2.1 and 2.2; step 8 is not blocked by 2.1. [M-4](claude-analysis/balance-loop-findings.md) is partly done (the mock and the off-Pi build exist; the physics do not), and [#12](https://github.com/BiPed-Capstone/ROCKO-env/issues/12) is the same item |
| 2.4 | **Let the loop run the sim** (`bootstrap-sim` step 9) | agent writes a proposal, human approves | — | open. Needs 2.3 first; the agent then writes a proposal, because it edits `.claude/settings.json`, a guardrail (`CLAUDE.md` §3) |
| 2.5 | **C-3**: teleop watchdog | agent writes, human sets the threshold | none (Tier 3 is not built). Tier 2 may show the velocity reference dropping after the timeout, since the mock runs `Joystick.py` (untried) | open. The right failsafe is counter-intuitive: read [C-3](claude-analysis/balance-loop-findings.md) before writing any code |
| 2.6 | **H-4**: tipover cutoff | agent writes, human sets the threshold | none (Tier 3 is not built) | open. *Adding* a safeguard is allowed; its threshold goes in a proposal |

## Wave 3 — needs the robot

| # | Item | Who | Status and what to know |
| --- | --- | --- | --- |
| 3.1 | **H-5**: real-time scheduling and CPU isolation | human | open. Verify with a histogram of loop periods, not an average |
| 3.2 | **H-1**: close the wheel-velocity loop | agent + robot | open. Obviated by a motor-driver change; see below |
| 3.3 | **Retune the gains, last** ([`tune-gains`](../.claude/skills/tune-gains/SKILL.md)) | human | open. Gains are gated |

**A hardware decision is pending** (findings, "Related hardware work"): brushless motors with ODrive
controllers. It would obviate H-1 and M-2 and partly C-1. The audit says **not to wait for it**: the
other fixes are independent, and several are needed to judge it.

## Decisions waiting on the maintainer

Not agent work. Each is recorded where it was raised.

- The `settings.json` allow-list (it can run code around the hook) and whether `CLAUDE.md`, the
  agents and the skills should be guarded: [ADR 0005](decisions/0005-ai-forward-development.md).
- What to allow, deny and enforce for a run nobody is watching: the permission mode, the loop's
  commands, a narrower `gh`, a deny list, git hooks and a rule on `main`:
  [proposal 0002](proposals/0002-unattended-run-settings.md): A, B, C and D are applied (2026-10-30);
  E and F are open. Guarding `CLAUDE.md`, the agents and the skills is still undecided and matters
  more now: under `acceptEdits` edits to them are expected not to prompt (not tried).
- A PyYAML-based gain check, because the hook reads lines, not YAML: the hook's docstring.
- The single-file `build_ros.sh` mount in `docker/compose.yml`, which goes stale: the ladder's blind
  spots and the [2026-10-27 journal](journal/2026-10-27-ai-forward-bootstrap.md).
- Whether the dev container should be one per checkout (a compose project name per checkout and no
  fixed `container_name`, so a worktree gets its own container and build volume), or whether
  `check_container_mount.sh` is enough: the ladder's
  [blind spots](architecture/verification-ladder.md#blind-spots-stated-plainly). (The script has been
  on the allow-list since 2026-10-30: [proposal 0001](proposals/0001-allow-container-mount-check.md).)
- Gazebo vs `SimPlant` (2.1).
- Whether `main` should adopt any of the `ai-forward` workflow: ADR 0005.
- Which wheel radius is right (0.7).

## Documentation gaps

An agent can draft these; a person checks the facts. They are listed in
[`architecture/README.md`](architecture/README.md) and [`guides/README.md`](guides/README.md).

## GitHub issues against the code

Issues #10 to #17 as of 2026-10-28 (none labelled), compared with the tree. #15 and #17 were closed
that day, at the maintainer's instruction, with a comment naming the evidence on `main`. Closing or
commenting on an issue is the maintainer's call: an agent lists what it thinks can be closed in its
report, and closes one only when told to.

| Issue | State in the code |
| --- | --- |
| [#10](https://github.com/BiPed-Capstone/ROCKO-env/issues/10) `use_mock_hardware` does nothing | **Done on `ai-forward`**: it selects `mock_components/GenericSystem`. Left open on purpose: `main` does not have the fix. Close it when the branch merges |
| [#11](https://github.com/BiPed-Capstone/ROCKO-env/issues/11) `view_robot.launch.py` default | **Open** (0.6) |
| [#12](https://github.com/BiPed-Capstone/ROCKO-env/issues/12) no simulation | **Partly**: the mock landed, as the issue suggested first; no physics (2.3) |
| [#13](https://github.com/BiPed-Capstone/ROCKO-env/issues/13) no tests | **Partly** (0.9) |
| [#14](https://github.com/BiPed-Capstone/ROCKO-env/issues/14) unpinned Python dependencies | **Open** (0.8) |
| [#15](https://github.com/BiPed-Capstone/ROCKO-env/issues/15) `setup.py` is dead code | **Closed 2026-10-28.** Already fixed on `main` (the file was removed in `a7d302a`) |
| [#16](https://github.com/BiPed-Capstone/ROCKO-env/issues/16) wheel radius disagrees | **Open** (0.7), and wider than the issue says |
| [#17](https://github.com/BiPed-Capstone/ROCKO-env/issues/17) template metadata in `package.xml` | **Closed 2026-10-28.** Already fixed on `main`: both packages have real descriptions and maintainers |

## Keeping this page true

Update a row in the same commit that changes its status. If a row here disagrees with the code, the
code is right and this page is a bug. Re-check the issues table whenever an issue is opened or closed.
