# Run journal — what the AI did, and what it actually checked

**New here?** Read [the project history](history.md) first: the whole journal in one page, with links to the
entries that matter.

One file per goal that changed behavior. It is the record a human reads to decide whether to trust a
change, and the thing that lets a stranger reproduce it. It is also the explanation trail: *why* the
change exists, in prose, next to the commit.

**Write one** for any change to code, launch files, URDF, build config or tooling. **Skip it** for
typo fixes and for documentation-only changes.

The rule that matters: **the journal records tiers that ran, not tiers that exist.** Each line under
Verification names a command and what it printed. If a tier was not run, the line says *not run* and
why. See the [verification ladder](../architecture/verification-ladder.md) for what each tier proves
and what it cannot.

## Template

Copy to `YYYY-MM-DD-short-slug.md`.

```markdown
# <Goal, as a sentence>

- **Date / commit:** YYYY-MM-DD, built on <base sha> (an entry cannot name the commit that adds
  it; say "the commit that adds this file")
- **Branch:** <branch>
- **Environment:** container image / Pi / laptop; ROS distro; anything non-default

## Goal and success criteria

What the human asked for, then the *checkable* statements that mean it is done.
1. ...

## Plan

Files to touch and the order. Which tier will prove each criterion. Risks.

## What changed

File → one line on what and why. Link the ADR or doc that explains it.

## Verification

| Tier | Command | Result |
| --- | --- | --- |
| 0 static | `verify_urdf.sh`; hook tests | pass / FAIL / not run (why) |
| 1 build | `build_ros.sh` | ... |
| 2 mock | `verify_mock.sh`, `verify_mock.sh tank` | ... |
| 3 sim | | see the ladder |

Warnings read and what they meant.

## Review rounds

| Round | Reviewer | Findings | Fixed / declined (why) |
| --- | --- | --- | --- |

## Not verified — needs a human

What this does NOT prove, and the procedure for a person to test it on the robot (tethered; success
looks like ...; abort if ...). Or a link to the proposal that holds it.

## Reproduce

Exact commands a stranger runs to get the same result.
```

## Index

| Date | Goal |
| --- | --- |
| 2026-10-27 | [Make the AI-forward loop real: mock hardware, off-Pi build, verify scripts](2026-10-27-ai-forward-bootstrap.md) |
| 2026-10-28 | [Let the agent push `ai-forward`](2026-10-28-allow-ai-forward-push.md) |
| 2026-10-29 | [Make the verify scripts notice when they are testing the wrong checkout](2026-10-29-container-mount-check.md) |
| 2026-10-29 | [Get the repo ready for a run nobody is watching: rules, queue and git hooks](2026-10-29-unattended-run-readiness.md) |
| 2026-10-29 | [Stop asking before every push to `ai-forward`](2026-10-29-push-without-asking.md) |
| 2026-10-30 | [Apply items B, C and D of proposal 0002 to the settings](2026-10-30-apply-proposal-0002-b-d.md) |
| 2026-10-30 | [Make `acceptEdits` the project's default permission mode](2026-10-30-default-mode-accept-edits.md) |
| 2026-10-31 | [Fix C-1/C-2/C-3 (+M-5, H-4, H-6) and build Tier 3: a `SimPlant` physics simulation](2026-10-31-sim-plant-and-critical-fixes.md) (stages 2 to 4: foundations, the plant, and the tipover cutoff / teleop watchdog / pitch rate with their sim checks; C-1 itself was finished on 2026-11-24, below) |
| 2026-11-03 | [Stage 5: find why the model drifts off (finding H-12), and write the balance scenarios before any tuning](2026-11-03-stage5-balance-in-sim.md) |
| 2026-11-03 | [Find why checks C3 and FF are flaky, and make them judge send times](2026-11-03-c3-ff-flaky-checks.md) |
| 2026-11-03 | [Stage 5c-1: tune the sim gains until scenarios U, I, V pass (thresholds unchanged)](2026-11-03-stage5c1-sim-gain-tuning.md) |
| 2026-11-03 | [Stage 5c-2: the ±30 % robustness sweep of the sim gains (mass, centre-of-mass height, motor gain, IMU delay)](2026-11-03-stage5c2-sweep.md) |
| 2026-11-04 | [Final report, stages 4 to 6: tiers reached, failures, declined findings, proposals 0003 to 0009, what was not verified, the tethered procedure, the sim's omissions](2026-11-04-final-report-stage-4-6.md) |
| 2026-11-07 | [Two noisy verifiers: scenario V's overshoot metric (limit unchanged) and check C3's second flake](2026-11-07-c3-v-verifier-noise.md) |
| 2026-11-07 | [Stage 5c-2 sweep re-run with V's corrected overshoot metric: PASS 18, MARGINAL 4, FAIL 2, COULD NOT JUDGE 1](2026-11-07-stage5c2-sweep-rerun.md) |
| 2026-11-07 | [Stage 5c review fixes (a tuned gain on every joint; simpler sim scripts) and why V could not judge two `d+` runs](2026-11-07-stage5c-review-fixes.md) |
| 2026-11-07 | [Make check H4 wait for the re-arm refusal instead of recording a fixed 16 s (and re-send a lost re-arm)](2026-11-07-h4-verifier-fix.md) |
| 2026-11-07 | [Stage 6, bounded: the BNO055 UART framing as a tested header, the rest of C-1 as a proposal](2026-11-07-c1-bno055-uart-framing.md) |
| 2026-11-08 | [Measure what the agent loop cost, and change the tools and instructions that drive it](2026-11-08-agent-loop-efficiency.md) |
| 2026-11-16 | [Readiness review, ramped drive commands (H-13), keyboard driving, a web dashboard and logging](2026-11-16-teleop-dashboard-logging.md) (in progress: its HANDOFF box says which stage is next) |
| 2026-11-19 | [Helper scripts for running ROCKO on the Pi as root, and the Pi workspace name](2026-11-19-pi-helper-scripts.md) |
| 2026-11-19 | [First run on the robot since `ai-forward`: tank mode drives, the motors hum; known red checks; the plan for the Xbox controller and self-balancing](2026-11-19-first-robot-run-tank-mode.md) |
| 2026-11-20 | [Drive tank mode with an Xbox controller over Bluetooth from the Mac, split arcade](2026-11-20-xbox-controller-split-arcade.md) |
| 2026-11-22 | [Drive tank mode from an 8BitDo controller plugged into the Pi; remove keyboard and laptop driving](2026-11-22-usb-controller-tank-mode.md) |
| 2026-11-22 | [First USB-controller drive on the robot, and why tank mode reverses slowly](2026-11-22-tank-reversal-lag.md) |
| 2026-11-22 | [Start the web dashboard at boot with a systemd service](2026-11-22-dashboard-boot-service.md) |
| 2026-11-22 | [Tank mode starts at boot; the 160 m/s² result; handoff to balance mode](2026-11-22-tank-boot-service-and-handoff.md) |
| 2026-11-23 | [One encoder service per wheel, speed over the measured time (row 4.10, finding U-2; since replaced by encoders in C++)](2026-11-23-encoder-service-u2.md) |
| 2026-11-23 | [Gyro check on the robot: which gyro axis is pitch, its sign and unit](2026-11-23-gyro-check.md) |
| 2026-11-23 | [A supervisor node owns tank/balance mode and enable, switched from the controller or the dashboard](2026-11-23-robot-supervisor.md) |
| 2026-11-24 | [Read the IMU from C++ over I2C (C-1, IMU half) and make a failed zero-on-deactivate visible (row 4.14)](2026-11-24-c1-imu-i2c-and-zero-on-deactivate.md) |
| 2026-11-24 | [Count the wheel encoders inside the C++ motor plugin (C-1, encoder half)](2026-11-24-c1-encoders-in-cpp.md) |
| 2026-11-24 | [Measure the control loop's timing on the Pi, and decide what may run during tuning](2026-11-24-loop-timing-on-the-pi.md) |
| 2026-11-24 | [Prepare the gyro D term for the robot: handle finding L-8 and propose the YAML change](2026-11-24-pitch-d-term-from-the-gyro.md) |
| 2026-11-24 | [The simulation runs the robot's measured signs, and its outer loop is retuned for the true wheel speed](2026-11-24-sim-measured-signs-retune.md) |
| 2026-11-24 | [Stand sign check on the robot: the encoder reads positive under a positive command (proposal 0006)](2026-11-24-stand-sign-check.md) |
| 2026-11-25 | [Make level read 0 (pitch offset), and propose a clamp on the target lean after the first floor test](2026-11-25-level-pitch-offset-and-lean-clamp.md) |
| 2026-11-25 | [Work without the robot: the S2a failure explained, the floor test reproduced in a model, proposal 0008 brought up to date](2026-11-25-offline-sim-work.md) |
| 2026-11-26 | [Cleanup while the robot is away: remove dead code and files, bring stale docs up to date](2026-11-26-cleanup.md) |
| 2026-11-26 | [Rename the IMU plugin from `ICM20948` to `BNO055` (finding L-2)](2026-11-26-rename-imu-plugin-bno055.md) |
| 2026-11-26 | [Make the repo easier to start in: fewer files, a shorter path to the open decisions](2026-11-26-repo-tidy.md) |
