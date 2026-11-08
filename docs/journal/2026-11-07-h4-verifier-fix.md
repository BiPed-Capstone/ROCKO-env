# Make check H4 wait for the re-arm refusal instead of recording a fixed 16 s

- **Date / commit:** 2026-11-07, built on `dc45e0c`. The change is in the commit that adds this file.
- **Branch:** `ai-forward`. Local commit only: this run was told not to push.
- **Environment:** `rocko-env-dev` container (`ros:jazzy`, arm64, Docker Desktop on macOS).
  `./check_container_mount.sh` printed `ok    the container sees this checkout` at the start. This run
  was the only user of the container; `ps` in it showed only the ROS daemon before the first run.
  `build_ros.sh`: `Summary: 2 packages finished` (no C++ changed in this run).

**Everything here is about the model and the check around it.** Nothing says anything about the real
robot. No gain, threshold, controller or YAML was changed.

## Goal and success criteria

`verify_sim.sh H4` (the tipover cutoff, finding H-4) failed 2 of 4 runs on 2026-11-07
([C-1 journal](2026-11-07-c1-bno055-uart-framing.md#sim-run)). Each time the cutoff itself passed
(1599 of 1599 commands past 35 deg exactly 0.0, both controllers tripped and refused the re-arm); the
check failed on its own rule "the commands were watched for 1 s after the refused re-arm", because the
refusal was logged after its fixed 16 s recording window had closed. A verifier that fails on its own
timing is a broken sensor (CLAUDE.md §4). Fix the check without loosening what it proves:

1. The cause named from run logs (H4 alone about 10 times, logs kept, the re-arm's send time and each
   controller's refusal time recorded).
2. H4 fails only on a real violation: no trip, a non-zero command while tripped, a controller that
   accepts the re-arm at 60 deg, a refusal missing, or commands not watched after it.
3. 20 consecutive green H4 runs.
4. Red on the deliberate breaks: the cutoff off; the re-arm band bypassed in a scratch build so the
   60 deg re-arm is accepted. The repo and the real build untouched afterwards.
5. The default `verify_sim.sh` once (expect PASS S1..FF) and `verify_sim.sh scenarios` once (exit 0).

## What was found

### Cause 1: the re-arm goes out 3 to 19 s after the first write, and the window was a fixed 16 s

The checker (`scenario_checks.py tipover`) started its 16 s window at the controllers' first written
command. The shell meanwhile ran the wiring check (`wait_for_wiring`: three `ros2 control` CLI calls
per look, repeated each second until everything is up), waited for the trip lines, and only then
published the re-arm. Whether the refusal plus 1 s fitted into 16 s depended on how long those CLI
calls took. It is not the ~0.3 s process stalls of C3/FF: it is seconds.

Evidence, times in seconds after the first written command (from the controllers' own log stamps and
the checker's sample stamps, one clock):

- **The two failures of 2026-11-07**, run directories still in the container
  (`/tmp/tmp.bDj5NqSs4v`, `/tmp/tmp.hxB7x4CCUB`): both controllers refused at **18.95** and **16.76**;
  the last sample was at 15.99 and 16.00.
- **Every older kept H4 run** (11 directories, 2026-11-03 and 11-04): refused between 3.83 and 12.71.
- **10 runs of H4 as it was, in a scratch copy that kept every run and stamped the shell's steps**
  (all 10 passed):

| Run | Wiring check done | `ros2 topic pub` started | returned | Both refused | Last sample | Margin over the 1 s rule |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 4.90 | 4.91 | 5.98 | 5.74 | 16.00 | 9.26 |
| 2 | 8.27 | 8.27 | 9.35 | 9.11 | 16.00 | 5.89 |
| 3 | 6.98 | 6.98 | 9.06 | 8.82 | 16.00 | 6.18 |
| 4 | 7.48 | 7.48 | 8.53 | 8.30 | 16.00 | 6.70 |
| 5 | 2.81 | 2.81 | 3.89 | 3.66 | 16.00 | 11.34 |
| 6 | 5.24 | 5.24 | 6.32 | 6.09 | 16.00 | 8.91 |
| 7 | 7.24 | 7.24 | 8.29 | 8.05 | 15.99 | 6.94 |
| 8 | 13.04 | 13.05 | 15.04 | 14.78 | 16.00 | **0.22** |
| 9 | 6.97 | 6.97 | 8.03 | 7.79 | 16.00 | 7.21 |
| 10 | 9.74 | 9.74 | 11.07 | 10.81 | 16.00 | 4.19 |

In the two runs looked at (5 and 8) all the controllers were active at the moment of the first write
(the spawner's `Configured and activated` line is stamped within 2 ms of it), so the 2.8 to 13.0 s before the wiring check finished
is the CLI calls, not the controllers. The publish itself took 1.05 to 2.07 s (not the ~3 s written
earlier), and both refusals were stamped 0.24 to 0.27 s **before** it returned. Run 8 passed by 0.22 s.

### Cause 2 (found while checking the fix): a re-arm message can be lost

In the first batch of 20 runs of the fixed check, run 5 failed a new way:
`ros2 topic pub --once -w 2` printed `publishing #1` and exited 0, and **neither** controller logged a
refusal, then or later (no line at all in the launch log). The cutoff part passed (2084 of 2084
commands exactly 0.0). So the one message was lost on its way, once in 30 runs (10 diagnostic + 20).
Likely mechanism, **not shown**: `-w 2` waits until the *publisher* has matched both subscribers, but
discovery is not symmetric, and a subscriber that has not yet matched the publisher drops the sample;
`--once` exits 0.1 s later. Run directory: `/tmp/tmp.KZBZa3H1ad`. The old check would have failed the
same way; it just had not been seen.

## What changed

- `sim/scenario_checks.py`, `tipover`: no fixed recording time. A new required `--until-file`: the
  checker records until it holds a sample **stamped** at or after the epoch-seconds time in that file.
  `--record` is now only a cap (120 s) so it cannot wait forever if its samples stop. A slow shell or a
  stalled checker can only make the recording longer, never cut it short.
- `verify_sim.sh`, `check_tipover`:
  - writes that file once it has both refusals: the **later** refusal's log stamp + `REARM_OBSERVED_S`
    (1 s); on any path without a refusal, the current time (the failure is already recorded), so the
    checker stops at its next sample.
  - prints when `ros2 topic pub` started and returned, and when each controller refused.
  - counts a refusal only if it is stamped after the publish of that attempt started, and fails if a
    controller logged a refusal before any re-arm was sent.
  - sends the re-arm again, up to `REARM_ATTEMPTS` = 3, when an attempt is not refused by **both**
    controllers within `REARM_REPLY_WAIT_S` (5 s) of the publish returning. Each attempt is judged
    alone: it counts only if both controllers refuse **that** message, so "one message on the shared
    topic reaches both wheels" is still what is shown. Attempts are more than 5 s apart and the
    controller rate-limits its refusal line to one a second, so an attempt that arrives is always
    logged. A controller that accepts any attempt logs `RE-ARMED`, which fails the check as before.
  - `TIPOVER_RECORD_S` (16) became `TIPOVER_RECORD_MAX_S` (120, a cap).

What it proves is unchanged or stricter: tripped in the first cycle, every command past 35 deg exactly
0.0, both controllers refuse one re-arm message, nobody re-arms, commands watched at least 1 s after
the later refusal, controllers active. New: no refusal before a re-arm was sent, and each refusal is
tied to its attempt. What is shorter: the recording now ends 1 s after the refusal (618 to 1458
samples in the 20 runs below, against about 1600 before) instead of at a fixed 16 s, which used to
watch up to 11 s more than the rule asked for (0.2 s more in the closest run). Nothing was claimed about that extra time. What is
weaker, said plainly: a single lost message no longer fails the run; two lost in a row still cost a
retry each, and three fail it.

## Verification

All runs in the container, from the repo root on the laptop, after `./check_container_mount.sh`
(`ok`). Container hygiene: before each batch `ps` in the container showed only the ROS daemon.

**A harness mistake of this run, recorded so it is not repeated:** a first fixed batch of 20 ended
12 `PASS`, then 8 exits 2 (`a controller_manager (ros2_control_node) is already running`). The cause
was this agent: a waiting loop it ran inside the container had the text `ros2_control_node` in its own
command line, and `verify_sim.sh`'s `pgrep -f ros2_control_node` matched it. Run 13's check itself had
passed. The loop was killed and the whole batch of 20 run again (below); the 12 passes are not counted.

| What | Command | Result |
| --- | --- | --- |
| Tier 1 | `build_ros.sh` (start, and again after the breaks) | `Summary: 2 packages finished` both times |
| H4 as it was, 10 runs, scratch copy keeping every run | `/tmp/h4diag/verify_sim.sh H4` | 10 of 10 `PASS`; timeline in the table above (run 8 by 0.22 s) |
| Fixed H4, first version (no re-send) | `verify_sim.sh H4` x 20 | 19 `PASS`, 1 `FAIL` (run 5: the lost message, cause 2), which led to the re-send |
| **Fixed H4, committed version** | `verify_sim.sh H4` x 20, consecutive | **20 of 20 `PASS`**, exit 0 every time; every run's first message was refused by both (no second attempt); 618 to 1458 samples with commands, all exactly 0.0 past 35 deg |
| Break (a): cutoff off | scratch workspace `/tmp/h4ws_off` (`tipover_pitch_deg` default 0.0), `ROS2_WS=/tmp/h4ws_off .../verify_sim.sh H4` | exit 1: `10943 of 10943 samples past the limit had a non-zero wheel command; first: pitch 40.00 deg, commands -600, -600 rad/s`; both `never logged 'TIPOVER CUTOFF TRIPPED'`; the publish timed out (exit 124: with the cutoff off the controllers do not subscribe to `/tipover_rearm`) |
| Break (b): re-arm accepted at 60 deg | scratch workspace `/tmp/h4ws_rearm` (`tipover_latch_.update(..., 1.0e9)` in the running block of `AddFeedforwardController.cpp`) | exit 1: `a controller logged 'RE-ARMED' although the robot was past the limit`; both `did not log 'Tipover re-arm refused' within 5 s of any of 3 /tipover_rearm messages`; `3 of 2674 samples past the limit had a non-zero wheel command; first: pitch 60.05 deg, commands -900.8, -900.8 rad/s` (each accepted re-arm let one cycle through before the cutoff tripped again) |
| Break (c): every message lost | scratch copy: all attempts published to `/tipover_rearm_lost` | exit 1: both controllers `did not log 'Tipover re-arm refused' within 5 s of any of 3 /tipover_rearm messages` |
| (d): first message lost | scratch copy: attempt 1 to `/tipover_rearm_lost` | exit 0: attempt 1 `refused by left at never, by right at never`, attempt 2 refused by both |
| (e): re-arm 12 s late | scratch copies with `sleep 12` before the publish: the check as it was, and the new one | old: exit 1, `the commands were not watched for 1 s after the refused re-arm (refused ...956.17, last sample ...954.81)` (the flaky failure, made on purpose); new: exit 0 |
| Default run | `verify_sim.sh` | exit 0, `PASS (simulation: S1 S2a H4 C3 C2 H6 FF)`, every run `ok`; H4's re-arm refused by both on the first message |
| Scenarios | `verify_sim.sh scenarios` | exit 0, `SCENARIOS: PASS`: U, I, V `PASS` (V overshoot 15.4 %), S2b and AB `ok` (AB fell at 3.770 s) |

After the breaks: the repo was never edited for them (`git status` showed only this change), the
real workspace was rebuilt (`Summary: 2 packages finished`), and `ps` in the container showed only
the ROS daemon. The scratch directories are left in the container's `/tmp` (`h4diag`, `h4fix*`,
`h4late_*`, `h4lost*`, `h4ws_*`).

Tiers 0 and 2 (`verify_urdf.sh`, `verify_mock.sh`) were not run: nothing they check changed (no URDF,
launch, YAML or C++ edit).

## Review rounds

Not run in this sub-task: the orchestrator that set it did not include the reviewer agents. The change
touches no §6 file. `explainability-reviewer` (code) and `docs-reviewer` (docs) are owed.

## Not verified — needs a human

Nothing here touches the robot. Not shown: why one re-arm message was lost (the mechanism above is a
guess); whether a lost message can also reach only ONE controller (not seen; the check would re-send
and require both to refuse the new message). Still wall time: the 5 s wait for each refusal after the
publish returned, which a stall would have to exceed by about 4.7 s to matter. The re-send path ran
once on purpose (break d), never on its own in the 20 runs. The check still does not test an accepted re-arm in the sim or the 10
to 35 deg re-arm band (the host test `tools/tipover_latch_test.cpp` does), and its other blind spots are
unchanged ([ladder](../architecture/verification-ladder.md), H4 blind spot).

## Reproduce

From the repo root on the laptop:

```bash
./check_container_mount.sh
docker compose -f docker/compose.yml exec -T ros2 bash -c 'cd /ros2_ws && src/ROCKO-env/build_ros.sh'
docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/verify_sim.sh H4
```

Success: `PASS (simulation: H4)`, exit 0, with two `re-arm: ..._add_feedforward_controller refused it
at <stamp>` lines, both stamps after the `'ros2 topic pub' started` stamp.
