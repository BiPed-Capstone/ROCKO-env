# Find why checks C3 and FF are flaky, and make them judge what the node did, not when the checker ran

- **Date / commit:** 2026-11-03, built on `f05e18a`. The change is in the commit that adds this file.
- **Branch:** `ai-forward`. Local commit only: this run was told not to push.
- **Environment:** `rocko-env-dev` container (`ros:jazzy`, arm64, Docker Desktop on macOS, 14 CPUs,
  7.6 GiB, no CPU quota, `cap_sys_nice` not granted). `./check_container_mount.sh` printed
  `ok    the container sees this checkout` at the start. This run was the only user of the container.

**Everything here is about the model and the checks around it.** Nothing in this entry says anything
about the real robot, its watchdog timing on the Pi, or whether 0.2 s is a good limit.

## Goal and success criteria

`verify_sim.sh C3` (the teleop watchdog, finding C-3) passed 2 of 4 times on 2026-11-03. `FF` (the
feedforward age limit, finding H-11, added in `71095e0`) printed notes about delivery gaps above 200
ms. A flaky verifier is a broken sensor (CLAUDE.md §4). Goal: find out why, and fix it without
loosening what the checks prove.

1. C3 run 10+ times as it was, every failure's message recorded.
2. The cause named with evidence; three candidates tested: the fake gamepad (or the checker) being
   starved, today's H-11 change, or a real race in the watchdog.
3. If the cause is in the harness: C3 still goes red on its deliberate breaks, and green 20 of 20.
4. The same for FF (10 runs as it was; fixed if the cause is shared).

## What was found

### The cause: the checking process stalls for about a third of a second

Not the node under test. Three kinds of evidence:

- **C3's own failure, read closely.** C3 as it was failed 1 of 10 (run 7). Its numbers: 40 commands
  took 2.25 s instead of 1.95 s. `DiffDriveController.py`'s log shows it received command 39 on time,
  logged `teleop command timed out` 0.200 s later (correct), and `teleop commands resumed` when
  command 40 arrived 0.35 s after command 39. Meanwhile the checker recorded the node's `True`, sent
  at `...39.378`, as *arriving* at `...39.520`, the moment it sent command 40: the checker had done
  nothing for 0.35 s. Its verdict, `command_timed_out went True 0.000 s after the last command`, was
  the timeout from the gap *the checker made*, timed with arrival stamps the stall had bunched up.
  The watchdog was right; the check was wrong. The two failures on record from earlier
  (stage 5 journal) have the same signatures (`True` in the middle of the drive, and `True 0.003 s
  after the last command` with the drive 0.2 s long).
- **A stall injected on purpose reproduces it exactly.** In a scratch copy of the old check, a 0.35 s
  `sleep` before command 40 gave `FAIL: command_timed_out went True 0.001 s after the last command`,
  the same message as run 7.
- **Three signatures, one cause.** The checker's stall shows as (a) `True` in the middle of the drive
  (a command sent late), (b) `True` about 0 s after the last command (command 40 sent late), or (c)
  `True` too late (the flag read late). In every failing run whose log was kept, the node's own log
  stamps show the watchdog firing 0.20 s after the last command it received.
- **The stalls are measurable, and in more than one process.** FF's new listener records the DDS
  source timestamp of every feedforward message. In 40 runs of FF, a `ros2 topic pub` publisher
  stopped sending for 0.327 to 0.342 s in 18 of them (only gaps after the listener had discovered
  the publisher can be seen). In 5 of those 18, both publishers, two separate processes, stopped
  within 3 ms of each other for the same length. One C3 run (a deliberate-break run) caught a
  natural 0.372 s gap between two of the checker's own commands.

What the stall is **not**, measured:

- **Not Python garbage collection** in C3: instrumented runs (22) saw at most 3.9 ms of GC during the
  drive. A long soak (240 s) did see 63 to 109 ms GC pauses, but only with 25,000+ messages held in
  memory; C3 holds about 500.
- **Not CPU starvation of the container**: a plain `time.sleep(0.01)` loop (no ROS) ran 240 s
  beside a running launch and never overslept by more than 36 ms; the cgroup reports `nr_throttled 0`.
- **Not leftover processes** in these runs: `ps` in the container showed only the ROS daemon before
  each batch. (One trap found: stopping a host-side `docker compose exec` does **not** stop the
  `verify_sim.sh` it started inside the container; it runs on, up to a minute, and a new run started
  meanwhile exits 2 because a controller manager is up. Waited each time until it was gone.)
- **Not H-11.** C3 as it was, run 10 times against a scratch build with `kMaxFeedforwardAgeNs` set to
  1000 s (the age limit effectively off), still failed once, with a third signature of the same stall
  (the flag read 0.18 s late; the node's log shows the time-out on time). See the table. H-11 cannot touch what C3 judges in any case:
  the age limit lives in `AddFeedforwardController`, and C3 judges `DiffDriveController.py`'s flag and
  the velocity reference it publishes.
- **Not a race in the watchdog**: the arrival time and the expiry check run in the same
  single-threaded executor of `DiffDriveController.py`, and in every failure its own log showed
  correct timing.

Not established: **what** stops two independent Python ROS processes at the same moment for about
333 ms. Since a ROS-free loop never stalled, the cause is somewhere in the ROS 2 / Fast DDS stack in
this container (discovery or the shared-memory transport are guesses, not tested). Whether a Pi shows
it is unknown. On the robot, `DiffDriveController.py` publishes the feedforward from Python too, so a
stall like this there would make the feedforward age out for a moment: the limit doing its job, but
worth knowing (see *Not verified*).

### The fix: judge send times, and judge every gap

No retry, no wider window. Both checks now take every time from when a message was **sent** (its
DDS source timestamp, the system clock, the same clock as ROS log stamps), never from when this
process got round to reading it.

- **C3** (`sim/scenario_checks.py`, `check_teleop`): the checker listens to its own gamepad topic to
  learn when each command really left. A stall can still hold back one of its commands; then the gap
  is real, and the watchdog **should** fire in it. So the rule is now judged on every silence: a
  `True` may never come sooner than the timeout (less one 10 ms cycle) after the latest command, and
  every gap longer than timeout + margin must hold a `True` within [timeout, timeout + margin] of its
  start. The final silence is judged exactly as before. The old rule "no `True` while driving" is
  replaced by the first, which is stricter where it matters (it fails a watchdog that fires early at
  any point) and does not fail a watchdog that fired correctly in a gap the checker made.
- **FF** (`verify_sim.sh`, `check_feedforward_age`, and the new `scenario_checks.py
  feedforward-sends` listener): each wheel's time-out is timed from that wheel's **last message
  sent**, not from the kill. The earliest allowed time tightens from 0.1 s after the kill to 0.19 s
  after the last message (`FF_EARLIEST_S` is gone). And, because the controller logs at most one
  time-out line per second, the publishers now keep going (at most 5 s more, `FF_EXTEND_MAX_S`) until
  a second has passed with no time-out line, so a stall just before the kill cannot hide the line FF
  needs. Before that, 2 of 20 FF runs ended `COULD NOT JUDGE` for exactly that reason.

## What changed

| File | What and why |
| --- | --- |
| `sim/scenario_checks.py` | `Recorder` keeps each message's DDS source timestamp (`sent_s`). `check_teleop` uses send times throughout, hears its own commands back, and judges every gap (above). New subcommand `feedforward-sends` for FF |
| `verify_sim.sh` | FF: the listener, timing from the last message sent, the bounded extension before the kill, `FF_EXTEND_MAX_S` and `FF_LISTEN_S` added, `FF_EARLIEST_S` removed; the C3 and FF header comments and the closing summary line |
| `docs/architecture/verification-ladder.md` | The C3 and FF rows, the observation in the Tier 3 status row (cause found), the deliberate breaks |
| `docs/journal/2026-11-03-stage5-balance-in-sim.md` | Its open item "why C3 is flaky" points here |
| this file, and the journal index | |

## Verification

All Tier 3 runs below are `verify_sim.sh` in the container. "old" = the check as at `f05e18a`, run
from a scratch copy; "new" = this commit.

| What | Runs | Result |
| --- | --- | --- |
| C3 old | 10 | 9 pass, **1 fail** (run 7: `command_timed_out went True 0.000 s after the last command`, drive 2.25 s) |
| C3 old, instrumented copy (GC, CPU time, context switches; a stack sampler) | 22 | 22 pass; largest command gap 60.6 ms; GC at most 3.9 ms. No stall caught, so no stack |
| C3 old, against the H-11-off build (`kMaxFeedforwardAgeNs` = 1000 s, scratch workspace) | 10 | 9 pass, **1 fail** (run 5: `command_timed_out went True 0.383 s after the last command, outside [0.2, 0.3] s`, drive a normal 1.95 s). `DiffDriveController.py`'s own log puts that time-out 0.20 s after the last command: the checker read the flag 0.18 s late (a stall on its receiving side). So the old check is flaky with H-11 off too |
| C3 old, a 0.35 s stall injected before command 40 | 1 | `FAIL: ... went True 0.001 s after the last command` (the run-7 message) |
| C3 new, the same injection; also injected at command 21 (0.35 s and 0.25 s) | 3 | all `PASS`, each with the note that the script sent a command more than 0.3 s late; the watchdog timed out in each gap and was judged so |
| C3 new, deliberate break `command_timeout_s:=1000` | 1 | `FAIL: command_timed_out never went True in the 2.0 s after the last command`, and the bash check `never logged 'teleop command timed out'`; exit 1 |
| C3 new, deliberate break `command_timeout_s:=0.03` | 1 | `FAIL: command_timed_out went True only [0.0377, 0.0415, ...] s after a command, sooner than the 0.2 s timeout`; exit 1 |
| C3 new, deliberate break `command_timeout_s:=0.5` | 1 | `FAIL: the gamepad went silent for longer than 0.3 s at [0.106] s into the drive and command_timed_out did not go True within [0.2, 0.3] s of it` (a natural 0.372 s stall happened in this run; without it the final latency 0.506 s fails too); exit 1 |
| **C3 new** | **20 consecutive** | **20 pass**: timed out 0.203 to 0.214 s after the last command; largest command gap 0.071 s, so no natural stall happened in these 20 (the injections above are what show the stall case) |
| FF old | 10 | 10 pass; 2 printed a time-out while the publisher ran |
| FF new, before the extension | 20 | 18 pass, **2 `COULD NOT JUDGE`** (a 0.332 s send gap less than 1 s before the last message hid the final line); 0 fail |
| **FF new, with the extension** | **20 consecutive** | **20 pass**: time-out 0.201 to 0.213 s after each wheel's last message; 12 of 20 printed a mid-run time-out (send gaps 0.327 to 0.342 s where the listener saw one) |
| FF new, deliberate break: `kMaxFeedforwardAgeNs` = 50 ms (scratch build) | 1 | `FAIL: ... logged the time-out less than 0.2 s after the last message was sent` (0.057 s and 0.055 s), both wheels; exit 1 |
| FF new, deliberate break: limit 1000 s (scratch build) | 1 | `FAIL: ... never logged 'Feedforward on ... ms old' after its last message`, both wheels; exit 1 |
| **FF new, final code** (adds only the line that prints the extension) | **20 consecutive** | **20 pass**: 0.201 to 0.213 s after each wheel's last message. In run 7 a 0.332 s send gap came just before the planned kill: `kept publishing 1.0 s past FF_PUBLISH_S`, then a clean pass (the case that gave `COULD NOT JUDGE` before) |
| Tier 1 `build_ros.sh` | 1 | `Summary: 2 packages finished`, `Done!` (no C++ changed) |
| Tier 0 `sim/check_sim_config.py` (host) | 1 | all `ok`, exit 0 (it still finds `TELEOP_TIMEOUT_S` and `FF_AGE_S`) |
| Tier 0 `verify_urdf.sh`, hook tests, Tier 2 `verify_mock.sh` | | not run: no URDF, launch, C++, hook or mock file changed |
| The rest of `verify_sim.sh` (S1 S2a H4 C2 H6) and `scenarios` | | not run: they do not use the changed code, except that `Recorder` now also stores source timestamps (`tipover`, `imu-hold`, `pitch-rate` and the scenarios ignore them) |

Not compared: the pre-H-11 commit `387d6dc`. It was copied into the container but not built or run;
the H-11-off build is the direct test of the same question.

## Review rounds

None: this run was not asked to start reviewer agents. The diff waits for `explainability-reviewer`
(code) and `docs-reviewer` (docs). No §6 critical-path file changed, so `realtime-auditor` is not
required.

## Not verified — needs a human

- What causes the ~333 ms stalls of Python ROS processes in this container. If a Pi shows the same,
  `DiffDriveController.py`'s 100 Hz feedforward would age out for a moment now and then (each time
  logged once, the controller keeps balancing on the reference). Worth a look on the robot: run the
  robot's normal launch tethered, drive gently for a minute, and count `Feedforward on ... ms old`
  lines in the log; any at all means a stall of 200 ms or more somewhere between the laptop's
  gamepad node and the Pi's controllers.
- C3's judging of a mid-drive gap was shown only with injected stalls (and one natural stall in a
  break run); none of the 20 green runs happened to contain one.
- `FF_EXTEND_MAX_S` (5 s) is a guess at "longer than any stall cluster seen"; the extension was seen to fire once (1.0 s) and never to run out.
- If a publisher's very last message reaches the listener but not the controller, FF would call a
  correct limit too short. Not seen; the transport is reliable and local.

## Reproduce

From the repo root on the laptop:

```bash
docker compose -f docker/compose.yml up -d
./check_container_mount.sh
docker compose -f docker/compose.yml exec -T ros2 bash -c 'cd /ros2_ws && src/ROCKO-env/build_ros.sh'
for i in $(seq 1 20); do docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/verify_sim.sh C3 | tail -1; done
for i in $(seq 1 20); do docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/verify_sim.sh FF | tail -1; done
```

Wait for each `verify_sim.sh` to finish before starting another (if you interrupt one, the copy
inside the container keeps running for up to a minute).

Stopped because: the goal's criteria are met or reported above; the reviewers were not part of this
run.
