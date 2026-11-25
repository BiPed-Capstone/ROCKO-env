# Measure the control loop's timing on the Pi, and decide what may run during tuning

- **Date / commit:** 2026-11-24, built on `78b179d`; the tool and this entry are added by the next commit
- **Branch:** `ai-forward`
- **Environment:** the robot's Pi 5 (Ubuntu 24.04, kernel 6.8.0-1068-raspi, Jazzy), over `ssh rocko` with the owner's
  permission and the owner present; robot on the stand, **disabled** throughout

## Goal and success criteria

Item 4 of the [next-steps handoff](../next-steps.md): with C-1 fixed (no service waits left in any `read()`), measure the
100 Hz loop's timing on the Pi with the balance stack running but disabled, with and without the dashboard (row 4.3),
Foxglove, and recording (row 4.2), and `joint_state_broadcaster`'s share (row 4.15). Then decide what may run during a
tuning session.

Done means:

1. A repeatable tool turns controller_manager's own per-cycle statistics into median / 99th percentile / maximum of
   the cycle's read, update, write and total time and of each component's period, with a host test of its arithmetic.
2. Each configuration was measured for 60 s on the Pi, and the numbers are recorded here with how to reproduce them.
3. A recommendation for what runs during tuning, stated with what it rests on and what it cannot show.

## Plan

- `tools/loop_timing.py` (runs on the Pi, rclpy, subscribes to `/controller_manager/statistics/names` and `values`)
  and `tools/loop_timing_test.py` (host test of the summary maths).
- The supervisor launch in balance mode (`pi/launch.sh robot initial_mode:=balance`), so the cascade controllers stay
  **inactive** (a hand launch of `rocko.launch.py` would activate them). Five runs: minimal (no dashboard, no Foxglove),
  dashboard only, Foxglove only, the boot configuration (both), and the boot configuration while recording (started
  through the dashboard's `/api/record/start`). `joint_state_broadcaster` runs in all of them (the supervisor needs
  it); its cost is its own execution time in the statistics.
- Risk: none to the hardware (disabled, nothing commands the wheels). Limits: the cascade controllers are inactive,
  so their own `update()` time is not in these numbers; no Foxglove or browser client was connected.

## What changed

- `tools/loop_timing.py` (new): listens to controller_manager's statistics for `--seconds` and prints median, 99th
  percentile and maximum per quantity. `tools/loop_timing_test.py` (new): host test of its arithmetic (nearest-rank
  percentiles, NaN and empty input, rate to period); `tools/run_ladder.sh` picks it up by its name.
- `rocko_robot.launch.py`: the comment that called the dashboard's cost "still unmeasured" now points here.
- Docs: the next-steps handoff item 4 and rows 4.2, 4.3, 4.15; the "not known" section of the dashboard guide; the
  IMU page's "not verified" line about the burst's duration.

## Verification

| Tier | Command | Result |
| --- | --- | --- |
| host | `python3 -I tools/loop_timing_test.py` | 4/4 passed; **red** with the percentile rank off by one (`ceil(q*n)` in place of `ceil(q*n) - 1`) |
| 0 to 2 | `tools/run_ladder.sh --no-sim` | all PASS (Tier 0, Tier 1 with 0 compiler warnings, Tier 2 mock both modes and supervisor, every host test, hook tests, doc links) |
| 3 | not run | nothing the simulation runs changed (a tool, a launch comment, docs) |
| 4 (agent, owner present) | the matrix below on the Pi | five configurations measured |

## The measurement, 2026-11-24, on the robot

Commit `78b179d` on the Pi, with `tools/loop_timing.py` copied to `/tmp` before it was committed (the committed
version differs only in its messages and checks, from review: the same arithmetic). `pi/launch.sh robot initial_mode:=balance log_root:=/home/rocko/rocko_logs` plus the
arguments in the table; balance mode, **disabled** (the four cascade controllers inactive), wheels still on the stand,
no browser or Foxglove client connected, robot on the bench stand (no floor). Battery voltage not recorded. After
10 s of settling, 60 s of `pi/rocko_run.sh python3 tools/loop_timing.py --seconds 60 --label <name>` and, at the same
time, one 60 s whole-system `top` sample (`top -b -d 60 -n 2 -o %CPU`, the second frame). The recording run started and
stopped the recording through the dashboard on the Pi:
`curl -s -X POST -H 'Content-Type: application/json' -d '{"label":"loop-timing-test"}' http://127.0.0.1:8080/api/record/start`
and the same with `/api/record/stop` and `-d '{}'`. The raw outputs
were in `/tmp/lt/` on the Pi and were not kept; the numbers below are copied from them. The recording run left
`~/rocko_logs/20261124_233257_loop-timing-test` (62 MB) on the Pi.

Times in ms. "Cycle" is `controller_manager.total_time` (read + update + write of one cycle; the budget is 10 ms).
"IMU read" is the `Gyro` component's `read()`. "Write period" is the time between two `write()`s (the same for every
component and `joint_state_broadcaster`).

| Configuration | Cycles received of ~6000 | Cycle median / p99 / max | IMU read median / max | IMU read period max | `update` / `write` max | `joint_state_broadcaster` median / max | Write period p99 / max | CPU idle (4 cores) |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| minimal: `dashboard:=false foxglove:=false` | 6001 | 2.39 / 3.47 / 3.67 | 2.31 / 3.59 | 10.08 | 0.091 / 0.051 | 0.017 / 0.051 | 11.83 / 11.90 | 78 % |
| `dashboard:=true foxglove:=false` | 5960 | 2.34 / 3.48 / 3.66 | 2.26 / 3.57 | 10.06 | 0.097 / 0.051 | 0.017 / 0.056 | 11.84 / 11.90 | 71 % |
| `dashboard:=false foxglove:=true` | 5999 | 2.38 / 3.48 / 3.68 | 2.30 / 3.57 | 10.06 | 0.100 / 0.065 | 0.017 / 0.051 | 11.84 / 11.94 | 71 % |
| boot: `dashboard:=true foxglove:=true` | 5600 | 2.39 / 3.50 / 3.73 | 2.28 / 3.58 | 10.08 | 0.112 / 0.054 | 0.018 / 0.071 | 11.83 / 11.91 | 62 % |
| boot + recording (dashboard's Record) | 5901 | 2.28 / 3.52 / 3.71 | 2.18 / 3.59 | 10.07 | 0.110 / 0.063 | 0.018 / 0.063 | 11.87 / 11.94 | 56 % |

What the numbers say:

- **The cycle is IMU-bound and well inside its budget.** The IMU's 12-byte I2C burst is 95 % of every cycle (median
  2.2 to 2.3 ms, never above 3.6 ms in about 29 500 cycles, 0 failed reads); `update()` and `write()` together stay
  under 0.2 ms. The longest cycle seen was 3.73 ms of 10.
- **The dashboard, Foxglove and recording do not measurably change the loop.** Between the minimal and the busiest
  configuration the cycle's 99th percentile moved by 0.05 ms and its maximum by 0.06 ms, about the run-to-run noise.
  The reads stay on time: the IMU read's period never strayed more than 0.08 ms from 10 ms. They do cost CPU (system
  idle 78 % minimal, 71 % with either the dashboard or Foxglove, 62 % with both, 56 % recording; one `top` sample each,
  with the measuring tool itself running). Fewer statistics samples reached the tool in the busier runs (5600 in the
  boot run); the periods show no gaps, so these are samples lost between controller_manager and the tool, not missed
  cycles. Whether the topic or the tool dropped them is not known.
- **`joint_state_broadcaster` costs about 0.02 ms per cycle** (median; at most 0.07 ms) (row 4.15).
- **The jitter that matters is in the write, and the IMU is most of it.** The cycle starts on time, but `write()`
  happens after the IMU read, whose length varies from about 2.2 to 3.6 ms. The moment the motors get their new
  command wanders by up to about 1.9 ms from cycle to cycle (write period p99 11.8 ms, max 11.9 ms, against a steady
  10 ms read period). The IMU's spread explains about 1.4 ms of that; where the rest comes from is not known. For a 100 Hz balance loop that is a varying delay of up to a fifth of a cycle between measuring
  and acting; whether it matters is for the tuning data. Shortening it means a faster or shorter I2C transaction
  (400 kHz, or fewer bytes), each with its own risks (the BNO055's clock stretching); not proposed yet.

Not measured: the cost with the cascade controllers **active** (their `update()`s, from the hand-launch or a balance
test), with a browser or Foxglove client connected, with the wheels moving (the encoder interrupts), or on battery
voltage sag.

## Decision: what may run during tuning

The agent's recommendation, for the owner: **the boot configuration (dashboard and Foxglove on) and recording may run
during tuning.** None of them changed the loop's timing measurably here, and a recording is what makes a tuning run
reproducible (`record:=true`, [guide](../guides/record-a-run.md)). Re-check with `tools/loop_timing.py` on the first
enabled balance run (active controllers, a client connected). If the cycle's maximum then passes about 6 ms, turn off
what that run shows costs the most; this run could not rank the three by CPU (dashboard and Foxglove tied at 71 % idle).

## Review rounds

| Round | Reviewer | Findings | Fixed / declined (why) |
| --- | --- | --- | --- |
| 1 | explainability-reviewer | one error message for three failures (one of them silent), a copied owner expression, lambdas with side effects, quantities dropped silently, the tool's own load not stated, the rate flag's source, the median rule, a `noqa`, errors to stdout, a bare traceback without ROS | all fixed; a `report()` split for testing `main` declined (the three failure checks are simple, and the tool was rerun on the Pi after the fix) |
| 1 | docs-reviewer | no index row, a "Review rounds" reference with no section, placeholders, the commit the tool ran from, claims without a number in the table, the sample-loss cause and the Foxglove CPU ranking unsupported, "30 000", numbers copied into other pages, a missing caveat in the handoff, the `top` and recording commands | all fixed; the other pages now link to this table instead of repeating it |

## Not verified, and what is left for a human

The cascade controllers active (an enabled balance run), a client connected, the wheels moving. The owner decides
whether the recommendation above stands for tuning.

**Stopped because:** the measurement and the recommendation are done; nothing else in item 4 can be measured without
enabling balance mode.
