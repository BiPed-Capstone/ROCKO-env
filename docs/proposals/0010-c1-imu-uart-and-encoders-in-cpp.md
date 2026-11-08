# Proposal 0010 — Read the IMU over UART and the encoders in C++, inside the plugins (finding C-1)

- **Status:** Open
- **Gate:** on-robot step (a wiring change, a Pi build, a bench test and a tethered bring-up). The code
  itself is not gated, but none of it can be checked without the hardware, so the person asked for it
  to wait here ("land only what the compile-only check / the ladder can cover; anything else becomes
  a proposal", 2026-11-07).
- **Raised:** 2026-11-07, built on `4374d5c`, run journal:
  [C-1 UART framing helper](../journal/2026-11-07-c1-bno055-uart-framing.md).

## In one paragraph, for a student

Every 10 ms the controller asks the IMU and the encoders for a reading. Today it asks by calling a
Python program over ROS 2 (a "service call") and **waits with no time limit** for the answer
([C-1](../claude-analysis/balance-loop-findings.md#c-1--unbounded-blocking-service-calls-inside-the-real-time-loop)).
The answer arrives late, and late by a different amount every cycle; if the Python program stops, the
controller stops with it and the motors keep their last command. A balancing robot cannot tolerate
that: feedback delay eats phase margin, and a delay that changes every cycle makes the robot a
different plant from one cycle to the next. The fix
([ADR 0004](../decisions/0004-device-io-in-cpp.md), Proposed) is to read the devices **directly from
the C++ plugin**, with a hard time limit, and treat a reading that misses the limit as a failed read.

## What has already landed, and what it covers

| Landed (2026-11-07) | What it covers | What it does NOT cover |
| --- | --- | --- |
| `rocko_env/rocko_env/hardware/sensors/icm20948/include/rocko_env/Bno055UartProtocol.hpp`, header-only, standard library only, no I/O, no allocation. **No plugin includes it.** | Building a read request (`0xAA 0x01 <reg> <len>`) and a write request; checking a reply (`0xBB <len> <data>`, or `0xEE <status>`) with an explicit result: ok, device error (with the status byte), too short, bad header, length mismatch, too long; decoding the 12 bytes of registers `0x14`–`0x1F` into gyro X/Y/Z (deg/s) and heading/roll/pitch (deg) as **one** snapshot; all six values NaN on any failure | Whether the frame layout, status codes, registers and scale factors match the real BNO055 (each is marked UNVERIFIED in the header: written from memory of the datasheet, which was not open); timing; termios; the serial port; the sensor's reply delay; which axis is the robot's pitch, its sign, the offset |
| `tools/bno055_uart_protocol_test.cpp`, a host test (not a tier) | That the header does what its comments say: 11 test functions, 388 checks; red on 5 deliberate breaks (wrong scale, swapped bytes, header check removed, no two's complement, an oversized frame accepted) | Anything about the device or the robot |

The plugins (`ICM20948.cpp`, `Motor12VoltQuadEncoder.cpp`, `QuadEncoder.cpp`) are **unchanged**.
Nothing that runs on the robot changed.

## The change (proposed, not written)

### A. Wiring (a human does it; every fact here is UNVERIFIED unless the repo already states it)

1. **Put the BNO055 in UART mode.** The repo's source (Adafruit's guide, quoted in
   [bno055-improvements.md §2](../claude-analysis/bno055-improvements.md#2-move-from-ic-to-uart)) says:
   tie `PS1` to 3.3 V; the `SCL` and `SDA` pins then become the serial pins. *UNVERIFIED:* which of the
   two is the sensor's TX and which its RX, and the state `PS0` must be in (low, on the breakout's
   default, as far as the author knows). Check the breakout's own guide.
2. **Wire it to the Pi's UART.** *UNVERIFIED for this robot (not in the repo):* on a Raspberry Pi
   40-pin header the primary UART is GPIO 14 (TXD, physical pin 8) and GPIO 15 (RXD, physical pin 10).
   Sensor TX goes to Pi RX and sensor RX to Pi TX; common ground. Check those pins are free on ROCKO:
   the encoders use 26, 20, 19 and 16 (`rocko.launch.py`; whether those numbers are BCM or WiringPi
   numbering is not stated there) and the motor pins are in the xacro.
3. **Free the UART on the Pi.** *UNVERIFIED:* remove the serial console (`console=serial0,...` in
   `/boot/firmware/cmdline.txt`), set `enable_uart=1` in `/boot/firmware/config.txt`, and on a Pi with
   Bluetooth give the full UART (PL011) to the header with `dtoverlay=disable-bt` (or `miniuart-bt`),
   because the mini-UART's baud rate follows the core clock. The device is then `/dev/serial0`.
4. **Baud rate:** 115200, 8 data bits, no parity, 1 stop bit, the BNO055's default as far as the author
   knows (*UNVERIFIED*). One byte is 10 bits on the wire, 86.8 µs.

### B. The IMU read in `ICM20948.cpp` (the C++ to write once A is done)

- **`on_configure`** (not in the loop): open `/dev/serial0` with `O_RDWR | O_NOCTTY | O_NONBLOCK`;
  termios raw mode (`cfmakeraw`), `B115200`, `CS8`, `CLOCAL | CREAD`, no flow control, `VMIN = 0`,
  `VTIME = 0`. Then, with the header's `make_write_request` and `parse_write_reply`: page 0, operating
  mode CONFIG, set `UNIT_SEL` (`0x3B`) explicitly to deg/s and degrees (the header's scale factors
  assume that; *UNVERIFIED* register value), optionally an axis remap
  ([bno055-improvements.md §1](../claude-analysis/bno055-improvements.md#1-use-gyro-rate-for-the-derivative-term)),
  then the fusion mode (IMUPLUS, §3 of that page, or NDOF as today (*UNVERIFIED*: the repo never sets the mode, so the sensor is in whatever mode it powers up in or was left in)). Mode switches take milliseconds
  (*UNVERIFIED* how many): fine in configure, never in `read()`. A failed configure step returns
  `ERROR`.
- **`read()`**, once per cycle, the one transaction of [H-3](../claude-analysis/balance-loop-findings.md#h-3--imu-sample-is-three-separate-ic-reads):
  1. drop any stale input (`tcflush(fd, TCIFLUSH)`), so a late reply from the last cycle cannot be
     taken for this one;
  2. send the 4 bytes of `make_read_request(kBurstFirstRegister, kBurstLength, ...)`
     (`0xAA 0x01 0x14 0x0C`);
  3. collect bytes with `poll()` and non-blocking `::read()` into a fixed 14-byte member buffer until
     `reply_frame_size()` says the frame is complete, **or a deadline measured on `steady_clock`
     passes**. Deadline **3 ms (PROPOSED**, a named constant; a human confirms it with the bench
     test). Why 3 ms: the bytes alone take 4 × 86.8 + 14 × 86.8 ≈ 1.56 ms at 115200 baud, which leaves
     about 1.4 ms for the sensor to start answering, and 7 ms of the 10 ms budget for everything else;
  4. `parse_gyro_euler_burst_reply()`; anything but `kOk`, or the deadline, is **a failed read**:
     record it in the existing `ImuFaultTracker` (5 in a row, or 10 of the last 20), hold the last good
     values, log with `_THROTTLE`, and return `ERROR` while the tracker is tripped: the C-2 path, now
     reachable, because a dead sensor can no longer block `read()` forever;
  5. a good snapshot then goes through the plugin's existing plausibility limits (|pitch|, |roll|
     ≤ 180 deg, |yaw| ≤ 360 deg, |rate| ≤ 2000 deg/s), the pitch offset, and the chosen pitch-rate axis
     and sign, which move from `ICM20948.py`'s parameters to hardware parameters in the xacro.
- **Removed from the hot path:** the `rclcpp` client and node in the plugin, the `wait_for_service`
  loop in `on_init`, the `icm20948` node in the launch files, and later `Icm20948Data.srv` and
  `ICM20948.py`. The all-equal-Euler rule of `ICM20948.py` (a guess at what a reset sensor returns)
  moves into the plugin or is replaced by the H-9 check below.
- **The rejected alternative, stated because it is not obvious:** send the request at the end of one
  cycle and collect the reply at the start of the next. That never waits, but every sample is then
  about 10 ms old, which costs the balance loop more phase than a bounded 2–3 ms wait. Determinism
  beats average speed (`control-loop-change` §4): a wait that is bounded is acceptable; an unbounded
  one is C-1 again.

### C. The encoders in C++ (rows 1.3 and 1.10)

- Count edges in C++ with WiringPi's interrupt callback (`wiringPiISR`), which `Motor12VoltQuadEncoder`
  can use because it already links WiringPi; the callback runs on WiringPi's own thread (*UNVERIFIED*) and only
  updates a `std::atomic<std::int64_t>` count; `read()` loads that count (lock-free) and computes
  position and velocity from it with the real `period` argument. No service, no Python, no blocking.
- **L-6 pattern** (tracker, `ERROR`, throttled log): with interrupt counting a `read()` cannot fail the
  way a service call can; the failures left are a GPIO or ISR setup failure (return `ERROR` from
  `on_configure`) and an implausible value (a speed above the motor's maximum counts as a failed read
  in a tracker, held, `_THROTTLE` log, `ERROR` past the limit). A disconnected encoder that reads a
  steady zero while the motor is driven is H-9's twin; detecting it needs a rule (counts expected when
  the command is large) with a **proposed** threshold.
- **This changes the velocity signal the cascade sees, so it is not a drop-in.** Today one Python node
  answers both motor plugins with the **left** position and the average of both wheels' filtered
  velocities (low-pass `alpha = 0.05`), assuming a 10 ms period, and the two calls per cycle halve the
  measured speed (finding U-2, [sim-plant.md](../architecture/sim-plant.md), `model_encoder_node`). The
  velocity loop's gains were tuned against that signal. Before the C++ version runs on the robot: run
  the sim with `model_encoder_node:=false` (the plant already has the switch), re-tune or confirm the
  velocity loop there, and decide whether the filter stays (and where its constant lives). Gains are
  gated (proposal [0008](0008-candidate-gains-and-tethered-test.md) and the gains table in
  [README](README.md)).
- `QuadEncoder.cpp` is registered in `rocko_env_hardware.xml` but loaded by no xacro at `4374d5c`;
  delete it or give it the same treatment, not both kinds of encoder code.

### D. H-9 becomes writable

With the bytes in C++ the plugin can (a) treat N identical 12-byte bursts in a row as a frozen sensor
(the gyro's noise makes exact repeats unlikely on a robot that is balancing, *UNVERIFIED*; N **proposed**, checked on
the bench by leaving the robot still), and (b) read `SYS_STATUS`, `SYS_ERR` and `CALIB_STAT`
(*UNVERIFIED* addresses `0x39`, `0x3A`, `0x35`) and the operating mode **at configure time**, and not
in the 10 ms loop unless the bench test shows the time is there. A sensor found back in CONFIG mode
(a brownout reset) is a failed read. Row [1.8](../next-steps.md).

## Why

C-1 is ranked first in the findings' order of work; C-2's `ERROR` path, the H-4 tipover cutoff and the
fault tracker all depend on `read()` returning at all. H-3 (one snapshot) and H-6 (gyro rate with the
angle, not a second read) fall out of the one burst.

## Evidence

| Check | Tier | Result |
| --- | --- | --- |
| The framing helper and its host test | host test, not a tier | 388 of 388 checks pass on the laptop (Apple clang 21) and in the container (GCC 13.3, also with `-Wpedantic -Wconversion`); red on 5 of 5 deliberate breaks ([journal](../journal/2026-11-07-c1-bno055-uart-framing.md)) |
| The tree still builds | Tier 1 | `build_ros.sh` exit 0, 0 warnings; nothing includes the header, so this only shows nothing else broke |
| Anything in sections A–D | **none** | Not written, not compiled, not run. No tier here can check a serial port, a BNO055, a GPIO interrupt or the Pi's timing |

## Risk

- A wrong axis or sign for the pitch rate makes the D term push the wrong way (worse than none).
- A deadline too short for the sensor's real reply delay makes every read fail: the tracker trips and
  `read()` returns `ERROR` within 50 ms of activation. That fails safe (the controllers deactivate),
  but whether the motors stop then is finding H-7, unconfirmed on hardware.
- Wrong UART wiring or a console still on the UART: the sensor answers garbage or nothing; the same
  safe failure.
- The encoder velocity change (C above) changes the velocity loop's gain by about a factor of two.
  Running it with today's gains is the risky step: do it wheels-off first.
- `tcflush` and `poll` are syscalls inside `read()`. They are bounded, but how long they take on the Pi
  under load is unmeasured; the bench test measures it.

## What a human provides, and the tests

**Before any code is merged into a plugin**, on a bench, not on the robot:

1. A Pi with Ubuntu 24.04 and Jazzy, a BNO055 breakout wired as in A, and (ideally) a logic analyser or
   scope on TX and RX.
2. **Check the datasheet facts** marked UNVERIFIED in `Bno055UartProtocol.hpp` against the datasheet
   revision you have (frame bytes, status codes, register addresses `0x14`–`0x1F`, 16 LSB per deg/s and
   per deg, `UNIT_SEL`). Correct the header and its test if any differ.
3. **Bench measurement** (a small C++ or Python script on the Pi, not the robot code): send
   `AA 01 14 0C` 10 000 times at 100 Hz. Record: replies that are `BB 0C` plus 12 bytes, replies that are
   `EE xx` (which `xx`, how often), timeouts; the time from the last request byte to the first reply
   byte (median, 99th percentile, maximum); the whole transaction time. **Success:** no reply later than
   the proposed 3 ms deadline in 10 000; failures below 1 in 1 000. Otherwise report the numbers; the
   deadline or the baud rate changes before the plugin does.
4. **Axes and signs:** with the sensor held as it is mounted on ROCKO, tilt the top of the robot
   towards the direction it drives forward, slowly: which Euler field changes, and with which sign; tilt quickly: which
   gyro axis shows the rate, and with which sign. Record them; they become the plugin's parameters.
5. **Frozen and reset:** hold it still for 60 s and count identical 12-byte bursts in a row (the
   largest run decides N for H-9); briefly cut the sensor's 3.3 V and record what the replies look like.

**Then the plugin change** is written (the realtime rules of `CLAUDE.md` §6;
**`realtime-auditor` review is mandatory**, plus `ros2-control-reviewer` and `explainability-reviewer`),
built on the Pi (Tier 1 does not build the motor plugin), and Tier 2 run off-Pi for the wiring.
Measure `read()` on the Pi with a scoped timer (steady clock, kept in a fixed array, printed after the
run, not logged each cycle) or by toggling a spare GPIO around `read()` and scoping it: worst case over
10 minutes must be under 4 ms (PROPOSED).

**Tethered bring-up**, in this order; abort at any step on: a wheel turning when nothing commands it,
the wheels running away, `ERROR` from the IMU plugin more than once a minute, a buzz or heat.

1. **Wheels off the ground** (the robot on a stand, power cut within reach, no gamepad):
   `ros2 launch rocko_env rocko.launch.py`. Watch `ros2 control list_hardware_components` (both motor
   components and the IMU `active`) and `ros2 topic echo /dynamic_joint_states`: the pitch must follow
   a hand tilt with the sign found in step 4, and must not be NaN or a frozen number.
2. Unplug the IMU's TX wire for 1 s: the plugin must log the failure (throttled) and, past 50 ms,
   return `ERROR`; record what `controller_manager` did and **whether the wheels stopped** (H-7).
   Plug it back and relaunch.
3. Still wheels-off, tilt by hand at most 3 deg each way: the wheels must turn the way that rolls under
   the lean (proposal [0006](0006-velocity-loop-sign-check.md)'s sign check comes first).
4. Only then the tethered balance test of proposal [0008](0008-candidate-gains-and-tethered-test.md),
   with the velocity-loop question of C settled.

Record for every run: the commit SHA, the launch file and config, battery voltage, the surface, and
where the log and bag went ([reproducing results](../guides/reproducing-results.md)).

## Version and real-time checks

- No version pin changes: termios, `poll` and WiringPi's ISR are already available on Ubuntu 24.04;
  C++17 and `ros2_control` ≥ 4.0.0 stay ([ADR 0001](../decisions/0001-version-lock.md)).
- `ICM20948.cpp` and `Motor12VoltQuadEncoder.cpp` are on the critical path: `realtime-auditor` must
  review the change, and the motor plugin must be built on a Pi (the container leaves it out; only
  `check_motor_plugin_compiles.sh` type-checks it).
- No gain, limit or safety behaviour is changed by this proposal; the deadline, N for H-9 and the
  encoder plausibility limit are new **proposed** values a human confirms.

## Decision

(Human fills in: date, who, accepted/declined, why.)
