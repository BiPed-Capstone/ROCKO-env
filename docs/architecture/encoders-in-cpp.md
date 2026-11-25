# How the wheel encoders are read: edges counted inside the motor plugin

**What this page covers:** how each `Motor12VoltQuadEncoder` hardware plugin gets its wheel's position and speed
every control cycle, why it changed on 2026-11-24, and what is still unverified. The IMU's half of the same change
is [imu-i2c-read.md](imu-i2c-read.md).

## The short version, for a student

Each wheel has a **quadrature encoder**: two signal wires, A and B, that switch on and off as the motor shaft turns, a
quarter of a cycle apart. Every switch of either wire is one **count**; which wire switched first says which way the
wheel turned. Counting them gives the wheel's position; position change over time gives its speed.

Until 2026-11-24 a Python program (`QuadEncoders.py`) counted the switches, and every 10 ms each motor plugin's
`read()` asked it for the count over a ROS service **and waited with no time limit** for the answer. That is finding
[C-1](../claude-analysis/balance-loop-findings.md#c-1--unbounded-blocking-service-calls-inside-the-real-time-loop):
a late answer delays the whole control loop, by a different amount each cycle, and a dead Python program freezes the
loop with the motors on their last command.

Now the plugin counts the switches itself. WiringPi (the GPIO library the plugin already used for the motors) calls a
small function on every switch of A or B, on its own thread; that function adds +1 or -1 to a counter. `read()` just
looks at the counter. It never asks anyone and never waits.

## The path, step by step

| When | What happens | Where |
| --- | --- | --- |
| `on_init` | Reads the hardware parameters from the xacro: `encoder_pin_a`, `encoder_pin_b` (WiringPi pin numbers), `wheel_radius_m`, `encoder_counts_per_wheel_rev`. A missing or malformed one refuses to start. | `Motor12VoltQuadEncoder.cpp` |
| `on_configure` | Sets both pins as inputs, reads where the encoder is now, then asks WiringPi for an interrupt on **both edges** of each pin (`wiringPiISR2`). WiringPi starts one thread per pin (its source asks for real-time priority 55, which only a root process gets; not checked on the robot). If either interrupt cannot start, configure fails. | `Motor12VoltQuadEncoder.cpp` |
| Every switch of A or B | On WiringPi's thread: read both pins, look up the step (+1, -1, ±2 if a state was missed, 0) from the last state, add it to the counter. | `on_encoder_edge()`; the table is `quadrature_step()` in `WheelEncoder.hpp` |
| `read()`, every cycle | Load the counter, divide by counts per metre for the position, then position change over the time since the last `read()`, measured with `steady_clock` when the count is taken, through a low-pass filter for the speed. | `Motor12VoltQuadEncoder.cpp`, `WheelVelocityFilter` in `WheelEncoder.hpp` |
| `on_cleanup`, `on_shutdown`, destructor | Stop both interrupts. WiringPi cancels each thread and waits for it to end, so no callback can run on a plugin that is gone. | `stop_encoder_interrupts()` |

The counter keeps counting while the controllers are inactive, so the position does not jump when they come back.

### Two threads, one counter: why it is safe

- The counter is a `std::atomic<std::int64_t>`. The interrupt threads add to it; `read()` loads it. On the Pi's 64-bit
  ARM that needs no lock (a `static_assert` stops the build if it ever would), so `read()` can never be held up by an
  interrupt.
- An edge on A and an edge on B arrive on **different** WiringPi threads and could run at the same moment. Both read
  and update the decoder's last state, so they share a mutex. Only those two threads take it; `read()` never does.

### The same signal as before, on purpose

The controllers' gains, the simulation's encoder replica (`model_encoder_node`, [sim-plant.md](sim-plant.md)) and the
2026-11-23/24 robot measurements were all made with the Python node's signal. The plugin copies it exactly:

| | Python node (until 2026-11-24) | Plugin (now) |
| --- | --- | --- |
| Decoding | the `Encoder` library's table, both edges of both pins | the same table (`tools/wheel_encoder_test.cpp` compares all 16 cases) |
| Units | metres of travel, m/s, forward positive | the same |
| Scale | 145 x 1.355 counts per wheel turn, 0.06 m radius | the same, now hardware parameters in the xacro |
| Speed | position change over the real time since the last call, low-pass 0.05 | position change over the time measured in `read()` (`steady_clock`), low-pass 0.05 (`kWheelVelocityFilterAlpha`) |
| Right wheel | A and B swapped so forward counts up | the same swap, in the xacro |

### Where the numbers live

| Number | Value | Home |
| --- | --- | --- |
| Encoder pins (WiringPi) | left A 25, B 28 (BCM 26, 20); right A 27, B 24 (BCM 16, 19: swapped) | both real ros2_control xacros; `verify_urdf.sh` checks them |
| Wheel radius | 0.06 m (unmeasured; see the [next-steps handoff](../next-steps.md), item 1) | the xacros, and `DiffDriveController.py` has its own copy |
| Counts per wheel turn | 196.475 (145 x 1.355) | the xacros |
| Speed filter weight | 0.05 per cycle | `kWheelVelocityFilterAlpha` in `WheelEncoder.hpp` |

## What is NOT verified

- **The container does not build this plugin** (it needs WiringPi). It passes the compile-only check
  (`check_motor_plugin_compiles.sh`: syntax and types against stub headers); the decoding and the filter pass a host
  test. Whether it builds, starts and counts on the Pi is checked on the robot only; the results, when there are
  some, are in the [journal entry](../journal/2026-11-24-c1-encoders-in-cpp.md).
- **Missed edges at speed.** Each edge has to reach a user-space thread before the next one. The Python library had the
  same exposure through a slower path. How fast the wheel can turn before counts are lost is not measured.
- **A broken or unplugged encoder is not detected.** It stops counting and reads as a wheel standing still. Detecting
  it needs a rule such as "a large command but no counts", with a threshold nobody has measured
  ([proposal 0010](../proposals/0010-c1-imu-uart-and-encoders-in-cpp.md) section C; row 1.10 of
  [next-steps](../next-steps.md)).
- No pull-up or pull-down is set on the encoder pins, as the Python library set none. That the encoder's outputs drive
  the lines on their own is assumed from the fact that the Python node counted correctly.
