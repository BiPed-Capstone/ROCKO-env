# How the IMU is read: one bounded I2C burst inside the plugin

**What this page covers:** how the `ICM20948` hardware plugin gets pitch, roll, yaw and pitch rate
from the BNO055 every control cycle, why it was changed on 2026-11-24, and what is still unverified.
(The part fitted is a BNO055; the `ICM20948` name is historical, see `CLAUDE.md` section 10.)

## The short version, for a student

The balance loop runs every 10 ms. Each cycle, `controller_manager` calls the IMU plugin's `read()`.
Until 2026-11-24 `read()` sent a ROS service request to a Python program (`ICM20948.py`) and **waited
with no time limit** for the answer. That is finding
[C-1](../claude-analysis/balance-loop-findings.md#c-1--unbounded-blocking-service-calls-inside-the-real-time-loop):
a late answer delays every controller, by a different amount each cycle, and a dead Python program
freezes the loop with the motors still on their last command.

Now `read()` talks to the sensor itself. It asks the BNO055 for 12 bytes (registers `0x14` to
`0x1F`: the three gyro rates, then heading, roll and pitch) in **one** I2C transaction through the
Linux device `/dev/i2c-1`. The kernel gives up on that transaction after **10 ms** (a timeout this
plugin sets). So `read()` always returns: with a fresh sample (normally about 1.5 ms of bus time at
the Pi's default 100 kHz, *not measured on the Pi*), or with a failed read that the fault tracker
counts.

## The path, step by step

| When | What happens | Where |
| --- | --- | --- |
| `on_init` | Reads the hardware parameters from the xacro: `i2c_device`, `i2c_address`, `pitch_offset_deg`, `pitch_rate_axis`, `pitch_rate_sign`. A bad axis or sign refuses to start. | `ICM20948.cpp` |
| `on_configure` (about 1 s, never in the loop) | Opens the bus; sets the I2C timeout (10 ms) and no retries; checks the chip id (`0xA0`); then starts the sensor the way Adafruit's Python driver did: CONFIG mode, reset, normal power, page 0, NDOF fusion mode. Then waits up to 3 s for two believable samples in a row that agree, because a freshly started BNO055 returns all zeros for a while. No sample means `ERROR`: the controllers never start on NaN. | `ICM20948.cpp` |
| `read()`, every cycle | One 12-byte burst read. The bytes are decoded (16 counts per degree and per deg/s) and judged by `check_imu_snapshot()`. | `ICM20948.cpp`, `ImuSnapshotCheck.hpp`, decoding in `Bno055UartProtocol.hpp` (the register layout is the same on I2C and UART) |
| A good sample | pitch = sensor pitch + `pitch_offset_deg`; pitch rate = `pitch_rate_sign` x gyro[`pitch_rate_axis`]. All four values are stored for the controllers. | `ImuSnapshotCheck.hpp` |
| A failed read | A bus error or timeout, or a sample the check rejects (see below). **All four values hold their last good numbers** (never 0.0, finding C-2). The fault tracker counts it; `read()` returns `ERROR` after 5 failures in a row or 10 of the last 20. | `ICM20948.cpp`, `ImuFaultTracker.hpp` |
| `on_cleanup` / `on_shutdown` | Closes the bus. | `ICM20948.cpp` |

### What the check rejects

`check_imu_snapshot()` rejects the **whole** sample, angles and rate together, if:

- heading, roll and pitch are all exactly equal: what a starting or reset sensor returns (all
  zeros). The gyro is left out of this rule, because a still gyro really does read 0, 0, 0;
- an angle is not finite, |pitch| (offset included) or |roll| is above 180 deg, or |heading| is
  above 360 deg;
- the pitch rate is not finite or is above 2000 deg/s, the gyro's full scale.
and `read_snapshot()` then also rejects it, through `pitch_step_plausible()`, if:

- **pitch jumped further than the gyro allows** since the last good sample: more than
  (|last good rate| + ½ x 20 000 deg/s² x elapsed time) x elapsed time + 5 deg (5 deg PROPOSED): the furthest pitch can move if the rate grew at the acceleration limit below. I2C has no checksum, and a flipped high
  byte moves pitch by 16, 32 or 64 deg while staying inside the limits above;
- **the rate itself jumped** more than 20 000 deg/s² x elapsed time + 50 deg/s (PROPOSED, generous):
  the same corruption in a rate byte. A flip of 256 deg/s or more in one cycle is caught, smaller ones
  are not.

The check is skipped after a gap of more than 0.1 s (by then the fault tracker has already stopped
the controllers), and `on_configure` only succeeds after **two** good samples in a row, the second
checked against the first, so one corrupted sample cannot become the reference. The first sample
after a long gap is accepted unchecked; if it was corrupted, the following real samples are rejected
until the gap passes 0.1 s again, which stops the controllers (it fails toward stopping).

Why all four together: they come from the same 12 bytes. If the rate is garbage, the transaction
is suspect, so its angles are not trusted either. Before this change a bad rate was only held and
logged, and never counted as a fault. That gap had to be closed before any controller uses
`pitch_rate` (finding H-6).

### Where the numbers live

| Number | Value | Home |
| --- | --- | --- |
| Pitch offset | 7.6 deg (added); where it came from is not recorded | `rocko.ros2_control.xacro`, `pitch_offset_deg` |
| Pitch-rate axis and sign | gyro X, sign -1, measured 2026-11-23 ([journal](../journal/2026-11-23-gyro-check.md)) | `rocko.ros2_control.xacro` |
| I2C bus and address | `/dev/i2c-1`, `0x28` | `rocko.ros2_control.xacro` |
| Pitch-jump slack; rate-step limit | 5 deg; 20 000 deg/s² + 50 deg/s (PROPOSED) | `kMaxPitchJumpSlackDeg`, `kMaxPitchAccelDegS2`, `kMaxRateStepSlackDegS` in `ImuSnapshotCheck.hpp` |
| I2C timeout | 10 ms (PROPOSED) | `kI2cTimeoutMs` in `ICM20948.hpp` |
| Fault limits | 5 in a row, 10 of 20 (PROPOSED) | `ICM20948.hpp`, `ImuFaultTracker.hpp` |

## Why I2C and not UART

[Proposal 0010](../proposals/0010-c1-imu-uart-and-encoders-in-cpp.md) planned to move the sensor to
UART, because the Pi's I2C hardware handles the BNO055's clock stretching badly. The owner chose
I2C for this step (2026-11-24): it needs no rewiring, and the same bus already worked from Python on
this robot, which read through the same kernel driver. If the bus turns out to be unreliable from
C++ (the failure count in the log keeps climbing, or the fault tracker trips), UART is the next
step. A corrupted byte that still looks plausible is caught only by the pitch-jump check above:
small corruptions (under about 5 deg) are not caught at all. The UART framing code is already written and tested (`Bno055UartProtocol.hpp`).

## What is NOT verified

- **Nothing here has run on the robot.** The container builds the plugin (Tier 1), but it has no
  `/dev/i2c-1` and no BNO055. The mock (Tier 2) replaces this plugin with a mock, and the
  simulation (Tier 3) replaces it with `SimPlant`.
- The register addresses and start-up values were written from memory of Adafruit's driver and the
  datasheet, not checked against the datasheet. The chip-id check fails loudly if the address is wrong;
  a wrong mode would show as all-zero samples, which the plugin rejects.
- That the Pi's I2C driver honours the 10 ms timeout, and **that it is a bound at all**. The
  2026-11-24 realtime audit, from memory of the kernel source: on a Pi 4 with a 100 Hz kernel tick
  the 10 ms is one jiffy and can fire anywhere from 0 to 10 ms (failing healthy reads at random); on a
  Pi 5 the bus driver may poll up to 20 ms for a busy bus first, so the worst case can pass 30 ms.
  The timeout is also set for the whole bus, not only for this device.
- How long the burst takes on the Pi was measured on 2026-11-24 ([loop timing](../journal/2026-11-24-loop-timing-on-the-pi.md)):
  the numbers are in that journal's table (in short: never near the 10 ms timeout, 0 failures). It is most of the
  control cycle, and its variation moves the moment the motors are written. `read()` keeps logging
  `IMU read timing: slowest ... us of the last 3000 reads, ... failed` every 30 s.
- The H-9 checks (a frozen sensor that repeats the same bytes, or reads `SYS_STATUS`) are not done.

The procedure for the first run on the robot is in the
[2026-11-24 journal entry](../journal/2026-11-24-c1-imu-i2c-and-zero-on-deactivate.md#robot-procedure).
