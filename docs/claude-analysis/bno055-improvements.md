# BNO055 — Improvements Available Without Changing the Part

**Date:** 2026-09-20
**Companion to:** [`balance-loop-findings.md`](balance-loop-findings.md)
**Applies to:** `rocko_env/rocko_env/hardware/sensors/icm20948/` (the directory and classes are named
for an ICM20948; the part actually fitted is a BNO055 — see [Naming](#naming) below)

---

## Summary

Four changes to how the BNO055 is configured and read. None require new hardware, and together they
address the sensor-side contribution to the balancing problem.

| # | Change | Effort | Expected benefit |
| --- | --- | --- | --- |
| 1 | Use gyro rate for the derivative term instead of differentiating fused pitch | Small | **Large.** Removes filter delay from the D term |
| 2 | Move the BNO055 from I²C to UART | Small, plus one wire | **Large.** Eliminates a known, frequent failure mode |
| 3 | Switch fusion mode from NDOF to IMUPLUS | Trivial | Moderate. Removes magnetometer disturbance |
| 4 | Read the quaternion instead of Euler angles | Small | Minor, but strictly safer |

Ordering note: these are worth doing, but they are **not** the top of the overall list. The blocking
DDS/Python round-trip in the control loop (finding C-1) dominates everything here. A better sensor
signal sampled through a blocking service call is still a bad signal. Fix C-1 first, or fix it
alongside these — the UART rewrite is a natural time to move the read into C++ anyway.

---

## 1. Use gyro rate for the derivative term

### Current behaviour

`ICM20948.cpp` exports exactly three state interfaces — `yaw`, `pitch`, `roll` — all of them fused
Euler angles. The pitch PID's derivative term therefore has nothing to work with except the
numerical derivative of a fused angle.

### Why this is a problem

That fused angle has already been through Bosch's BSX fusion filter and arrives with a group delay
on the order of tens of milliseconds. Differentiating a delayed signal produces a *more* delayed
signal. A derivative term built on delayed feedback does not damp the system — past a certain lag it
actively destabilises it, which is the opposite of the term's purpose.

### What to do instead

The BNO055 reports **raw angular rate** from the gyroscope alongside its fusion output, at registers
`0x14`–`0x19` (`GYR_DATA_X/Y/Z`). This signal has essentially no filter delay — it is what the
sensor is measuring, not an inference from it.

- Export pitch rate as an additional state interface.
- Feed it to the pitch controller as the measured derivative, rather than letting the controller
  differentiate the angle.

Using measured angular rate for the D term is standard practice for balancing robots, and is likely
the single largest control improvement available without changing any hardware.

### Implementation notes

- **Which gyro axis corresponds to robot pitch depends on how the board is mounted.** Determine this
  empirically rather than assuming; tilt the robot by hand and watch which axis moves.
- The BNO055 has axis remap registers (`AXIS_MAP_CONFIG` `0x41`, `AXIS_MAP_SIGN` `0x42`) that can
  align the sensor frame to the robot frame on the sensor itself. Doing the remap there keeps the
  sign conventions in one place instead of scattered through the control code.
- Check the sign convention against the existing pitch angle. A D term with an inverted sign is
  worse than no D term.

---

## 2. Move from I²C to UART

### This is a known hardware incompatibility, not bad luck

The `try`/`except` in `ICM20948.py:36-41` — finding C-2, the one that reports `pitch = 0.0` on
failure — may be catching a documented hardware incompatibility rather than a rare fluke.

Adafruit's BNO055 guide for the Raspberry Pi states plainly:

> "the BNO055's use of I2C clock stretching will cause problems with a hardware I2C clock stretching
> bug in the Raspberry Pi"

and recommends:

> "To work around this clock stretching issue you can instead connect to the BNO055 using its serial
> UART mode."

Adafruit's CircuitPython-on-Linux guide lists the BNO055 as one of three parts (with the BNO085 and
CCS811) that require clock-stretching support the Pi's hardware I²C does not properly provide.

Sources:
- [BNO055 Absolute Orientation Sensor with Raspberry Pi & BeagleBone Black — Hardware](https://learn.adafruit.com/bno055-absolute-orientation-sensor-with-raspberry-pi-and-beaglebone-black/hardware)
- [CircuitPython Libraries on Linux and Raspberry Pi — I2C Clock Stretching](https://learn.adafruit.com/circuitpython-on-raspberrypi-linux/i2c-clock-stretching)
- [Raspberry Pi I2C Clock Stretching Fixes](https://learn.adafruit.com/raspberry-pi-i2c-clock-stretching-fixes/example-issue)

### 2.1 Does this apply to the Pi 4?

Unresolved, and worth stating precisely rather than assuming either way.

- The **BCM2711 datasheet claims the bug is fixed**: "The BSC controller in the BCM2711 fixes the
  clock-stretching bug that was present in BCM283x devices."
- **Users report it is not.** [raspberrypi/linux issue #4884](https://github.com/raspberrypi/linux/issues/4884)
  documents intermittent failures on a Pi 4 with a clock-stretching device, and a
  [Raspberry Pi forum thread](https://forums.raspberrypi.com/viewtopic.php?t=329543) includes
  oscilloscope captures showing "the old bug in the clock width when a stretch occurs, leaving it
  smaller than it should many times" on a Pi 4 at 100 kHz.
- **Raspberry Pi engineers engaged but did not resolve it**, noting only that it "isn't a
  straightforward problem to reproduce." The issue remains open with no fix and no denial.
- Reducing the baud rate helps but does not eliminate it — issue #4884 reports that at 10 kHz
  "errors still occur occasionally."
- Adafruit's guidance is current (it references the Bookworm-era `/boot/firmware/config.txt` path)
  and still lists the BNO055 as requiring the workaround, with no carve-out for the Pi 4 or 5.
- One secondary source claims the fix applies only to the *additional* BSC blocks on the Pi 4 and
  not to the original buses on GPIO 2/3. **This is uncorroborated** in the primary sources above;
  treat it as a lead to test, not a fact.
- The **Pi 5** routes I/O through the RP1 southbridge, a different I²C controller entirely, so it
  is plausibly unaffected. Unverified.

**Therefore: measure it before assuming.** See § 2.2 for how.

If it measures clean on ROCKO's actual Pi, this drops down the priority list — but UART is still the
better destination, because the read is being rewritten for finding C-1 anyway, and moving to UART
removes a contested variable rather than betting on it.

### 2.2 How to tell whether this is happening to you

Two distinct failure signatures, and counting only the first will undercount:

1. **Failed reads** — add a counter to the `except` block and log the rate. Run the sensor alone for
   30 minutes and record failures per minute.
2. **Silently corrupted reads** — these throw no exception. Log pitch to a bag with the robot held
   perfectly still and look for discontinuities. A one-byte corruption in the Euler high byte turns
   a 2° reading into something on the order of ±100°, which is obvious in a plot and invisible in a
   log.

Supporting checks:

```bash
cat /proc/device-tree/model                   # which Pi is actually fitted
dmesg | grep -i i2c                           # kernel-level transfer errors
grep -i baudrate /boot/firmware/config.txt    # current bus speed (default 100 kHz)
```

### The usual I²C workaround is not viable for this robot

The standard mitigation is to slow the bus down in `/boot/firmware/config.txt`:

```
dtparam=i2c_arm_baudrate=10000
```

with the guide suggesting 5 kHz or 1 kHz if 10 kHz is not enough.

**Do not do this here.** At 10 kHz, a six-byte burst read plus addressing and restart overhead is
roughly 90 bits on the wire, which is about **9 ms** — essentially the entire 10 ms control period,
for one sensor read. At 1 kHz it is ten times worse. The workaround that makes I²C reliable is the
same workaround that makes it far too slow for a 100 Hz control loop.

This is why UART is not merely the cleaner option for ROCKO; it is the only viable one short of
changing the part.

### How to do it

1. **Tie the `PS1` pin to 3.3 V.** Per the Adafruit guide: "You'll need to connect the PS1 pin from
   the top row to 3.3V power to put the BNO055 into UART mode."
2. **Rewire.** "Once in UART mode the BNO055's SCL and SDA pins will become serial RX and TX pins."
3. **Free the Pi's serial port** — disable the kernel console on the UART, and on a Pi with
   Bluetooth be aware the good PL011 UART is claimed by the Bluetooth modem by default.
4. **Implement the BNO055 UART protocol.** It is a small register read/write protocol framed with a
   `0xAA` start byte; straightforward to implement directly in C++ with `termios`, which is where
   the read should end up anyway once C-1 is addressed.

### Budget

At 115200 baud, a register read of the Euler or quaternion block is roughly 12 bytes each way — on
the order of 1 ms. That is comfortably inside a 10 ms period but is not free, so:

- Do **one** block read per cycle, not one per axis (see finding H-3).
- Use a bounded read with a timeout. Never let a serial read block the control loop indefinitely —
  that would reintroduce C-1 through a different door.

---

## 3. Switch fusion mode from NDOF to IMUPLUS

### Why

NDOF is the BNO055's 9-axis mode and includes the magnetometer. ROCKO needs **pitch**, not heading.
The magnetometer contributes nothing to balancing and introduces a disturbance path: fusion output
is perturbed by nearby ferrous metal.

The deployment environment is a wedding venue — structural steel, aisle stanchions, metal table
legs, and a floor whose contents are unknown until the day. This is a disturbance that cannot be
tested for in advance and does not need to be accepted at all.

### Secondary benefit: much easier calibration

In NDOF the magnetometer must be calibrated by the figure-eight motion, and it drifts out of
calibration. IMUPLUS needs only gyroscope and accelerometer calibration — the gyro calibrates by
sitting still, the accelerometer by being held in a handful of orientations. Both are repeatable and
can be done in the venue parking lot if needed.

### How

Set the operating mode to `IMUPLUS` after initialisation (`adafruit_bno055.IMUPLUS_MODE` in the
CircuitPython driver; `OPR_MODE` register `0x3D` when talking to the part directly).

Note that heading becomes relative rather than absolute. This does not matter — nothing in the
balance loop consumes yaw.

### Also: persist the calibration offsets

The BNO055 exposes its calibration offsets and will accept them back at startup. Save a known-good
set and restore it on boot so the robot does not need to re-converge before it can be trusted. Pair
this with a preflight check that reads the calibration status registers and refuses to arm until
gyro and accelerometer both report calibrated.

---

## 4. Read the quaternion instead of Euler angles

The current code reads `.euler`, which has two problems: the driver performs a separate I²C
transaction per property access (finding H-3), and Euler output has wraparound discontinuities.

Read the quaternion block (registers `0x20`–`0x27`, 8 bytes) in a single transaction and derive
pitch from it. This gives a coherent single-instant sample and no discontinuity.

Near zero pitch the discontinuity rarely bites, so this is the smallest of the four changes — but it
costs nothing extra once the read is being rewritten for UART anyway.

---

## Related: the hardcoded pitch offset

`ICM20948.py:31` contains:

```python
self.pitch_offset = 7.6
```

This is a mounting-tilt correction — the sensor board is not perfectly aligned with the robot's
balance axis. Two problems with it as written:

1. It is a magic constant in source, so re-determining it means editing code.
2. It will silently become wrong if the IMU is ever remounted, the battery moves, or mass is added
   to the chassis — all of which are likely between now and the wedding.

Make it a ROS parameter, and add a small routine that determines it: hold the robot at its true
balance point, average the reported pitch over a few seconds, and store the result. The correct
value is the pitch at which the robot's centre of mass is over the axle, which is a property of the
assembled robot, not of the sensor.

---

## Naming

The class, file, directory, service type (`Icm20948Data`), pluginlib export, and the
`rocko.ros2_control.xacro` plugin reference all say **ICM20948**. The part fitted is a **BNO055** —
`ICM20948.py:29` instantiates `adafruit_bno055.BNO055_I2C`, and the git history records the change
("changed to bno055"). The `adafruit_icm20x`, `numpy`, `ahrs.filters.Madgwick`, `acc2q` and
`Quaternion` imports are all dead.

Rename while the file is being rewritten anyway. Anyone reading the xacro to find out what sensor is
on the robot is currently being told the wrong answer.

---

## When to replace the part instead

The changes above fix what is fixable in software and wiring. What they cannot fix:

- **Fusion latency.** The BSX fusion filter is tuned for general orientation tracking, not for
  closing a control loop. Its delay is not specified for that purpose and cannot be adjusted.
  Improvement 1 sidesteps this for the derivative term — which is where it hurts most — but the
  pitch *angle* driving the proportional term still carries it.
- **The 100 Hz fusion ceiling.** The BNO055's fusion output updates at 100 Hz, so no control rate
  above 100 Hz can see new information.

If those become the binding constraint, the move is to a raw-output IMU over **SPI** — candidates
worth evaluating are the **ICM-42688-P** (low noise, current default in drone and robotics work),
the **BMI088** (designed for vibration-heavy robotics), or the **LSM6DSV**. SPI is deterministic,
megahertz-class, and has no clock stretching, which removes this entire class of problem rather than
working around it.

The trade is that none of those parts do fusion for you. A pitch-only complementary filter blending
gyro integration against the accelerometer's gravity vector is short, but it is code that has to be
written, tuned and tested — and the git history suggests an ICM20948 was already tried and abandoned
in favour of the BNO055's built-in fusion. Understand why that happened before repeating it.

**Recommendation:** do all four changes above first. They are cheap and they are prerequisites for
judging whether the part itself is the limit. Reassess only once C-1 is fixed and the loop period
and feedback age have been measured.
