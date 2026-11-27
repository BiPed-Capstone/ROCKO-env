# Fit the screen, buzzer and battery monitor

**Goal:** wire the 3.5" touch screen, a passive buzzer and an INA226 battery monitor to the Pi, set the Pi up for them,
and check each one. How they work and why: [the robot extras](../architecture/robot-extras.md).

**Machine:** the robot's Raspberry Pi 5 (Ubuntu 24.04), over SSH, as your normal user. Every command is run from the
repo root, `~/ROCKO-env`. You need the robot powered and **disabled** for everything here; nothing in this guide
enables it.

**Status: written from datasheets and the code, not yet done on the robot (2026-11-27).** Step 2's config lines, the
panel's pin table and the touch directions are the parts most likely to need adjusting; each step says how to tell.
Each extra is optional: if one does not work, the robot still drives, and the dashboard says what is wrong.

## 1. The pins, and the conflict to fix first

The screen plugs onto header pins 1 to 26 and uses the Pi's SPI bus. **Today the two motor drivers use four of those
pins** (21 to 24), and the IMU uses pins 3 and 5 underneath the screen. So before the screen goes on, the four motor
wires move, and the screen is connected so that pins 3, 5, 15 and 16 stay reachable: with **jumper wires** to the pins
in the screen's table below, or through a **stacking header** (long pins that pass through). The motor drivers use
software PWM, which works on any GPIO, so moving them changes nothing else.

The pin numbers below: *physical* is the position on the header (1 to 40); *BCM* is the GPIO number Linux and the
config lines use; *WiringPi* is the number in the robot's xacro files (`gpio readall` on the Pi shows all three).

| What | Today: physical / BCM / WiringPi | With the screen: physical / BCM / WiringPi |
| --- | --- | --- |
| Left motor speed (PWM) | 23 / 11 / 14 | **29 / 5 / 21** |
| Left motor direction | 24 / 8 / 10 | **31 / 6 / 22** |
| Right motor speed (PWM) | 21 / 9 / 13 | **33 / 13 / 23** |
| Right motor direction | 22 / 25 / 6 | **40 / 21 / 29** |
| Left encoder A, B | 37, 38 / 26, 20 / 25, 28 | unchanged |
| Right encoder A, B | 36, 35 / 16, 19 / 27, 24 | unchanged |
| IMU (BNO055) SDA, SCL | 3, 5 / 2, 3 (I²C bus 1) | unchanged |
| Screen | – | 19, 21, 23 (SPI), 24 (screen select), 26 (touch select), 18 (data/command, BCM 24), 22 (reset, BCM 25), 11 (touch interrupt, BCM 17), power 1/2/4, ground 6/9/14/20/25 |
| Buzzer | – | **32 / 12** (hardware PWM, channel 0) |
| Battery monitor SDA, SCL | – | **15, 16 / 22, 23** (its own I²C bus, bus 3; under the screen, like the IMU's) |

**Check the screen's row against the pin table that came with your screen.** It is the usual layout of these 3.5"
ILI9486 panels, not read from this seller's manual. If yours differs, the battery or buzzer pins may need to move too.

The code still has today's motor pins. When you have moved the wires, change the four numbers in **both**
`rocko_env/rocko_env/description/ros2_control/rocko.ros2_control.xacro` and `rocko_tank_mode.ros2_control.xacro`
(`pinNumberSpeed` 14 → 21 and `pinNumberDirection` 10 → 22 under `LeftDriveMotor`; 13 → 23 and 6 → 29 under
`RightDriveMotor`), or ask an agent to, then rebuild on the Pi (`./build_ros.sh`). Pulled code with new pins and old
wiring (or the reverse) means the motors do not move.

## 2. Wire the parts

Power the Pi off first.

- **Buzzer** (passive). A small piezo can hang straight off the pin through a 100 Ω resistor: GPIO 12 (pin 32) → resistor
  → buzzer → ground. A magnetic buzzer draws more than a pin can give: drive it with an NPN transistor (2N2222 or
  similar): pin 32 → 1 kΩ → base; emitter → ground; buzzer between 5 V (pin 2 or 4) and the collector, with a diode
  (1N4148) across the buzzer, stripe towards 5 V.
- **INA226** breakout. VCC → **3.3 V** (pin 1 or 17), never 5 V: its I²C pull-up resistors go to VCC, and the Pi's pins
  are not 5 V tolerant. GND → ground. SDA → pin 15, SCL → pin 16. For voltage only (the default), join **IN+ and IN−
  together** and connect them, through a small inline fuse (0.5 A), to the battery's **+**. The motor current does not go
  through the board: its 0.1 Ω shunt would saturate at 0.8 A. The battery's **−** must share ground with the Pi (it
  normally does already, through the regulator). A0 and A1 to ground gives address 0x40.

## 3. Turn the hardware on in the Pi's config

Edit `/boot/firmware/config.txt` (`sudo nano /boot/firmware/config.txt`) and add at the end, under `[all]`:

```text
# ROCKO extras (docs/guides/fit-the-extras.md)
dtparam=spi=on
dtoverlay=piscreen,speed=16000000
dtoverlay=pwm,pin=12,func=4
dtoverlay=i2c-gpio,bus=3,i2c_gpio_sda=22,i2c_gpio_scl=23
```

What each line does: SPI on for the screen; `piscreen` loads the ILI9486 panel driver and the ADS7846 touch driver on the
pins in the table; `pwm` gives GPIO 12 to hardware PWM channel 0 (`func=4` is that pin's PWM function); `i2c-gpio`
makes I²C bus 3 on GPIO 22 and 23 (a software bus: slow, which is fine for one reading a second). Then
`sudo reboot`, and check, on the Pi:

```bash
cat /sys/class/graphics/fb*/name
ls /sys/class/pwm/
ls /dev/i2c-3
```

Success: one name starts with `fb_` (for example `fb_ili9486`), `ls /sys/class/pwm/` lists a `pwmchip`, and
`/dev/i2c-3` exists. `grep -i ads7846 /proc/bus/input/devices` prints a line when the touch driver loaded. With the INA226 wired, `sudo apt install i2c-tools` and then `sudo i2cdetect -y 3` shows `40` in
the grid.

If no `fb_` name appears: the `piscreen` overlay's support on a **Pi 5** is not confirmed. Check `dmesg | grep -i -E
"fbtft|ili9486|spi"` for an error and note it. Do **not** run the seller's `LCD-show` script: it is written for
Raspberry Pi OS, replaces `config.txt` and does not support Ubuntu. Ask an agent to try the `mipi-dbi-spi` route
instead, giving it the `dmesg` lines. If `ls /sys/class/pwm/` lists more than one chip, set `pwm_chip` in
`extras.yaml` (step 5).

## 4. Python packages

The screen draws with Pillow and numpy. On the Pi:

```bash
sudo apt install python3-pil python3-numpy
pi/rocko_shell.sh
```

`pi/rocko_shell.sh` opens a **root** shell with the robot's own Python environment (the venv at `~/.venv`, see
[Run ROCKO on the Pi](run-on-the-pi.md)), the same one the robot's programs use. In it, run
`python3 -c "import PIL, numpy; print('ok')"`. Success: `ok`. If it says `No module named`, the venv does not see the
system packages; which way the venv on this Pi was made is not recorded, so this is the fallback, not tested: still in
that shell, run `pip install pillow numpy`, run the `import` line again, and expect `ok`. Then `exit`.

## 5. Restart, and check each one

If you edited the two xacro files in step 1, commit them first (`git commit -am "motor pins moved for the screen"`),
or `git pull` may refuse. Then:

```bash
git pull
./build_ros.sh
sudo systemctl restart rocko.service
```

This build is also the rebuild step 1 asks for. Success: `./build_ros.sh` ends with `Summary: 2 packages finished`
and exits without an error line.

Then open the dashboard (`http://<robot-ip>:8080`, [the dashboard guide](use-the-dashboard.md)). The three cards under
the state card say what each extra reports. An ERROR card's message names the cause.

**Buzzer.** About 10 s after the restart, a rising four-note chirp (the stack is ready). Press LB + RB: two rising
beeps (enabled); again: one low beep (disabled). Press **Mute** on the dashboard: the next enable and disable are
silent; **Unmute** plays one short chirp. Restart the service while muted: no boot chirp (the setting is kept).

**Battery.** Measure the battery with a multimeter at its terminals, and compare with the dashboard's battery card
within a minute (the reading is smoothed over about 10 s). Write both numbers in the journal or tell an agent. They
should agree within about 0.05 V; a bigger difference means a wiring or address problem. A reading of 0 V, or above
16 V, shows as an error and is not used.

**Screen.** The top strip turns red with DISABLED, and green with ENABLED when you press LB + RB. Tap the right third of
the bottom bar: the System page; again: the ROS page. If the picture is sideways or upside down, set `rotate_deg` in
`extras.yaml`. If a tap on the right lands on the left button (or a tap at the bottom does nothing), set
`touch_swap_xy`, then `touch_invert_x` or `touch_invert_y`, one at a time, until the right buttons respond. The file is
`rocko_env/rocko_env/extras/extras.yaml` in the checkout; uncomment the line you change (delete only its leading
`# `). `build_ros.sh` builds with `--symlink-install`, which links the installed copy to this file, so an edit needs
only `sudo systemctl restart rocko.service`. To check the link: `ls -l install/rocko_env/share/rocko_env/extras/extras.yaml`
shows `->` pointing into `rocko_env/rocko_env/extras/`.

**The CPU cost, before a balance test.** Measure the loop's timing with the screen running, the same way as
[the loop timing journal](../journal/2026-11-24-loop-timing-on-the-pi.md), and compare. If the loop's timing got
worse, set `refresh_period_s: 1.0` or start the robot with `screen:=false`.

## What commonly goes wrong

| You see | Why | Do |
| --- | --- | --- |
| Battery card: `cannot open /dev/i2c-3` | The `i2c-gpio` line is missing, or no reboot since | Step 3 |
| Battery card: `no answer from 0x40` | Wiring, power, or another address | `sudo i2cdetect -y 3`; set `i2c_address` (decimal) in `extras.yaml` |
| Battery card: `implausible reading 0.00 V` | IN+/IN− (or VBUS) not connected to the battery + | Step 2 |
| Buzzer card: `no /sys/class/pwm/pwmchip*` | The `pwm` line is missing, or no reboot since | Step 3 |
| Buzzer card OK, but silent | Muted, wrong pin, or a magnetic buzzer with no transistor | Unmute; check pin 32; step 2 |
| Screen card: `no panel framebuffer` | The panel driver did not load | Step 3's `dmesg` check |
| Screen card: `cannot draw: No module named 'PIL'` | Pillow is missing where the robot's Python looks | Step 4 |
| Screen card WARN: `touch: ...` | The picture works but the touch device was not found | `cat /proc/bus/input/devices` should list an `ADS7846` |
| Motors do not move after the change | The xacro pins and the wires do not match | Step 1 |
