# Robot extras: a status screen, a buzzer and a battery monitor, none of which can stop the robot

- **Date / commit:** 2026-11-27, built on `1e20aba`
- **Branch:** `ai-forward`
- **Environment:** macOS laptop (host tests); dev container (Jazzy, image rebuilt with `python3-pil` and
  `python3-numpy`) for the build, the mock and the screen drawing. Nothing ran on the Pi.

## Goal and success criteria

The owner asked for, and answered the questions about (2026-11-27):
- a **status screen** on the robot (Hosyond 3.5" 480x320 SPI touch panel, Pi 5) showing enabled/disabled, balance
  angle, battery, PID gains, gamepad connected, the IP address, and on touch more pages: CPU, RAM, the ROS nodes and
  controllers; easy to change;
- **battery monitoring** (12 V 10 Ah LiFePO4, an INA226 on a second I²C bus; warn when low, refuse to enable when
  critical, never disable a running robot);
- the **dashboard's enabled/disabled** made obvious: green enabled, red disabled, filling the state card and its
  buttons, not the page;
- a **passive buzzer** (pin chosen by the agent) with sounds on boot finished, enable, disable, and the warnings the
  agent proposed, and **mute** on the dashboard and the screen;
- the hard rule: **every extra is optional.** Failing to start the screen, buzzer or battery read must not stop the
  robot; the error is logged and shown on the dashboard.

Checkable criteria:
1. With none of the extras' hardware present (the container), the robot launch reaches `ready`, enables and disables,
   and each extra reports ERROR on `/diagnostics` with its cause. (T2 `verify_supervisor.sh` section 12)
2. Killing an extra changes nothing for the supervisor. (T2, section 12)
3. A fresh battery voltage at or below `CRITICAL_V` refuses an enable in both modes, from the dashboard and the
   gamepad; no reading allows it; a disable is never refused. (host `supervisor_state_machine_test.py`, T2 section 12)
4. The battery, sound, health and screen-text logic is right, including edge cases. (host `tools/extras_test.py`)
5. Every screen page draws at 480x320 with a green, red or grey strip, and the pixel conversion is right. (container
   `tools/screen_render_check.py`)
6. The dashboard still passes its own check, which limits the services it may call. (T3 `verify_tools.sh`)
7. The launch still stops cleanly on SIGINT, with nothing left running. (T2 section 10)
8. Every tier that ran before still passes. (`tools/run_ladder.sh --clean`)

## Plan

New package folder `rocko_env/extras/`: pure-logic modules with no ROS (`battery.py`, `sounds.py`, `health.py`,
`screen_layout.py`, `sysinfo.py`, `log_phrases.py`), drawing (`screen_draw.py`, Pillow and numpy), three nodes
(`BatteryMonitor.py`, `Buzzer.py`, `StatusScreen.py`), `launch_extras.py`, and `extras.yaml` for the settings a person
changes on the robot. The supervisor reads `/rocko/battery`; the dashboard reads `/diagnostics` and calls the mute
service. Risks: anything an extra does that could block or kill the robot launch; the panel driver on a Pi 5 (cannot
be checked here); a pin conflict with today's wiring. No §6 file changes, so the realtime audit is not required.

## What changed

- **The optional rule.** Each extra is its own process; a missing part makes it log once, report ERROR every second
  and retry every 30 s; a crash ends only that process. [The robot extras](../architecture/robot-extras.md) explains
  it; `health.py` is the reporting.
- **Battery** (`battery.py`, `BatteryMonitor.py`): INA226 over `/dev/i2c-3` with the standard library only; a 10 s
  smoothing; a typical LiFePO4 charge table (not measured on this battery); warn and critical levels with hysteresis;
  readings outside 5 to 16 V are refused as wiring faults. `state_machine.py` refuses enable when a fresh voltage is
  at or below `CRITICAL_V`; `Supervisor.py` treats a reading older than 5 s as unknown, which allows it.
- **Buzzer** (`sounds.py`, `Buzzer.py`): hardware PWM on GPIO 12 through `/sys/class/pwm`; tunes for boot, enable,
  disable, fault, tipover, gamepad lost, battery low and critical; each once, on the change; mute through
  `/rocko/set_buzzer_muted`, saved to a file.
- **Screen** (`screen_layout.py`, `screen_draw.py`, `StatusScreen.py`): three pages (Main, System, ROS), a bottom bar
  of three large touch buttons (previous, mute, next), drawn twice a second into the panel's framebuffer, written only
  when it changed; `preview_png:=` draws into a picture instead.
- **Dashboard**: the state card is green/red/grey with green Enable and red Disable; three extras cards; a Mute button.
  `sim/check_dashboard.py` now allows the mute service besides the supervisor's two.
- **Shared tipover phrases** (`log_phrases.py`), now used by the dashboard, the screen and the buzzer; a host test
  fails if the C++ log lines change.
- **Build**: three programs and `extras.yaml` installed; `package.xml` exec-depends on `diagnostic_msgs`, `std_srvs`,
  `python3-numpy`, `python3-pil`, `python3-yaml`; the Dockerfile installs Pillow and numpy.
- **Not changed: the motor pins.** The panel needs header pins 19 to 26, and the four motor driver lines use 21 to 24
  today. The guide proposes new pins (and pins for the buzzer and the battery bus that avoid the panel); the code keeps
  today's until the owner rewires (next-steps row 4.17), so pulled code still matches the robot.

## Verification

- **Host, `python3 -I tools/extras_test.py`:** 49 checks, all PASS. Made red on purpose: `enable_allowed` returning
  always True (1 FAIL), the enable/disable sound removed (2 FAIL), a tipover phrase misspelled (2 FAIL); each restored.
- **Host, `python3 -I tools/supervisor_state_machine_test.py`:** 28 tests PASS, including the new
  `test_flat_battery_refuses_enable` (tank and balance; service and gamepad combo; unknown allowed; disable never
  refused). Made red with `enable_allowed` always True: `AssertionError: tank: refused at CRITICAL_V`.
- **Container, `tools/screen_render_check.py`:** 12 checks PASS (9 page and state pictures, 3 pixel checks). Made red by
  writing the pixels big-endian: 2 FAIL. The pictures were looked at; the first version cut the main page's last two
  lines, so the layout was tightened and a host check added that the main page fits.
- **T2, `verify_supervisor.sh`:** 39 checks PASS plus section 10 (clean stop), with section 12 new: each extra reported
  ERROR (`cannot open /dev/i2c-3 ...`, `no /sys/class/pwm/pwmchip* ...`, `no panel framebuffer ...`), enable worked
  anyway, a made-up 11.5 V battery was refused (`cannot enable: battery 11.50 V is at or below 12.0 V; charge it`),
  enable was allowed again 5.1 s after the readings stopped, and a SIGKILLed `Buzzer.py` changed nothing.
- **A real problem found by running it:** with `respawn=True` on the extras, a SIGINT to the process group (what Ctrl-C
  and `systemctl stop` do) let them exit cleanly before launch began shutting down; launch restarted them and never
  ended (seen: still running 20 s later, with fresh Buzzer, BatteryMonitor and StatusScreen processes). Respawn was
  removed; section 10 then passed with no leftovers.
- **The dashboard page** was looked at in a browser, served from a laptop with made-up `/api/state` (not the robot):
  green card while enabled, red after Disable, Mute turned into Unmute.
- **Full ladder, `tools/run_ladder.sh --clean`** (container recreated from the rebuilt image first): `RESULT: PASS`, 31 steps; Tier 1 with 0 compiler warnings, Tier 2 (mock both modes, supervisor), the screen drawing, Tier 3 (`verify_sim.sh`, `verify_tools.sh` with the updated dashboard check, the scenarios), every host test, the hook tests, the drift check and the link scan.
- **Not run:** anything on the Pi; the panel, its Pi 5 driver, the touch axes; the PWM and I²C overlay lines; the INA226;
  the buzzer's sound; the extras' CPU cost beside the balance loop; the buzzer's shutdown-silence fix (needs a buzzer).

## Review

Three reviewers in one round.
- **explainability-reviewer:** layout numbers outside `screen_layout.py` (fixed: named constants, used by both files);
  `unmuted` is not played by `SoundEvents` (fixed: said so); the ioctl number was bit-packing (fixed: named parts);
  the tipover phrases had no check (fixed: host test, made red); smaller comments (fixed). Declined: moving
  `CRITICAL_V` into `supervisor/` (the battery numbers belong together; the state machine importing `extras.battery`
  is a plain module import with no hardware).
- **ros2-control-reviewer:** uncommenting a line in `extras.yaml` made it invalid YAML (fixed: each block holds
  `use_sim_time: false`; an uncommented `rotate_deg` was then loaded without error); the buzzer could keep sounding at
  shutdown (fixed: a stop event checked between notes, the thread joined, then the output switched off; not run with
  a buzzer); the supervisor's check and the screen's red can differ by the hysteresis (documented); numpy added to the
  Dockerfile; screen mute taps no longer pile up behind an unanswered request.
- **docs-reviewer:** the dashboard's check and two docs said it calls only the supervisor's services (fixed: the check
  and the docs allow the mute switch); "no other node calls an extra's service" was false (fixed); the guide's Python
  and rebuild steps were incomplete (fixed: root shell and venv, the build's success line, commit before pull, the
  symlink check); smaller wording.

## What a human should do on the robot

[Fit the screen, buzzer and battery monitor](../guides/fit-the-extras.md), in order: move the four motor wires and
tell an agent to change the pins (row 4.17); wire the parts; add the config lines and check the devices appear; install
Pillow; restart and check each extra on the dashboard; compare the battery reading with a multimeter; measure the
loop's timing with the screen on before any balance test.
