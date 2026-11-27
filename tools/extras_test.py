#!/usr/bin/env python3
"""Host test of the optional extras' logic (rocko_env/extras/): battery arithmetic, when the buzzer plays what,
how the dashboard and the screen read the health reports, the screen's text and touch buttons, and the /proc
parsers. No ROS, no hardware. Run by tools/run_ladder.sh (--host too), or alone: python3 tools/extras_test.py
What it cannot check: the I2C, PWM, framebuffer and touch I/O (docs/guides/fit-the-extras.md has the procedure on
the robot) and the drawing itself (tools/screen_render_check.py, in the container, where Pillow is).
"""
import pathlib
import sys

REPO = pathlib.Path(__file__).resolve().parent.parent
sys.path.insert(0, str(REPO / "rocko_env"))
from rocko_env.extras import battery, health, log_phrases, screen_layout, sounds, sysinfo  # noqa: E402

failures = []


def check(name, ok, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}  {name}" + (f"  ({detail})" if detail else ""))
    if not ok:
        failures.append(name)


def near(a, b, tol=1e-6):
    return abs(a - b) <= tol


# ---- battery -------------------------------------------------------------------------------------------------
check("INA226 bus voltage: 10400 counts = 13.0 V", near(battery.bus_voltage_v(10400), 13.0))
check("INA226 shunt: 0xFFFF = -2.5 uV, over 0.002 ohm = -1.25 mA",
      near(battery.shunt_current_a(0xFFFF, 0.002), -2.5e-6 / 0.002))
check("INA226 shunt: 4000 counts over 0.002 ohm = 5 A", near(battery.shunt_current_a(4000, 0.002), 5.0))
check("charge: table points come back exactly",
      all(near(battery.percent_from_voltage(v), p) for v, p in battery.CHARGE_TABLE))
check("charge: halfway between 13.1 V (40 %) and 13.2 V (70 %) is 55 %", near(battery.percent_from_voltage(13.15), 55.0))
check("charge: clamped to 0..100", battery.percent_from_voltage(3.0) == 0.0 and battery.percent_from_voltage(20.0) == 100.0)
check("charge: never decreases as the voltage rises",
      all(battery.percent_from_voltage(v / 100) <= battery.percent_from_voltage((v + 1) / 100) for v in range(900, 1500)))
check("smoothing: the first reading is taken as it is", battery.smooth(None, 12.5, 1.0) == 12.5)
v = 13.0
for _ in range(10):                         # 10 s of 1 s readings at 12.0 V, after 13.0 V
    v = battery.smooth(v, 12.0, 1.0)
check("smoothing: one time constant moves ~63 % of a step", 0.60 < (13.0 - v) < 0.70, f"moved {13.0 - v:.3f} V")
check("smoothing: one 1 s sag of 1 V moves the average only 0.1 V", near(battery.smooth(13.0, 12.0, 1.0), 12.9))
check("level: thresholds", [battery.level(x) for x in (13.0, 12.8, 12.5, 12.0, 11.0)] ==
      ["ok", "warn", "warn", "critical", "critical"])
check("level: hysteresis holds warn just above WARN_V, releases above WARN_V + HYSTERESIS_V",
      battery.level(12.9, "warn") == "warn" and battery.level(13.05, "warn") == "ok")
check("level: hysteresis holds critical just above CRITICAL_V",
      battery.level(12.1, "critical") == "critical" and battery.level(12.3, "critical") == "warn")
check("enable rule: unknown battery allowed, critical refused, just above allowed",
      battery.enable_allowed(None) and not battery.enable_allowed(12.0) and battery.enable_allowed(12.01))

# ---- sounds ------------------------------------------------------------------------------------------------------
e = sounds.SoundEvents()
check("sounds: nothing before the supervisor is heard", e.update(None, None, False, None, None) == [])
check("sounds: no boot tune while the stack is still starting", e.update(False, "starting", False, 0.1, "ok") == [])
check("sounds: boot tune once the first stack is ready", e.update(False, "ready", False, 0.1, "ok") == ["boot"])
check("sounds: boot tune only once", e.update(False, "ready", False, 0.1, "ok") == [])
check("sounds: enabled", e.update(True, "ready", False, 0.1, "ok") == ["enabled"])
check("sounds: no repeat while it stays enabled", e.update(True, "ready", False, 0.1, "ok") == [])
check("sounds: gamepad lost while enabled, once",
      e.update(True, "ready", False, 2.0, "ok") == ["gamepad_lost"] and e.update(True, "ready", False, 3.0, "ok") == [])
check("sounds: tipover, once", e.update(True, "ready", True, 0.1, "ok") == ["tipover"]
      and e.update(True, "ready", True, 0.1, "ok") == [])
check("sounds: disabled", e.update(False, "ready", False, 0.1, "ok") == ["disabled"])
check("sounds: gamepad silence while DISABLED is not 'lost'", e.update(False, "ready", False, 9.0, "ok") == [])
check("sounds: fault, once", e.update(False, "fault", False, 0.1, "ok") == ["fault"]
      and e.update(False, "fault", False, 0.1, "ok") == [])
check("sounds: battery low, then critical, once each",
      e.update(False, "ready", False, 0.1, "warn") == ["battery_low"]
      and e.update(False, "ready", False, 0.1, "critical") == ["battery_critical"]
      and e.update(False, "ready", False, 0.1, "critical") == [])
check("sounds: a lost battery reading (None) is silent", e.update(False, "ready", False, 0.1, None) == [])
check("sounds: every event has a tune, and every note is (Hz >= 0, s > 0)",
      all(name in sounds.TUNES for name in ("boot", "enabled", "disabled", "fault", "tipover", "gamepad_lost",
                                            "battery_low", "battery_critical", "unmuted"))
      and all(f >= 0 and d > 0 for tune in sounds.TUNES.values() for f, d in tune))

# ---- health and the tipover phrase ------------------------------------------------------------------------------
reports = {"battery": health.report("battery", health.WARN, "12.70 V, low", {"voltage": 12.7}),
           "buzzer": health.report("buzzer", health.OK, "ready", {"muted": "false"})}
s = health.summary(reports, {"battery": 0.5, "buzzer": health.STALE_AFTER_S + 1})
check("health: a fresh report is shown with its level", s["battery"]["state"] == "WARN" and s["battery"]["values"]["voltage"] == "12.7")
check("health: an old report is NOT RUNNING, not its last level", s["buzzer"]["state"] == "NOT RUNNING")
check("health: an extra never heard is NOT RUNNING", s["screen"]["state"] == "NOT RUNNING")
cpp = (REPO / "rocko_env/rocko_env/controllers/add_feedforward/AddFeedforwardController.cpp").read_text()
check("tipover phrases are still in AddFeedforwardController.cpp (the screen, buzzer and dashboard read them)",
      log_phrases.TIPOVER_TRIPPED in cpp and log_phrases.TIPOVER_REARMED in cpp)
check("tipover phrases", log_phrases.tipover_change("x TIPOVER CUTOFF TRIPPED at 31 deg") is True
      and log_phrases.tipover_change("Tipover cutoff RE-ARMED") is False and log_phrases.tipover_change("hello") is None)

# ---- the screen's text and buttons ---------------------------------------------------------------------------
L = screen_layout
check("banner: enabled is green, disabled red, no supervisor grey",
      L.banner({"robot": {"enabled": True, "mode": "tank"}}) == ("ENABLED  tank", L.GREEN)
      and L.banner({"robot": {"enabled": False, "mode": "balance"}})[1] == L.RED
      and L.banner({"robot": None})[1] == L.GREY)
view = {"robot": {"mode": "balance", "enabled": False, "phase": "fault", "message": "IMU read failed",
                  "nodes": [("supervisor", True), ("balance_teleop", False)],
                  "controllers": [("pitch_pid_controller", "inactive", "inactive"), ("velocity_pid_controller", "missing", "inactive")]},
        "pitch_deg": -2.34, "battery": {"voltage": 11.9, "percent": 8.4, "level": "critical"}, "gamepad": False,
        "tipover": True, "gains": {"pitch": (10.0, 0.0, 0.1), "velocity": (-2.0, -5.0, -0.2)},
        "ips": {"wlan0": "192.168.1.42"}, "system": {"cpu_percent": 91.0, "ram_used_mb": 900, "ram_total_mb": 4000,
                                                    "temp_c": 60.0, "uptime_s": 3725, "hostname": "rocko"},
        "extras": {"screen": {"state": "OK", "message": "page Main", "values": {}}}, "muted": True}
main = [line.text for line in L.main_page(view)]
check("main page: one line per field, in order", len(main) == len(L.MAIN_FIELDS) and main[0] == "Pitch: -2.3 deg", main[0])
check("main page: every field fits on the screen (nothing cut to '... more')",
      L.fit_lines(L.main_page(view), L.CONTENT_HEIGHT) == L.main_page(view))
check("main page: the battery line says charge now, in red",
      L.battery_text(view) == ("11.90 V  ~8 %  CHARGE NOW", L.RED), L.battery_text(view)[0])
check("main page: gamepad missing and tipover tripped are red",
      L.gamepad_text(view)[1] == L.RED and L.tipover_text(view)[1] == L.RED)
check("main page: the fault reason is shown", "IMU read failed" in L.state_text(view)[0])
check("main page: the gains and the IP", "p 10  i 0  d 0.1" in main[5] and "192.168.1.42" in main[7], main[5])
empty = {}
check("main page: an empty view shows unknowns, never a crash or a guess",
      [line.text for line in L.main_page(empty)][:4] == ["Pitch: unknown", "Battery: not measured",
                                                        "Gamepad: NOT CONNECTED", "Tipover: no report yet"])
check("system page: CPU over 80 % is amber, uptime formatted",
      L.system_page(view)[0].colour == L.AMBER and "1 h 02 min" in L.system_page(view)[3].text)
ros = L.ros_page(view)
check("ROS page: a missing node and a controller in the wrong state are red",
      any("balance_teleop: MISSING" in l.text and l.colour == L.RED for l in ros)
      and any("velocity_pid_controller: missing / inactive" in l.text and l.colour == L.RED for l in ros))
check("buttons: thirds of the bottom bar, nothing above it",
      [L.button_at(10, 300), L.button_at(240, 300), L.button_at(470, 300), L.button_at(240, 100)]
      == ["prev", "mute", "next", None])
check("buttons: labels name the neighbouring pages and the mute state",
      L.button_labels(view, 0) == ["< ROS", "Muted", "System >"])
many = [L.Line(f"line {i}", L.WHITE, 16) for i in range(30)]
cut = L.fit_lines(many, 200)
check("fit_lines: cut to the height, with a count of what was left out",
      sum(l.size + 4 for l in cut) <= 200 and cut[-1].text == f"... +{30 - len(cut) + 1} more", cut[-1].text)
check("fit_lines: everything fits -> unchanged", L.fit_lines(many[:3], 200) == many[:3])

# ---- /proc parsers ---------------------------------------------------------------------------------------------
before = sysinfo.parse_cpu_times("cpu  100 0 100 700 100 0 0 0 0 0\ncpu0 ...")
after = sysinfo.parse_cpu_times("cpu  150 0 150 750 100 0 0 0 0 0\n")
check("cpu: 100 busy of 150 jiffies = 66.7 %", near(sysinfo.cpu_percent(before, after), 100 * 100 / 150, 1e-9))
check("meminfo: used = total - available",
      sysinfo.parse_meminfo("MemTotal:  4096000 kB\nMemFree: 1 kB\nMemAvailable: 3072000 kB\n") == (1000.0, 4000.0))

print("RESULT:", "FAIL" if failures else "PASS", f"({len(failures)} failed)" if failures else "")
sys.exit(1 if failures else 0)
