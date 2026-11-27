"""What the status screen shows: every page's lines, as text and colour. No drawing, no ROS.

To change the screen, edit this file. StatusScreen.py collects a `view` (a dict, described below) and
screen_draw.py paints whatever these functions return, so nothing else needs to change:
  * a new line on the main page: write a function view -> (text, colour) and add (label, function) to MAIN_FIELDS;
  * a new page: write a function view -> [Line, ...] and add it to PAGES.

The view (StatusScreen.make_view builds it; a key is None when that value is not known right now):
  robot      {"mode", "enabled", "phase", "message", "nodes": [(name, alive)], "controllers": [(name, state, expected)]}
             from the supervisor's /rocko/state, or None when no supervisor is heard
  pitch_deg  the IMU's pitch [deg], or None
  battery    {"voltage", "percent", "level"} from /rocko/battery, or None
  gamepad    True if /joy was heard recently
  tipover    True tripped / False armed / None never seen a log line
  gains      {"pitch": (p, i, d), "velocity": (p, i, d)} from rocko_controllers.yaml, or None
  ips        {interface: address}
  system     {"cpu_percent", "ram_used_mb", "ram_total_mb", "temp_c", "uptime_s", "hostname"}
  extras     health.summary(): {extra: {"state", "message", "values"}}
  muted      True / False / None (buzzer not heard)
  disk       sysinfo.read_disk_gb("/"): {"used_gb", "total_gb", "free_gb"}, or None
  recording  the dashboard's "rocko/recording" report values (all text), or None when no dashboard is heard:
             {"running": "true"/"false", "seconds_left": seconds, or "" for a recording with no end,
              "folder", "log_root", "free_gb" ("" when unknown), "low_space": "true"/"false", "port",
              "button_s"}. The dashboard does the recording; the screen only asks it to.
"""
from collections import namedtuple

from rocko_env.extras import battery, sysinfo

WIDTH, HEIGHT = 480, 320           # [px] the 3.5" panel, landscape
BANNER_H = 56                      # [px] the ENABLED / DISABLED strip at the top
BANNER_SIZE = 36                   # [px] its font height
BAR_H = 56                         # [px] the four touch buttons at the bottom
BAR_Y = HEIGHT - BAR_H
CONTENT_TOP = BANNER_H + 6         # [px] where a page's first line starts
CONTENT_HEIGHT = BAR_Y - 4 - CONTENT_TOP   # [px] room for a page's lines; the main page must fit in it whole
MARGIN_X = 10                      # [px] left edge of the text
LINE_GAP = 4                       # [px] between one line and the next
BUTTON_INSET = 4                   # [px] space around each button inside its quarter of the bar
BUTTON_RADIUS = 8                  # [px] rounded corners
BUTTON_SIZE = 20                   # [px] the buttons' font height
PAGE_SIZE = 20                     # [px] font height of the System page's lines
SMALL_SIZE = 16                    # [px] the extras' lines and the ROS page, which have more to fit
CPU_WARN_PERCENT = 80              # [%] CPU busier than this shows amber
TEMP_WARN_C = 75                   # [deg C] the Pi 5 starts slowing itself down at about 80-85

# Colours (red, green, blue). Green = enabled and red = disabled, the same as the dashboard.
GREEN = (24, 140, 80)
RED = (190, 35, 24)
AMBER = (220, 130, 20)
GREY = (110, 118, 135)
WHITE = (240, 240, 240)
DIM = (160, 165, 175)
BACKGROUND = (12, 14, 18)
BUTTON = (45, 50, 60)

Line = namedtuple("Line", "text colour size")   # size [px], the font height


# ---- the strip at the top -------------------------------------------------------------------------------
def banner(view):
    """(text, background colour). Unknown is grey, never a guess: an old "disabled" could be wrong."""
    robot = view.get("robot")
    if robot is None:
        return "NO SUPERVISOR", GREY
    word, colour = ("ENABLED", GREEN) if robot["enabled"] else ("DISABLED", RED)
    return f"{word}  {robot['mode']}", colour


# ---- the main page, one line per field --------------------------------------------------------------------
def pitch_text(view):
    pitch = view.get("pitch_deg")
    return ("unknown", GREY) if pitch is None else (f"{pitch:+.1f} deg", WHITE)


def battery_text(view):
    b = view.get("battery")
    if b is None:
        return "not measured", GREY
    colour = {battery.LEVEL_WARN: AMBER, battery.LEVEL_CRITICAL: RED}.get(b["level"], WHITE)
    note = {battery.LEVEL_WARN: "  LOW", battery.LEVEL_CRITICAL: "  CHARGE NOW"}.get(b["level"], "")
    return f"{b['voltage']:.2f} V  ~{b['percent']:.0f} %{note}", colour


def gamepad_text(view):
    return ("connected", WHITE) if view.get("gamepad") else ("NOT CONNECTED", RED)


def tipover_text(view):
    tripped = view.get("tipover")
    if tripped is None:
        return "no report yet", GREY
    return ("TRIPPED", RED) if tripped else ("armed", WHITE)


def state_text(view):
    robot = view.get("robot")
    if robot is None:
        return "no supervisor", GREY
    colour = RED if robot["phase"] == "fault" else WHITE
    return (robot["phase"] + (f": {robot['message']}" if robot["message"] else "")), colour


def gains_text(loop):
    def text(view):
        gains = (view.get("gains") or {}).get(loop)
        if gains is None:
            return "unknown", GREY
        p, i, d = gains
        return f"p {p:g}  i {i:g}  d {d:g}", DIM
    return text


def ip_text(view):
    ips = view.get("ips") or {}
    return (" ".join(ips.values()), WHITE) if ips else ("no network", AMBER)


MAIN_FIELDS = [
    ("Pitch", pitch_text),
    ("Battery", battery_text),
    ("Gamepad", gamepad_text),
    ("Tipover", tipover_text),
    ("State", state_text),
    ("Pitch PID", gains_text("pitch")),
    ("Speed PID", gains_text("velocity")),
    ("IP", ip_text),
]
MAIN_SIZE = 20   # [px] 8 lines of 20 + LINE_GAP fit CONTENT_HEIGHT; a ninth field needs a smaller size


def main_page(view):
    lines = []
    for label, field in MAIN_FIELDS:
        text, colour = field(view)
        lines.append(Line(f"{label}: {text}", colour, MAIN_SIZE))
    return lines


# ---- more information, one tap away --------------------------------------------------------------------------
EXTRA_COLOURS = {"OK": WHITE, "WARN": AMBER, "ERROR": RED, "NOT RUNNING": GREY}


def storage_text(view):
    d = view.get("disk")
    if d is None:
        return "Storage: ?", GREY
    return (f"Storage: {d['used_gb']:.1f} of {d['total_gb']:.1f} GB used, {d['free_gb']:.1f} free",
            RED if sysinfo.disk_low(d["free_gb"]) else WHITE)


def system_page(view):
    s = view.get("system") or {}
    size = PAGE_SIZE
    cpu = s.get("cpu_percent")
    ram = (s.get("ram_used_mb"), s.get("ram_total_mb"))
    temp = s.get("temp_c")
    hot = (cpu or 0) > CPU_WARN_PERCENT or (temp or 0) > TEMP_WARN_C
    lines = [
        Line(f"CPU: {'?' if cpu is None else f'{cpu:.0f} %'}    Temp: {'?' if temp is None else f'{temp:.0f} C'}",
             AMBER if hot else WHITE, size),
        Line(f"RAM: {'?' if ram[0] is None else f'{ram[0]:.0f} of {ram[1]:.0f} MB'}", WHITE, size),
        Line(*storage_text(view), size),
        Line(f"Up: {sysinfo.format_uptime(s.get('uptime_s'))}   Host: {s.get('hostname', '?')}", WHITE, size),
    ]
    for name, address in (view.get("ips") or {}).items():
        lines.append(Line(f"{name}: {address}", WHITE, SMALL_SIZE))
    for extra, info in (view.get("extras") or {}).items():
        # The extras report on themselves here too, so a broken one is visible without the dashboard.
        lines.append(Line(f"{extra}: {info['state']} {info['message']}", EXTRA_COLOURS.get(info["state"], RED), SMALL_SIZE))
    return lines


def format_seconds(seconds):
    """62.4 -> "1:02" (minutes:seconds)."""
    whole = max(0, int(round(seconds)))
    return f"{whole // 60}:{whole % 60:02d}"


def seconds_left(rec):
    """The recording report's time left [s], or None for a recording with no end (sent as "")."""
    text = rec.get("seconds_left", "")
    return float(text) if text else None


def record_page(view):
    """Where recordings go and how to get them: the dashboard records (on the robot); a laptop downloads them."""
    rec = view.get("recording")
    size = PAGE_SIZE
    if rec is None:
        return [Line("No dashboard: recording needs it.", GREY, size),
                Line("Start the robot with the dashboard on (it is, by default).", DIM, SMALL_SIZE)]
    lines = []
    if rec.get("running") == "true":
        left = seconds_left(rec)
        lines.append(Line(f"RECORDING  {'(no time limit)' if left is None else format_seconds(left) + ' left'}",
                          RED, size))
        lines.append(Line(f"  {rec.get('folder', '?')}", WHITE, SMALL_SIZE))
    else:
        lines.append(Line("Not recording", WHITE, size))
    lines.append(Line("Saved on the robot in:", DIM, SMALL_SIZE))
    lines.append(Line(f"  {rec.get('log_root', '?')}", WHITE, SMALL_SIZE))
    if rec.get("free_gb"):
        lines.append(Line(f"Free there: {float(rec['free_gb']):.1f} GB",
                          RED if rec.get("low_space") == "true" else WHITE, SMALL_SIZE))
    lines.append(Line("Download to a laptop: open the dashboard,", DIM, SMALL_SIZE))
    ips = list((view.get("ips") or {}).values())
    address = f"http://{ips[0]}:{rec.get('port', '8080')}" if ips else "(no network)"
    lines.append(Line(f"  {address}, Record a run", WHITE, SMALL_SIZE))
    return lines


def ros_page(view):
    robot = view.get("robot")
    size = SMALL_SIZE
    if robot is None:
        return [Line("No supervisor: the robot launch is not running.", GREY, 20)]
    lines = [Line("Controllers (state / expected):", DIM, size)]
    for name, state, expected in robot["controllers"]:
        lines.append(Line(f"  {name}: {state} / {expected}", WHITE if state == expected else RED, size))
    lines.append(Line("Nodes:", DIM, size))
    for name, alive in robot["nodes"]:
        lines.append(Line(f"  {name}: {'running' if alive else 'MISSING'}", WHITE if alive else RED, size))
    return lines


PAGES = [("Main", main_page), ("System", system_page), ("Record", record_page), ("ROS", ros_page)]


# ---- the touch buttons ------------------------------------------------------------------------------------
BUTTONS = ("prev", "mute", "record", "next")   # the bottom bar's quarters, left to right


def record_label(view):
    """The record button: (text, colour). Red while recording, with the time left."""
    rec = view.get("recording")
    if rec is None:
        return "Rec ?", BUTTON                     # no dashboard to record with
    if rec.get("running") == "true":
        left = seconds_left(rec)
        return ("Stop" if left is None else f"Stop {format_seconds(left)}"), RED
    if not rec.get("button_s"):
        return "Rec", BUTTON                       # the dashboard always sends it; never guess a duration
    seconds = float(rec["button_s"])
    return (f"Rec {seconds / 60:g} min" if seconds % 60 == 0 else f"Rec {seconds:.0f} s"), BUTTON


def button_labels(view, page):
    """[(text, colour)] for BUTTONS, in order."""
    muted = view.get("muted")
    mute = "Sound ?" if muted is None else ("Muted" if muted else "Sound on")
    return [(f"< {PAGES[(page - 1) % len(PAGES)][0]}", BUTTON), (mute, BUTTON), record_label(view),
            (f"{PAGES[(page + 1) % len(PAGES)][0]} >", BUTTON)]


def button_at(x, y):
    """Which button a tap at screen pixel (x, y) hit, or None. A quarter of the width each, 120 px, is large on
    purpose: an uncalibrated resistive touch panel can be off by a few tens of pixels."""
    if y < BAR_Y or not 0 <= x < WIDTH:
        return None
    return BUTTONS[min(len(BUTTONS) - 1, int(x * len(BUTTONS) // WIDTH))]


def fit_lines(lines, height):
    """As many lines as fit in `height` pixels (each takes its size plus LINE_GAP), with a "+N more" line when cut."""
    used, kept = 0, []
    for i, line in enumerate(lines):
        if used + line.size + LINE_GAP > height:
            more = len(lines) - i + (1 if kept else 0)
            if kept:
                kept.pop()   # make room for the note
            kept.append(Line(f"... +{more} more", DIM, line.size))
            break
        kept.append(line)
        used += line.size + LINE_GAP
    return kept
