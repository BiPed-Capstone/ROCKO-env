#!/usr/bin/env python3
"""Check the status screen's drawing (rocko_env/extras/screen_draw.py), where Pillow and numpy are: the container.

    docker compose -f docker/compose.yml exec -T ros2 python3 /ros2_ws/src/ROCKO-env/tools/screen_render_check.py [out_dir]

Draws every page for an enabled, a disabled and an unknown robot, and checks: the picture is 480x320; the top strip
is green / red / grey; the RGB565 conversion matches a pixel-by-pixel reference; and it writes the pictures to
out_dir (default /tmp/rocko_screen) so a person can look at them. Not a check of the panel or its driver: that is
on the robot (docs/guides/fit-the-extras.md).
"""
import os
import pathlib
import struct
import sys

sys.path.insert(0, str(pathlib.Path(__file__).resolve().parent.parent / "rocko_env"))
from rocko_env.extras import screen_draw, screen_layout as L  # noqa: E402

failures = []


def check(name, ok, detail=""):
    print(f"{'PASS' if ok else 'FAIL'}  {name}" + (f"  ({detail})" if detail else ""))
    if not ok:
        failures.append(name)


def robot(enabled):
    return {"mode": "balance", "enabled": enabled, "phase": "ready", "message": "",
            "nodes": [("supervisor", True), ("game_controller_node", True), ("balance_teleop", True)],
            "controllers": [("joint_state_broadcaster", "active", "active"),
                            ("pitch_pid_controller", "active" if enabled else "inactive",
                             "active" if enabled else "inactive")]}


def view(robot_state):
    return {"robot": robot_state, "pitch_deg": 1.8, "battery": {"voltage": 13.12, "percent": 46, "level": "ok"},
            "gamepad": True, "tipover": False, "gains": {"pitch": (10.0, 0.0, 0.1), "velocity": (-2.0, -5.0, -0.2)},
            "ips": {"wlan0": "192.168.1.42"},
            "system": {"cpu_percent": 23.0, "ram_used_mb": 812, "ram_total_mb": 3956, "temp_c": 51.0,
                       "uptime_s": 1234, "hostname": "rocko"},
            "extras": {"battery": {"state": "OK", "message": "13.12 V, ok", "values": {}},
                       "buzzer": {"state": "ERROR", "message": "no PWM output", "values": {}},
                       "screen": {"state": "OK", "message": "page Main", "values": {}}},
            "muted": False, "disk": {"used_gb": 12.3, "total_gb": 58.0, "free_gb": 45.7},
            "recording": {"running": "true", "seconds_left": "42", "folder": "20261127_101500_screen",
                          "log_root": "/home/rocko/rocko_logs", "free_gb": "45.7", "low_space": "false", "port": "8080",
                          "button_s": "60"}}


out = pathlib.Path(sys.argv[1] if len(sys.argv) > 1 else "/tmp/rocko_screen")
out.mkdir(parents=True, exist_ok=True)
for label, state, colour in (("enabled", robot(True), L.GREEN), ("disabled", robot(False), L.RED),
                             ("unknown", None, L.GREY)):
    for page, (name, _) in enumerate(L.PAGES):
        image = screen_draw.render(view(state), page)
        path = out / f"{label}-{name.lower()}.png"
        image.save(path)
        check(f"{label} / {name}: 480x320, top strip {colour}",
              image.size == (L.WIDTH, L.HEIGHT) and image.getpixel((5, 5)) == colour, str(path))

# The record button (third quarter of the bar) is red while recording.
image = screen_draw.render(view(robot(True)), 0)
inside = (L.WIDTH * 2 // 4 + 10, L.BAR_Y + 10)
check("the record button is red while recording", image.getpixel(inside) == L.RED, str(image.getpixel(inside)))

# RGB565: compare numpy's conversion with the formula written out for single pixels.
image = screen_draw.render(view(robot(True)), 0)
data = screen_draw.to_rgb565(image)
check("RGB565: 2 bytes per pixel", len(data) == L.WIDTH * L.HEIGHT * 2, f"{len(data)} bytes")
ok = True
for x, y in ((0, 0), (5, 5), (100, 120), (479, 319), (240, 290)):
    r, g, b = image.getpixel((x, y))
    expected = ((r >> 3) << 11) | ((g >> 2) << 5) | (b >> 3)
    got = struct.unpack_from("<H", data, (y * L.WIDTH + x) * 2)[0]
    ok = ok and got == expected
check("RGB565: matches the formula at sample pixels, low byte first", ok)
pure = screen_draw.to_rgb565(screen_draw.Image.new("RGB", (2, 1), (255, 0, 0)))
check("RGB565: pure red is 0xF800", pure == b"\x00\xf8\x00\xf8", pure.hex())

print("RESULT:", "FAIL" if failures else "PASS", f"pictures in {out}" if not failures else "")
sys.exit(1 if failures else 0)
