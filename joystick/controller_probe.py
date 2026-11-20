#!/usr/bin/env python3
"""Print what SDL (the library under pygame) sees of a game controller. Runs ON THE LAPTOP; needs
pygame-ce, no robot. Use it the first time you pair a controller, and whenever xbox_teleop.py drives
the wrong way or does not find the controller.

    python3 joystick/controller_probe.py          # Ctrl-C to stop

It prints, once: SDL's version, every joystick with its GUID and whether SDL has a GameController
mapping for it (xbox_teleop.py needs one), and that mapping. Then, ten times a second, the NAMED
values xbox_teleop.py uses (left stick Y, right stick X, the bumpers, Menu) next to the RAW joystick
axes, so you can see whether the names match the sticks you move.

What to check (write the answers in the guide's table if they differ):
  * left stick pushed UP gives left_y near -1.00, DOWN near +1.00;
  * right stick pushed RIGHT gives right_x near +1.00;
  * with your hands off, how far from 0.00 each stick rests (that sets --deadzone);
  * click another window: do the numbers keep changing? (if not, input stops when Terminal is behind).
"""
import os
import time

os.environ.setdefault("SDL_JOYSTICK_ALLOW_BACKGROUND_EVENTS", "1")  # same as xbox_teleop.py
os.environ.setdefault("PYGAME_HIDE_SUPPORT_PROMPT", "1")
import pygame  # noqa: E402
from pygame._sdl2 import controller as sdl_controller  # noqa: E402

FULL = 32767  # SDL GameController axis full scale

pygame.init()
sdl_controller.init()
print(f"pygame {pygame.version.ver}, SDL {'.'.join(map(str, pygame.get_sdl_version()))}")
print("Waiting up to 5 s for a controller...")
deadline = time.monotonic() + 5.0
while pygame.joystick.get_count() == 0 and time.monotonic() < deadline:
    pygame.event.pump()
    time.sleep(0.1)
count = pygame.joystick.get_count()
if count == 0:
    raise SystemExit("No joystick. Switch the controller on and check System Settings > Bluetooth.")

pad = None
for index in range(count):
    stick = pygame.joystick.Joystick(index)
    is_pad = sdl_controller.is_controller(index)
    print(f"[{index}] {stick.get_name()!r}  GUID {stick.get_guid()}  axes {stick.get_numaxes()}  "
          f"buttons {stick.get_numbuttons()}  GameController mapping: {'yes' if is_pad else 'NO'}")
    if is_pad and pad is None:
        pad = sdl_controller.Controller(index)
        raw = stick
        print("    mapping:", pad.get_mapping())
if pad is None:
    raise SystemExit("SDL has no GameController mapping for this device: xbox_teleop.py cannot use it.")

print("Move the sticks. Ctrl-C to stop.\n")
try:
    while True:
        pygame.event.pump()
        named = (f"left_y {pad.get_axis(pygame.CONTROLLER_AXIS_LEFTY) / FULL:+.2f}  "
                 f"right_x {pad.get_axis(pygame.CONTROLLER_AXIS_RIGHTX) / FULL:+.2f}  "
                 f"LB {pad.get_button(pygame.CONTROLLER_BUTTON_LEFTSHOULDER)} "
                 f"RB {pad.get_button(pygame.CONTROLLER_BUTTON_RIGHTSHOULDER)} "
                 f"Menu {pad.get_button(pygame.CONTROLLER_BUTTON_START)}")
        raw_axes = " ".join(f"{raw.get_axis(i):+.2f}" for i in range(raw.get_numaxes()))
        print(f"\r{named}   raw axes: {raw_axes}   attached {pad.attached()}  ", end="", flush=True)
        time.sleep(0.1)
except KeyboardInterrupt:
    print()
finally:
    pygame.quit()
