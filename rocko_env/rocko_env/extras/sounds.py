"""Which sound the buzzer plays, and when. No ROS, no I/O: Buzzer.py feeds it and plays what it returns.

To change a sound, edit TUNES. To add one, add a tune here and the line in SoundEvents.update() that returns
its name. One exception: "unmuted" is played by Buzzer.py itself when the mute is switched off, not by update().
Design: docs/architecture/robot-extras.md.
"""
from rocko_env.extras import battery

# name -> notes, each (frequency [Hz], duration [s]); frequency 0 is a rest. The buzzer is passive, so it can play
# any pitch; around 1-4 kHz is loudest on a typical small one. Rising = something started, falling = stopped.
TUNES = {
    "boot":             [(1047, 0.10), (1319, 0.10), (1568, 0.10), (2093, 0.20)],   # C6 E6 G6 C7: ready to drive
    "enabled":          [(1568, 0.08), (0, 0.04), (2093, 0.12)],                     # two rising beeps
    "disabled":         [(784, 0.30)],                                               # one low beep
    "fault":            [(2093, 0.15), (1047, 0.15), (2093, 0.15), (1047, 0.15)],    # siren: look at the screen
    "tipover":          [(1568, 0.12), (1175, 0.12), (784, 0.30)],                   # falling: the cutoff tripped
    "gamepad_lost":     [(1319, 0.08), (0, 0.08), (1319, 0.08), (0, 0.08), (1319, 0.08)],
    "battery_low":      [(988, 0.20), (0, 0.10), (988, 0.20)],
    "battery_critical": [(988, 0.20), (0, 0.10), (988, 0.20), (0, 0.10), (988, 0.20), (0, 0.10), (988, 0.40)],
    "unmuted":          [(2093, 0.05)],                                              # confirms the mute toggle
}

# [s] While enabled, no /joy for this long counts as "gamepad lost". joy republishes an unchanged state at
# gamepad.JOY_AUTOREPEAT_RATE_HZ, so silence this long means the gamepad or its node is gone.
GAMEPAD_LOST_S = 1.0


class SoundEvents:
    """Turns the robot's state, seen again and again, into the events that deserve a sound: each fires once, on the
    change, never on every update."""

    def __init__(self):
        self.booted = False
        self.enabled = None          # None until the supervisor is first heard
        self.phase = None
        self.tipover = False
        self.gamepad_lost = False
        self.battery_level = battery.LEVEL_OK

    def update(self, enabled, phase, tipover, gamepad_age_s, battery_level):
        """enabled/phase: the supervisor's (None if not heard); tipover: True while the cutoff is tripped;
        gamepad_age_s: seconds since the last /joy (None: never); battery_level: battery.LEVEL_* or None.
        Returns the names of the tunes to play, in order."""
        events = []
        if enabled is not None:
            # "Finished booting": the supervisor's first stack is up. Once per run of the buzzer node.
            if not self.booted and phase == "ready":
                self.booted = True
                events.append("boot")
            if self.enabled is not None and enabled != self.enabled:
                events.append("enabled" if enabled else "disabled")
            if phase == "fault" and self.phase != "fault":
                events.append("fault")
            self.enabled, self.phase = enabled, phase

        if tipover and not self.tipover:
            events.append("tipover")
        self.tipover = bool(tipover)

        lost = bool(self.enabled) and (gamepad_age_s is None or gamepad_age_s > GAMEPAD_LOST_S)
        if lost and not self.gamepad_lost:
            events.append("gamepad_lost")
        self.gamepad_lost = lost

        if battery_level is not None:
            if battery_level == battery.LEVEL_CRITICAL and self.battery_level != battery.LEVEL_CRITICAL:
                events.append("battery_critical")
            elif battery_level == battery.LEVEL_WARN and self.battery_level == battery.LEVEL_OK:
                events.append("battery_low")
            self.battery_level = battery_level
        return events
