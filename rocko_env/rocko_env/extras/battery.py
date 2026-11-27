"""Battery arithmetic: the INA226's registers, voltage to charge, and the warning levels. No ROS, no I/O.

The battery is a 12 V (12.8 V nominal, four cells) 10 Ah LiFePO4. BatteryMonitor.py reads its voltage with a
TI INA226 on its own I2C bus and uses these functions; the supervisor uses CRITICAL_V to refuse an enable.
Design: docs/architecture/robot-extras.md.

Why the percentage is rough: a LiFePO4 cell's voltage is almost flat from about 90 % down to 20 %, so a
0.1 V error moves the estimate by tens of percent there. The voltage is the number to trust; the percentage is
for a glance. It is also only right for a RESTING battery: under load the voltage sags, so while driving the
estimate reads low. The smoothing below keeps one hard acceleration from setting off a warning.
"""

# ---- the INA226 (datasheet SBOS547) ------------------------------------------------------------------
INA226_ADDRESS = 0x40          # A0 and A1 tied to ground, the usual breakout board's default
REG_CONFIG = 0x00
REG_SHUNT_VOLTAGE = 0x01       # signed, 2.5 uV per count
REG_BUS_VOLTAGE = 0x02         # unsigned, 1.25 mV per count: the battery voltage, wired as the guide says
REG_MANUFACTURER_ID = 0xFE
MANUFACTURER_TI = 0x5449       # "TI" in ASCII: what a real INA226 answers
# Configuration register: bit 14 is always 1; averaging 16 samples (AVG=010), 1.1 ms per bus and per shunt
# conversion (100, 100), continuous shunt and bus (MODE=111). A new reading is ready every ~35 ms; we read once a
# second, so each reading is already the average of 16.
CONFIG_AVG16_CONTINUOUS = (1 << 14) | (0b010 << 9) | (0b100 << 6) | (0b100 << 3) | 0b111   # = 0x4527
BUS_VOLTS_PER_COUNT = 1.25e-3
SHUNT_VOLTS_PER_COUNT = 2.5e-6


def bus_voltage_v(raw):
    """REG_BUS_VOLTAGE's 16-bit value -> volts."""
    return raw * BUS_VOLTS_PER_COUNT


def shunt_current_a(raw, shunt_ohms):
    """REG_SHUNT_VOLTAGE's 16-bit value (two's complement) -> amps through a shunt of shunt_ohms.
    + is current from IN+ to IN-. Only meaningful when the motor current really flows through the shunt."""
    signed = raw - 0x10000 if raw & 0x8000 else raw
    return signed * SHUNT_VOLTS_PER_COUNT / shunt_ohms


# ---- what a reading may be ------------------------------------------------------------------------------
# [V] Outside this range the reading is a wiring fault, not a battery: 0 V is an unconnected VBUS pin, and a 4-cell
# LiFePO4 never reads above ~14.6 V even on the charger.
PLAUSIBLE_MIN_V = 5.0
PLAUSIBLE_MAX_V = 16.0

# ---- voltage -> charge, for a RESTING 4-cell LiFePO4 ------------------------------------------------------
# (volts, percent), rising. A typical published chart for 12.8 V LiFePO4 packs, not measured on this battery:
# check yours against its datasheet and edit this table. Note how steep it is: 20 % to 90 % lies between 12.9 and
# 13.3 V, so the screen's "~%" is rough.
CHARGE_TABLE = (
    (10.0, 0), (12.0, 9), (12.5, 14), (12.8, 17), (12.9, 20), (13.0, 30),
    (13.1, 40), (13.2, 70), (13.3, 90), (13.4, 99), (13.6, 100),
)


def percent_from_voltage(volts):
    """Charge left [0..100 %] from the resting voltage, by straight lines between CHARGE_TABLE's points."""
    if volts <= CHARGE_TABLE[0][0]:
        return 0.0
    for (v0, p0), (v1, p1) in zip(CHARGE_TABLE, CHARGE_TABLE[1:]):
        if volts <= v1:
            return p0 + (p1 - p0) * (volts - v0) / (v1 - v0)
    return 100.0


# ---- warning levels --------------------------------------------------------------------------------------
# [V] Proposed, from CHARGE_TABLE, not tested on this battery. At WARN_V (about 17 %) the screen turns amber and the
# buzzer beeps once; at CRITICAL_V (about 9 %) it turns red, beeps, and the supervisor refuses to ENABLE. Nothing
# disables a robot that is already enabled: cutting the motors while balancing drops the robot.
WARN_V = 12.8
CRITICAL_V = 12.0
# [V] To LEAVE a level the voltage must climb this far above its threshold, so a voltage sitting on a threshold does
# not make the buzzer beep every second. (The supervisor's enable check uses CRITICAL_V alone.)
HYSTERESIS_V = 0.2
# The battery's levels. Not the same thing as health.OK/WARN/ERROR, which say whether the MONITOR works.
LEVEL_OK, LEVEL_WARN, LEVEL_CRITICAL = "ok", "warn", "critical"

# [s] Time constant of the smoothing: a step change in voltage shows about 63 % of the way after this long.
SMOOTHING_S = 10.0


def smooth(previous_v, reading_v, dt_s):
    """One step of a first-order low-pass filter (an exponential moving average). previous_v None: start at the
    reading. Long enough to ride out one acceleration's sag, short enough to see a real drop within a minute."""
    if previous_v is None:
        return reading_v
    alpha = min(1.0, dt_s / SMOOTHING_S)
    return previous_v + alpha * (reading_v - previous_v)


def level(volts, previous_level=LEVEL_OK):
    """LEVEL_OK / LEVEL_WARN / LEVEL_CRITICAL for a smoothed voltage, with HYSTERESIS_V on the way back up.
    Example: at 12.9 V a battery that was already "warn" stays "warn" (its threshold is now 12.8 + 0.2 = 13.0 V);
    it is "ok" again only above 13.0 V."""
    # Each threshold is raised by the hysteresis while we are already at or below it.
    warn_v = WARN_V + (HYSTERESIS_V if previous_level in (LEVEL_WARN, LEVEL_CRITICAL) else 0.0)
    critical_v = CRITICAL_V + (HYSTERESIS_V if previous_level == LEVEL_CRITICAL else 0.0)
    if volts <= critical_v:
        return LEVEL_CRITICAL
    if volts <= warn_v:
        return LEVEL_WARN
    return LEVEL_OK


def enable_allowed(volts):
    """The supervisor's rule: refuse to enable at or below CRITICAL_V. volts None (no fresh reading: the battery
    monitor is optional and may not be fitted) allows it, because an unknown battery is the robot as it was before
    the monitor existed."""
    return volts is None or volts > CRITICAL_V
