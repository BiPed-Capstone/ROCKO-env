"""How the optional extras (battery monitor, buzzer, status screen) report whether they work.

The extras are optional: the robot drives and balances without any of them, and none of them can stop it
(docs/architecture/robot-extras.md). So a broken extra must not crash or block anything. It logs the error
and says so here instead, and the dashboard and the screen show it.

Each extra publishes one diagnostic_msgs/DiagnosticStatus, inside a DiagnosticArray on /diagnostics (the
standard ROS topic for "is this part healthy"), every REPORT_PERIOD_S. Its name is "rocko/<extra>". An extra
that is not running at all publishes nothing, which a reader sees as a report older than STALE_AFTER_S.
No ROS import here, so the tests run on a laptop; the nodes build the message from report().
"""

EXTRAS = ("battery", "buzzer", "screen")   # every extra, in the order the dashboard lists them
NAME_PREFIX = "rocko/"                     # DiagnosticStatus.name = NAME_PREFIX + extra
REPORT_PERIOD_S = 1.0                      # [s] how often each extra reports
STALE_AFTER_S = 5.0                        # [s] no report for this long = "not running"
RETRY_PERIOD_S = 30.0                      # [s] how often a failed extra tries its hardware again

# The levels, with diagnostic_msgs/DiagnosticStatus's numbers (OK=0, WARN=1, ERROR=2).
OK, WARN, ERROR = 0, 1, 2
LEVEL_NAMES = {OK: "OK", WARN: "WARN", ERROR: "ERROR"}


def report(extra, level, message, values=None):
    """One extra's status as plain data: {"name", "level", "message", "values": {key: text}}."""
    return {"name": NAME_PREFIX + extra, "level": level, "message": message,
            "values": {str(k): str(v) for k, v in (values or {}).items()}}


def to_message(status):
    """report() -> diagnostic_msgs/DiagnosticArray, ready to publish. Imports ROS, so call it in a node."""
    from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
    array = DiagnosticArray()
    # DiagnosticStatus.level is a ROS `byte`, which rclpy gives as Python bytes, not an int: converted both ways here.
    item = DiagnosticStatus(name=status["name"], level=bytes([status["level"]]), message=status["message"],
                            hardware_id="rocko")
    item.values = [KeyValue(key=k, value=v) for k, v in status["values"].items()]
    array.status = [item]
    return array


def from_message(msg):
    """A received DiagnosticArray -> {extra: report()} for the rocko/ entries in it (other nodes may publish
    on /diagnostics too; theirs are skipped)."""
    found = {}
    for item in msg.status:
        if not item.name.startswith(NAME_PREFIX):
            continue
        extra = item.name[len(NAME_PREFIX):]
        level = item.level[0] if isinstance(item.level, (bytes, bytearray)) else int(item.level)
        found[extra] = report(extra, level, item.message, {kv.key: kv.value for kv in item.values})
    return found


def summary(reports, ages_s):
    """What a reader shows for each extra: {extra: {"state", "message", "values"}}. state is "OK", "WARN",
    "ERROR" or "NOT RUNNING" (never heard, or no report for STALE_AFTER_S). reports: {extra: report()};
    ages_s: {extra: seconds since its newest report}."""
    out = {}
    for extra in EXTRAS:
        status, age = reports.get(extra), ages_s.get(extra)
        if status is None or age is None or age > STALE_AFTER_S:
            # An old "OK" is not repeated: the extra may have died since.
            out[extra] = {"state": "NOT RUNNING", "message": "no report: not started, or it crashed (see the launch log)",
                          "values": {}}
        else:
            out[extra] = {"state": LEVEL_NAMES.get(status["level"], "ERROR"), "message": status["message"],
                          "values": status["values"]}
    return out
