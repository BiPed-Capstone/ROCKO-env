"""Log lines other programs read to learn a state that is not published on a topic.

The tipover cutoff (AddFeedforwardController.cpp) only LOGS when it trips or re-arms. The dashboard, the screen
and the buzzer all read /rosout for these two phrases, from this one place. If the C++ log lines are reworded,
reword these too; tools/extras_test.py fails if either phrase is no longer in AddFeedforwardController.cpp.
"""
TIPOVER_TRIPPED = "TIPOVER CUTOFF TRIPPED"
TIPOVER_REARMED = "Tipover cutoff RE-ARMED"


def tipover_change(text):
    """True if this log line says the cutoff tripped, False if it re-armed, None if it says neither."""
    if TIPOVER_TRIPPED in text:
        return True
    if TIPOVER_REARMED in text:
        return False
    return None
