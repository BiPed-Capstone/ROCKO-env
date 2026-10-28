#!/usr/bin/env python3
"""ROCKO-env PreToolUse hook (ai-forward branch). Run via guard-protected-files.sh.

On this branch the AI works without stopping to ask. A few things still need a human, because
the AI cannot verify them:

  1. Hardware gains  PID gains, integral clamps, limits and update_rate in bringup/config/*.yaml
                     are experimental results from one robot, surface and battery voltage.
  2. Version pins    Jazzy, ros2_control >= 4.0.0, C++17, CMake >= 3.16 (docs/decisions/0001).
  3. Safety words    An edit that leaves FEWER mentions of tipover / watchdog / clamp / fault ...
                     than the file had. It counts words; it does not understand code.
  4. The guardrails  This hook, settings.json, and the files that can run code behind them.

For 1-3 the hook DENIES the edit and says what to do instead (write a proposal in
docs/proposals/), so an unattended run carries on rather than waiting on a prompt nobody answers.
For 4 it ASKS: a human can approve a change to the rules, and a headless run is refused.

THE HOOK FAILS CLOSED. If it crashes for any reason (a huge file, odd bytes, an unreadable file, a
malformed payload) it ASKS rather than stay silent, because the harness treats a hook that exits
non-zero as "no opinion" and lets the edit through. That is why main() catches everything.

HOW IT JUDGES AN EDIT. It works out the WHOLE file as it would be after the edit and compares
it with the file now, not just the edited fragment: an edit that only swaps "4.0.0" for "5.0.0" has
no "controller_manager" in its old_string, but the file as a whole shows the pin moving.
  - Gains (rule 1) and pins (rule 2) are compared as whole LINES. A line that mentions a gain key
    or a pin is reduced to a comparable form (comment removed, spacing normalised), so
    reformatting is not a change but renaming a key, swapping values between two controllers, or
    editing a pin line is. What "normalised" means depends on the language: see comparable().
  - For gains ANY difference counts, including adding a gain: a new bringup/config/foo.yaml with
    gains is denied (put sim gains in foo_sim.yaml, which is not guarded).
  - For pins only DISAPPEARANCES count: a pin line the file had that is gone or changed. So
    creating a second CMakeLists.txt, or adding a source file, is fine.
  - The Edit tool deletes a line's newline along with the text when new_string is empty, which can
    glue the next line onto a comment and silence it. The hook judges BOTH outcomes.

LIMITS, stated plainly. This is a tripwire, not a proof.
  - It watches Edit, Write and MultiEdit. It does not watch Bash (`sed -i`, `git diff --output`),
    nor anything on the settings.json allow-list that can write files or run code.
  - It knows the gain keys, pin patterns and safety words listed below, and nothing else.
  - GAIN PROTECTION IS LINE-BASED, NOT A YAML PARSER. It does not see structure that only a parser
    can: a quoted key (`{"p": 5}`), `? p` explicit keys, a `gains: |` block scalar that turns the map
    into a string, or an edit that moves a line out of its map. Leading indentation IS compared, so
    de-indenting a gain line is caught, but it cannot tell a harmless re-indent from a structural
    one. A parser-based check (PyYAML) would close this; it is not here because PyYAML may not be
    installed on the host, and the hook must work without it.
  - Wrapping an unchanged pin line in `if(FALSE)`, or ADDING something that overrides a pin (a
    later FROM line, `add_compile_options(-std=c++11)`), is not caught: that takes parsing CMake
    and Docker, not reading lines. Whitespace inserted INSIDE a token (`cxx_std_1 7`) usually just
    breaks the build and is not caught either.
  - A launch file can load a different YAML or override update_rate; the ros2-control-reviewer
    agent checks that, this hook does not.
  - CLAUDE.md, .claude/agents/ and .claude/skills/ are not guarded; a human reads their diffs.
  - The safety check is a word count. It can be offset by adding a comment that repeats the word.
  - A git worktree OUTSIDE the project directory is outside its view (inside it is covered).
  - If python3 is missing, or guard-protected-files.sh loses its execute bit, the hook cannot run
    at all and the edit proceeds.
Tests: .claude/hooks/test-guard-protected-files.sh   Mutation check: .claude/hooks/mutation-check-guard.py
"""
import json
import os
import re
import stat
import sys
from collections import Counter

MAX_BYTES = 8 * 1024 * 1024   # a file this big is not a source file; ask rather than read it

PROPOSE = (
    "Do not stop. Write what you want to change to docs/proposals/NNNN-<slug>.md "
    "(template in docs/proposals/README.md): the change, why, the highest verification tier "
    "you ran, and the test a human should do on the robot. Then carry on with the rest of the task."
)

# --- 4. The guardrails (checked first; needs no file contents) --------------------------------
# .git/config and .git/hooks run code on a plain `git status` (so do a submodule's); .mcp.json and
# .claude/commands add tools and commands. Anything here can change what the agent may do.
GUARDRAIL = re.compile(
    r"(^|/)(\.claude/(hooks/|commands/|settings(\.local)?\.json$)"
    r"|\.git/(config$|hooks/|modules/.*/(config$|hooks/))|\.mcp\.json$)"
)

# --- 1. Gains: any line that has one of these keys on it ---------------------------------------
GAIN = re.compile(
    r"\b(?:p|i|d|i_clamp_max|i_clamp_min|u_clamp_max|u_clamp_min|feedforward_gain|"
    r"tracking_time_constant|antiwindup\w*|update_rate|cmd_vel_timeout|"
    r"has_(?:velocity|acceleration|jerk)_limits|(?:max|min)_(?:velocity|acceleration|jerk))\s*:",
    re.I,
)

# --- 2. Version pins: (which files, what a pin line looks like, how that file writes comments) --
PINS = [
    (r"(^|/)cmakelists\.txt$",
     r"find_package\s*\(\s*controller_manager\b|controller_manager_found|cxx_std_\d+|"
     r"cmake_cxx_standard|cxx_standard\s|-std=|cmake_minimum_required\s*\(",
     "cmake"),
    (r"(^|/)dockerfile$",
     r"^\s*from\s|ubuntu[:\s-]*\d+\.\d+|ros-(?:jazzy|humble|iron|rolling)|/opt/ros/\w+",
     "cmake"),
    (r"(^|/)package\.xml$", r"version_(?:gte|gt|lte|lt|eq)\s*=", None),
]

# --- 3. Safety words. `fault` must not match inside "default"/"Defaults" but must match "imuFault"
SAFETY = re.compile(
    r"tipover|watchdog|cutoff|fail[_-]?safe|clamp|(?<!de)fault|safe_state|limiter|saturat|"
    r"return_type::error",
    re.I,
)

# How each language starts a comment and ends a line.
#  - YAML: '#' is a comment only at the start of a line or after a space, so `{k: x#y, p: 99.0}`
#    keeps its '#' and its p. A comment ends at ANY YAML line break: LF, a lone CR, NEL, LS, PS (a
#    lone CR ends it in libyaml, so text after one is live YAML, not comment).
#  - CMake: any '#' starts a comment, '#[[ ... ]]' spans lines, and a line ends at "\n" only.
#  - Dockerfile: handled like CMake. Stripping generously can only hide a pin from BOTH sides of
#    the comparison, never invent a difference.
YAML_COMMENT = re.compile(r"(^|\s)#.*$")
YAML_LINE_BREAK = re.compile(r"[\r\n\x85\u2028\u2029]")
CMAKE_COMMENT = re.compile(r"#.*$")
BRACKET_OPEN = re.compile(r"#\[(=*)\[")


def decide_and_exit(decision, reason):
    """Print the verdict and stop. Never returns."""
    print(json.dumps({"hookSpecificOutput": {
        "hookEventName": "PreToolUse",
        "permissionDecision": decision,
        "permissionDecisionReason": reason,
    }}))
    sys.exit(0)


def text_of(value):
    """A string with LF line endings; anything that is not a string counts as empty.

    CRLF files exist in this tree even though .gitattributes says LF, and a multi-line
    old_string would otherwise fail to match them and make an edit look like a no-op.
    """
    return value.replace("\r\n", "\n") if isinstance(value, str) else ""


def read_current(abs_path):
    """The file's text, "" if it does not exist yet. Anything odd raises, and main() then asks."""
    try:
        info = os.stat(abs_path)
    except FileNotFoundError:
        return ""  # a new file
    # A FIFO would block forever; a huge file is not source. (An unreadable file raises
    # PermissionError from open() below, which is also an "ask", not "treat it as new".)
    if not stat.S_ISREG(info.st_mode) or info.st_size > MAX_BYTES:
        raise ValueError("not a regular file, or too big to judge")
    # surrogateescape: a stray non-UTF-8 byte (a latin-1 degree sign in a comment) must not crash us.
    with open(abs_path, encoding="utf-8", errors="surrogateescape", newline="") as f:
        return text_of(f.read())


def apply_edits(current, edits, eat_newline):
    """(the text after applying `edits` in order, whether any old_string failed to match).

    eat_newline models the Edit tool's habit: when new_string is empty and old_string does not
    end in a newline, it deletes the newline that follows old_string too.
    """
    after, unmatched = current, False
    for e in edits:
        old, new = text_of(e.get("old_string")), text_of(e.get("new_string"))
        if not old:
            if not after:          # an empty old_string on an empty/new file means "create it"
                after = new
            continue
        if old not in after:
            unmatched = True       # the tool may match more loosely than we do (curly quotes...)
            continue
        if eat_newline and not new and not old.endswith("\n") and (old + "\n") in after:
            old = old + "\n"
        # The Edit tool replaces the first match (old_string must be unique), or all with replace_all.
        after = after.replace(old, new) if e.get("replace_all") else after.replace(old, new, 1)
    return after, unmatched


def before_and_after(tool, tool_input, abs_path):
    """(the file now, every form it could take after this call, whether an edit failed to match)."""
    current = read_current(abs_path)
    if tool == "Write":
        return current, [text_of(tool_input.get("content"))], False
    if tool == "MultiEdit":
        edits = tool_input.get("edits")
        edits = edits if isinstance(edits, list) else []
    else:  # Edit
        edits = [tool_input]
    plain, unmatched = apply_edits(current, edits, eat_newline=False)
    eaten, _ = apply_edits(current, edits, eat_newline=True)
    return current, ([plain] if eaten == plain else [plain, eaten]), unmatched


def strip_bracket_comments(text):
    """Remove CMake's #[[ ... ]] (and #[=[ ... ]=]) comments, which can span lines.

    A linear scan on purpose: a regex like `#\\[(=*)\\[.*?\\]\\1\\]` is quadratic on a file with
    many openers and no closers (400 KB took 84 s), and a hook that is slow enough to be killed is
    a hook that lets the edit through.
    """
    kept, i = [], 0
    while True:
        opener = BRACKET_OPEN.search(text, i)
        if not opener:
            kept.append(text[i:])
            return "".join(kept)
        kept.append(text[i:opener.start()])
        closer = "]" + opener.group(1) + "]"
        end = text.find(closer, opener.end())
        if end < 0:                # never closed: the rest of the file is comment
            return "".join(kept)
        i = end + len(closer)


def comparable(line, comment_kind):
    """A line reduced to the form in which two lines are 'the same'.

    YAML keeps case and the space after ':' (key `P` is not `p`, and `{p:10.0}` is the key
    "p:10.0", not p = 10.0), keeps leading indentation (it is structure), and ignores only spacing
    that YAML ignores: runs of spaces, spaces before ':' and around , { } [ ].
    Everything else (CMake, Docker, XML) ignores case and all whitespace.
    """
    if comment_kind == "yaml":
        indent = line[:len(line) - len(line.lstrip(" "))]
        body = re.sub(r"\s+", " ", line.strip())
        body = re.sub(r" ?([,{}\[\]]) ?", r"\1", body)
        return indent + re.sub(r" +:", ":", body)
    return re.sub(r"\s+", "", line).lower()


def lines_with(regex, text, comment_kind=None):
    """The lines of `text` in which `regex` finds something, each reduced by comparable()."""
    if comment_kind == "cmake":
        text = strip_bracket_comments(text)
    # str.splitlines() splits on form feeds and more, which CMake and XML do not treat as line
    # ends; and YAML ends a line at a lone CR, NEL, LS or PS where CMake does not. So split by hand.
    lines = YAML_LINE_BREAK.split(text) if comment_kind == "yaml" else text.split("\n")
    found = []
    for line in lines:
        if comment_kind == "cmake":
            line = CMAKE_COMMENT.sub("", line)
        elif comment_kind == "yaml":
            line = YAML_COMMENT.sub("", line)
        if regex.search(line):
            found.append(comparable(line, comment_kind))
    return found


def safety_words(text):
    """How many times each safety word is mentioned (comments included: a comment is a record)."""
    return Counter(m.group(0).lower() for m in SAFETY.finditer(text))


def judge(abs_path, old, new, is_gain_file, pin_rule, is_safety_file):
    """Apply rules 1-3 to one possible outcome of the edit. Exits (deny) on a violation."""
    # 1. Hardware gains and the loop rate. ANY difference in the gain lines counts, a swap between
    # controllers and a newly added gain included.
    if is_gain_file and Counter(lines_with(GAIN, old, "yaml")) != Counter(lines_with(GAIN, new, "yaml")):
        decide_and_exit("deny", f"{abs_path}: this edit changes a PID gain, a limit or the "
                        "update_rate. These are experimental results measured on one robot, "
                        "surface and battery voltage; you cannot verify them, and sim-tuned "
                        "values do not transfer. Gains found in simulation belong in a "
                        "separate *_sim.yaml, which is not guarded. Wiring changes "
                        f"(interfaces, new controllers without gains) are fine. {PROPOSE}")

    # 2. Version pins. Counter counts how many times each pin line appears; subtracting keeps only
    # those whose count went DOWN. So a pin that is removed, changed or commented out is "lost",
    # while adding pins (a second target, a new CMakeLists.txt) is not.
    if pin_rule:
        regex, kind = pin_rule
        pins = re.compile(regex, re.I)
        lost = Counter(lines_with(pins, old, kind)) - Counter(lines_with(pins, new, kind))
        if lost:
            decide_and_exit("deny", f"{abs_path}: this edit removes or changes a version pin line "
                            f"({', '.join(sorted(lost))}). ROS 2 Jazzy, ros2_control >= 4.0.0, "
                            "Ubuntu 24.04, C++17 and CMake >= 3.16 are locked deliberately; see "
                            "docs/decisions/0001-version-lock.md. Adding a dependency or a "
                            f"source file is fine. {PROPOSE}")

    # 3. Safety words: fewer mentions than before means a safeguard was removed or moved.
    if is_safety_file:
        lost = safety_words(old) - safety_words(new)
        if lost:
            decide_and_exit("deny", f"{abs_path}: this edit leaves fewer mentions of "
                            f"{', '.join(sorted(lost))} than the file had. A tipover cutoff, "
                            "watchdog, command clamp or error return is not removed or weakened "
                            "on sim evidence alone; the simulator cannot see a real fall or a "
                            "lost gamepad link. If this is a refactor that keeps the behavior, "
                            f"keep the safeguard and its comment. Otherwise: {PROPOSE}")


def check(tool, tool_input):
    abs_path = tool_input.get("file_path")
    if not isinstance(abs_path, str) or not abs_path:
        return

    # Resolve symlinks and fold case, so `.CLAUDE/Hooks/..` on a case-insensitive filesystem,
    # a symlink, or `/tmp` vs `/private/tmp` cannot dodge the path rules.
    root = os.path.realpath(os.environ.get("CLAUDE_PROJECT_DIR") or os.getcwd()).casefold()
    real = os.path.realpath(abs_path).casefold()
    path = os.path.relpath(real, root).replace(os.sep, "/")
    outside_project = path.startswith("../") or path == ".."

    # 4. The guardrails. Also covers another checkout's .claude/ and ~/.claude/settings.json
    # (outside the project), but not the rest of ~/.claude/ (memory files are fine to write).
    if GUARDRAIL.search(real if outside_project else path):
        decide_and_exit("ask", f"{abs_path} is part of the agent's own guardrails, or can run code "
                               "behind them. Only a human changes the rules the agent works "
                               "under. Describe the change you want and why.")
    if outside_project:
        return

    # Which rules apply to this file? If none, never read it (and never hang on it).
    name = os.path.basename(path)
    is_gain_file = (re.search(r"bringup/config/.*\.ya?ml$", path)           # any subfolder too
                    and not re.search(r"(^|[_.-])sim([_.-]|$)", name))      # `sim` as a whole word
    pin_rule = next(((regex, kind) for path_re, regex, kind in PINS if re.search(path_re, path)), None)
    is_safety_file = (re.search(r"\.(cpp|cxx|cc|hpp|hxx|hh|h|py)$", path)
                      and re.search(r"(^|/)rocko_env/rocko_env/", path))   # also inside a worktree
    if not (is_gain_file or pin_rule or is_safety_file):
        return

    old, outcomes, unmatched = before_and_after(tool, tool_input, abs_path)
    if unmatched:
        decide_and_exit("ask", f"{abs_path}: part of this edit does not match the file as the guard "
                               "hook reads it, so the hook cannot tell what it changes. A human "
                               "should look.")
    for new in outcomes:
        judge(abs_path, old, new, is_gain_file, pin_rule, is_safety_file)


def main():
    try:
        payload = json.load(sys.stdin)
        check(payload.get("tool_name") or "", payload.get("tool_input") or {})
    except Exception as err:  # noqa: BLE001 - fail closed on ANY failure; see module docstring
        decide_and_exit("ask", "The guard hook itself failed "
                        f"({type(err).__name__}: {err}), so it cannot vouch for this edit. "
                        "A human should look at it.")


if __name__ == "__main__":
    main()
