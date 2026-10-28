#!/usr/bin/env python3
"""Mutation check for the guard hook's tests. Run it after changing the hook or its tests:

    PYTHONDONTWRITEBYTECODE=1 .claude/hooks/mutation-check-guard.py

A test suite that has only ever passed has not shown it can fail. This reintroduces one known bug at
a time into a COPY of guard_protected_files.py (your working tree is never touched), runs
test-guard-protected-files.sh against the copy, and requires the suite to go red. It reports:
  - caught        the suite failed, so some test can see this bug
  - blind spot    the suite stayed green, so a test is missing
  - invalid       the bug's text is no longer in the hook, so the mutation did not apply; update
                  MUTATIONS below to match the hook (the result says nothing about the tests)
Exit status is 0 only if every mutation was caught. Needs only python3 and bash.
"""
import os
import shutil
import subprocess
import sys
import tempfile

REPO = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
HOOK = ".claude/hooks/guard_protected_files.py"

# (what the bug is, the exact text in the hook to break, what to break it into)
MUTATIONS = [
    # --- failing open / reading the file
    ("hook crashes instead of failing closed",
     "    try:\n        payload = json.load(sys.stdin)\n        check(payload.get(\"tool_name\") or \"\", payload.get(\"tool_input\") or {})\n    except Exception as err:",
     "    if True:\n        payload = json.load(sys.stdin)\n        check(payload.get(\"tool_name\") or \"\", payload.get(\"tool_input\") or {})\n    if False:\n        err = None"),
    ("non-UTF-8 bytes crash the read", 'errors="surrogateescape"', 'errors="strict"'),
    ("an unreadable file is read as new",
     'with open(abs_path, encoding="utf-8", errors="surrogateescape", newline="") as f:\n        return text_of(f.read())',
     'try:\n        with open(abs_path, encoding="utf-8", errors="surrogateescape", newline="") as f:\n            return text_of(f.read())\n    except OSError:\n        return ""'),
    ("CRLF is not normalised", 'return value.replace("\\r\\n", "\\n") if isinstance(value, str) else ""', 'return value if isinstance(value, str) else ""'),
    # --- which file is it?
    ("paths are not case-folded", ".casefold()", ""),
    ("symlinks are not resolved", "real = os.path.realpath(abs_path).casefold()", "real = os.path.abspath(abs_path).casefold()"),
    (".git/config and hooks are not guardrails", r"|\.git/(config$|hooks/|modules/.*/(config$|hooks/))", ""),
    ("a submodule's git hooks are not a guardrail", r"|modules/.*/(config$|hooks/)", ""),
    ("'sim' matches inside any word", r"(^|[_.-])sim([_.-]|$)", "sim"),
    ("gain files in a subfolder of config/ are not guarded", r'r"bringup/config/.*\.ya?ml$"', r'r"bringup/config/[^/]*\.ya?ml$"'),
    ("safety path anchored to the project root only", 'r"(^|/)rocko_env/rocko_env/"', 'r"^rocko_env/rocko_env/"'),
    # --- what an edit does to the file
    ("an edit that does not match is waved through", "            unmatched = True       # the tool may match more loosely than we do (curly quotes...)", "            pass"),
    ("an empty old_string does not create the file", "            if not after:          # an empty old_string on an empty/new file means \"create it\"\n                after = new", "            pass"),
    ("the Edit tool's newline-eating is not modelled", "    return current, ([plain] if eaten == plain else [plain, eaten]), unmatched", "    return current, [plain], unmatched"),
    # --- comments and lines
    ("CMake line comments are not stripped", 'CMAKE_COMMENT = re.compile(r"#.*$")', 'CMAKE_COMMENT = re.compile(r"(?!)")'),
    ("CMake #[[ ]] comments are not stripped", 'BRACKET_OPEN = re.compile(r"#\\[(=*)\\[")', 'BRACKET_OPEN = re.compile(r"(?!)")'),
    ("CMake #[[ ]] comments are stripped with a quadratic regex",
     "        text = strip_bracket_comments(text)", "        text = re.sub(r\"#\\[(=*)\\[.*?\\]\\1\\]\", \"\", text, flags=re.S)"),
    ("YAML '#' always starts a comment (hides a second p)", 'YAML_COMMENT = re.compile(r"(^|\\s)#.*$")', 'YAML_COMMENT = re.compile(r"#.*$")'),
    ("a lone CR / NEL / LS / PS is not a YAML line break", 'YAML_LINE_BREAK = re.compile(r"[\\r\\n\\x85\\u2028\\u2029]")', 'YAML_LINE_BREAK = re.compile(r"\\n")'),
    ("lines split with splitlines() (a form feed hides a pin)", 'if comment_kind == "yaml" else text.split("\\n")', 'if comment_kind == "yaml" else text.splitlines()'),
    # --- how two lines are compared
    ("a gain is a token, not a whole line", "found.append(comparable(line, comment_kind))", "found.append(regex.search(line).group(0).lower())"),
    ("YAML is compared case-insensitively with all spacing deleted", '    if comment_kind == "yaml":\n        indent', '    if False:\n        indent'),
    ("YAML indentation is ignored", 'indent = line[:len(line) - len(line.lstrip(" "))]', 'indent = ""'),
    ("additions also count as pin changes", "lost = Counter(lines_with(pins, old, kind)) - Counter(lines_with(pins, new, kind))",
     "lost = (Counter(lines_with(pins, old, kind)) - Counter(lines_with(pins, new, kind))) + (Counter(lines_with(pins, new, kind)) - Counter(lines_with(pins, old, kind)))"),
    # --- which words and keys count
    ("'fault' matches inside 'default'", "(?<!de)fault", "fault"),
    ("'fault' is not counted when camelCased (imuFault)", "(?<!de)fault", "(?<![a-z])fault"),
    ("'limiter' is not a safety word", "limiter|saturat|", ""),
    ("limit switches are not gains", r"has_(?:velocity|acceleration|jerk)_limits|", ""),
]


def run_suite(root):
    result = subprocess.run([os.path.join(root, ".claude/hooks/test-guard-protected-files.sh")],
                            capture_output=True, text=True, timeout=300)
    return [line[:120] for line in result.stdout.splitlines() if line.startswith("FAIL")], result.returncode


def main():
    work = tempfile.mkdtemp()
    copy = os.path.join(work, "repo")
    shutil.copytree(REPO, copy, symlinks=True,
                    ignore=shutil.ignore_patterns(".git", "build", "install", "log", "__pycache__"))
    pristine = open(os.path.join(copy, HOOK)).read()

    fails, rc = run_suite(copy)
    if fails or rc != 0:
        print("BASELINE is not green, so the mutation results would mean nothing:", fails)
        sys.exit(2)
    print("BASELINE (unmutated copy): green")

    blind, invalid = [], []
    for label, old, new in MUTATIONS:
        if old not in pristine:
            print(f"=== {label}\n   invalid: the text to break is not in the hook any more")
            invalid.append(label)
            continue
        with open(os.path.join(copy, HOOK), "w") as f:
            f.write(pristine.replace(old, new, 1))
        fails, rc = run_suite(copy)
        if not fails and rc == 0:
            print(f"=== {label}\n   BLIND SPOT: the suite stayed green, so no test can see this bug")
            blind.append(label)
        else:
            print(f"=== {label}\n   caught by {len(fails)} case(s), e.g. " + " | ".join(s[6:70] for s in fails[:2]))
    shutil.rmtree(work, ignore_errors=True)

    caught = len(MUTATIONS) - len(blind) - len(invalid)
    print(f"\n{caught} of {len(MUTATIONS)} mutations caught; blind spots: {blind or 'none'}; invalid: {invalid or 'none'}")
    sys.exit(1 if blind or invalid else 0)


main()
