#!/usr/bin/env python3
"""Mutation check for the git hooks' tests. Run it after changing a hook or its test:

    .githooks/mutation-check.py

A test that has only ever passed has not shown it can fail. This breaks one check at a time in a
COPY of this folder (your working tree is never touched), runs test-git-hooks.sh from the copy, and
requires it to go red. It first runs the test unbroken: if that is not green, nothing below would
mean anything, so it stops. It prints which cases failed.
  - caught          the test failed, so some case can see this break
  - NOT CAUGHT      the test stayed green, so a case is missing
  - TEST BROKE      the test failed without a single FAIL line, so it crashed rather than noticed
  - did not apply   the text to break is no longer in the hook; update MUTATIONS below
Exit status:  0 = every break was caught   1 = at least one was not   2 = the unbroken test is not green
Needs only python3, git and bash.
"""
import os
import shutil
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
ESCAPE = '[ "${ROCKO_HUMAN:-}" = 1 ] && exit 0'
FAST_FORWARD = 'if [ "$remote_sha" != "$no_commit" ] && ! git merge-base'

# (what is broken, which hook, the exact text to break, what to break it into)
MUTATIONS = [
    ("pre-push: any ref name is allowed", "pre-push",
     '"$remote_ref" != refs/heads/ai-forward', '"$remote_ref" != "$remote_ref"'),
    # Plausible "simplification": match the branch by its ending. It lets a tag named ai-forward through.
    ("pre-push: ref matched by its ending", "pre-push",
     'if [ "$remote_ref" != refs/heads/ai-forward ]; then',
     'if [ "${remote_ref%ai-forward}" = "$remote_ref" ]; then'),
    ("pre-push: any remote is allowed", "pre-push",
     'if [ "$1" != origin ]; then', 'if false; then'),
    ("pre-push: a deletion is not recognised as one", "pre-push",
     'if [ "$local_ref" = "(delete)" ]; then', 'if false; then'),
    ("pre-push: no fast-forward check", "pre-push",
     FAST_FORWARD, 'if false && ! git merge-base'),
    # Plausible "tidy-up": check the commit exists before merge-base, to quiet its "bad object" error.
    # It waves through a force over commits we never fetched.
    ("pre-push: refuses only when we have the remote's commit", "pre-push",
     FAST_FORWARD,
     'if [ "$remote_sha" != "$no_commit" ] && git cat-file -e "$remote_sha" 2>/dev/null && ! git merge-base'),
    ("pre-push: a refusal that forgets its exit 1", "pre-push",
     'not $remote_ref" >&2\n    exit 1', 'not $remote_ref" >&2'),
    ("pre-push: the person's ROCKO_HUMAN escape removed", "pre-push", ESCAPE, 'true'),
    ("pre-push: any ROCKO_HUMAN value is the escape", "pre-push",
     ESCAPE, '[ -n "${ROCKO_HUMAN:-}" ] && exit 0'),
    ("pre-commit: main is not detected", "pre-commit", '"$branch" = main', '"$branch" = nomatch'),
    ("pre-commit: the person's ROCKO_HUMAN escape removed", "pre-commit", ESCAPE, 'true'),
    ("pre-commit: any ROCKO_HUMAN value is the escape", "pre-commit",
     ESCAPE, '[ -n "${ROCKO_HUMAN:-}" ] && exit 0'),
]


def run_test(folder):
    """Run folder/test-git-hooks.sh. Returns its exit status and its FAIL lines (not the summary)."""
    run = subprocess.run([os.path.join(folder, "test-git-hooks.sh")], stdin=subprocess.DEVNULL,
                         capture_output=True, text=True, timeout=300)
    return run.returncode, [line for line in run.stdout.splitlines() if line.startswith("FAIL ")]


def main():
    # One copy for the whole run; each break is written into it and then undone. The with-block
    # deletes the copy on the way out, also after an error or Ctrl-C.
    with tempfile.TemporaryDirectory(prefix="hook-mutation.") as work:
        copy = os.path.join(work, ".githooks")
        shutil.copytree(HERE, copy)
        # If the test is red before anything is broken, every break below would look "caught".
        rc, red = run_test(copy)
        if rc != 0:
            print(f"BASELINE is not green (exit {rc}), so the results would mean nothing:")
            for line in red or ["(no FAIL lines: the test itself broke)"]:
                print(f"    {line}")
            return 2
        print("BASELINE (nothing broken): green")
        all_caught = True
        for name, hook, old, new in MUTATIONS:
            path = os.path.join(copy, hook)
            with open(path, encoding="utf-8") as f:
                original = f.read()
            if old not in original:
                print(f"did not apply   {name}")
                all_caught = False
                continue
            with open(path, "w", encoding="utf-8") as f:
                f.write(original.replace(old, new, 1))
            rc, red = run_test(copy)
            with open(path, "w", encoding="utf-8") as f:   # undo, so the next break starts clean
                f.write(original)
            if rc != 0 and red:
                print(f"caught          {name}: {len(red)} case(s) red")
                for line in red:
                    print(f"                  {line}")
            elif rc != 0:
                print(f"TEST BROKE      {name}: exit {rc} and no FAIL line")
                all_caught = False
            else:
                print(f"NOT CAUGHT      {name}")
                all_caught = False
    print()
    print("PASS: every break was caught." if all_caught else "FAIL: see above.")
    return 0 if all_caught else 1


if __name__ == "__main__":
    sys.exit(main())
