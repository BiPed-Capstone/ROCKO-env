# Run journal — what the AI did, and what it actually checked

One file per goal that changed behavior. It is the record a human reads to decide whether to trust a
change, and the thing that lets a stranger reproduce it. It is also the explanation trail: *why* the
change exists, in prose, next to the commit.

**Write one** for any change to code, launch files, URDF, build config or tooling. **Skip it** for
typo fixes and for documentation-only changes.

The rule that matters: **the journal records tiers that ran, not tiers that exist.** Each line under
Verification names a command and what it printed. If a tier was not run, the line says *not run* and
why. See the [verification ladder](../architecture/verification-ladder.md) for what each tier proves
and what it cannot.

## Template

Copy to `YYYY-MM-DD-short-slug.md`.

```markdown
# <Goal, as a sentence>

- **Date / commit:** YYYY-MM-DD, built on <base sha> (an entry cannot name the commit that adds
  it; say "the commit that adds this file")
- **Branch:** <branch>
- **Environment:** container image / Pi / laptop; ROS distro; anything non-default

## Goal and success criteria

What the human asked for, then the *checkable* statements that mean it is done.
1. ...

## Plan

Files to touch and the order. Which tier will prove each criterion. Risks.

## What changed

File → one line on what and why. Link the ADR or doc that explains it.

## Verification

| Tier | Command | Result |
| --- | --- | --- |
| 0 static | `verify_urdf.sh`; hook tests | pass / FAIL / not run (why) |
| 1 build | `build_ros.sh` | ... |
| 2 mock | `verify_mock.sh`, `verify_mock.sh tank` | ... |
| 3 sim | | see the ladder |

Warnings read and what they meant.

## Review rounds

| Round | Reviewer | Findings | Fixed / declined (why) |
| --- | --- | --- | --- |

## Not verified — needs a human

What this does NOT prove, and the procedure for a person to test it on the robot (tethered; success
looks like ...; abort if ...). Or a link to the proposal that holds it.

## Reproduce

Exact commands a stranger runs to get the same result.
```

## Index

| Date | Goal |
| --- | --- |
| 2026-10-27 | [Make the AI-forward loop real: mock hardware, off-Pi build, verify script](2026-10-27-ai-forward-bootstrap.md) |
