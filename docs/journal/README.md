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
| 2026-10-27 | [Make the AI-forward loop real: mock hardware, off-Pi build, verify scripts](2026-10-27-ai-forward-bootstrap.md) |
| 2026-10-28 | [Let the agent push `ai-forward`](2026-10-28-allow-ai-forward-push.md) |
| 2026-10-29 | [Make the verify scripts notice when they are testing the wrong checkout](2026-10-29-container-mount-check.md) |
| 2026-10-29 | [Get the repo ready for a run nobody is watching: rules, queue and git hooks](2026-10-29-unattended-run-readiness.md) |
| 2026-10-29 | [Stop asking before every push to `ai-forward`](2026-10-29-push-without-asking.md) |
| 2026-10-30 | [Apply items B, C and D of proposal 0002 to the settings](2026-10-30-apply-proposal-0002-b-d.md) |
| 2026-10-30 | [Make `acceptEdits` the project's default permission mode](2026-10-30-default-mode-accept-edits.md) |
| 2026-10-31 | [Fix C-1/C-2/C-3 (+M-5, H-4, H-6) and build Tier 3: a `SimPlant` physics simulation](2026-10-31-sim-plant-and-critical-fixes.md) (in progress: stages 2 to 4 are logged: foundations, the plant, and the tipover cutoff / teleop watchdog / pitch rate with their sim checks; stage 4: review round 1 is done and round 2 is pending; Tier 3 is only partly built) |
| 2026-11-03 | [Stage 5: find why the model drifts off (finding H-12), and write the balance scenarios before any tuning](2026-11-03-stage5-balance-in-sim.md) |
| 2026-11-03 | [Find why checks C3 and FF are flaky, and make them judge send times](2026-11-03-c3-ff-flaky-checks.md) |
| 2026-11-03 | [Stage 5c-1: tune the sim gains until scenarios U, I, V pass (thresholds unchanged)](2026-11-03-stage5c1-sim-gain-tuning.md) |
| 2026-11-03 | [Stage 5c-2: the ±30 % robustness sweep of the sim gains (mass, centre-of-mass height, motor gain, IMU delay)](2026-11-03-stage5c2-sweep.md) |
| 2026-11-04 | [Final report, stages 4 to 6: tiers reached, failures, declined findings, proposals 0003 to 0009, what was not verified, the tethered procedure, the sim's omissions](2026-11-04-final-report-stage-4-6.md) |
| 2026-11-07 | [Two noisy verifiers: scenario V's overshoot metric (limit unchanged) and check C3's second flake](2026-11-07-c3-v-verifier-noise.md) |
| 2026-11-07 | [Stage 5c-2 sweep re-run with V's corrected overshoot metric: PASS 18, MARGINAL 4, FAIL 2, COULD NOT JUDGE 1](2026-11-07-stage5c2-sweep-rerun.md) |
