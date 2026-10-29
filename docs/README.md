# ROCKO documentation

Start with the [root README](../README.md) — it has what ROCKO is, the signal path from gamepad to
wheel, the folder map, and the build and run instructions. This directory is the depth behind it.

| Directory | What's in it |
| --- | --- |
| [`architecture/`](architecture/) | How a subsystem works, and **why it is built that way** |
| [`guides/`](guides/) | Procedures — do this, get that |
| [`decisions/`](decisions/) | Architecture Decision Records: choices, alternatives, consequences |
| [`journal/`](journal/README.md) | One entry per AI-written change: the goal, what changed, and which checks **actually ran** |
| [`proposals/`](proposals/README.md) | Changes waiting on a human decision: PID gains, version pins, safety behavior |
| [`claude-analysis/`](claude-analysis/) | AI-produced audits of the code, with findings and severities |

## If you are new

1. [Root README](../README.md) — the signal path diagram is the fastest way to understand the robot.
2. [`architecture/control-cascade.md`](architecture/control-cascade.md) — why "go faster" means
   "lean further", and why the loops are nested in that order.
3. [`decisions/0001-version-lock.md`](decisions/0001-version-lock.md) — why the ROS 2 version is not
   negotiable, and what breaks if you use the wrong one.
4. [`claude-analysis/balance-loop-findings.md`](claude-analysis/balance-loop-findings.md) — the honest
   current state: what is wrong with the balance loop today and in what order to fix it.

## If you are reviewing AI-written changes

- [`decisions/0005-ai-forward-development.md`](decisions/0005-ai-forward-development.md) — how this
  branch is worked on and why.
- [`architecture/verification-ladder.md`](architecture/verification-ladder.md) — what the AI can
  check, and what it cannot. Read this before trusting any "verified" claim.
- [`journal/`](journal/README.md) — what each change did and which checks ran.
- [`proposals/`](proposals/README.md) — decisions the AI left for you.

## If you want to know what to work on

- [`next-steps.md`](next-steps.md) — the prioritized work queue: the order, who can do each item,
  and what can check it. Start here when you have no specific goal.

## If you are about to change something

- [`guides/verify-in-container.md`](guides/verify-in-container.md) — check a change on your laptop
  with no robot.
- [`guides/reproducing-results.md`](guides/reproducing-results.md) — record a result so someone else
  can get the same number.
- [`decisions/README.md`](decisions/README.md) — write an ADR when the answer to "why is it like
  this?" isn't obvious from the code.

## House rules

- A behavior change is not done until its docs change in the same commit.
- Write for a capstone student in week one. Define jargon on first use.
- **Say what is not true.** Documenting an intention as a fact costs the next reader an afternoon.
  Conventions are in [`.claude/skills/write-docs/SKILL.md`](../.claude/skills/write-docs/SKILL.md).
