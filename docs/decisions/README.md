# Architecture Decision Records

An ADR records **one decision**: what was chosen, what else was considered, and what it costs. They
exist so that the next person — often you, eight months later — does not have to reverse-engineer
intent from code.

Write one when the answer to *"why is it like this?"* is not obvious from reading the code: a version
pin, a framework choice, a rejected alternative, a deliberate limitation.

## Conventions

- One decision per file, named `NNNN-short-slug.md`, numbered in order.
- **Never renumber and never delete.** A decision that turned out wrong gets `Status: Superseded by
  ADR-NNNN` and stays, because the reasoning is still useful.
- `Status` is `Proposed`, `Accepted`, `Superseded by ADR-NNNN`, or `Rejected`. Something that is only
  an intention stays **`Proposed`** — do not mark an unimplemented plan `Accepted`.
- Cite code as `path/to/file.cpp:42`.

## Index

| ADR | Decision | Status |
| --- | --- | --- |
| [0001](0001-version-lock.md) | ROS 2 Jazzy and ros2_control 4.x are pinned | Accepted |
| [0002](0002-cascaded-balance-control.md) | Balance via nested velocity → pitch PID cascade | Accepted |
| [0003](0003-simulation-target.md) | Simulate with `gz_ros2_control` for off-robot development | Proposed |
| [0004](0004-device-io-in-cpp.md) | Device I/O belongs in C++ hardware plugins, not Python nodes | Proposed |
| [0005](0005-ai-forward-development.md) | Develop with an AI-forward agent loop (the `ai-forward` branch) | Proposed |

## Template

```markdown
# ADR NNNN — <the decision, as a statement>

- **Status:** Proposed | Accepted | Superseded by ADR-NNNN | Rejected
- **Date:** YYYY-MM-DD
- **Deciders:** <names>

## Context

What situation forces a choice? Constraints, including the educational ones. What does a reader need
to know to judge this fairly?

## Decision

What we are doing, stated plainly.

## Alternatives considered

Each one, with why it was not chosen. An ADR with no alternatives is a note, not a decision.

## Consequences

- **Good:** what this buys.
- **Bad:** what it costs. Be honest — this section is the reason ADRs are worth writing.
- **Follow-on work:** what now has to happen.

## References

Code (`file:line`), issues, external docs.
```
