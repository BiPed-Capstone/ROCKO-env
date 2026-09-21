---
name: write-docs
description: Conventions for writing and updating ROCKO documentation in docs/ — architecture notes, how-to guides, ADRs, and README updates. Use when asked to document something, write a guide or README, record a decision, explain how part of the robot works, or after any change that alters behavior a user would notice.
---

# Writing ROCKO documentation

This is an educational platform. The docs are a deliverable on the same level as the code, and a
change without its docs is not finished.

## Where things go

| Directory | Holds | Test for "does it belong here" |
| --- | --- | --- |
| `README.md` (root) | Front door: what ROCKO is, the signal path, quick build/run | Would a stranger's first 15 minutes need it? |
| `docs/architecture/` | How a subsystem works and **why it is that way** | Explains, doesn't instruct |
| `docs/guides/` | Do-this-get-that procedures | Has numbered steps and a success check |
| `docs/decisions/` | ADRs — a choice, its alternatives, its consequences | Someone will later ask "why on earth did they…" |
| `docs/claude-analysis/` | AI-produced audits and findings | Dated, with severities and `file:line` |

Keep the README as the front door and **link down** into `docs/` rather than duplicating. If you
find yourself copying a section, move it to `docs/` and leave a link.

## House style

- **Write for a capstone student in week one.** Assume one controls course and no ROS.
- **Define jargon on first use**, in line. The README's "xacro is XML with macros" is the model.
- **Never write "simply", "just", or "obviously".** If a step is easy, it needs no adjective; if it
  isn't, the word is a lie.
- **Say what is *not* true.** The README's note that `use_mock_hardware` "currently does nothing at
  all" is the standard. Documenting an aspiration as a fact is the worst failure mode available here —
  it costs a student a whole afternoon.
- **Prefer a table or an ASCII diagram** over a paragraph, for anything with more than three parts.
- **Cite the code.** `rocko_controllers.yaml:3` beats "the config file". Paths from the repo root.
- Explain the *why* alongside the *what*. "`--symlink-install` links instead of copying, so you can
  edit a gain and re-run without rebuilding" teaches; "use `--symlink-install`" does not.

## Every procedure must be complete

A guide is broken unless it states:

1. **Which machine** — your laptop, the Pi, or inside the Docker container. These are genuinely
   different here, and the gamepad script runs on a *different machine* from everything else.
2. **What to source first** — `source /opt/ros/jazzy/setup.bash`, then the workspace overlay. Every
   new terminal. Say it every time.
3. **Which directory** — workspace root vs. `src/ROCKO-env` matters for `colcon`.
4. **What success looks like** — the expected output, not just the command.
5. **What goes wrong** — the common failure and its cause.

Copy-pasteable blocks, one command per line, no `$` prefix, no interleaved output.

## ADRs

One decision per file, `docs/decisions/NNNN-short-slug.md`, numbered in order, never renumbered.
Template is in [`docs/decisions/README.md`](../../../docs/decisions/README.md). Status is
`Proposed`, `Accepted`, `Superseded by ADR-NNNN`, or `Rejected` — a decision that is only an
intention stays `Proposed`. Write one whenever the answer to "why is it like this?" is not obvious
from the code: a version pin, a framework choice, a rejected alternative.

## Reproducibility

Any doc that reports a **result** — a gain set, a calibration, a latency number — must carry the
context to reproduce it: commit SHA, launch file and config, battery voltage, surface, log location.
Format in [`docs/guides/reproducing-results.md`](../../../docs/guides/reproducing-results.md). A
number without those is a rumor.

## Before you call it done

- Every claim traced to code you actually read, or explicitly marked as untested.
- Every link resolves; every path exists.
- Nothing contradicts the README (or the README was updated in the same change).
- New pages linked from their directory's index, and from the README if a newcomer needs them.
- Run the `docs-reviewer` subagent — it checks exactly the above.
