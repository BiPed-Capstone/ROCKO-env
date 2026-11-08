---
name: orchestrate-run
description: Playbook for the agent that orchestrates a long ROCKO run with subagents - which model for which job, how to brief an agent, how to wait, when to start a fresh session. Use when continuing a multi-stage run from a journal handoff, when about to launch more than one subagent, or when asked about cost or efficiency of the loop.
---

# Orchestrating a run cheaply

Cost of a run is roughly *number of model calls x size of the conversation each call re-reads*
(96 % of tokens in the 2026-10/11 runs were cache reads). Measurements and sources:
[agent-loop-efficiency.md](../../../docs/claude-analysis/agent-loop-efficiency.md). The rules below come from it.
CLAUDE.md still wins: §3 gates, §4 honesty, the three-review-round cap, the push rule in §2.8.

## 1. Start

1. `git branch --show-current`, `git status --short`, `git log --oneline -5`; `git fetch origin` and
   `git log --oneline HEAD..origin/ai-forward` must print nothing.
2. Read the **last section of the newest journal** (the handoff), then only the pages it names. Do not read
   whole 1,000-line journals; use `Grep` or a line range.
3. `mcp__ccd_session_mgmt__get_usage` (once). If the 5-hour window is above 70 % or extra usage is nearly spent,
   do one small stage, write the handoff in the journal and stop; an agent killed by a limit leaves half-edited
   files and pays to re-read its context when resumed.
4. Run `tools/run_ladder.sh` once (not a Haiku agent, not command by command). It prints a table; paste nothing
   else into your context.

## 2. Pick the model for the job

| Job | Model | Why |
| --- | --- | --- |
| Orchestration (you) | Sonnet, low or medium effort | Dispatch and relay; bound by context size, not reasoning |
| `realtime-auditor`, `ros2-control-reviewer` | Opus (set in the agent file) | Small and they find the High findings |
| `explainability-reviewer`, `docs-reviewer` | Sonnet (set in the agent file) | Found real issues on Sonnet |
| Root cause of a failure nobody understands; sign and dynamics analysis; a new check's design | Opus, once per problem | One-off reasoning |
| Applying a list of confirmed findings; docs; rerunning checks; mechanical refactors | Sonnet | The decision is already made |
| Running the ladder | No agent: `tools/run_ladder.sh` | A script costs no model tokens |

If a Sonnet agent fails the same task twice, escalate that one task to Opus; do not move the whole class up.

## 3. Brief an agent (copy this skeleton)

```
Repo <path>, branch <b>, HEAD <sha>, tree <clean|expected dirty files>. Read CLAUDE.md §<n> and <one skill>.
TASK: <one goal, with the checkable success criteria>.
FILES YOU OWN: <list>. Do not touch: <list>. Another agent is editing <files> concurrently.
CONTAINER: <you are the sole container user | no container commands>.
TOOLS: use Grep/Read with line ranges, not cat/grep through Bash. Run the ladder with tools/run_ladder.sh,
a check N times with tools/repeat_check.sh, links with tools/check_doc_links.py. Never write your own wait
loop; never use a pgrep pattern that can match your own command line.
WAITING: finish only when all your work is done. If a command takes over 10 minutes, run it in the
background inside ONE Bash call that waits with `timeout`, do not end your turn with work running.
GIT: commit locally with the Co-Authored-By trailer; push only if told.
REPORT: at most 25 lines: what changed (files, commit shas), the exact commands run and their result lines,
what you could not verify. Put anything longer in the journal and link it.
```

Why each line: the file lists stop two agents editing one file; the tool line removes the 29 % of Bash calls that
ran checks by hand; the waiting line removes the "Waiting" notifications that each cost you a turn; the
report limit keeps your context from growing by 1.5 to 3 K tokens per agent.

## 4. While agents run

- Do not poll and do not answer a pure "still waiting" notification with more than one line. Do not read an agent's
  transcript file (it overflows your context).
- One container user at a time. Reviewers are read-only and never use the container; run them in one message.
- `git status` before trusting any report; a limit can kill an agent mid-edit.
- Relay results to the user in at most six lines per update; the detail belongs in the journal.

## 5. Review and stop rules

- Reviews are the cheapest part of a run (about 5 % of tokens) and find the real problems. Keep them: one
  message, all needed reviewers, at most three rounds (CLAUDE.md §2.6).
- Name findings with their final ID in code comments from the start (H-11, not "H-A"); a rename cost a
  review finding in two reviews.
- A red result stays red until a cause is found. Rerunning with `tools/repeat_check.sh <check> 10` gathers
  evidence (is it intermittent, and how often?); it never clears a failure, and any red in N runs is reported
  with its tally. A check that fails at random is a broken sensor: fix it (see `bootstrap-sim`, "Designing a check that
  does not fail at random") before building on it.

## 6. End of a stage

Write the handoff in the journal (state, what each tier said, open items, the next step), commit, push per
CLAUDE.md §2.8, then **start the next stage in a new session** that begins at the handoff. A single
conversation that spans days is re-read in full on every call (run 3's reached 350 K tokens).

At the end of a run, `python3 -I tools/session_usage.py <session-id>` and put the totals in the journal.
