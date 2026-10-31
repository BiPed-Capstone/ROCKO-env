# Make `acceptEdits` the project's default permission mode

- **Date / commit:** 2026-10-30, built on `96592fb`; the commit that adds this file
- **Branch:** `ai-forward`
- **Environment:** the maintainer's laptop (macOS), `~/projects/ROCKO-env`; Claude Code desktop app
  2.1.309. This session ran in `auto` mode, so nothing about the new default could be tried

## Goal and success criteria

Item A of [proposal 0002](../proposals/0002-unattended-run-settings.md) asks which permission mode a
session that does not pick one should start in. The maintainer chose **`acceptEdits`** from three
options (`acceptEdits`, stay in `auto`, `default`) with the agent's recommendation first.

1. `.claude/settings.json` gets exactly one new line, `"defaultMode": "acceptEdits"`, as the first key
   inside `permissions`, and stays valid JSON with the guard hook still wired up.
2. The pages that talk about the permission mode say what is true now, including what is not known.

## Plan

One edit to the guardrail file (Claude Code asks the maintainer about it), then the docs from the
proposal's *If accepted* list. Risks: the desktop app might ignore the setting; `acceptEdits` is
expected to stop prompting for edits to `CLAUDE.md`, `.claude/agents/` and `.claude/skills/`, which the hook does not
guard; sessions that do not pick `auto` get more prompts for commands off the allow-list.

## What changed

| File | Change |
| --- | --- |
| `.claude/settings.json` | `"defaultMode": "acceptEdits"` added. Nothing else |
| `CLAUDE.md`, `docs/decisions/0005-ai-forward-development.md`, `docs/architecture/verification-ladder.md`, `docs/next-steps.md` | The default is named; the push wording says `default` mode and, untested, `acceptEdits`; ADR 0005 has an amendment row; the guard-the-instruction-files decision is flagged as more pressing |
| `docs/proposals/0002-…`, `docs/proposals/README.md`, `docs/journal/README.md` | Status: A, B, C and D applied; E and F open; the index row |

## Verification

| Tier | Command | Result |
| --- | --- | --- |
| 0 | `python3 -m json.tool .claude/settings.json`; `git diff` of the file | valid; the diff is the one added line |
| 0 | `.claude/hooks/test-guard-protected-files.sh` | exit 0 (81 `ok`, including the check that the hook is wired up) |
| — | The settings edit's permission record | `decision: accept`, `source: user_temporary`, `reasonCode: safetyCheck`, `grantScope: once`: Claude Code asked the maintainer, who approved |
| 1, 2, 3, 4 | not run | nothing they exercise changed |

## Review rounds

| Round | Reviewer | Findings | Disposition |
| --- | --- | --- | --- |
| 1 | `docs-reviewer` | 0 High, 4 Medium, 3 Low. Medium: ADR 0005's push row still said "the default permission mode"; a `grep` on the `claude` binary on the PATH (2.1.290) was cited for a session that ran 2.1.309; the proposal's step 4 no longer tested the setting (a person who picks `acceptEdits` in the picker would pass it); "edits do not prompt" was stated as fact | All **fixed**. The grep row is dropped: it proved only that two words exist in a program. Step 4 now says to start without choosing a mode. The fixes were not re-reviewed |

## Not verified — needs a human

- **That a new session starts in `acceptEdits`.** The desktop app has its own mode picker and may not
  read `defaultMode`. Start a session without choosing a mode and read `permissionMode` in its
  transcript records.
- **Everything about `acceptEdits` itself:** that edits do not prompt, that a command off the
  allow-list does, and that the guard hook still denies a gain edit there (and, once item E's step 1 is done, asks about
  `.githooks/`).
  The *Test for a human* in proposal 0002 (steps 1 to 4) is the check, and it must be run in that mode.
- **Whether `acceptEdits` also lets `mkdir`, `mv`, `cp` and `rm` through without a prompt.**

## Reproduce

```bash
python3 -m json.tool .claude/settings.json >/dev/null && echo valid
git diff 96592fb -- .claude/settings.json                       # before it is committed: one added line
.claude/hooks/test-guard-protected-files.sh; echo "exit=$?"
```

The checks that matter need a new session, outside `auto`; see *Not verified*.
