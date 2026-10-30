# Stop asking before every push to `ai-forward`

- **Date / commit:** 2026-10-29, built on `72b27ee`; the commit that adds this file
- **Branch:** `ai-forward`
- **Environment:** the maintainer's laptop (macOS), `~/projects/ROCKO-env`; Claude Code desktop app,
  `auto` permission mode

## Goal and success criteria

The maintainer wrote, after the agent asked "Push 72b27ee to origin/ai-forward?" for the second time
that day: "update the rules and permissions that make the agent keep asking for permission to push
code. the agent should always be able to push code".

1. Find what makes the agent ask, and say which cause is a permission and which is an instruction.
2. `CLAUDE.md` tells the agent to push every `ai-forward` commit before the turn ends, without
   asking, and keeps the limits: the fetch check, `git push origin ai-forward` in exactly those words
   and as its own command, never `main`, another branch or a force.
3. No guardrail file (`CLAUDE.md` §3) is edited unless that is what fixes the cause.

## Plan

Read the transcript's permission records for every push, and the places that tell the agent to wait.
Change the instructions (`CLAUDE.md` step 8 and §3, an ADR note, the agent's memory). Leave
`.claude/settings.json` alone unless a push the documented way is shown to prompt. Docs-only apart
from the memory, so Tier 0 is the whole ladder here. Risk: wording that reads as permission to push
more than `ai-forward`.

## What was found

- **The permission already allows the documented command.** `.claude/settings.json` has
  `Bash(git push origin ai-forward)` and `Bash(git push -u origin ai-forward)` as exact allow rules.
  The session transcript links each push to its permission record. The bare `git push origin
  ai-forward` ran three times, and each record reads `reasonType: rule`, `ruleSource:
  projectSettings`: the rule matched.
- **A wrapped form does not match.** On 2026-10-28 the agent twice ran `cd ... && git push origin
  ai-forward 2>&1 | tail -N; echo "exit=..."`. Neither record is a rule match; the `auto` classifier
  approved both. In `default` mode that form should have prompted (not tried). The one real prompt was
  `git push -u origin main ai-forward`, which pushed `main` too and was approved by the maintainer
  before the rule existed.
- **The asking was mostly an instruction.** Two things told the agent to wait for a go-ahead: a note
  in its own memory ("a commit is pushed only after the user says to"), and the general default
  "commit or push only when the user asks". `CLAUDE.md` step 8 said to push after the review loop but
  never said the maintainer had waived the question.
- **Not found:** any push in the documented words that prompted.

## What changed

| File | Change |
| --- | --- |
| `CLAUDE.md` §2 step 8, §3 | Push without asking: no "shall I push?", no choice offered, no wait; push every `ai-forward` commit before the turn ends and report what went out. Run the push as its own command, nothing chained or piped. If it is refused, declined or fails, stop and report; no other form. The only reasons not to push are the fetch check and a third review round that is still dirty |
| `docs/decisions/0005-ai-forward-development.md` | An *Amended 2026-10-29* note on item 6; the "not established" sentence about whether the rule or the mode let the push through now cites the transcript's records |
| The agent's memory (outside the repo) | The "push only after the user says to" note is replaced by the standing instruction |

**Deliberately not changed:** `.claude/settings.json`. Widening the exact rules to
`Bash(git push origin ai-forward:*)` would let `git push origin ai-forward --force` and
`... --delete` through without a prompt, and nothing in this clone would stop them. The deny list in
[proposal 0002](../proposals/0002-unattended-run-settings.md) (item D) matches only spellings that
begin with the flag (`git push --force ...`), as that proposal's *Risk* section says, so it would not
catch them either; the `pre-push` hook (item E) would, and it is not installed here. The documented
words already work, so the cost of the exact rule is only that the agent must type them, bare.

## Verification

| Tier | Command | Result |
| --- | --- | --- |
| 0 | `.claude/hooks/test-guard-protected-files.sh` | exit 0 (81 `ok`) |
| 0 | A relative-link check of the changed documents (a throwaway script that resolves each link; not kept) | passes |
| 0 | `git diff --stat -- .claude/settings.json .claude/hooks .claude/commands .mcp.json` | empty: no guardrail file was touched |
| 0 | The transcript query (below) | the six pushes and their decision records as described above |
| 1, 2, 3, 4 | not run | nothing they exercise changed |

## Review rounds

| Round | Reviewer | Findings | Disposition |
| --- | --- | --- | --- |
| 1 | `docs-reviewer` | 1 High, 2 Medium, 5 Low. High: this entry said the proposed deny list would stop a force-push, and it would not. Medium: the ADR still said it was not established whether the rule let the push through, and the transcript settles it; `CLAUDE.md` did not say what to do when a push is refused or fails. Low: a wrapped push does not match the rule (so step 8 should say to run it bare), wording, a long line, the journal template | All **fixed**. I ran the transcript query myself before citing it. Not re-reviewed |

## Not verified — needs a human

- **That the agent now stops asking.** The change is to instructions the agent reads; only a later
  session shows whether it follows them. The next commit is the test.
- **That a wrapped push prompts in `default` mode.** The records show the rule did not match; nobody
  tried the form outside `auto`.
- **That no push in the documented words prompts under any other mode.**

## Reproduce

From the repo root, a few seconds in all:

```bash
.claude/hooks/test-guard-protected-files.sh; echo "exit=$?"
git diff --stat -- .claude/settings.json .claude/hooks .claude/commands .mcp.json   # prints nothing
grep -n "git push" .claude/settings.json                                            # the two exact allow rules
```

To repeat the transcript query you need the session transcript on your machine
(`~/.claude/projects/<this project's folder>/<session id>.jsonl`). Link each `git push` command to
its permission record by pairing an assistant record's `uuid` with the
user record whose `sourceToolAssistantUUID` equals it, and read that record's `permissionDecision`
(`decision`, `source`, `reasonType`, `ruleSource`).
