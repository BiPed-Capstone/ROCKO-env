# Let the agent push `ai-forward`

- **Date / commit:** 2026-10-28, built on `d581f84`; the commit that adds this file
- **Branch:** `ai-forward`
- **Environment:** the maintainer's laptop, `~/projects/ROCKO-env`. No container or ROS involved.

## Goal and success criteria

The maintainer asked to update the rules so the agent can push on `ai-forward`, because that branch
is likely to hold only commits from the AI. Until now `CLAUDE.md` said the agent never pushes and
`settings.json` made every `git push` prompt.

1. `CLAUDE.md` and ADR 0005 say the agent may push `ai-forward`, and say exactly how and what stays
   forbidden.
2. `settings.json` lets the agent push `ai-forward` without a prompt and, for `git push`, **only**
   that: not `main`, not another branch, not `--force`.
3. A human's commits on the branch cannot be overwritten, even though "only AI commits" is expected
   rather than guaranteed.
4. No document still says the agent never pushes, or that the work is local-only.
5. The documents do not claim more protection than the rules give.

## Plan

Change the permission rule first (remove the broad `ask`, add exact `allow` rules), then the two
documents that state the rule, then fix the statements the change made false, then test the
procedure as written and have the docs reviewed. Risk: the permission system matches command text,
so the rule can be narrower or wider than it looks; the wording has to say what it cannot stop.

## What changed

| File | Change |
| --- | --- |
| `.claude/settings.json` | Removed the broad `ask` rule for `git push`; added `allow` for the exact commands `git push origin ai-forward`, `git push -u origin ai-forward` and `git fetch origin` |
| `CLAUDE.md` | §2 step 8: fetch, then `git log --oneline HEAD..origin/ai-forward` must print nothing, then push; stop and report if it lists commits. §3: pushing is allowed for that one thing, never force, never another branch, and what the rules do not stop |
| `docs/decisions/0005-ai-forward-development.md` | Item 6 and the permissions table amended (dated); a consequence added about what a push publishes and what is unverified |
| `docs/journal/2026-10-27-ai-forward-bootstrap.md` | Three statements that the work was local-only now say it was pushed on 2026-10-28 at the maintainer's instruction |

**Why exact-match rules and not a prefix.** Claude Code checks rules in the order deny, then ask,
then allow, and unlisted commands prompt. A prefix allow such as `git push origin ai-forward:*` would
also match `git push origin ai-forward --force`. An exact rule matches only itself, so in the default
permission mode every other `git push` form falls back to a prompt. The old broad `ask` rule had to
be removed because `ask` outranks `allow` and would have kept prompting.

**Why the remote-commits check.** "Only AI commits" is likely, not certain. A push without `--force`
cannot overwrite anything, so the real risks are a human pushing in between (the push is rejected)
and someone reaching for `--force` to get past that. The rule says to stop and report, never to force.

**Why `git log HEAD..origin/ai-forward` and not `git status -sb`.** The first draft told the agent
that `git status -sb` "must show `ahead`". The docs review pointed out that when the branch matches
the remote it prints no bracket at all (`## ai-forward...origin/ai-forward`), and before the first
`-u` push it prints only the branch name, so that check could not do what it claimed. `git log
HEAD..origin/ai-forward` lists exactly the commits the remote has and the branch lacks, whatever
`status` prints, and `git log` is already on the allow-list.

## Verification

| Tier | Check | Result |
| --- | --- | --- |
| 0 static | `settings.json` is valid JSON; no broad `git push` rule remains in `allow` or `ask` | pass |
| 0 static | `.claude/hooks/test-guard-protected-files.sh`, which includes the check that `settings.json` wires the hook up | pass; its last line read `--- 81 cases (80 hook cases plus the wiring check); exit status asserted on every hook case` |
| — | `git log --oneline HEAD..origin/ai-forward` with local equal to remote | printed nothing, exit 0 |
| — | the same against a branch that is not on the remote | `fatal: ambiguous argument ...: unknown revision or path not in the working tree` |
| — | `git status -sb` with local equal to remote | `## ai-forward...origin/ai-forward` (no bracket) |
| — | `git fetch origin`, then `git push origin ai-forward` with nothing new to send, as a **test of the allow rule** (step 8 itself says to push only after a commit) | `Everything up-to-date`, exit 0 |

No tier of the verification ladder applies to a permission change; the rows above are what could be
checked.

## Review rounds

| Round | Reviewer | Findings | Disposition |
| --- | --- | --- | --- |
| 1 | `docs-reviewer` | 2 High: the docs claimed the push limits were enforced although `gh api` (allowed) can move or delete a remote ref, and the push target comes from `.git/config`; `git status -sb` cannot show what step 8 asked of it. 3 Medium: the ADR claimed more testing than was done; "still prompts" needed the default-mode caveat; "pushed" did not say by whom. 2 Low: the journal template; an ADR reason that did not follow | All **fixed** in the documents. `gh api` was **not** removed from the allow-list: it was already there and removing it is the maintainer's call |

No second round: the fixes are wording and a changed check, each re-verified by running the command
(above) and by searching the repo for the stale phrases. They were not re-reviewed.

## Not verified — needs a human

- **That the allow rule is what let the test push through.** This session may have approved or
  auto-approved the command, so the no-op push does not show the rule matched. The first push of a
  real commit under a default permission mode will.
- **That every other push form still prompts.** That follows from Claude Code's documented rule order
  and the exact-match form of the rules; none of the refused forms was run, because running them
  would only trigger prompts. `--force`, `main`, and a chained `git push ... && git push ...` are
  the ones to try once if you want it confirmed. A chain in which every part is allowed, such as
  `git fetch origin && git push origin ai-forward`, is expected to run without a prompt.
- **What the rules cannot stop:** `gh api` (allowed) moving or deleting a remote ref; a push target
  changed in `.git/config`; a rule added by answering "don't ask again" or kept in
  `settings.local.json` or a user's own settings. Whether `gh` is logged in on a given machine was
  not checked as part of this change.

## Reproduce

```bash
python3 -c "import json; p = json.load(open('.claude/settings.json'))['permissions']; print([a for a in p['allow'] + p['ask'] if 'push' in a])"
.claude/hooks/test-guard-protected-files.sh
git fetch origin
git log --oneline HEAD..origin/ai-forward
```

The first prints only the two exact allow rules. The last prints nothing when the branch is level
with, or ahead of, the remote.
