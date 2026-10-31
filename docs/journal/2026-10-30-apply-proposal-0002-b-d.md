# Apply items B, C and D of proposal 0002 to the settings

- **Date / commit:** 2026-10-30, built on `a7b28c4`; the settings are in `ea12edc`, the rest is in
  the commit that adds this file
- **Branch:** `ai-forward`
- **Environment:** the maintainer's laptop (macOS), `~/projects/ROCKO-env`; Claude Code desktop app
  2.1.309, `auto` permission mode throughout

## Goal and success criteria

The maintainer answered "Apply B–D" to the question whether to apply items B, C and D of
[proposal 0002](../proposals/0002-unattended-run-settings.md). Items A (permission mode), E (git
hooks) and F (GitHub ruleset) are not decided and stay open.

1. `.claude/settings.json` has exactly the allow, remove and deny changes the proposal lists, nothing
   else, and is still valid JSON with the guard hook still wired up.
2. The `gh api *` entry is gone from `.claude/settings.local.json` (item C).
3. Each new rule string is tried in a live session, and the result says whether a *rule* made the
   decision, so the proposal's "do the rule strings match" doubt is answered where it can be.
4. The pages that described the old state are corrected in the same change (`CLAUDE.md` §7).

## Plan

Edit the two settings files with the Edit/Write tools (the guard hook and Claude Code both gate
them), commit them alone so the working tree is clean, then try the rules (the `git reset --hard HEAD`
test is only harmless on a clean tree), then fix the docs from the proposal's *If accepted* list and
the results. Risk: a deny rule that is a typo blocks a command the loop needs; a rule that does not
match gives false comfort.

## What changed

| File | Change |
| --- | --- |
| `.claude/settings.json` | Ten allow entries added (`./check_container_mount.sh`, `.claude/hooks/test-guard-protected-files.sh`, `docker inspect:*`, four read-only `git` commands, `gh issue view:*`, `gh issue list:*`, `gh pr view:*`); `gh api:*` removed; nine deny rules added. `ask`, the hook wiring and the other allow entries are untouched; no `defaultMode` (item A) |
| `.claude/settings.local.json` (git-ignored) | The `Bash(gh api *)` entry removed; the allow list is empty |
| `CLAUDE.md`, `docs/decisions/0005-ai-forward-development.md`, `docs/architecture/verification-ladder.md`, `docs/next-steps.md` | The sentences that said the mount check prompts, that `gh api` is allowed, that `--force` prompts and that nothing blocks a merge; ADR 0005 amended |
| `docs/proposals/0001-…`, `0002-…`, `README.md` | Statuses and *Decision* sections |
| `docs/journal/2026-10-29-push-without-asking.md` | The wrapped-push wording corrected (below) |

**A correction.** The first draft of these pages said a `cd ... &&` prefix does not stop a command
matching its allow rule, and `CLAUDE.md` step 8 was loosened because of it. That was wrong. In the
session transcript the `cd <project folder> &&` prefix appears only in `api-request-blob` records,
which add it to every Bash call; the recorded (`assistant`) commands have none. So the tests say
nothing about a `cd` prefix. What separates the calls that matched a rule from those that did not is
only the trailing `2>&1 | tail -N; echo ...` or `> /dev/null 2>&1; echo ...`. `CLAUDE.md` again says
to run the push with nothing chained, piped, redirected or appended.

## Verification

Rows with a tier are checks that need no live permission system. The rows marked "—" are live tries
in this session (`auto` mode): no tier of the [ladder](../architecture/verification-ladder.md)
covers a permission rule, so none is claimed. For those, the *record* is the `permissionDecision` of
the call in the session transcript (its `decision`, `source`, `reasonType` and `ruleSource`).

| Tier | Command | Result |
| --- | --- | --- |
| 0 | `python3 -m json.tool` on both settings files; a Python diff against `HEAD` | valid; ten allow entries added, `gh api:*` removed, `ask` and the hook wiring unchanged, nine deny rules |
| 0 | `.claude/hooks/test-guard-protected-files.sh` | exit 0 (81 `ok`; it includes the check that the hook is wired up) |
| — | Nine deny rules: `git push --force`, `-f`, `--force-with-lease`, `--delete` (a branch that does not exist), `--no-verify`, `git merge HEAD`, `git reset --hard HEAD` (clean tree), `gh pr merge --help`, `git commit --no-verify` (nothing staged) | all nine **refused**; each record reads `decision: reject`, `reasonType: subcommandResults`, `ruleSource: projectSettings`. Each was chosen so that, had it run, it would have lost nothing (the branch was one commit ahead of `origin`, so a push would only have published it) |
| — | `git commit -m "..." --no-verify` (nothing staged): the spelling the proposal says the deny rule misses | it **ran** ("nothing to commit"); its record has the same shape as the seven read-only commands below, so it is consistent with the `:*` prefix rule `git commit:*` having matched. The documented gap is real |
| — | `./check_container_mount.sh` | ran; `reasonType: rule`, `ruleSource: projectSettings`: the new exact rule matched |
| — | `.claude/hooks/test-guard-protected-files.sh` (bare) | ran, 81 cases; `rule` / `projectSettings`: the exact rule matched |
| — | `git branch --show-current` | ran; `rule` / `projectSettings` |
| — | `docker inspect`, `git ls-files`, `git ls-tree`, `git rev-parse`, `gh issue view`, `gh issue list`, `gh pr view` | ran; `subcommandResults` with **no rule source**, so it is not known whether the new rule or a built-in allowance for read-only commands let them through |
| — | `.claude/hooks/test-guard-protected-files.sh > /dev/null 2>&1; echo "exit=$?"` | ran; `classifier`, not the rule: the wrapped form does not match |
| — | `gh api repos/BiPed-Capstone/ROCKO-env --jq .name` (a read) | ran; `classifier`: the rule is gone, but in `auto` the call is not blocked |
| — | The two settings edits (`Write` to `settings.json` and to `settings.local.json`) | each record: `decision: accept`, `source: user_temporary`, `reasonCode: safetyCheck`, `grantScope: once`, with `circuitBreaker: claudeSettingsFile`. That is Claude Code's own protection of its settings file asking the maintainer, who approved; the guard hook's reason text was not seen |
| 1, 2, 3, 4 | not run | nothing they exercise changed |

## Review rounds

| Round | Reviewer | Findings | Disposition |
| --- | --- | --- | --- |
| 1 | `docs-reviewer` | 1 High, 3 Medium, 8 Low. High: the `cd ... &&` claim came from the wrong transcript records (above). Medium: live permission tries were filed under Tier 0; the push text still said `--force` prompts, which the deny list now refuses; this entry's Reproduce block was hazardous. Low: an undercount of the allow rules tried, field names, "harmless" overstated, a stale "no refusal seen", wrapping, grammar | All **fixed**. I checked the High finding in the transcript myself before changing anything (all 19 recorded Bash commands that mention the hook test have no `cd` prefix). The settings change itself was found exact. The fixes were not re-reviewed |

## Not verified — needs a human

- **Any behavior outside `auto` mode.** Whether the allow rules stop prompts in `default` or
  `acceptEdits`, and whether a command that matches no rule prompts there, was not tried.
- **Whether the seven read-only commands are allowed by the new rules or by built-ins.** The record
  does not say.
- **Whether a `cd ... &&` prefix, a pipe, a redirect or an `echo` is what breaks a match.** Only the
  combined wrapped forms were seen.
- **That the guard hook still asks for guardrail edits under `acceptEdits`.**
- **Items A, E and F.**

## Reproduce

Ask the agent to run these in a Claude Code session from the repo root (they are about the permission
system, so a terminal does not test them; do not type the third one in a terminal). Each should be
refused with no approve button, or, for the allow checks, run without a prompt:

```bash
python3 -m json.tool .claude/settings.json >/dev/null && echo valid
git diff ea12edc~1 ea12edc -- .claude/settings.json          # the whole settings change
git push --force --dry-run origin ai-forward                 # should be refused; a dry run sends nothing if it is not
```

The third command is the safe way to see a deny rule fire: it still starts `git push --force`, so the
rule matches it, and if the rule failed the dry run would still change nothing. To see why a call was
allowed or refused, read its `permissionDecision` record in the session transcript
(`~/.claude/projects/<this project's folder>/<session id>.jsonl`): the user record whose
`message.content[].tool_use_id` is the call's id. Use the `assistant` record for the command text, not
the `api-request-blob` copy, which adds a `cd <project folder> &&` prefix to every command.
