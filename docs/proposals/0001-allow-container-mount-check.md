# Proposal 0001 — allow `./check_container_mount.sh` without a permission prompt

- **Status:** Accepted (applied 2026-10-30 with [proposal 0002](0002-unattended-run-settings.md), item B)
- **Gate:** guardrail (`.claude/settings.json`)
- **Raised:** 2026-10-29, the commit that adds this file, run journal:
  [2026-10-29-container-mount-check.md](../journal/2026-10-29-container-mount-check.md)

## The change

In `.claude/settings.json`, in `permissions.allow`, directly after line 21 (`"Bash(docker compose:*)",`),
insert:

```json
"Bash(./check_container_mount.sh)",
```

It is an exact rule: the command with no arguments, in those words, run from the repo root. An
absolute path, `bash check_container_mount.sh` or a command chained to it is a different string and
is expected to prompt (not tried). `CLAUDE.md` §8 tells the agent to run it as `./check_container_mount.sh`.

## Why

`check_container_mount.sh` tells an agent whether the dev container is looking at the checkout it is
editing. Without that, a session in a git worktree runs the container's verification commands
against another folder's files, and they can pass. `CLAUDE.md` §8 tells the agent to run it first, at the start of a session
and whenever it changes checkout. The script is not on the allow-list, so in the default permission
mode it prompts every time, and a check that interrupts a person gets skipped.

## Evidence

No tier of the [verification ladder](../architecture/verification-ladder.md) covers a permission
change, so none was reached for the rule itself. What was checked about the script is in the journal:
it passes from the main checkout 20 times out of 20, fails with exit 1 from a real git worktree, and
exits 2 with no container, no `docker`, or no `docker/compose.yml` beside it. Not verified: that the
rule matches the command (the session that wrote this may have been auto-approved), and Linux or WSL2.

## Risk

The script lives in the repo, so an agent could edit it, and an allowed script runs its current
contents without a prompt. That is the same trust the list already gives `docker compose:*`,
`colcon build:*` and `git commit:*`, which also run code the agent can change; the guard hook does
not watch Bash. The script creates and deletes one empty file in the repo root and runs
`docker compose exec ... true` and `test -e`; it writes nothing else and never touches the robot.

## Test for a human

Not a robot step. In a new session in the default permission mode, with the container up:

1. Check that nothing else already allows it, or step 3 proves nothing: no `check_container_mount`
   in `.claude/settings.local.json` or `~/.claude/settings.json`.
2. Ask the agent to run exactly `./check_container_mount.sh` from the repo root.
3. Success: it runs with no prompt and prints `ok    the container sees this checkout: <path>`.
4. Ask it to run `./check_container_mount.sh --help`. It should prompt, because the rule is exact. (The
   script ignores its arguments, so the point of the test is the prompt.)

## Decision

2026-10-30, the maintainer ("Apply B–D" to proposal 0002, whose item B contains this rule):
accepted and applied. In a live session the permission record for `./check_container_mount.sh` reads
`reasonType: rule`, `ruleSource: projectSettings`, so the rule matched (`auto` mode; see the
[journal](../journal/2026-10-30-apply-proposal-0002-b-d.md)). The `--help` step of the test above
was not tried.
