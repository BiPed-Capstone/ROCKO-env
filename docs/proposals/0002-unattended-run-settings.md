# Proposal 0002 — settings and enforcement for a run nobody is watching

- **Status:** Partly accepted: A, B, C and D applied 2026-10-30; E and F are still open
- **Gate:** guardrail (`.claude/settings.json`, `.claude/settings.local.json`, `.git/hooks/`, the guard hook) and a GitHub setting
- **Raised:** 2026-10-29, the commit that adds this file, run journal:
  [2026-10-29-unattended-run-readiness.md](../journal/2026-10-29-unattended-run-readiness.md)

Six independent items, A to F. Each can be accepted or declined on its own; the table says what
each depends on. Nothing here is a robot step.

Words used here. A **headless** run is started with `claude -p`, with nobody at the terminal. A
**fast-forward** push only adds commits on top of what the remote already has, so the remote loses
nothing. A rule written without `:*`, such as `Bash(git push origin ai-forward)`, is **exact**: only
that command matches.

| Item | Where | What it does | Depends on |
| --- | --- | --- | --- |
| A | `.claude/settings.json` | Choose the permission mode on purpose (was `auto`; `acceptEdits` since 2026-10-30) | — |
| B | `.claude/settings.json` | Allow the commands the loop runs | — (includes [0001](0001-allow-container-mount-check.md)'s line) |
| C | `.claude/settings.json`, `.claude/settings.local.json` | Replace the open-ended `gh api` with read-only `gh` commands | — |
| D | `.claude/settings.json` | Refuse force-push, merge and a few more, even if a prompt is approved | — |
| E | `.git/hooks/` (a copy), `.claude/hooks/guard_protected_files.py` | Make git itself refuse commits on `main` and pushes of anything but `ai-forward` | the files in `.githooks/` (already in the tree, inactive) |
| F | GitHub, repository settings (needs an admin) | Make GitHub refuse force-push to and deletion of `main` | — |

## The change

### A. Permission mode

The session transcript records the mode this repo is worked in: `auto` (every record that carries a
mode says so). In `auto` a classifier decides most calls: of the 670 permission decisions recorded
when counted, all accepted, 525 were accepted by a "classifier", 67 by the mode, 43 by a rule (36 given
on the command line, 7 from `.claude/settings.json`), 7 by a person answering a prompt (5 of them
Claude Code's own `safetyCheck`, one the agent's own question) and 28 for another or no recorded reason. So `auto` still stops for a
person on some calls. Nothing in this repo says what the classifier will approve, and a deny rule that
fails to match could be approved by it and run. (The field meanings are read off their names. A call refused by a deny rule reads `decision: reject`,
`reasonType: subcommandResults`, `ruleSource: projectSettings`; seen on 2026-10-30.) For a run nobody watches, the predictable choice is `acceptEdits`
together with B and D: whatever is not allowed prompts. A person who is away leaves the run waiting at
the prompt; under `claude -p` the call is refused.

To make that the default, in `.claude/settings.json`, as the first key inside `permissions`, add:

```json
"defaultMode": "acceptEdits",
```

or choose that mode in the app for the session. Whether the app honors `defaultMode` over a mode
picked for a session is not verified. Declining A and staying in `auto` is a legitimate choice; the
other items still apply, and the *Test for a human* says how to run it in the right mode.

### B. Allow the commands the loop runs

In `permissions.allow`, directly after `"Bash(docker compose:*)",` insert:

```json
"Bash(./check_container_mount.sh)",
"Bash(.claude/hooks/test-guard-protected-files.sh)",
"Bash(docker inspect:*)",
```

and directly after `"Bash(git show:*)",` insert:

```json
"Bash(git ls-files:*)",
"Bash(git ls-tree:*)",
"Bash(git rev-parse:*)",
"Bash(git branch --show-current)",
```

The first line is [proposal 0001](0001-allow-container-mount-check.md); accepting B accepts it, so
set 0001 to `Accepted` too. `CLAUDE.md` §8 names the first two commands.

**Left out on purpose.** An earlier draft also allowed `Bash(bash -n:*)` and `Bash(chmod +x:*)`.
A review showed both are too wide. `bash -n +n -c 'cmd'` turns the no-exec flag back off and runs
`cmd` (tried on bash 3.2 and 5.2), so the rule would allow any code. `chmod +x,a-x file` clears
execute bits, and a guard hook without its execute bit does not run
(`guard_protected_files.py`'s docstring). The cost is a prompt for each syntax check and each new
script's `chmod`; row 0.2 in [next-steps](../next-steps.md) will add a script. If you want a
particular check, add it as an exact rule, for example `Bash(bash -n check_container_mount.sh)`.
`.githooks/test-git-hooks.sh` (named in `CLAUDE.md` §8) is not added either: it runs only when the
hooks change, and after E's step 1 an unattended run cannot change them without a person.

### C. Narrow `gh api`

In `.claude/settings.json`, replace the last allow entry `"Bash(gh api:*)"` with:

```json
"Bash(gh issue view:*)",
"Bash(gh issue list:*)",
"Bash(gh pr view:*)"
```

In `.claude/settings.local.json` (git-ignored, so no diff ever shows it) delete the entry
`"Bash(gh api *)"`.

### D. Deny list

In `.claude/settings.json`, after the `"ask"` array (it then needs a comma after its closing `]`), add:

```json
"deny": [
  "Bash(git push --force:*)",
  "Bash(git push --force-with-lease:*)",
  "Bash(git push -f:*)",
  "Bash(git push --delete:*)",
  "Bash(git push --no-verify:*)",
  "Bash(git commit --no-verify:*)",
  "Bash(git merge:*)",
  "Bash(git reset --hard:*)",
  "Bash(gh pr merge:*)"
]
```

Run `python3 -m json.tool .claude/settings.json >/dev/null` afterwards: a stray comma makes the
whole file invalid.

### E. Git hooks

Two scripts are in [`.githooks/`](../../.githooks/) and do nothing until copied into `.git/hooks/`:
`pre-commit` refuses `git commit` on `main`, and `pre-push` refuses everything except a fast-forward
of `ai-forward` on `origin` (no other branch, tag, remote or deletion, and no force). A person who
means it sets `ROCKO_HUMAN=1` for that one command. Two steps, in this order:

1. In `.claude/hooks/guard_protected_files.py`, in the `GUARDRAIL` pattern, insert `|\.githooks/`
   before `|\.mcp\.json$)`, so the line reads
   `r"|\.git/(config$|hooks/|modules/.*/(config$|hooks/))|\.githooks/|\.mcp\.json$)"`, and add a case
   for it to `.claude/hooks/test-guard-protected-files.sh`. What you copy in step 2 should be what was
   reviewed; the copies in `.git/hooks/` are guarded already.
2. From the repo root, run
   `cp .githooks/pre-commit .githooks/pre-push "$(git rev-parse --git-common-dir)/hooks/"`, and again
   after any change to `.githooks/`. (Inside a git worktree `.git` is a file, so a plain
   `.git/hooks/` would fail there; the command above finds the hooks that a worktree shares with its
   clone.) It is per clone: a second clone needs it too, and all the worktrees of one clone share it.

Do **not** use `git config core.hooksPath .githooks` instead. Git would then run the hooks from the
`.githooks/` folder of whatever files are checked out (a relative or an absolute path alike), and `main`
has no such folder, so after `git switch main` the hooks vanish. Tried in a throwaway repository set
up that way: a commit on `main` and a push of `main` both went through.

### F. GitHub

Needs an administrator of the repository: this account has `WRITE` on `BiPed-Capstone/ROCKO-env`
(`gh repo view --json viewerPermission`), and rulesets need admin. The repository is public. Settings
→ Rules → Rulesets → new branch ruleset: set *Enforcement status* to **Active**, target `main`, turn
on *Restrict deletions* and *Block force pushes*, and leave the bypass list empty, or the rule does
not bind an administrator's own account. Those two rules do not stop an ordinary push of new commits
to `main`; only *Require a pull request before merging* does (rulesets also have *Restrict updates*, not tried
here), and it also means you open a pull request for your own changes to `main`.

## Why

The goal is to let an agent work through the work queue ([next-steps](../next-steps.md)) while
nobody is watching, without a prompt nobody answers stopping it and without giving it more power
than the loop needs. What stands in the way today:

- In `default` mode every edit prompts, which would stall a run. This repo is worked in `auto`, where
  a classifier decides instead and its behavior is not written down anywhere (A).
- The loop's own commands, `CLAUDE.md` §8's Tier 0 host test among them, are not on the allow-list (B).
- `gh api` is allowed twice over (once in each settings file) and can make any GitHub REST call. The
  `gh` login behind it has the `repo` and `workflow` scopes, and it is also what `git push` uses, so
  the agent holds the account's power; the one rule that limits a push is an exact allow entry, which
  decides when to *ask*, not what is *possible* (C, D, E, F).
- There are no deny rules, and "no commits to `main`" is a sentence in `CLAUDE.md` that nothing
  enforces (D, E, F).

## Evidence

No tier of the [verification ladder](../architecture/verification-ladder.md) covers a permission
rule, so none was reached for A to D and F. What was checked:

- **Read this session:** `.claude/settings.json` (no `defaultMode`, no `deny`),
  `.claude/settings.local.json` (`Bash(gh api *)`, git-ignored), the guard hook's source,
  `gh auth status` (scopes `gist`, `read:org`, `repo`, `workflow`; HTTPS; credential in the keyring),
  `gh repo view --json viewerPermission,visibility` (`WRITE`, `PUBLIC`), and
  `git config --show-origin --get-regexp credential` (for github.com the credential helper is
  `gh auth git-credential`, so `git push` uses that same login). The `claude` on the PATH is 2.1.290; this session ran in the
  desktop app, 2.1.308 and then 2.1.309 (the transcript's `version` field).
- **The permission records** in this session's transcript (it has run since 2026-10-27), counted at
  one moment while the session went on: the mode and the decisions given in A.
- **Commands this session ran,** counted from the main transcript only (not its subagents'): the first
  line of each Bash call, split at `&&`, `;` and `|`, so approximate, and still growing.
  `.claude/hooks/test-guard-protected-files.sh` 31 times, `./check_container_mount.sh` 14,
  `git ls-files` 10, `git ls-tree` 9, `gh issue` 10, `docker inspect` 8. None is on the allow-list.
  `python3` (75), `bash` (33), `chmod` (18), `rm` (10), `mkdir` (9), `docker exec` (50) and
  `docker cp` (10) also ran and are **not** proposed: each can run anything, delete files or (`chmod`)
  clear execute bits, so allowing them would empty the list of meaning. An unattended run that
  needs one avoids it; under `claude -p` a call to one is refused and the run records it in a proposal
  (`CLAUDE.md` §2, *Unattended runs*).
- **Tier 0, item E only:** `.githooks/test-git-hooks.sh` builds a throwaway repository and remote,
  installs the hooks into `.git/hooks/` (the worktree form of step 2 was checked separately), and tries 24 things (listed in the script): refused, a
  commit on `main`, a push of `main`, a tag, a tag named `ai-forward`, another branch name, another
  remote, a deletion, a forced push over a diverged branch (`--force` and `--force-with-lease`), a forced push over commits
  we have never seen, and a push with one bad ref alongside a good one; allowed, a commit on `ai-forward`, a
  fast-forward push, a detached-HEAD commit, and a person with `ROCKO_HUMAN=1` (while `ROCKO_HUMAN=0` is refused). All pass (git 2.48.1, macOS;
  also under `dash`, and with SHA-256 repositories). `.githooks/mutation-check.py` first runs the
  unbroken test (and stops if that is red), then breaks each of twelve checks in a copy, one at a time;
  the test went red every time. Three defects were found and fixed: the test counted a push that *git*
  refused as the hook's work (a refusal now has to contain the hook's own `refused:`); with `GIT_DIR`
  set it acted on whatever repository that named (in a copy without the `unset`, pointed at a stand-in
  repository, it committed to the stand-in, added a remote to its config and pushed several refs to its
  remote; it now clears `GIT_DIR` and the settings `git -c` hands down); and the mutation check printed
  PASS even when the unbroken test was red (it now checks first). The `core.hooksPath` failure in E, and
  `git push --dry-run origin ai-forward:main` being refused by the installed hook, in a scratch
  repository, were also tried.
- **The new `GUARDRAIL` pattern in E,** tried on its own in a throwaway Python snippet on nine paths:
  it matches `.githooks/pre-push`, `.claude/hooks/...`, `.claude/settings.json`, `.git/config` and
  `.mcp.json`, and not `.githooks-old/x`, `docs/githooks.md` or a source file. The hook itself was
  not edited, so its tests were not rerun against the new pattern.

**Not verified:**

- That the rule strings match the commands. In particular whether `git push --force:*` also catches
  `--force-with-lease` (the extra line is there in case it does not), and whether
  `Bash(.claude/hooks/test-guard-protected-files.sh)` matches the command as the agent types it.
- How the guard hook's *deny* and *ask* behave under `acceptEdits` or `auto`. The hook's own tests
  exercise the script, not how Claude Code reacts to its answer. The test below is the check.
- What the `auto` classifier approves. Whether `acceptEdits` also lets simple file commands such as
  `mkdir`, `mv`, `cp` and `rm` through inside the project without a prompt.
- Whether a reviewer subagent running in the background can use any command that prompts. It cannot
  answer a prompt.
- Items E and F on any machine but this one, and F on GitHub at all.

## Risk

- **A.** After a switch to `acceptEdits` there is no prompt for edits to `CLAUDE.md`,
  `.claude/agents/` and `.claude/skills/`, which the hook does not guard, so an unattended agent
  could rewrite its own instructions and a person finds out from the diff afterwards (in `auto` that
  is already so). Whether to guard those paths (the hook would *ask*) is already waiting on you in
  [next-steps](../next-steps.md#decisions-waiting-on-the-maintainer); decide it together with A.
- **B.** An allowed script runs its current contents, and `check_container_mount.sh` sits outside the
  guarded folder, so the agent can edit it. That is the trust the list already gives
  `docker compose:*` and `git commit:*`. `.claude/hooks/test-guard-protected-files.sh` is safer: the
  hook asks before any edit to it.
- **Not proposed: scoping `docker compose:*` and `git commit:*`.** [ADR 0005](../decisions/0005-ai-forward-development.md)
  says to move both, and `gh api`, back to `ask` or scope them for an unattended run. `docker compose
  exec` is how Tiers 1 and 2 run, so moving it to `ask` stops the loop at its first step, and a prefix
  such as `docker compose -f docker/compose.yml:*` still allows `run` and `exec` of anything. The
  container bind-mounts the whole repo read-write, `.claude/` and `.git/hooks/` included, and has
  passwordless `sudo` (ADR 0005), so an agent that wants to can edit the guardrails from inside it.
  D, E and F stop accidents, not that. It is the largest gap this proposal leaves, and whether to
  accept it is yours. `git commit:*` is left because commit messages are free text, so a prefix rule
  cannot narrow it; E covers `main` instead.
- **D.** A deny rule matches the start of the command. It catches only the spelling that begins
  with the flag: `git commit -m "x" --no-verify`, `git commit -n`, `git -C . push -f`, a deletion
  written `git push origin :ai-forward`, and anything wrapped in `bash -c` get past it. `git push
  origin ai-forward --force` is not denied either, but it also fails the exact allow rule, so it
  should prompt in `default` or `acceptEdits` (not verified; in `auto` the classifier decides).
  `git merge --abort` is blocked too. It is a net for the careless case, not a fence.
- **E.** The scripts stop an accident, not a determined agent: `--no-verify` skips them (D denies
  only one spelling of it), and `ROCKO_HUMAN=1 git push ...` is not on the allow-list, so it should
  prompt in `default` or `acceptEdits` (not verified; in `auto` the classifier decides).
  `ROCKO_HUMAN=1 git commit ...` may match `Bash(git commit:*)` and not prompt at all. The scripts do
  not print the override, but it is in their headers and here: it is a person's override, and an agent
  never sets it. Only `git commit` runs `pre-commit`: `git merge`, `cherry-pick`, `revert` and `rebase` can
  still add commits to `main` in this clone, and `pre-push` is what keeps them off GitHub. The
  scripts also stop *you* committing to `main`, or pushing any other branch (this clone has local
  `main`, `consolidate_controller_chain` and `debug-webserver`), unless you set `ROCKO_HUMAN=1`. They are
  POSIX `sh` and were tried on macOS, under `dash` as well.
- **F.** A fine-grained token limited to this repository, used for the agent's pushes instead of the
  account's login, is stronger still and is not covered here.

## Test for a human

Not a robot step. Apply the items you accepted, commit the changes to tracked files to `ai-forward` and
push them (E's step 2 changes no tracked file), then do this in a new session (skip the steps for the
items you declined), in **`default` or `acceptEdits` mode, not `auto`**: in `auto` a classifier can
approve a call whose rule does not match, and the steps would then pass for the wrong reason. Before
step 1:

- `git fetch origin && git status -sb` shows `## ai-forward...origin/ai-forward` with nothing in
  brackets (neither ahead nor behind) and no changed files. Step 2's commands are harmless only then.
- Nothing else already allows what is tested: no `check_container_mount`, `test-guard-protected-files`
  or `gh api` in `.claude/settings.local.json` or `~/.claude/settings.json`.

1. **The hook still fires.** Ask the agent to change a `p:` value in
   `rocko_env/rocko_env/bringup/config/rocko_controllers.yaml`: expect a denial that mentions a PID
   gain. If the edit lands, run `git checkout -- rocko_env/rocko_env/bringup/config/rocko_controllers.yaml`
   before anyone launches a robot, and do not accept A. To see the hook *ask*, do this after step 1 of
   E: ask the agent to add a comment line to `.githooks/pre-push` and expect a prompt that gives the
   hook's reason ("... part of the agent's own guardrails ..."); decline it. If the prompt gave no
   reason, repeat the edit and approve it once, read that call's `permissionDecision` record (below),
   then undo the edit with `git checkout -- .githooks/pre-push` (the installed copy does not change).
   Do not use `.claude/settings.json` or `.claude/hooks/` for this: Claude Code asks for its own
   settings file, and for edits under `.claude/hooks/`, anyway, so a prompt there does not show that
   the hook fired.
2. **Each deny rule refuses without asking.** Ask the agent to run, one at a time,
   `git push --force origin ai-forward`, `git push -f origin ai-forward`,
   `git push --force-with-lease origin ai-forward`, `git merge HEAD`, `git reset --hard HEAD` and
   `gh pr merge --help`. Success: each is refused with no approve button. If any shows a prompt
   instead, that rule does not match: note which, and do not approve it.
3. **B and C.** Ask for `./check_container_mount.sh` and `.claude/hooks/test-guard-protected-files.sh`
   (with the container up). Expect no prompt; `ok ...` and exit 0 from the first, `ok` lines and
   exit 0 from the second. Then ask for `gh api repos/BiPed-Capstone/ROCKO-env` (a read): after C,
   expect a prompt; decline it.
4. **A.** Start the session **without choosing a mode**. Its records in the transcript should read
   `permissionMode: acceptEdits`; if they read `auto` or `default`, the app ignored `defaultMode`.
5. **E.** Run `.githooks/test-git-hooks.sh` and `.githooks/mutation-check.py`. Success: `PASS` from
   each, exit 0. If you have not done step 2 of E yet, do it now; then `git switch main` and run
   `git commit --allow-empty -m test`:
   expect `refused: no commits on main`. Run `git push --dry-run origin ai-forward:main`: expect
   `refused: only refs/heads/ai-forward may be pushed from this checkout, not refs/heads/main`. (A dry
   run sends nothing even if the hook is not active. If the commit was not refused, undo it with
   `git reset --hard HEAD~1` and fix the setup.) Then `git switch ai-forward`.
6. **F** (an administrator). Try it on a scratch repository you do not mind, not by pushing to the real
   `main`: create the same ruleset there and try a force push and a branch deletion.

To see why a call was accepted or refused, read its `permissionDecision` record in the session
transcript, `~/.claude/projects/<this project's folder>/<session id>.jsonl`: `decision` (`accept`, or
`reject` for a call a deny rule refused), `source`, `reasonType` (`rule`, `classifier`,
`mode` ...), `ruleSource` and `reasonCode`.

Record: commit SHA, permission mode, the Claude Code version that ran the session (the transcript's
`version` field, or `claude --version` for a terminal session), git version, and where you ran it.

## If accepted, change in the same commit

These pages say what is true today and would be wrong afterwards (`CLAUDE.md` §7):

- **A, B:** the `CLAUDE.md` §8 sentence "not on the `settings.json` allow-list yet, so expect a prompt
  in the default permission mode", §3's "in the default permission mode" about pushes, the same
  phrase in ADR 0005 (its decision table's push row and *Consequences*) and in the ladder's blind spots, and the decision bullets for
  0001 and 0002 in [next-steps](../next-steps.md) and the index in [proposals/README.md](README.md).
- **C:** `CLAUDE.md` §3's remark that `gh api` is on the allow-list (and the place it lists the
  allowed commands), and ADR 0005's *Consequences* and the "(not so for `gh api`, see below)" in its
  decision table's push row.
- **D, E:** `CLAUDE.md` §3's guardrail row and its "Also off limits: committing to `main` and merging.
  Nothing stops those, so don't"; add a line there saying `ROCKO_HUMAN=1` is a person's override that
  an agent never sets, and that a git hook's `refused:` counts as a denial. Also the gate table in
  [proposals/README.md](README.md), the comment above `GUARDRAIL` in the hook, the ladder's count of hook cases
  and its "not installed in this clone" lines, ADR 0005's "Committing to `main` is policy only", and
  the `README.md` line for `.githooks/`, and the sentence "the hooks are not installed in this clone until
  a person does it" in `.claude/skills/build-verify/SKILL.md`.
- **Not proposed** (the paragraph in *Risk*): a pointer from ADR 0005's advice on unattended runs to
  this proposal.

## Decision

2026-10-30, the maintainer ("Apply B–D"): **B, C and D accepted and applied** to
`.claude/settings.json` (commit `ea12edc`), and the `gh api *` entry removed from
`.claude/settings.local.json`. All nine deny rules and all ten new allow rules were tried in a live
session (`auto` mode): the nine deny rules refused, three exact allow rules show a rule match, and the
seven `:*` ones show no rule source. See the [journal](../journal/2026-10-30-apply-proposal-0002-b-d.md).
Later the same day the maintainer chose **A: `acceptEdits`**, and `"defaultMode": "acceptEdits"` was added
to `.claude/settings.json` (see the [journal](../journal/2026-10-30-default-mode-accept-edits.md)).
**E (git hooks) and F (GitHub ruleset) are not decided.** Nor is guarding `CLAUDE.md`, the agents and the skills,
which Risk A said to decide with A: it stays open in [next-steps](../next-steps.md#decisions-waiting-on-the-maintainer).
