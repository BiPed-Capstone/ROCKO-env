# ADR 0005 — Develop ROCKO with an AI-forward agent loop

- **Status:** Proposed — implemented on the `ai-forward` branch only; `main` still uses the
  human-gated workflow
- **Date:** 2026-10-27
- **Deciders:** Garrett Hart (maintainer). Drafted by Claude at the maintainer's direction.

## Context

`main` treats the AI as a tutor: investigate, explain, ask, **wait for a go-ahead**, then implement.
That fits a teaching repo, but it makes a human the bottleneck on every change, and it means the AI
never gets to find out whether its work was right — it hands over a patch and stops.

The maintainer wants the opposite default for this branch: the repo is code *intended to be written
by AI*, the AI plans, implements, runs things to check its own work, and iterates without being
walked through each step. Three things stay as important as before: code stays simple, code stays
explainable to other people, and documentation remains a deliverable.

Two facts constrain this:

1. **The AI can only be as autonomous as its verifier is honest.** A loop checked by nothing that
   can fail is open-loop — the same mistake as finding C-2, an IMU that reports "upright" no matter
   what. At the time of writing the only checks that exist are static (Tier 0), an off-Pi build
   (Tier 1) and a wiring check on mock hardware (Tier 2). There is **no physics simulation**
   ([ADR 0003](0003-simulation-target.md)), so nothing the AI can run observes balance.
2. **Some things cannot be verified without the robot and damage hardware if wrong:** the PID gains
   (experimental results on one robot, surface and battery voltage), the version pins
   ([ADR 0001](0001-version-lock.md)), and the safety behaviors (tipover, watchdog, clamps).

## Decision

On the `ai-forward` branch:

1. **Goal in, verified change out.** The AI works a whole goal without mid-loop approval: restate the
   goal as checkable criteria, plan in writing, implement, verify, run reviewer subagents in parallel,
   fix, repeat. Capped at three review rounds; a dirty third round is handed to the human.
2. **Autonomy is bounded by hardware, not by process.** The AI changes code, launch files, URDF,
   tests, tooling and docs freely. A human decides: hardware gains and limits (the key list is in [`docs/proposals/`](../proposals/README.md)), version pins,
   and removing or weakening a safety behavior. A hook *denies* those edits and tells the AI to write
   a proposal ([`docs/proposals/`](../proposals/README.md)) and carry on, so an unattended run never
   stalls on a prompt. Edits to the guardrails themselves (the hook, `settings.json`, and the other files that can run
   code behind them; the list is in [`docs/proposals/`](../proposals/README.md)) ask a human; a
   headless run is refused.
3. **The verification ladder is the loop's sensor.** Five tiers from static checks to the robot,
   each with a stated status in [`verification-ladder.md`](../architecture/verification-ladder.md).
   The AI reports the highest tier it actually ran, never above it, and a tier counts as built only
   once it has been seen to fail on a deliberately broken input.
4. **An explanation trail replaces teach-first.** Every behavior change leaves a *why* in the code, a
   run-journal entry ([`docs/journal/`](../journal/README.md)), updated architecture docs, and an ADR
   where the choice is not obvious. A dedicated `explainability-reviewer` checks the diff against the
   test "could a student explain this in five sentences?".
5. **Building better verifiers is first-class work.** The `bootstrap-sim` skill is an ordered
   backlog toward Tier 3.
6. **Checkpoints, and publishing one branch.** The AI may commit verified work to its working branch
   in logical commits and, on `ai-forward`, push it to `origin` after the review loop. It first
   fetches and pushes only if its branch is not `behind` the remote, and it never forces. It never
   commits to `main` and never pushes any other branch. Committing to `main` is policy only; the
   push limits are *limited* (not guaranteed) by permission rules (item 7). *Amended 2026-10-28:*
   this item originally said the AI "never pushes". The maintainer lifted that for `ai-forward`
   because the branch is expected to hold only the AI's own commits, so publishing it is the natural
   end of the loop and there is little human work to disturb. A push without `--force` cannot
   overwrite anything whatever the branch holds, and that is not relied on either: the check for
   commits the remote has and the agent lacks, and the no-force rule, cover a human who does push to
   it. *Amended 2026-10-29:* the maintainer asked that the agent push without asking each time
   ("the agent should always be able to push code"); `CLAUDE.md` step 8 says so. The permission rules
   are unchanged: the exact allow entries already let the command through, and widening them would
   also let `--force` or `--delete` through.
7. **Permissions that follow from the above** (`.claude/settings.json`), and why:

   | Setting | Why |
   | --- | --- |
   | `allow` `docker compose` (was `ask` on `main`) | Tiers 0–2 run in the container; a prompt per run defeats an unattended loop |
   | `allow` `git add`, `git commit` | Checkpoint commits (item 6) |
   | `allow` `colcon test`, `ros2 topic echo` | Build/test, and read-only; but `colcon` runs repo-controlled code (see the trust decision below) |
   | `ask` `ros2 launch`, `ros2 run` (unchanged) | On a Pi these drive real motors |
   | `allow` exactly `git push origin ai-forward` and `git push -u origin ai-forward`; `allow` `git fetch origin` (amended 2026-10-28; was `ask` on every `git push`) | Publishing the AI's own branch is the end of the loop. Exact-match rules cannot be stretched into `--force` or another branch, and the broad `ask` rule had to go because `ask` outranks `allow`. In `default` mode (and, untested, `acceptEdits`) every other `git push` falls back to a prompt, except that `--force`, `-f`, `--force-with-lease`, `--delete` and `--no-verify` spelled first are refused outright by the deny list added 2026-10-30 |
   | *Amended 2026-10-30* ([proposal 0002](../proposals/0002-unattended-run-settings.md) B, C, D): `allow` `./check_container_mount.sh`, the guard hook's test, `docker inspect`, four read-only `git` commands and `gh issue view`/`list`, `gh pr view`; `gh api` removed from `allow`; a `deny` list for force-push, `--delete`, `--no-verify`, `git merge`, `git reset --hard` and `gh pr merge` | The loop's own commands and read-only lookups; `gh api` could make any REST call with a login that has `repo` scope. A deny rule matches the start of the command, so it is a net for the careless case: `git commit -m x --no-verify` gets past it (tried) |
   | *Amended 2026-10-30* ([proposal 0002](../proposals/0002-unattended-run-settings.md) A): `permissions.defaultMode` set to `acceptEdits` | A session that does not pick its own mode is meant to start where edits do not prompt but a command off the allow-list does, so an unattended run waits (or is refused under `claude -p`) instead of a classifier deciding, as it does in `auto`. Not verified: that the desktop app honors it over its mode picker, or anything about this mode (nothing was run in it), including whether edits to `CLAUDE.md`, the agents and the skills stop prompting there |
   | `NotebookEdit` dropped from the hook matcher | There are no notebooks here, and the tool's path field is `notebook_path`, which the hook does not read |

   The first row is a real trust decision; see "The allow-list is a trust decision" below.

## Alternatives considered

- **Keep `main`'s human-gated workflow.** Safest, and the best fit for a course where the point is
  that a student does the work. Rejected *for this branch* because it caps the AI's usefulness at
  "writes a patch"; it remains the right model for `main` until this one proves itself.
- **Fully autonomous, including gains and pins.** Fastest. Rejected: the AI cannot observe the
  consequence of a gain on a real robot, and sim-tuned values do not transfer. A confident wrong gain
  is worse than a slow right one, and a fall damages hardware.
- **Keep the gates as prompts (`ask`) instead of denials.** Familiar. Rejected: in an unattended run a
  prompt is a stall, and in a headless run it is a silent refusal with no record. A denial that says
  *write a proposal* turns the block into a visible, reviewable artifact.
- **A separate repository.** Cleaner separation. Rejected: the two workflows share all the code, and
  divergence would be expensive to reconcile.

## Consequences

- **Good:** the AI can verify its own work and iterate; every change comes with the evidence for it;
  a human reviews proposals and journals instead of re-deriving intent from diffs.
- **Bad:**
  - **Verifier weakness is the central risk.** Until Tier 3 exists, an agent can "pass" Tier 2 with a
    controller change that is physically wrong, because the mock has no physics. The journal's
    discipline of recording only tiers that ran is the mitigation, and it relies on honesty.
  - **The hook is a tripwire, not a proof.** It sees edits, not intent; it cannot detect every way
    to weaken a safeguard and does not watch shell commands. Reviewers and `CLAUDE.md` carry the rest.
    Its limits are listed in the docstring of `guard_protected_files.py` and in `CLAUDE.md` §3. An
    adversarial review found it failed *open* on a crash; it now fails closed, and its tests assert
    the exit status and are checked against reintroduced bugs.
  - **The allow-list is a trust decision.** Anything the agent may run without a prompt can write
    files or run code *around* the hook: `docker compose` (the container bind-mounts the repo
    read-write, `.claude/` included, and the container user has passwordless `sudo`), `colcon` and
    `xacro` (both run repo-controlled code), `git commit`, and
    `git diff --output=<file>`. The branch accepts that for an interactive loop run by the
    maintainer. For an unattended or less-trusted run, move `docker compose` and
    `git commit` back to `ask`, or scope them; that is the maintainer's call, not the agent's.
    *Amended 2026-10-30:* `gh api` (any HTTP verb) was taken off the list and a deny list was added
    ([proposal 0002](../proposals/0002-unattended-run-settings.md), items C and D). `docker compose`
    and `git commit` stay unscoped; that proposal says why.
  - **A push publishes, and its limits rest on permission matching.** Everything on `ai-forward` is
    readable by whoever can read the GitHub repo, and the run journals hold local paths and tool
    output, so nothing secret may ever be committed. That another branch or extra arguments prompt
    relies on Claude Code's documented rule order (deny, then ask, then allow; unlisted commands
    prompt), on the exact-match form of the two allow rules, and on the permission mode being
    `default` (or, untested, `acceptEdits`) with no extra rule from `settings.local.json`, a user's
    own settings or an answered "don't ask again". Since 2026-10-30 a push that starts `git push
    --force`, `-f`, `--force-with-lease`, `--delete` or `--no-verify` is refused outright by the
    deny list (all five tried, in `auto` mode); `git push origin ai-forward --force` is not, and
    still falls to the prompt in `default` mode (not tried, nor in `acceptEdits`). It does not stop
    `gh api` (off the allow-list since 2026-10-30, but in `auto` the classifier approved a read-only
    call; it can move or delete any remote ref), a chain of allowed commands, or a changed push
    target in `.git/config`. The bare `git push origin ai-forward` has run three times, and each run
    has a permission record in the session transcript reading `reasonType: rule`, `ruleSource:
    projectSettings`: the exact rule let it through. Written `git push origin ai-forward 2>&1 | tail
    -N; echo ...`, it did not match the rule and the `auto` classifier approved it instead, which in
    `default` mode should have prompted (not tried). Which of the pipe, the redirect or the `echo`
    broke the match is not known, and neither is whether a `cd ... &&` prefix would. If a human
    starts committing to `ai-forward` it becomes a shared branch, and the agent stops when the
    remote has commits it lacks, which is the intended behavior.
  - **`CLAUDE.md`, the agent definitions and the skills are not guarded.** An agent could edit the
    rules it works under. The control is a human reading the diff of those files before merging.
  - **Students learn less by doing.** The explanation trail narrows the gap but does not close it.
    If a course outcome depends on a student writing the control code, that student should work on
    `main`.
  - **Sim-tuned gains may mislead.** The `tune-gains` skill requires held-out scenarios and a stated
    sim-to-real gap; neither exists until Tier 3 does.
- **Follow-on work:** build Tier 3 (`bootstrap-sim`; choose between `SimPlant` and Gazebo, see
  ADR 0003); decide whether `main` should adopt any of this; add the tipover cutoff and watchdog
  (findings H-4, C-3), which the AI may write but whose thresholds are a hardware decision.

## References

- [`CLAUDE.md`](../../CLAUDE.md) — the operating instructions
- [`.claude/hooks/guard-protected-files.sh`](../../.claude/hooks/guard-protected-files.sh) and its
  tests, `test-guard-protected-files.sh` — the tripwire for that boundary (not a proof; see the
  docstring of `guard_protected_files.py` for its limits)
- [Verification ladder](../architecture/verification-ladder.md), [proposals](../proposals/README.md),
  [run journal](../journal/README.md)
- [`balance-loop-findings.md`](../claude-analysis/balance-loop-findings.md) C-2 — the failure mode a
  weak verifier repeats
