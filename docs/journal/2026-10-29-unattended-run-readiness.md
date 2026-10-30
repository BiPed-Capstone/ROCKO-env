# Get the repo ready for a run nobody is watching: rules, queue and git hooks

- **Date / commit:** 2026-10-29, built on `b101768`; the commit that adds this file
- **Branch:** `ai-forward`
- **Environment:** the maintainer's laptop (macOS), `~/projects/ROCKO-env`; git 2.48.1, Claude Code
  2.1.309 (the desktop app that ran this session; the `claude` on the PATH is 2.1.290). No container was needed (see *Verification*)

## Goal and success criteria

The maintainer asked what settings, files, instructions or permissions would have to change for the
agent to implement the work queue "fully autonomously", and then told me to write that up as a
proposal and to update the instruction files that are not guarded ("Yes, do both").

1. Every change to a guardrail (`.claude/settings.json`, `.claude/settings.local.json`, `.git/hooks/`,
   the guard hook, a GitHub setting) is in a proposal a person can accept item by item, with the
   exact lines, and **none of those files is edited** by this change.
2. `CLAUDE.md` says what to do when a call is refused, how to pick work, when to stop and where to
   report. `docs/next-steps.md` has a fixed set of status words and a rule that picks the rows an
   unattended run takes; run against the current table, the rule picks exactly 0.1 to 0.6 and 0.9.
3. The two git hooks refuse what the proposal says they refuse (a commit on `main`; any push but a
   fast-forward of `ai-forward` on `origin`) and allow the rest, installed the way the proposal says,
   and each check can be seen to fail.
4. Nothing claims more than was tested. In particular the permission-rule strings and the guard
   hook's behavior under another permission mode are marked *not verified*.
5. The guard hook's tests still pass.

## Plan

New: `docs/proposals/0002-unattended-run-settings.md`, and `.githooks/` (`pre-commit`, `pre-push`,
`test-git-hooks.sh`, `mutation-check.py`). Edited: `CLAUDE.md` §2 (an *Unattended runs* paragraph),
`docs/next-steps.md` (status words, item 6 of *How to use it*, status wordings, one decision bullet),
the ladder (an evidence row, a blind spot, a Tier 0 script), `README.md` (folder map), the two
indexes. The git hooks are tooling, so this has a journal entry; the rest is documentation. Risks: the
rule must not take a row nothing can check; the hooks must not be live before a person has read them.

**The boring alternative, rejected:** let an unattended run take *any* row whose *Who* is `agent` or
`agent + robot`. That would include Wave 1: a UART migration that needs a wiring change (1.1) and a
gyro axis and sign that "must be established on the robot" (1.2). A person should name those rows. I
therefore narrowed what I told the maintainer ("rows whose Who is `agent`") to Wave 0, and this is
the one place this change differs from that message.

**Not done on purpose:** the `.githooks/` files are in the tree, but nothing runs them until a person
copies them into `.git/hooks/`; and the guard hook does not yet guard `.githooks/`, which is why the
proposal orders that step first.

## What changed

| File | Change |
| --- | --- |
| `docs/proposals/0002-unattended-run-settings.md` (new), `docs/proposals/README.md` | The proposal: six items A to F, the exact lines, evidence, risks, a test for a human. Index row |
| `.githooks/pre-commit`, `pre-push` (new) | Refuse `git commit` on `main`; refuse every push but a fast-forward of `ai-forward` on `origin`. `ROCKO_HUMAN=1` lets a person through |
| `.githooks/test-git-hooks.sh`, `mutation-check.py` (new) | 24 cases on a throwaway repository and remote; the check that runs the unbroken test, then breaks each of twelve hook checks and requires the test to go red |
| `CLAUDE.md` §2, §8 | *Unattended runs*: a refusal is a hook denial, how to pick work, when to stop, where to report; and the new `.githooks/test-git-hooks.sh` line in §8's command block |
| `.claude/skills/build-verify/SKILL.md`, `docs/guides/verify-in-container.md` | A line each on running the git hooks' test when `.githooks/` changes |
| `docs/next-steps.md` | Status words; item 6 of *How to use it*; 0.7 and 0.8 read *blocked on human*, 0.9 *partly done*, 2.3 *open*, 2.4 *open* (needs 2.3); proposal 0002 in the decisions list |
| `docs/architecture/verification-ladder.md` | A Tier 0 script, an evidence row for the hooks, a blind spot: no tier tests a permission rule |
| `README.md`, `docs/journal/README.md` | Folder map line; index row |

## Verification

| Tier | Command | Result |
| --- | --- | --- |
| 0 | `sh -n` on both hooks, `bash -n` on the test, `py_compile` on the mutation check | pass |
| 0 | `.githooks/test-git-hooks.sh` | 24 `ok`, `PASS: every case behaved.`, exit 0. Also with the hooks run by `dash` (a stand-in for Ubuntu's `/bin/sh`) and in SHA-256 repositories |
| 0 | `.githooks/mutation-check.py`: the unbroken test, then twelve checks broken one at a time in a copy | `BASELINE ... green`, then red each time, with between 1 and 11 cases: any ref name allowed; a ref matched by its ending; any remote allowed; a deletion not recognised; no fast-forward check; refusing only when we have the remote's commit; a refusal that forgets its `exit 1`; the `ROCKO_HUMAN` escape removed from `pre-push`, and any value of it accepted; `main` not detected; the escape removed from `pre-commit`, and any value accepted. `PASS`, exit 0 |
| 0 | The same check on a copy whose `pre-push` allows everything (so the unbroken test is red) | `BASELINE is not green (exit 1)`, the failing cases listed, exit 2, no temporary folder left behind |
| 0 | `GIT_DIR`, and separately `GIT_CONFIG_PARAMETERS` (which `git -c` hands down), pointed at a stand-in "real" repository or remote, then `.githooks/test-git-hooks.sh` | the stand-in kept its one commit and gained no hooks; the stand-in remote stayed empty. With the `unset` lines removed from a copy and `GIT_DIR` at a stand-in repository, the test committed to it (10 commits), added an `elsewhere` remote to its config and pushed five refs (three branches, two tags) to its remote |
| 0 | A scratch repository with the hooks as `core.hooksPath .githooks` (relative), then `git switch main` | a commit and a push of `main` both went through: `main` has no `.githooks/`. With the scripts copied into `.git/hooks/` instead, the same commit and `git push --dry-run origin ai-forward:main` were refused |
| 0 | A git worktree: `cp ... .git/hooks/` there, then the same with `"$(git rev-parse --git-common-dir)/hooks/"` | `cp: .git/hooks: Not a directory` (inside a worktree `.git` is a file); the second form installs, and the hooks then refuse a commit on `main` made from the clone's own folder |
| 0 | A Python snippet that parses the Wave 0 table and applies item 6's rule | takes 0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.9; skips 0.7 (human) and 0.8 (no check) |
| 0 | A Python snippet that tries the proposal's new `GUARDRAIL` pattern on nine paths | matches the five it should, not the four it should not. The hook file itself was not edited |
| 0 | `.claude/hooks/test-guard-protected-files.sh` | exit 0 (81 `ok`) |
| 0 | Guardrails untouched: `git diff --stat -- .claude/settings.json .claude/hooks .claude/commands .mcp.json` is empty, and `stat` on `.git/config`, `.claude/settings.json`, `.claude/settings.local.json`, the hook, plus `ls .git/hooks` | no tracked guardrail differs from `HEAD`; the modification times are 2026-10-28, 2026-10-28, 2026-08-11 and 2026-10-27; `.git/hooks` holds only samples. (`git status` alone cannot see `.git/` or ignored files, which is why `stat` is here.) |
| 0 | A link check of the changed documents (relative links; anchors by eye) | passes |
| 1, 2 | not run | nothing they exercise changed: no source, launch, URDF or build file |
| 3, 4 | not run | Tier 3 is not built; Tier 4 is a person, and this is not a robot step |

## Review rounds

| Round | Reviewer | Findings | Disposition |
| --- | --- | --- | --- |
| 1 | `explainability-reviewer` (the three scripts) | 2 High, 5 Medium, 6 Low. High: two "refused" cases passed because *git* refused the push, so the hook's protection against a forced overwrite was never tested (a mutant that kept the whole test green was shown); the test acted on a real repository and remote whenever `GIT_DIR` was set. Medium: the key "git already refuses a non-forced push" was missing; "refuse a commit on main" overclaimed (only `git commit` runs it); the headers did not say what a hook is; the `*[!0]*` glob needed explaining | All **fixed**. I reproduced the `GIT_DIR` case and kept the reviewer's mutant in the mutation check. The all-zeros test is now git's own idiom; `git branch --show-current` replaces `symbolic-ref`; an origin-only check was added (the reviewer left that to me) |
| 1 | `docs-reviewer` (the documents) | 4 High, 9 Medium, 8 Low. High: `core.hooksPath .githooks` does not protect `main`; `Bash(bash -n:*)` runs any code; I said the transcript does not record the permission mode, and it does (`auto`); stale numbers. Medium: a test step that printed the wrong result; test steps not all harmless; "(D denies it)" overstated; item F needs an administrator; a departure from ADR 0005; item 6 never runs out of work; gaps in the `CLAUDE.md` paragraph; a guardrail check that could not fail; `chmod +x:*` can clear bits | All **fixed** or answered in the proposal. I reproduced the first and third High findings myself (a commit and a push of `main` went through; the transcript says `auto`). The `bash -n +n` and `chmod +x,a-x` holes were accepted on the reviewer's demonstration, not re-run by me; the cost of removing them is a prompt. Not changed: `CLAUDE.md` §3 says a pushed branch is "readable by whoever can read the team's GitHub repo", and the repository is public, so that means anyone (a Low finding outside this change; I left a section of the rules alone) |
| 2 | `explainability-reviewer` (the four scripts) | 1 High, 5 Medium, 4 Low. High: `mutation-check.py` printed PASS when the unbroken test was already red (shown with a git setting that stops every hook), and labelled a crashing test "NOT CAUGHT". Medium: `git -c` settings (`GIT_CONFIG_PARAMETERS`) were not cleared, so the test could push to a remote that setting named; the `refused:0:*` line in `check` looked redundant and was not (without it four cases pass for the wrong reason); no case for a tag named `ai-forward`; `pre-commit` allowed a commit if `git branch --show-current` failed, and "moves no branch" is false during a rebase; the stated reason against `core.hooksPath` pointed at a wrong fix (an absolute path fails the same way) | All **fixed**. The baseline check is the reviewer's rewrite, which also removes a confusing `try/finally`. Added the cases for a tag named `ai-forward` and for `ROCKO_HUMAN=0`, and four mutations for them. Not added: a SHA-256 case (GitHub does not host SHA-256 repositories; I ran it by hand and the test header says so) |
| 2 | `docs-reviewer` (the documents) | 2 High, 8 Medium, 13 Low. High: the case counts were stale after the scripts changed during the review; the first human test step could not fail for `.claude/settings.json`. Medium: the permission-record breakdown hid the 7 person-answered prompts; wrong Claude Code version; step 2 of E fails inside a git worktree; the "without the fix" `GIT_DIR` evidence was stale; the "if accepted" list was incomplete; the `CLAUDE.md` paragraph missed `verify_urdf.sh`; "it prompts" and `ROCKO_HUMAN` stated as fact; the new Tier 0 script was only half wired in. Low: wording, a status word, reproducibility | All **fixed**. I reproduced the worktree failure, the permission breakdown, the version and the `GIT_DIR` numbers before changing the text. The hook's refusal message no longer prints the override |
| 3 | `docs-reviewer` (the last round the loop allows) | 0 High, 3 Medium, 8 Low and a nit. Medium: the `CLAUDE.md` mount-check rule fired only on a refusal, but the paragraph tells the agent to avoid commands that prompt (so the check would simply be skipped); the journal still said 2.4 was *blocked on human*; the human test's precondition (a clean tree) conflicted with applying the accepted items first. Low: two places missing from the "if accepted" list, a wrong ref count, a README line with literal backticks, and wording | All **fixed**, and the fixes are **not re-reviewed**: round three was the last, so they were checked by rerunning the scripts, the link check and a search for stale text only |

## Corrections to what I told the maintainer

- I said the previous session's transcript "does not say" whether prompts were approved or
  auto-approved. It does: the mode is `auto`, and a classifier accepted most calls (see the proposal,
  item A).
- I said "no default mode is set, so every edit prompts". That holds in `default` mode only; this
  repo is worked in `auto`.
- I proposed `core.hooksPath .githooks` for the git hooks. It silently stops working on `main`; the
  proposal now copies the scripts into the clone's hooks folder.
- I cited `claude --version` (2.1.290) as the version behind the evidence. The session ran in the
  desktop app, 2.1.308 and then 2.1.309.
- The proposal's first human test step asked for a prompt on `.claude/settings.json` as proof the hook
  fires. Claude Code asks for its own settings file anyway (the transcript shows two such prompts
  answered by a person), so that half could not fail; it now uses `.githooks/pre-push`.
- I broke the permission-record counts into too few kinds: 7 of the decisions are a person answering a
  prompt, so `auto` still stops for a person on some calls.

## Not verified — needs a human

- **Every `Bash(...)` rule string in the proposal.** Whether it matches the command as an agent types
  it, whether `git push --force:*` also catches `--force-with-lease`, and whether
  `.claude/hooks/test-guard-protected-files.sh` is matched as written.
- **The guard hook under `acceptEdits` or `auto`.** Its tests run the script; they do not show how
  Claude Code treats its answer. And what the `auto` classifier approves is unknown.
- **The git hooks in this clone.** They were tested on a throwaway repository only. Nothing is
  installed in `.git/hooks/` here.
- **The GitHub ruleset.** It needs an administrator; this account has `WRITE`.
- **The fixes made after round 3** (above): no reviewer has read them.
- **That a refusal really stops an unattended run cleanly.** The new `CLAUDE.md` paragraph says what
  the agent should do; nobody has watched a run do it.

## Reproduce

From the repo root on the laptop; none of this touches the container or the real remote:

```bash
sh -n .githooks/pre-commit && sh -n .githooks/pre-push && bash -n .githooks/test-git-hooks.sh
.githooks/test-git-hooks.sh; echo "exit=$?"            # PASS, exit 0
.githooks/mutation-check.py; echo "exit=$?"            # "BASELINE ... green", twelve lines starting "caught", PASS, exit 0
.claude/hooks/test-guard-protected-files.sh; echo "exit=$?"
git diff --stat -- .claude/settings.json .claude/hooks .claude/commands .mcp.json   # prints nothing (tracked files only)
git status --porcelain --ignored -- .claude .mcp.json   # lists the ignored settings.local.json and any new file under .claude/
stat -f '%Sm  %N' -t '%Y-%m-%d %H:%M' .git/config .claude/settings.json .claude/settings.local.json
ls .git/hooks | grep -v sample                          # prints nothing: no hook is installed in this clone
```

The `core.hooksPath` and worktree experiments used throwaway repositories and are described, not
scripted: `git init`, commit the two hooks on a branch, `git config core.hooksPath .githooks`,
`git switch main` (which has no `.githooks/`), then `git commit --allow-empty`. The Python snippets in
the table (the Wave 0 parser, the `GUARDRAIL` pattern) were typed at a prompt and are not kept; each
is a few lines that apply the rule or the regex quoted in the proposal.
