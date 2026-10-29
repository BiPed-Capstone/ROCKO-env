# Make the verify scripts notice when they are testing the wrong checkout

- **Date / commit:** 2026-10-29, built on `329be04`; the commit that adds this file
- **Branch:** `ai-forward`
- **Environment:** the maintainer's laptop (macOS, Docker Desktop, arm64), `~/projects/ROCKO-env`;
  Docker Engine 28.3.2, Compose 2.39.1-desktop.1; the `rocko-env-dev` container (`ros:jazzy`)

## Goal and success criteria

While answering "is this repo ready for an AI agent to work in a new session?" a gap turned up:
`docker/compose.yml` mounts `..` (the checkout holding the compose file) at `/ros2_ws/src/ROCKO-env`
and names the container `rocko-env-dev`. There is one container per laptop, and it shows whichever
checkout last ran `up -d`. A session in a git worktree or a second clone that does not run `up -d`
edits one folder while `build_ros.sh`, `verify_urdf.sh` and `verify_mock.sh` run against another, and
can pass. That is a verifier that cannot fail for the thing it is supposed to check. The maintainer
asked to document the single-checkout requirement and add a host-side check that the container mounts
the repo being edited.

1. A script run on the laptop exits 0 when the container sees the checkout the script lives in, 1
   when it does not (and prints the fix), and 2 when it cannot tell.
2. It fails (exit 1) in the real failure case: run from a git worktree while the container mounts the
   main checkout. It passes once the container is pointed at the worktree, and again at the main
   checkout.
3. It leaves no file behind in the checkout, on a pass or a fail.
4. The docs say that there is one container per laptop, say to run the check first, and do not claim
   more than was tested.
5. The existing tiers still pass after the container has been recreated and restored.

Criterion 4 grew during review (below) to include what the check does **not** prove.

## Plan

One new script, `check_container_mount.sh`, at the repo root next to the other scripts. Docs:
`CLAUDE.md` §8, `build-verify`, `add-hardware` (a pointer), `docs/guides/verify-in-container.md` (the
canonical explanation), `README.md`, the ladder (evidence, blind spots). The script is not on the
`settings.json` allow-list and that file is a guardrail, so the allow rule goes in
[proposal 0001](../proposals/0001-allow-container-mount-check.md) instead. Risks: the check must not
pass for the wrong reason, and testing it means recreating the maintainer's running container.

**The boring alternative, rejected:** compare the folder Docker reports as the mount source with the
current folder. Docker Desktop reports `/Users/ghart/...` as `/host_mnt/Users/ghart/...`, and
Windows and WSL2 spell paths differently again, so the comparison needs per-OS rules. A marker file
asks the question that matters directly ("can the container see the files I am editing?") and has no
OS-specific part.

## What changed

| File | Change |
| --- | --- |
| `check_container_mount.sh` (new) | Creates `mktemp .mount-check.XXXXXX` in its own checkout, asks the container with `exec -T ros2 test -e`, removes the file on exit. Exit 0, 1 or 2; the FAIL text names the fix (`up -d --force-recreate`) and the clean rebuild |
| `.gitignore` | `.mount-check.*`, for the one case the script cannot clean up after (SIGKILL) |
| `CLAUDE.md` §8 | The check in the command block, and a paragraph "One container, one checkout" |
| `docs/guides/verify-in-container.md` | The canonical explanation: definitions, why `up -d` is itself the switch, what `ok` does not prove, the fix and its costs |
| `.claude/skills/build-verify/SKILL.md`, `add-hardware/SKILL.md` | A short section / pointer, linking to the guide and the ladder |
| `README.md` | The script in the folder map; a paragraph in the container section |
| `docs/architecture/verification-ladder.md` | A precondition sentence, five evidence rows, two blind spots, a "caught" row, the last-verified note |
| `docs/proposals/0001-allow-container-mount-check.md` (new), `docs/proposals/README.md`, `docs/next-steps.md` | The allow-rule proposal, and the per-checkout-container question listed as waiting on the maintainer |

## Verification

| Tier | Command | Result |
| --- | --- | --- |
| pre | `bash -n check_container_mount.sh` | pass |
| pre | `./check_container_mount.sh` from the main checkout, then 20 times in a loop | `ok    the container sees this checkout: /Users/ghart/projects/ROCKO-env`, exit 0, 20 of 20; no stray `.mount-check.*`. Shows the marker was never missed on this Docker Desktop for macOS mount; says nothing about Linux or WSL2 |
| pre | `COMPOSE_PROJECT_NAME=zzz`; `PATH` without `docker`; run through a symlink | exit 2 each time: `service "ros2" is not running` then `ERROR: cannot reach ...`; `ERROR: no docker command here ...`; `ERROR: no docker/compose.yml next to this script ...` |
| pre | a `git worktree` in the scratchpad with the script copied in, container still on the main checkout | `FAIL  the container does NOT see this checkout: <worktree>`, exit 1, with the full fix text |
| pre | the printed `docker compose -f <worktree>/docker/compose.yml up -d --force-recreate`, then the check from the worktree and from main | `ok` (exit 0) from the worktree, `FAIL` (exit 1) from main |
| pre | delete that worktree and make a new one at the **same path** | `FAIL`; plain `up -d` printed `Running` and the check still failed; `up -d --force-recreate` fixed it. This is why the printed fix has the flag (an earlier draft said it was not needed: that held only when the path differs) |
| pre | `up -d --force-recreate` from the main checkout, then the check | `ok`, exit 0; `docker inspect` shows the mounts back on `/host_mnt/Users/ghart/projects/ROCKO-env` |
| pre | SIGINT, SIGTERM, SIGHUP, then SIGKILL sent to the script's process group (Python, `start_new_session`) while a stand-in `docker` hung on the second call and the marker existed | marker removed for INT, TERM and HUP (exit -2, -15, -1); SIGKILL (exit -9) left it |
| 1 | the stale-build experiment: append a comment to `AddFeedforwardController.cpp`, `colcon build --packages-select rocko_env`, restore the original with `git checkout` and `touch -r` to its old mtime, build again | build 1 had nothing to do; build 2 compiled the object (`Building CXX object ...AddFeedforwardController.cpp.o`); build 3 recompiled nothing (`Finished <<< rocko_env [0.20s]`). The file is restored (`git status` clean for it). So output from another checkout survives a switch |
| 1 | the printed clean rebuild: `exec -T ros2 bash -c 'cd /ros2_ws && rm -rf build install log && src/ROCKO-env/build_ros.sh'` | rosdeps reinstalled, `Summary: 2 packages finished [9.98s]`, `Done!`, exit 0 (a real compile, unlike the 0.5 s runs before it) |
| 0 | `verify_urdf.sh`; `.claude/hooks/test-guard-protected-files.sh` | `PASS`; exit 0 (81 cases) |
| 2 | `verify_mock.sh`, `verify_mock.sh tank` | both `PASS`, exit 0. A wiring check, no physics |
| 3, 4 | | not run: Tier 3 is not built; Tier 4 is a person |

The negative controls are the worktree, same-path, exit-2 and stale-build rows: the check failed for
the reason it exists and passed once the container was pointed at the folder under test.

## Review rounds

| Round | Reviewer | Findings | Disposition |
| --- | --- | --- | --- |
| 1 | `explainability-reviewer` | 1 High, 4 Medium, 8 Low. High: "the build volume is kept" was offered as reassurance, but it keeps another checkout's compiled output. Medium: the "one fixed folder" model was wrong; the line that makes the script work had no comment; Docker's error was thrown away so the message guessed; the FAIL text could not be acted on. Low: array instead of a function, `pwd -P`, mktemp error, style drift from the sibling scripts, wording in `CLAUDE.md` | All **fixed**. Most of the script now follows a rewrite the reviewer wrote and ran; I read it in full and kept it except for the FAIL text |
| 1 | `docs-reviewer` | 1 High (same stale-build point), 6 Medium, 12 Low. Medium: the ladder claimed more than was run; a recreate also discards the installed ROS packages; `up -d` is itself the switch; plain `up -d` can loop in the same-path case; the README pointed at the wrong machine; undefined jargon. Low: proposal wording, stale tier references, duplicated status, a missing pointer in `add-hardware` | All **fixed** or checked by running (the stale-build and same-path claims were reasoned by the reviewers and then **tested**; both held). Not taken: a `docker inspect` call inside the script to name the mounted checkout (the message prints the command instead) |

No second round: the fixes after this table were verified by running the commands above, not
re-reviewed.

## Not verified — needs a human

- **Linux and WSL2.** Only Docker Desktop for macOS was tried. The marker-file method uses nothing
  OS-specific, but a bind mount with a slow sync could miss a file that was created a moment before
  (none did in 20 runs here).
- **That `Bash(./check_container_mount.sh)` would work as an allow rule**, and that the script
  prompts without it. This session may have been auto-approved. See the proposal.
- **That the build volume is the only stale thing.** The experiment showed a compiled object
  surviving a switch. Python and launch files are symlinked (`--symlink-install`), so they follow the
  mounted checkout; that reasoning was not tested.
- **Two sessions at once.** The check says whether the container sees one checkout *now*. A second
  session that recreates the container for its own checkout takes it away again; run the check after
  switching. A container per checkout (a compose project name per checkout and no fixed
  `container_name`) would remove the problem and was not built: it is a bigger change to
  `docker/compose.yml`, and it is listed in [next-steps](../next-steps.md) for the maintainer.
- **The first-time image build** was not run (the image already existed).

## Reproduce

From the repo root on the laptop, with the container up (the commands in the first block do not
change it):

```bash
bash -n check_container_mount.sh
./check_container_mount.sh; echo "exit=$?"                                  # ok, exit 0
for i in $(seq 1 20); do ./check_container_mount.sh >/dev/null || echo "run $i failed"; done
COMPOSE_PROJECT_NAME=zzz ./check_container_mount.sh; echo "exit=$?"         # exit 2
git worktree add --detach ../ROCKO-env-wt HEAD && cp check_container_mount.sh ../ROCKO-env-wt/
../ROCKO-env-wt/check_container_mount.sh; echo "exit=$?"                    # FAIL, exit 1
```

(`cp` is needed only until the script is committed.) To see the fix work, which **recreates the
container**, run the first command the FAIL text printed, check from both folders, then put it back
and rebuild from clean:

```bash
docker compose -f ../ROCKO-env-wt/docker/compose.yml up -d --force-recreate
../ROCKO-env-wt/check_container_mount.sh; ./check_container_mount.sh       # ok, then FAIL
docker compose -f docker/compose.yml up -d --force-recreate
./check_container_mount.sh                                                  # ok
docker compose -f docker/compose.yml exec -T ros2 bash -c 'cd /ros2_ws && rm -rf build install log && src/ROCKO-env/build_ros.sh'
git worktree remove --force ../ROCKO-env-wt
```

Then the tiers, as in [Verify a change without the robot](../guides/verify-in-container.md).
