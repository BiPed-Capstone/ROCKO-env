# Make the AI-forward loop real: mock hardware, off-Pi build, verify scripts

- **Date / commit:** 2026-10-27, built on `923fa38`; the commits on `ai-forward` that add this file
- **Branch:** `ai-forward`, cut from `main` at `923fa38` (the maintainer's `build_ros.sh` and compose
  fix; both branches were pushed to `origin` on 2026-10-28, at the maintainer's instruction)
- **Environment:** `rocko-env-dev` container, `ros:jazzy` **arm64** on Docker Desktop for macOS,
  `pid_controller` 4.42.1. Nothing was run on a Raspberry Pi. x86_64 untried.

## Goal and success criteria

The maintainer asked for an alternative branch where the repo is code intended to be written by AI:
the agent loop does the programming, iterates by itself, and runs code to verify its work, while code
stays simple and explainable and documentation stays a priority. Choices they made: autonomy bounded
by hardware (a human decides gains, version pins and weakening safety behavior); build the first rung
of the simulation ladder as part of this change; commit their uncommitted work to `main` first; an
interactive loop with no mid-loop gates.

Checkable criteria:

1. The hook enforces that boundary, **fails closed**, and its tests can fail.
2. `build_ros.sh` builds `rocko_interfaces` and `rocko_env` in the container, **exits 1 when the
   build fails**, and a build without the opt-out flag is refused.
3. `use_mock_hardware:=true` selects a real mock in balancing and tank mode; the real (`false`) path
   is unchanged; every launch file defaults to `false`.
4. `verify_urdf.sh` and `verify_mock.sh` pass in every mode **and fail** on deliberate breaks, for
   the right reason.
5. Docs change in the same commit; reviewers come back clean or each finding is declined with a
   reason.

## Plan

Hook and settings first (so later edits to `CMakeLists.txt` follow the new policy), then the CMake
opt-out, then the mock in xacro and launch, then build and verify in the container, then the
instruction files, docs and reviews. Tier 2 needed controller packages and a working launch, so it
was expected to find problems; it found four (below), and the reviews found more, including in the
verifiers themselves.

## What changed

| Area | Change | Why |
| --- | --- | --- |
| `.claude/hooks/` | Guard rewritten as `guard_protected_files.py` (+ a short `.sh` wrapper that just runs it): fails closed, judges whole lines of the whole file, denies gain/pin/safety edits, asks on guardrail edits. 70-case test suite | [ADR 0005](../decisions/0005-ai-forward-development.md) |
| `settings.json` | `docker compose`, `git add/commit`, `colcon test`, `ros2 topic echo` allowed; `git push` asked (changed 2026-10-28: see [the push entry](2026-10-28-allow-ai-forward-push.md)) | ADR 0005, and its trust-decision caveats |
| `rocko_env/CMakeLists.txt` | `ROCKO_SKIP_WIRINGPI` option; default OFF fails when WiringPi is missing; ON where WiringPi exists also fails (stale cache) | Off-Pi build ([ADR 0003](../decisions/0003-simulation-target.md) follow-on 2) |
| `build_ros.sh`, `docker/compose.yml` | Always pass the flag (`ON` in the container); exit 1 if the build fails | A failed build used to say `Done!` and exit 0 |
| `*.ros2_control.xacro` (2) | `xacro:if` mock block (one `MockRocko` system); real blocks under `xacro:unless` | Follow-on 1: `use_mock_hardware` finally does something |
| `*.launch.py` (3) | `use_mock_hardware` default `false`; sensor nodes skipped and a banner printed under mock; URDF passed as a string parameter | Fail-closed default; mock launch works |
| `package.xml` | `exec_depend` on `pid_controller`, `diff_drive_controller` | The launch already needed them |
| 5 Python scripts | Executable bit set in git | `--symlink-install` runs them in place |
| `verify_urdf.sh`, `verify_mock.sh` | New Tier 0 and Tier 2 checks | The loop's sensors |
| `CLAUDE.md`, skills, agents | Autonomous loop, verification ladder, simplicity bar, `bootstrap-sim`, `explainability-reviewer` | The request |
| `docs/` | Ladder, verify guide, ADR 0005, ADR 0003 progress, proposals and journal homes, findings M-5 / L-3 / L-4 and M-4 status, README | Same-commit docs |

## Verification

All in the container unless noted.

| Tier | Command | Result |
| --- | --- | --- |
| 0 static | `verify_urdf.sh` | **pass**: 4 expansions parse and hold the right hardware; the mock exports the same 12 (balancing) / 6 (tank) joint interfaces as the real hardware |
| 0 static | real-mode `<ros2_control>` XML vs `HEAD`'s, compared semantically | identical: 3 of 3 blocks (balancing), 2 of 2 (tank) |
| 0 static | `.claude/hooks/test-guard-protected-files.sh` (host) | **pass**: 80 hook cases plus a settings-wiring check |
| 0 static | `.claude/hooks/mutation-check-guard.py`: 28 bugs reintroduced one at a time into copies of the hook | the suite went red for **28 of 28** |
| 1 build | `src/ROCKO-env/build_ros.sh` | **pass**, exit 0, `rosdep` installed `pid_controller` / `diff_drive_controller` from `package.xml` by itself |
| 1 build, negative | same, flag forced `OFF` (no WiringPi here) | `WiringPi not found`, `ERROR: colcon build failed`, **exit 1**, no `Done!`; the next normal run recovered |
| 1 build, negative | fake `libwiringPi` + flag `ON` | CMake stops: `WiringPi IS installed here ... stale CMake cache` |
| 1 build, Pi path | fake `libwiringPi` + flag `OFF` | configures with 0 errors and compiles `Motor12VoltQuadEncoder.cpp`, which fails on the missing `wiringPi.h` as it must off-Pi. **Not a Pi build** |
| 2 mock | `verify_mock.sh` | **pass**: all four cascade controllers `active`, wheel velocity commands claimed |
| 2 mock | `verify_mock.sh tank` | **pass** |
| 0 static, negatives | swap the mock's `if`/`unless`; delete a `yaw` state from the mock (parity); give only the mock a `<sensor>`; make the real hardware export an interface twice; put a third plugin in the mock block | **FAIL** exit 1 each, for the right reason |
| 2 mock, negatives | delete the `pitch` state; strip `Joystick.py`'s exec bit | **FAIL** exit 1 each, for the right reason |
| 2 mock, refusal | start a mock launch, then run `verify_mock.sh` | `ERROR: ... already running`, exit 2 |
| 3 sim | — | **not built** |
| 4 robot | — | **not run**: no robot |

Measured: the mock's pitch `feedback` is exactly `0.0` (not NaN) and the controller computes every
cycle. After the final runs, 0 ROS processes were left in the container. Command output was kept in the
session scratchpad, not in the repo; every row above can be regenerated with the commands under
"Reproduce" (plus `.claude/hooks/mutation-check-guard.py`).

**Warnings read:** `AddFeedforwardController.cpp:150` ignores a `[[nodiscard]]` `set_value()` result
(finding **M-5**); a deprecated `realtime_buffer.h` include and two invalid-escape `SyntaxWarning`s in
`RateLimiter.py` (**L-3**); "manually-specified variable not used" for `ROCKO_SKIP_WIRINGPI` in
`rocko_interfaces`, which is harmless because colcon passes `--cmake-args` to every package.

**Defects found by running the checks, and fixed:**

1. `pid_controller` and `diff_drive_controller` were not in `package.xml`, so neither launch worked on
   a fresh machine.
2. The URDF was passed to `robot_state_publisher` unwrapped; any `: ` in an xacro comment broke the
   launch with a YAML parse error. My own mock comments triggered it. Real mode was unaffected.
3. Five Python executables were committed as `100644` on `main` too, so under `--symlink-install`
   they could not run (`PermissionError`).
4. `build_ros.sh` could not fail: no `set -e`, ends in `echo`, so a failed build said `Done!` and
   exited 0. (Found by the docs review; reproduced; fixed; proven with a forced failure.)
5. My own verifiers had bugs, each found by running or reviewing them: `set -u` died inside ROS's
   `setup.bash`; `ros2 control` blocks forever with no controller manager; a background job ignores
   `SIGINT`; orphaned controller processes survived; `verify_urdf.sh` counted a plugin *name* in an
   XML comment and failed a correct description; its first negative control passed for that same
   wrong reason; two hook-test cases were denied for the wrong reason (a regex that picked up a value
   in a comment; a `.replace` that left a literal `@@` behind) and so could not tell a good hook
   from a broken one.
6. The guard hook failed **open**: any crash (an oversized `Write`, a stray non-UTF-8 byte) made the
   harness skip it. It was then reviewed twice more and rewritten around what the reviews found; the
   round-3 fixes include its comparison of YAML lines, which had lower-cased keys and deleted
   spacing that YAML treats as meaningful (`{P : 10.0}` and `{p:10.0}` both passed as "the same").
7. Testing the `build_ros.sh` fix showed the container's `/ros2_ws/build_ros.sh` is a **stale copy**:
   a single-file bind mount, pinned when the container started, that no longer matched the host
   file after it was edited (different checksum; it ended mid-statement). My first "exit 1" there was
   the truncated script's, not the fix's. Not fixed in `compose.yml` (the maintainer's file); the
   commands now use `src/ROCKO-env/build_ros.sh`.

## Review rounds

Five reviewers in round 1 and four in round 2. The three project reviewer agent types were
unavailable for part of round 1 (the folder holding their definitions was gone, see the notes), so
round 1 used `general-purpose` agents told to follow the same definition files; round 2 used the real
agent types, with an adversarial `general-purpose` agent for the hook. `realtime-auditor` ran in
round 1 only: no critical-path file was edited, and its report is below.

| Round | Reviewer | Findings | Disposition |
| --- | --- | --- | --- |
| 1 | `realtime-auditor` | 0 Critical/High. Sticky CMake cache (Med); mock banner (Low); missing plugin non-fatal (Low); unpinned new deps (Low) | Cache and banner **fixed**; non-fatal **declined** (fails safe, wheels cannot be driven); deps **documented** |
| 1 | `ros2-control-reviewer` | 0 Critical/High. Mock pitch may be NaN (Med, plausible); stub plugin for skip builds; no assertion that `mock=false` holds real plugins; stale GUI launch; missing RViz install | NaN **refuted by measurement**; stub **declined** (error text documented); assertion **fixed** (`verify_urdf.sh`); GUI and RViz **recorded** (L-4) |
| 1 | `explainability-reviewer` | `fault` matched `default` (16 hits); `CMAKE_CXX_STANDARD` unguarded; `simple_gains.yaml` escaped the sim exemption; comments overclaimed; magic numbers | **All fixed** (and verified against the code first) |
| 1 | `docs-reviewer` | Stale ADR 0003 / M-4; "won't link"; `§7` pointer; "every controller in the YAML"; guide inaccuracies; shorthand commands; duplicated status | **Fixed**; remaining duplicates are dated records |
| 1 | adversarial hook review | Fail-open on crash (High); token swaps; commented-out pins; case/symlink/root spellings; CRLF; false positives; allow-list bypasses | Hook **rewritten** and its tests hardened; allow-list and unguarded `CLAUDE.md` **left to the maintainer** and documented (ADR 0005) |
| 2 | `ros2-control-reviewer` | 0 Critical/High. `verify_mock.sh` could pass against another controller manager (Med); stale-cache only a warning; `verify_urdf.sh` assertions weak; timing text | All **fixed**: refuses to run if one is up (exit 2); the stale-cache case is now a hard stop; exact counts, parity check, error output shown |
| 2 | `explainability-reviewer` | 0 High, 3 Med: gains vs pins compared differently without saying why; "new files are fine" untrue for gains; a broken fixture could print `ok` | **Fixed** (comments, docstring, empty-payload guard in the tests) |
| 2 | `docs-reviewer` | 2 High: `build_ros.sh` could not fail; the ladder's evidence was not in the journal. Case-sensitive filesystems; Pi wording; gain-key lists; counts | **Fixed** (this entry is the second one) |
| 2 | adversarial hook review | 3 High (pin line dies but token survives, `#` not YAML-aware, unmatched `old_string` allowed); medium and low | **Fixed** by judging whole lines per language; `if(FALSE)` wrap and pin-overriding additions **documented as limits** |
| 3 | `ros2-control-reviewer` | 0 Critical/High. Parity ignored `<sensor>`/`<gpio>` (Med); `sort -u` hid a duplicated interface; a failing helper printed `ok ... 0`; a third plugin passed; only the last line of an error shown; `pgrep` missing read as "none running" | All **fixed** and each shown to fail first; two simultaneous runs racing is **documented** |
| 3 | `docs-reviewer` | 1 High (the README container block would stop with `WiringPi not found`), 4 Med (six `ok` lines; exit-2 list; reproducibility; a `pkill` that leaves a stale `Joystick.py`), about 10 Low | **Fixed**; the mutation check is now a script in the repo and the 0.0-pitch measurement has its command |
| 3 | adversarial hook review | 4 ways past the gain guard and 1 hang: the Edit tool eats a newline when `new_string` is empty; a lone CR ends a YAML comment; YAML is case-sensitive and `p:10` is not `p: 10`; structure only a parser sees; a quadratic comment regex (400 KB took 84 s); plus cheap Lows | Newline-eating, lone CR, case and spacing, indentation, the linear scan and the cheap Lows **fixed with a test each**; structural YAML (quoted keys, `gains: |`) **documented as a limit** |

Round 3 was the last the loop allows. **The fixes made after it were verified by running, not re-reviewed:** the rewritten hook (suite: 80 hook cases plus the wiring check; mutation check: 28 of 28), the hardened `verify_urdf.sh` (each new check shown to fail first), and the README and docs corrections. A reviewer should read `guard_protected_files.py` and `verify_urdf.sh` with that in mind.
## Not verified — needs a human

Nothing here was run on a Raspberry Pi, and none of it shows the robot balances. A person should:

1. **Build on the Pi with the default (no flag):** `colcon build --symlink-install` from `~/ros2_ws`
   (or `src/ROCKO-env/build_ros.sh`). Expect it to **include** `Motor12VoltQuadEncoder.cpp` and
   succeed as before; the `OFF` branch has only been taken as far as compiling that file off-Pi.
   Expect `WiringPi not found` if WiringPi is missing — that is intended. If a `build/` directory was
   ever used for a skip build, `rm -rf build/rocko_env` first.
2. **Tethered, wheels off the ground,** `ros2 launch rocko_env rocko_tank_mode.launch.py` with no
   arguments. Expect real hardware (the motors respond to the gamepad), **not** the mock. Abort if the
   wheels do not respond or the launch prints `MOCK HARDWARE`. Record commit SHA, battery voltage,
   surface, where the log went ([reproducing results](../guides/reproducing-results.md)).
3. Confirm `ICM20948.py` and `QuadEncoders.py` still start in balancing mode (they are skipped only
   when `use_mock_hardware:=true`).
4. Confirm a Pi checkout keeps the five scripts executable, and check which versions of
   `pid_controller` / `diff_drive_controller` a fresh `rosdep install` pulls (they are not pinned).
5. **Read `.claude/hooks/guard_protected_files.py` yourself.** It was rewritten during this work and I
   never saw an approval prompt for editing it, so I cannot say you approved it.

Not covered by any tier: the motor plugin compiles (Pi only), x86_64 containers, `foxglove_bridge`,
and `rocko_gui.launch.py`, which looks stale. See the
[verification ladder](../architecture/verification-ladder.md).

**Decisions left to the maintainer** (all recorded in ADR 0005 or the audit): the allow-list in
`settings.json` (`docker compose`, `gh api`, `git commit` run code around the hook); whether
`CLAUDE.md`, the agents and the skills should be guarded; whether to add a PyYAML-based gain check
so the hook sees YAML structure, not just lines (it may not be installed on the host); the single-file
`build_ros.sh` mount in `docker/compose.yml`; Gazebo vs `SimPlant` for Tier 3 (ADR 0003 vs finding M-4); findings M-5 and L-4.

## Notes on how this run went

- Partway through, the maintainer's OneDrive `Documents` folder was reset by something outside this
  work, and the repo disappeared from its path. It was recovered from the Docker bind mount's view
  and the work continued in a non-synced local copy (`~/projects/ROCKO-env`). `923fa38` and this
  branch were local-only until they were pushed on 2026-10-28.
- Several of my own checks were wrong before they were right (item 5 above). The mutation check on
  the hook tests found two tests that did not test what they claimed; that is the reason the ladder
  asks for a control that fails *for the right reason*.

## Reproduce

From the repo root on your laptop, with Docker Desktop running:

```bash
docker compose -f docker/compose.yml up -d
docker compose -f docker/compose.yml exec -T ros2 bash -c 'cd /ros2_ws && src/ROCKO-env/build_ros.sh'
docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/verify_urdf.sh
docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/verify_mock.sh
docker compose -f docker/compose.yml exec -T ros2 /ros2_ws/src/ROCKO-env/verify_mock.sh tank
.claude/hooks/test-guard-protected-files.sh
```

The walk-through, with expected output, is [Verify a change without the robot](../guides/verify-in-container.md).
