# Measure what the agent loop cost, and change the tools and instructions that drive it

- **Date / commit:** 2026-11-08, built on `29d6541`; the commit that adds this file.
- **Branch:** `ai-forward`
- **Environment:** laptop (macOS, bash 3.2, Python 3); dev container for the one `repeat_check.sh` smoke test.

## Goal and success criteria

The person asked what the 2026-10-27 to 2026-11-08 runs cost, what they could have cost, whether other
models or settings would do better, and for the instruction files, skills and tooling to be changed where that
would make the loop more efficient.
1. A page with measured token use per run, per model and per agent, and an honest statement of what is not
   measured. **Done:** [agent-loop-efficiency.md](../claude-analysis/agent-loop-efficiency.md).
2. Helper tools that remove the biggest measured waste, each shown to work and, where it checks something,
   shown to fail on a break. **Done** for the scanner and for the "could not run" path of `repeat_check.sh`; see below.
3. Instruction, skill and agent changes that follow from the findings, without loosening CLAUDE.md §3 or §4.
   **Done.**

## What changed

| File | Change |
| --- | --- |
| `tools/session_usage.py`, `tools/session_prices.json` | Token use per agent and model from a session transcript; dollars only if the person fills in prices |
| `tools/run_ladder.sh` | The whole ladder in one call; PASS/FAIL table; `--host`, `--no-sim`, `--clean` |
| `tools/repeat_check.sh` | One `verify_sim.sh` check N times, exact-name process wait, pass/fail/could-not-run tally |
| `tools/check_doc_links.py` | Relative links and `#anchors` in the Markdown files (GitHub slug rule) |
| `CLAUDE.md` | §7 rule on volatile numbers; new §12 "Running the loop cheaply"; `orchestrate-run` in the skills table |
| `.claude/skills/orchestrate-run/SKILL.md` | New: model routing, a brief template, waiting and reporting rules, stage handoff |
| `.claude/skills/bootstrap-sim/SKILL.md` | New section "Designing a check that does not fail at random" |
| `.claude/skills/build-verify/SKILL.md` | Pointer to `tools/run_ladder.sh` |
| `.claude/agents/*.md` | `docs-reviewer` and `explainability-reviewer` default to Sonnet; every reviewer has an output limit |
| `docs/proposals/0011-agent-loop-cost-settings.md` | Allow-list entries and plugin/MCP trimming: guardrail changes, so a human decides |

## Verification

| Tier / check | Command | Result |
| --- | --- | --- |
| Measurement | `python3 -I tools/session_usage.py <id>` for the three sessions | printed per-agent tables; totals are in the analysis page |
| Link scanner | `python3 -I tools/check_doc_links.py` | `files 61 links 715 broken 0`; first version reported 10 false breaks (it dropped underscores and kept the `²` in `I²C`), fixed |
| Link scanner, red | a scratch file with a missing file and a missing anchor | both reported, exit 1 |
| Ladder script | `tools/run_ladder.sh --host` | every host test, hook tests, drift check and link scan PASS; `RESULT: PASS`; the container tiers were NOT run by this command |
| Repeat script | `tools/repeat_check.sh S2a 2` | pass 2 of 2; a first run failed on bash 3.2 (`ENV_ARGS[@]: unbound variable`), fixed |
| Repeat script, could not run | `SIM_PLANT_OVERRIDES=bad tools/repeat_check.sh S2a 1` | `could not run 1`, exit 2 |
| Not run | the full `tools/run_ladder.sh` (container tiers), `--clean`, `repeat_check.sh` with a real failing check | NOT RUN; the container steps in `run_ladder.sh` use the commands already in CLAUDE.md §8 but the script as a whole was only exercised in `--host` mode |

## Not verified

- Dollars. No prices are in the repo; `tools/session_prices.json` is empty on purpose. The page gives a price-free
  "units" figure with assumed weights and says so.
- The savings in §6 of the analysis are estimates from token shares; the first stage run with the new tools and
  rules is the test.
- What "ultracode" does: nothing in the transcripts names it.
- Whether trimming plugins and MCP servers (proposal 0011) works as described.
- Two slips against the "use Edit, not a script" habit: a Python one-liner corrected five figures in the analysis page
  after I overstated them, and another appended the 0011 row to `docs/proposals/README.md`. Both files are
  unguarded and the diffs are in the commit.
