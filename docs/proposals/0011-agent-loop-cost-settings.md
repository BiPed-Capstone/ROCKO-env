# 0011 - Settings that would make the agent loop cheaper (a human decides)

**Status:** Open. **Date:** 2026-11-08. **Gate:** `.claude/settings.json` and the plugin list are guardrails
(CLAUDE.md §3), so an agent writes this proposal and a human applies it. **Evidence:**
[agent-loop-efficiency.md](../claude-analysis/agent-loop-efficiency.md), findings W5 and the allow-list part of W3.

## What this asks for

### A. Allow the new helper scripts by exact command

`.claude/settings.json` allows commands by exact match. The new scripts are read-only on the repo and run
the same commands the allow-list already permits one at a time (`docker compose ... exec`, `colcon` through
`build_ros.sh`, `g++` on `tools/*.cpp`). Add, as exact rules and nothing broader:

```
Bash(tools/run_ladder.sh)            Bash(tools/run_ladder.sh --no-sim)
Bash(tools/run_ladder.sh --host)     Bash(tools/run_ladder.sh --clean)
Bash(python3 -I tools/check_doc_links.py)
Bash(python3 -I tools/session_usage.py:*)
```

and, if you accept that the argument is a check name and a count, `Bash(tools/repeat_check.sh:*)`. Without
these, an unattended run in `default` or `acceptEdits` mode stops at a prompt on the very commands that now
replace dozens of allowed ones. (The 2026-11 run actually ran in `bypassPermissions`, which the allow-list
does not affect; CLAUDE.md §2 describes `acceptEdits`. Say which mode you intend.)

Note: `check_container_mount.sh` is allowed by exact words (proposal 0001). `run_ladder.sh` and `repeat_check.sh`
call it with a redirection inside the script, which is fine because the allow-list matches the command the
agent types, not what a script runs; but an agent that types `./check_container_mount.sh >/dev/null` itself
would prompt.

**Risk:** `run_ladder.sh` runs `rm -rf build install log` inside the container only with `--clean`; it runs
`g++` on files under `tools/`. Nothing in it writes outside `$TMPDIR` and the container's workspace.
**Test a human should run:** `tools/run_ladder.sh --host` on the laptop (seconds); it should end `RESULT: PASS`.

### B. Trim what every call carries

The main thread carries about 73 K tokens before any work (`get_usage`, 2026-11-08): MCP tool names 32 K,
system tools 11 K, system prompt 11 K, skills list 10 K, memory files 8.5 K. This repo uses none of the
plugin catalogues (product management, data, engineering productivity connectors) or most of the MCP
servers. Options, cheapest first:

1. In this project, disable the plugins and MCP servers you do not use for ROCKO (a project `.claude/settings.json`
   `enabledPlugins` / MCP allow-list, or `claude mcp` per project). **Check the effect with**
   `mcp__ccd_session_mgmt__get_usage` (the "MCP tools" and "Skills" rows) before and after.
2. Keep a separate, bare profile for long autonomous runs and use the full one for interactive work.

I did not edit these settings: they change what every future session can do, and I cannot see which
connectors you use elsewhere.

### C. Decide the seed model and effort with a measurement

The recommendation in §5 of the analysis (Sonnet at low or medium for the orchestrator, Opus only for
the two technical reviewers and for root-cause work, Sonnet for applying fixes) is a hypothesis. Test it on one
stage: run it, then `python3 -I tools/session_usage.py <session-id> --prices tools/session_prices.json`
(fill in the prices first) and compare with the 2026-11-03 run's 290 M tokens.

## Decision

(Human fills in: date, who, accepted/declined, why.)
