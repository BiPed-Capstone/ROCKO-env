# How the agent loop spent its tokens, and what to change

**Written 2026-11-08 by the orchestrating agent, from the three session transcripts of the
2026-10-27 to 2026-11-08 runs.** Every number below comes from `tools/session_usage.py` and a
tool-call count over the same transcripts (commands at the end). Where a figure is an estimate, a
guess, or something this page could not measure, it says so.

## 1. What this page can and cannot tell you

- **It can say how many tokens each run used, on which model, in which agent.** The transcripts
  record that exactly (de-duplicated by response id).
- **It cannot say what the runs cost in dollars.** This repo does not know your plan or the model
  prices, and a Pro/Max plan is charged against limits, not per token. The one hard dollar figure the
  app reports is *extra usage*: **$25.31 spent against a $25.00 monthly limit** (read 2026-11-08). That is
  the whole account for the month, not only this repo, and it explains why four subagent runs were killed
  mid-task by limit errors (2026-11-04 and 2026-11-07) and why the last turns ran on a grace allowance.
- **To get dollars,** fill the four numbers per model into `tools/session_prices.json` (dollars per
  million tokens) and rerun `tools/session_usage.py <session-id> --prices tools/session_prices.json`.
- **"Units" below** are a price-free way to compare runs: tokens weighted by the usual shape of API
  prices, where one uncached input token = 1, a cache write = 1.25, a cache read = 0.1 and an output token
  = 5. Those weights are an assumption about the price structure (they have held for earlier Claude
  models; I could not check them for the 5.5 family). Multiply a model's units by its own base input
  price to get dollars. Units are only comparable *within one model*.
- **Run 3's transcript kept growing while this page was written,** so its figures were taken at different
  moments: 290.5 M tokens at the start, 295 M for the group table, about 303 M at the end (main thread 39 M, 44 M and
  52 M). Runs 1 and 2 are final. Read run-3 figures as accurate to about 4 %; the per-call counts quoted in §3
  (170 calls, 38.3 M cache reads) are from the first snapshot.
- **"Cache read"** means a token the model already processed earlier in the same conversation and re-reads from
  a cache on every later call. It is cheap per token, but a long conversation pays it again on every call.
- **I cannot see what "ultracode" does.** Nothing in the transcripts names it. Compare a run with it on
  and off using the same script.

## 2. The three runs

| Run | Dates | Session id (first 8) | Main-thread model | Total tokens | Opus tokens | Sonnet tokens | Haiku | Output tokens | Cache read share |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 1 | 10-27 to 10-31 | `86d50601` | Sonnet 5.5 | 284.5 M | 32.6 M | 252.0 M | 0 | 2.05 M | 97 % |
| 2 | 10-31 to 11-03 | `e72bbf61` | Sonnet 5.5 | 226.0 M | 88.2 M | 136.3 M | 1.5 M | 1.87 M | 95 % |
| 3 | 11-03 to 11-08 | `a4c49b25` | Sonnet 5.5 | 290.5 M | 214.3 M | 75.4 M | 0.8 M | 1.31 M | 97 % |
| **Sum** | | | | **801 M** | 335 M (42 %) | 463 M (58 %) | 2.3 M | 5.2 M | |

Weighted units per run (assumed weights, §1): run 1 Sonnet 38.6 M + Opus 8.5 M; run 2 Sonnet 23.8 M +
Opus 17.0 M + Haiku 0.6 M; run 3 Sonnet 13.3 M + Opus 30.3 M + Haiku 0.2 M. **Reading the table:**

1. **About 96 % of all tokens are cache reads.** An agent that makes 150 tool calls re-reads its whole
   conversation 150 times. So cost is roughly *number of calls × size of context*, and the number of
   words the agents *wrote* (5.2 M output tokens in total) is the smaller part. Output is still about 20 %
   of the weighted units, because an output token is priced far above a cache read.
2. **Work moved from the main thread to subagents between run 1 and run 3.** In run 1 the main thread
   used 228 M of 284 M tokens (80 %) doing the work itself, at about 450 K tokens of context per call. In run 3 it
   used 39 M of 290 M (13 %). That change was correct and is the largest single saving so far.
3. **Run 3 shifted onto Opus** (74 % of its tokens) because the orchestrator routed long
   fix-and-rerun loops to Opus. §5 asks whether that was needed.
4. **Reviewers are cheap.** The 17 reviewer runs together used 14 M tokens in run 3 (4.8 %) and
   found the one High finding of the stage-4 round (the missing feedforward age limit, H-11).

### Run 3 in detail (the run that used the new instructions)

Grouping done by agent description over the 295 M tokens in the transcript at the time of writing (the
table above says 290.5 M: the main thread grew while this analysis ran):

| Group | Agents | Tokens | Share |
| --- | --- | --- | --- |
| Verifier fixes and flake hunts (C3 twice, H4 twice, the sweep and its re-run) | 6 | 117.4 M | 39.8 % |
| Fixes and stage 6 (stage 4 rounds 3 and 4, 5b and 5c review fixes, C-1 helper, docs for H-11) | 8 | 60.8 M | 20.6 % |
| Tuning, scenarios and diagnosis (stage 5a, 5b, 5c-1) | 3 | 46.5 M | 15.7 % |
| Main thread (orchestrator, 170 calls; includes this analysis) | 1 | 44.1 M | 14.9 % |
| Reviewers (all four kinds, every round) | 17 | 14.1 M | 4.8 % |
| Docs writing and docs fixes (gap page, review fixes) | 2 | 11.5 M | 3.9 % |
| Preflight ladder run (Haiku) | 1 | 0.8 M | 0.3 % |

Counting every agent whose main job was documentation (including the docs reviewers) gives about 27 M
tokens, 9 % of the run.

The first row is the finding that matters most, with one correction to its label: two of its six agents
(about 24 M tokens) were the gain-tuning sweep and its re-run, not flake hunts. **About 93 M tokens, roughly
31 % of the run, went on making timing checks stop failing at random** (C3, FF, V, H4; S2a still fails once
in a while). Section 4 gives
the cause and the rule that would have avoided it.

## 3. Where the waste was

Evidence is the 5,314 tool calls in the three runs (3,358 were `Bash`, 891 `Edit`, 610 `Read`,
159 `Write`, 108 `Agent`).

| # | Finding | Evidence | Rough size | Fix made here |
| --- | --- | --- | --- | --- |
| W1 | **Timing checks written so that they judge the checker's own pauses.** C3 and FF (run 2, stage 4) timed from when the checker *received* a message; H4 closed a fixed 16 s window around a step that takes 3.7 to 19 s; V took the single highest noisy sample. The container stalls processes for about 0.3 s, so each failed at random, and each failure cost an investigation. | 117 M tokens in run 3 (40 %), plus the repeated "flaky" notes in 5 journals | 25 to 35 % of run 3 if the rule had been followed at the start | `bootstrap-sim` skill: new section "Designing a check that does not fail at random" |
| W2 | **A turn for every "Waiting" notification.** A subagent that ends its turn with a background job still running notifies the orchestrator each time it stops. The sweep agent produced about 25 such notices; each made the orchestrator re-read about 230 K tokens to write one line. | In run 3, 76 of 167 main-thread API calls (46 %) used no tool; they carried 17.6 M of the main thread's 38.3 M cache-read tokens (46 %) | about 7 to 8 % of run 3 | `orchestrate-run` skill and `CLAUDE.md` §12: agents wait inside one `Bash` call, never end a turn with work running |
| W3 | **Re-running the ladder one command at a time.** 961 of 3,358 `Bash` calls (29 %) ran a verify script or a test; another 183 were hand-written wait/poll loops. Two of those loops cost hours (a `pgrep` that matched its own command line; runs that exited 2 because the last run's process was still alive). | tool-call census above | 15 to 25 % of agent calls | `tools/run_ladder.sh` (one call, 25 lines out), `tools/repeat_check.sh` (N runs, tally, correct cleanup) |
| W4 | **Reading files through the shell.** 1,380 `Bash` calls were `grep`/`cat`/`ls`/`head` where `Read` or `Grep` return only the lines asked for. | census | not measured; each call also returns whole files into context | `orchestrate-run`: brief template says "use Grep and Read with line ranges" |
| W5 | **Fixed overhead on every call.** The main thread carries about 73 K tokens before any work: MCP tool names 32 K, system tools 11 K, system prompt 11 K, skills list 10 K, memory files 8.5 K. Most of the MCP and skills list is for products this repo never uses (the plugin catalogues). Each subagent's first call also writes tens of thousands of cache tokens for the same reason (32 K for the one I sampled; the rest were not checked). | `get_usage` on this session; first call of each subagent transcript | about 32 % of the main thread's cache reads (73 K x 170 calls = 12 M); the share in subagents was not measured | Proposal 0011 (disable unrelated plugins and MCP servers for this project; a settings change, so a human decides) |
| W6 | **One long main conversation across stages.** Run 3's main thread ran for five days in a single context that grew to 350 K tokens. Every call re-read it. | `get_usage`: 354 K tokens, 275 K of them messages | A fresh session per stage would read about 60 to 80 K per call instead of about 230 K | `orchestrate-run`: end a stage with a handoff in the journal and start the next in a new session |
| W7 | **Long reports flowing back into the orchestrator.** Agent reports of 1.5 to 3 K tokens each, 108 times, stay in context for the rest of the run. | main-thread messages = 275 K tokens | part of W6 | Brief template: "report in at most 25 lines; details go in the journal" |
| W8 | **The same fact written in many pages.** Counts, durations and sweep tallies (389 checks, 157 s, PASS 18) sit in the README, the guide, the ladder, three journals and next-steps. Five of the six docs reviews found stale copies of one of them, and each finding cost a fix agent. | docs reviews in this run | about 9 % of run 3 on docs agents (27 M tokens), much of it chasing copies | `CLAUDE.md` §7 addition: a volatile number lives in one page; elsewhere link to it |
| W9 | **Agents killed by the usage limit, then resumed.** Four subagent runs died on limit errors in run 3. A resumed agent re-writes its cache: `a63dbeb4` (C3 fix) wrote 0.5 M cache tokens and read 43 M in total. | agent logs | small, but it also lost partial edits | `orchestrate-run`: call `get_usage` before a long agent; stop at a stage boundary when the 5-hour window is above 70 % or extra usage is nearly spent |
| W10 | **Pinned models that the orchestrator overrode.** All four reviewer agents were declared `model: opus`; the orchestrator passed `model: sonnet` by hand for two of them on every call. | `.claude/agents/*.md` | removes a per-call decision | Agent files: `docs-reviewer` and `explainability-reviewer` now default to Sonnet |

**What was not waste.** Reviews: 14 M tokens found the one High finding of round 3, several real doc errors
(a measurement step that could not produce its data, "scenarios fail today" after they passed) and a
missing check (one-wheel gain deletions). The verifiers that were made robust now exist. The
instruction files did their job on safety: no gated file was edited, no push other than `ai-forward`, and
every claim in the final report is labelled as being about a model.

## 4. The rule that would have saved the most

A check that measures time must take its timestamps **at the source of the event**, not when the checker
happens to run, and must not use a fixed window around something whose duration it does not control.

- C3/FF: stamp each message when it is *sent* and count it only once the publish call has returned.
- H4: wait for the event (the refusal) with a bounded cap, not a fixed 16 s.
- V: recompute the signal with each cycle's real length, not an assumed 10 ms.
- Every one of these also needs a deliberate break that turns it red, and 20 consecutive green runs.

That is now written into the `bootstrap-sim` skill, so the next check is designed this way the first time.

## 5. Models and effort

The data says what each model was used for, not how good the result would have been on another model. The
recommendations below are therefore hypotheses with a cheap way to test each.

| Job | Used in run 3 | Recommendation | Why, and how to test it |
| --- | --- | --- | --- |
| **Orchestrator** | Sonnet 5.5, medium | **Keep Sonnet 5.5; low or medium effort.** Do not use Haiku. | Its job is to dispatch, relay and keep the honesty rule. It is bound by context size, not by reasoning. Haiku would save little (the cost is cache reads of a large context) and risks the judgement calls (is this finding confirmed? is this claim allowed?). Measure effort with `session_usage.py`: compare the same stage at `low` and `medium`. |
| `realtime-auditor`, `ros2-control-reviewer` | Opus | **Keep Opus.** | Small (about 8 M tokens in run 3) and they found the High finding. |
| `explainability-reviewer`, `docs-reviewer` | Sonnet (overridden by hand) | **Sonnet by default** (now in the agent files) | Both found real issues on Sonnet. |
| Root cause of a failure nobody understands (the sign diagnosis, the first C3 investigation) | Opus | Keep Opus, once per problem | One-off reasoning; the 30 M tokens of C3 investigation were the right place for it. |
| **Applying a list of confirmed fixes; rerunning checks; writing docs** | Opus in several fix agents | **Sonnet** | The fix is already specified. Opus agents `ade63d25` (25.5 M tokens), `a47a5b27` (9.0 M) and `a6277f79` (9.5 M) were given a list. Test it: give the next such list to Sonnet and compare retries. |
| **Tuning loop** (5c-1) | Opus, 22 M tokens | Opus for choosing candidates, but have it run `--sweep` scripts and read only their output | The agent's calls were mostly waiting for container runs. |
| **Running the ladder** | One Haiku agent in run 3 | **No agent: `tools/run_ladder.sh`** | A script returns the same 25 lines for no model tokens. A Haiku agent still pays to boot (about 90 K cache-write tokens) and to read the output. |

If you want the cheapest first experiment, change only the two reviewer defaults (done) and the fix-agent
routing (a line in the skill), run one stage, and compare the totals with `session_usage.py`.

## 6. What it could have cost

These are estimates, not measurements, built from the shares in §3. They are not additive, because the
levers overlap (a fresh session also removes the chatter turns).

| Change | Estimated effect on a run like run 3 | Confidence |
| --- | --- | --- |
| W1: design timing checks right the first time | 25 to 35 % fewer tokens in the whole run (the flake-fixing share is about 31 %; this range is a judgement, not a measurement) | Medium. It depends on the checks being written once, correctly. |
| W2 + W6 + W7: no chatter turns, fresh session per stage, short reports | Main thread from 39 M to about 10 to 14 M tokens | Medium |
| W3 + W4: scripts instead of hand-run loops, `Grep` instead of `cat` | Up to about 18 % fewer calls (961 of 5,314 were verify or test runs; the poll loops add about 3 %) | Medium. The census counts the calls; the saving per call depends on context size. |
| W5: drop unrelated plugins and MCP servers | Up to about 18 % of main-thread cache reads (the MCP and skills list is 42 K of a roughly 230 K context); subagent share not measured | High on the count, low on the dollars. |
| W8: single home for volatile numbers | A share of the 10 % docs spend | Low |
| W10 and the model routing in §5 | Opus tokens down by perhaps a third | Low. It is untested. |

**Overall: a run of the same scope plausibly costs 40 to 55 % fewer weighted units (my judgement from the table, not a measurement)**, with the largest
single factor being W1 and the next largest the main-thread changes. Treat that as a target to measure
against, not a promise. The first thing to do with it is to run the next stage and compare.

## 7. What changed in the repo because of this page

| File | Change |
| --- | --- |
| `tools/session_usage.py`, `tools/session_prices.json` | New. Token use by agent and model from a session transcript; dollars if you supply prices. |
| `tools/run_ladder.sh` | New. The whole ladder in one call, one table out. |
| `tools/repeat_check.sh` | New. One check N times, with correct process cleanup and a tally. |
| `tools/check_doc_links.py` | New. The one link and anchor scanner (it replaces five throwaway ones). |
| `CLAUDE.md` | New section 12 "Running the loop cheaply"; §7 gains the rule on volatile numbers; the skills table lists `orchestrate-run`. |
| `.claude/skills/orchestrate-run/SKILL.md` | New. The orchestration playbook that used to live in the seed prompt. |
| `.claude/skills/bootstrap-sim/SKILL.md` | New section on designing a check that does not fail at random. |
| `.claude/skills/build-verify/SKILL.md` | A pointer to `tools/run_ladder.sh` at the top. |
| `.claude/agents/*.md` | Reviewers default to the right model; each has an output limit. |
| `docs/proposals/0011-agent-loop-cost-settings.md` | New. The settings changes that need a human (allow-list, plugins, MCP). |

## 8. About the seed prompt

The prompt that started run 3 was good where it mattered: explicit stop conditions, "decisions already
made", a cap on tuning iterations, "a sweep is a script". Four changes would make it cheaper:

1. **Move the orchestration paragraph out of the prompt and into a skill** (done: `orchestrate-run`). The
   prompt can then say "continue the run in the last journal handoff" in three lines.
2. **Resolve one conflict:** it said both "read these nine documents in full" and "do not read large files
   yourself". The first read is worth it (it is how the handoff state was verified), but say which agent
   should do it.
3. **Put model routing in the agent files,** not in the prompt (done for the reviewers).
4. **Add the missing limits:** a report length, a rule against ending a turn with work running, a
   `get_usage` check before launching long agents, and "start the next stage in a fresh session".

## 9. How the numbers were produced

```bash
python3 -I tools/session_usage.py a4c49b25-d9f0-4ce4-be35-9db89c974c24      # run 3; repeat for 86d50601-... and e72bbf61-...
```

The tool-call census (5,314 calls; categories by command text) and the no-tool-turn count were produced
by a one-off script over the same transcripts that is **not in the repo, so those figures (W2, W3, W4) cannot
be reproduced from it**; the categories are keyword matches on the command, so
treat them as approximate (a command can match two). Not measured: dollars, latency of the user's own
waiting, the quality difference between models, and whether the 0.3 s container stalls come from
Docker Desktop's file sharing or the VM. The week-by-week plan usage is available from the app; the
transcripts do not contain it.
