#!/usr/bin/env python3
"""Tokens used by one Claude Code session, split by model and by subagent.

Why this exists: an agent run is judged on cost as well as on whether it worked, and the
desktop app shows only a percentage of a plan limit. This reads the session's own transcript
files and answers "which agent, on which model, used how many tokens, for how long".

Usage (laptop, standard library only; run with -I as the other tools do):
    python3 -I tools/session_usage.py <session-id>                  # tokens only
    python3 -I tools/session_usage.py <session-id> --prices tools/session_prices.json
    python3 -I tools/session_usage.py <session-id> --csv out.csv    # one row per agent

<session-id> is the folder/file stem under ~/.claude/projects/<project>/ (for this repo the
project folder is -Users-<you>-projects-ROCKO-env). --project overrides the folder.

What it counts, and what it does not:
  * Every API response is listed several times in a transcript (once per content block).
    Responses are de-duplicated by their message id, keeping the largest output count.
  * Four token kinds, as the API reports them: input (uncached), cache write, cache read,
    output. Thinking tokens are inside output.
  * Cost is computed ONLY if you pass --prices. The prices file is yours: this repo does not
    know what you pay, so session_prices.json ships with null values on purpose. A plan
    (Pro/Max) is charged against limits, not per token, so a dollar figure is an estimate of
    what the same tokens cost at list price, not your bill.
  * Wall time per agent is last minus first timestamp in its transcript; it includes time
    waiting on tools, so it is not compute time.
  * The main thread's token count includes the cost of re-reading the conversation on each
    turn (cache read). A long main conversation is expensive for that reason alone.
"""
import argparse
import collections
import csv
import datetime
import json
import os
import sys

KINDS = ("input", "cache_write", "cache_read", "output")


def parse_ts(text):
    try:
        return datetime.datetime.fromisoformat(text.replace("Z", "+00:00"))
    except (AttributeError, ValueError):
        return None


def read_transcript(path):
    """Return ({model: {kind: tokens}}, api_calls, first_ts, last_ts) for one transcript."""
    best = {}  # message id -> (output_tokens, model, usage)
    first = last = None
    with open(path, encoding="utf-8", errors="replace") as handle:
        for line in handle:
            try:
                row = json.loads(line)
            except ValueError:
                continue
            stamp = parse_ts(row.get("timestamp"))
            if stamp:
                first = stamp if first is None or stamp < first else first
                last = stamp if last is None or stamp > last else last
            if row.get("type") != "assistant":
                continue
            message = row.get("message") or {}
            usage = message.get("usage")
            if not usage:
                continue
            key = message.get("id") or row.get("requestId") or row.get("uuid")
            out = usage.get("output_tokens", 0)
            if key not in best or out >= best[key][0]:
                best[key] = (out, message.get("model", "unknown"), usage)
    per_model = collections.defaultdict(lambda: dict.fromkeys(KINDS, 0))
    for out, model, usage in best.values():
        slot = per_model[model]
        slot["input"] += usage.get("input_tokens", 0)
        slot["cache_write"] += usage.get("cache_creation_input_tokens", 0)
        slot["cache_read"] += usage.get("cache_read_input_tokens", 0)
        slot["output"] += out
    return per_model, len(best), first, last


def cost_of(tokens, price):
    """Dollars at list price, or None when the price entry is incomplete."""
    if not price or any(price.get(k) is None for k in KINDS):
        return None
    return sum(tokens[k] * price[k] for k in KINDS) / 1_000_000.0


def main():
    parser = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    parser.add_argument("session_id")
    parser.add_argument("--project", help="folder under ~/.claude/projects (default: guessed from the cwd)")
    parser.add_argument("--prices", help="JSON file: {model: {input, cache_write, cache_read, output}} in $ per million tokens")
    parser.add_argument("--csv", help="write one row per agent to this file")
    args = parser.parse_args()

    root = os.path.expanduser("~/.claude/projects")
    project = args.project or os.getcwd().replace("/", "-")
    base = os.path.join(root, project)
    main_path = os.path.join(base, args.session_id + ".jsonl")
    if not os.path.exists(main_path):
        sys.exit(f"no transcript at {main_path} (pass --project <folder under {root}>)")
    prices = {}
    if args.prices:
        with open(args.prices, encoding="utf-8") as handle:
            prices = json.load(handle)

    rows = []  # (label, agent type, description, model, tokens, calls, seconds)

    def add(label, kind, description, path):
        per_model, calls, first, last = read_transcript(path)
        seconds = (last - first).total_seconds() if first and last else 0
        for model, tokens in per_model.items():
            rows.append((label, kind, description, model, tokens, calls, seconds))

    add("main", "main thread", "orchestrator", main_path)
    sub_dir = os.path.join(base, args.session_id, "subagents")
    if os.path.isdir(sub_dir):
        for name in sorted(os.listdir(sub_dir)):
            if not name.endswith(".jsonl"):
                continue
            meta = {}
            meta_path = os.path.join(sub_dir, name[: -len(".jsonl")] + ".meta.json")
            if os.path.exists(meta_path):
                with open(meta_path, encoding="utf-8") as handle:
                    meta = json.load(handle)
            add(name[len("agent-"):-len(".jsonl")][:8], meta.get("agentType", "?"),
                meta.get("description", ""), os.path.join(sub_dir, name))

    def line(label, tokens, extra=""):
        total = sum(tokens[k] for k in KINDS)
        return (f"{label:<44} in {tokens['input']:>9,} wr {tokens['cache_write']:>10,} "
                f"rd {tokens['cache_read']:>12,} out {tokens['output']:>9,} total {total:>12,}{extra}")

    by_model = collections.defaultdict(lambda: dict.fromkeys(KINDS, 0))
    by_type = collections.defaultdict(lambda: dict.fromkeys(KINDS, 0))
    calls_by_type = collections.Counter()
    print("PER AGENT (label, type, model)")
    counted = set()  # an agent that used two models has two rows; count its API calls once
    for label, kind, description, model, tokens, calls, seconds in rows:
        if sum(tokens.values()) == 0:
            continue  # a "<synthetic>" row: the app's own placeholder messages, no API call
        price = prices.get(model)
        dollars = cost_of(tokens, price)
        extra = f"  calls {calls:>4}  {seconds / 60:>6.1f} min" + (f"  ${dollars:,.2f}" if dollars is not None else "")
        print(line(f"{label} {kind[:14]} {model[-12:]} {description[:20]}", tokens, extra))
        for k in KINDS:
            by_model[model][k] += tokens[k]
            by_type[kind][k] += tokens[k]
        if label not in counted:
            counted.add(label)
            calls_by_type[kind] += calls
    print("\nBY MODEL")
    grand = 0.0
    priced = True
    for model, tokens in sorted(by_model.items()):
        dollars = cost_of(tokens, prices.get(model))
        priced = priced and dollars is not None
        grand += dollars or 0.0
        print(line(model, tokens, f"  ${dollars:,.2f}" if dollars is not None else ""))
    print("\nBY AGENT TYPE")
    for kind, tokens in sorted(by_type.items()):
        print(line(f"{kind} ({calls_by_type[kind]} calls)", tokens))
    if prices:
        print(f"\nESTIMATED LIST-PRICE TOTAL: ${grand:,.2f}" + ("" if priced else "  (INCOMPLETE: a model had no price)"))
    else:
        print("\nNo --prices file given, so no dollar figures. Copy tools/session_prices.json and fill it in.")

    if args.csv:
        with open(args.csv, "w", newline="", encoding="utf-8") as handle:
            writer = csv.writer(handle)
            writer.writerow(["label", "agent_type", "description", "model", *KINDS, "api_calls", "wall_seconds"])
            for label, kind, description, model, tokens, calls, seconds in rows:
                writer.writerow([label, kind, description, model, *(tokens[k] for k in KINDS), calls, int(seconds)])


if __name__ == "__main__":
    main()
