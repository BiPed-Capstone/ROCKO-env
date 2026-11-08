#!/usr/bin/env python3
"""Check every relative link and #anchor in the repo's Markdown files. Standard library only.

    python3 -I tools/check_doc_links.py            # docs/, README.md, CLAUDE.md, .claude/skills, .claude/agents
    python3 -I tools/check_doc_links.py path ...   # only these files or folders

Why it exists: in the 2026-10 to 2026-11 runs, five different agents each wrote their own throwaway link
scanner in a scratch folder (and three docs reviews found links the scanner of the moment had missed). This is the
one scanner. It checks that the target file exists and, for `file.md#anchor`, that the anchor matches a heading in
that file using GitHub's rule (lower-case, punctuation dropped, spaces to hyphens, duplicates get -1, -2, ...).
It skips http(s) and mailto links, links inside fenced code blocks, and links to files that do not exist ON
PURPOSE are not special-cased: if a page is known to be unwritten, the link must say so in plain text instead.

Exit status: 0 = no broken link, 1 = at least one broken link.
"""
import os
import re
import sys
import unicodedata

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
DEFAULT = ["docs", "README.md", "CLAUDE.md", ".claude/skills", ".claude/agents"]
LINK = re.compile(r"(?<!\!)\[[^\]]*\]\(([^)\s]+)(?:\s+\"[^\"]*\")?\)")
FENCE = re.compile(r"^\s*(```|~~~)")


def markdown_files(paths):
    for path in paths:
        full = os.path.join(ROOT, path)
        if os.path.isfile(full):
            yield full
        for folder, _, names in os.walk(full):
            for name in sorted(names):
                if name.endswith(".md"):
                    yield os.path.join(folder, name)


def slug(heading):
    text = re.sub(r"[`*]", "", heading.strip().lower())  # GitHub keeps underscores
    text = "".join(c for c in text if unicodedata.category(c) != "No")  # "I²C" becomes "ic"
    text = re.sub(r"[^\w\- ]", "", text)
    return text.replace(" ", "-")


def anchors(path, cache={}):
    if path not in cache:
        found, seen, fenced = set(), {}, False
        with open(path, encoding="utf-8", errors="replace") as handle:
            for line in handle:
                if FENCE.match(line):
                    fenced = not fenced
                match = None if fenced else re.match(r"^#{1,6}\s+(.*?)\s*#*\s*$", line)
                if match:
                    base = slug(match.group(1))
                    count = seen.get(base, 0)
                    seen[base] = count + 1
                    found.add(base if count == 0 else f"{base}-{count}")
        cache[path] = found
    return cache[path]


def main():
    paths = sys.argv[1:] or DEFAULT
    files = links = broken = 0
    for md in markdown_files(paths):
        files += 1
        fenced = False
        with open(md, encoding="utf-8", errors="replace") as handle:
            for number, line in enumerate(handle, 1):
                if FENCE.match(line):
                    fenced = not fenced
                if fenced:
                    continue
                for target in LINK.findall(line):
                    if re.match(r"^[a-z][a-z0-9+.-]*:", target) or target.startswith("#") is False and target.startswith("//"):
                        continue
                    links += 1
                    name, _, fragment = target.partition("#")
                    dest = md if not name else os.path.normpath(os.path.join(os.path.dirname(md), name))
                    problem = None
                    if not os.path.exists(dest):
                        problem = "file not found"
                    elif fragment and dest.endswith(".md") and fragment.lower() not in anchors(dest):
                        problem = f"no heading for #{fragment}"
                    if problem:
                        broken += 1
                        print(f"{os.path.relpath(md, ROOT)}:{number}: {target}: {problem}")
    print(f"files {files} links {links} broken {broken}")
    return 1 if broken else 0


if __name__ == "__main__":
    sys.exit(main())
