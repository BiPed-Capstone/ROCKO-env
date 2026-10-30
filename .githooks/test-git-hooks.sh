#!/usr/bin/env bash
# Tests for pre-commit and pre-push in this folder. Run from anywhere:  .githooks/test-git-hooks.sh
# It builds a throwaway repository and a throwaway "remote" in a temporary folder, installs these
# hooks in the repository, and tries things the hooks must refuse and things they must allow. It
# never touches the real repository or the real remote (see the comment on GIT_DIR below). Tried by
# hand as well with `git init --object-format=sha256` and with the hooks run by dash.
# Exit status:  0 = every case behaved   1 = at least one did not   2 = could not set up the temp repos
set -u
hooks="$(cd "$(dirname "$0")" && pwd)"
tmp="$(mktemp -d)" || exit 2
trap 'rm -rf "$tmp"' EXIT
fails=0

# git reads these before it looks at the current folder. Inside a git hook (and in some editors) they
# are set, and then every git command below would act on THAT repository and its remote, not the
# throwaway one. `git -c name=value` hands its settings down in the last two. Clear them, and ignore
# the user's own git settings, so the result is the same anywhere. (GIT_CONFIG_GLOBAL needs git 2.32
# or later.)
unset GIT_DIR GIT_WORK_TREE GIT_INDEX_FILE GIT_COMMON_DIR GIT_OBJECT_DIRECTORY GIT_PREFIX \
      GIT_CONFIG_PARAMETERS GIT_CONFIG_COUNT
export GIT_CONFIG_NOSYSTEM=1 GIT_CONFIG_GLOBAL=/dev/null
export GIT_AUTHOR_NAME=t GIT_AUTHOR_EMAIL=t@t GIT_COMMITTER_NAME=t GIT_COMMITTER_EMAIL=t@t

# check <allowed|refused> <label> <command...>
# "refused" means a non-zero exit AND the hook's own "refused:" in the output. git's own rejections
# say "rejected", so a push git turns down by itself, or a typo in this test, cannot pass as the
# hook's work. The output stays in $out for a follow-up look.
check() {
  local want="$1" label="$2" good=0; shift 2
  out="$("$@" 2>&1)"; rc=$?
  case "$want:$rc:$out" in
    allowed:0:*)           good=1 ;;
    refused:0:*)           good=0 ;;   # printed "refused:" yet exited 0: a refusal that forgot its exit 1. Keep this above the next line
    refused:*:*refused:*)  good=1 ;;
  esac
  if [ $good = 1 ]; then
    echo "ok    $label (exit $rc)"
  else
    echo "FAIL  $label: wanted it $want, got exit $rc"; echo "$out" | sed 's/^/        /'; fails=$((fails + 1))
  fi
}

git init -q --bare -b main "$tmp/origin.git"
git init -q --bare -b main "$tmp/elsewhere.git"
git init -q -b main "$tmp/work"
cd "$tmp/work" || exit 2
git remote add origin "$tmp/origin.git"
git remote add elsewhere "$tmp/elsewhere.git"
git commit -q --allow-empty -m seed          # main needs one commit: make it before the hooks are on, so they cannot refuse it

# Installed the way proposal 0002 says: copied into .git/hooks. Not core.hooksPath: git would then run
# the hooks from the .githooks folder of whatever files are checked out, and main has none, so after
# `git switch main` there would be no hooks at all.
cp "$hooks/pre-commit" "$hooks/pre-push" .git/hooks/
check refused "commit on main"                                 git commit --allow-empty -m x
check allowed "commit on main as a person (ROCKO_HUMAN=1)"     env ROCKO_HUMAN=1 git commit --allow-empty -m x
check refused "commit on main with ROCKO_HUMAN=0"              env ROCKO_HUMAN=0 git commit --allow-empty -m x
check allowed "commit on main with --no-verify (known gap)"    git commit --no-verify --allow-empty -m y

git checkout -q -b ai-forward
check allowed "commit on ai-forward"                           git commit --allow-empty -m a1
check allowed "first push of ai-forward"                       git push -q origin ai-forward
check refused "push main"                                      git push -q origin main
check refused "push main with ROCKO_HUMAN=0"                   env ROCKO_HUMAN=0 git push -q origin main
check refused "push ai-forward onto the remote's main"         git push -q origin ai-forward:main
git tag v1
check refused "push a tag"                                     git push -q origin v1
check refused "push a tag named ai-forward"                    git push -q origin ai-forward:refs/tags/ai-forward
check refused "push to some other branch name"                 git push -q origin ai-forward:other
check refused "push to a remote that is not origin"            git push -q elsewhere ai-forward
check refused "delete the remote ai-forward"                   git push -q origin :ai-forward
case "$out" in
  *deleting*) echo "ok    ...and the message says it is a deletion" ;;
  *) echo "FAIL  the deletion refusal blames something else"; fails=$((fails + 1)) ;;
esac
check allowed "commit, then fast-forward push"                 bash -c 'git commit -q --allow-empty -m a2 && git push -q origin ai-forward'

git reset -q --hard HEAD~1
git commit -q --allow-empty -m diverged
check refused "--force over a diverged branch"                 git push -q --force origin ai-forward
check refused "--force-with-lease over a diverged branch"      git push -q --force-with-lease origin ai-forward
check allowed "--force as a person (ROCKO_HUMAN=1)"            env ROCKO_HUMAN=1 git push -q --force origin ai-forward

# Someone else pushes to the remote; we force over it without having fetched it.
git clone -q "$tmp/origin.git" "$tmp/other" 2>/dev/null   # git warns that the remote has no main; harmless
( cd "$tmp/other" && git checkout -q ai-forward && git commit -q --allow-empty -m theirs \
    && git push -q --no-verify origin ai-forward ) \
  || { echo "FAIL  setup: the second clone could not push"; fails=$((fails + 1)); }
git commit -q --allow-empty -m mine
check refused "--force over commits we have never seen"        git push -q --force origin ai-forward

git fetch -q origin && git rebase -q origin/ai-forward
before="$(git ls-remote origin refs/heads/ai-forward)"
check refused "one bad ref in a push"                          git push -q origin ai-forward main
if [ "$(git ls-remote origin refs/heads/ai-forward)" = "$before" ]; then
  echo "ok    ...and the good ref in that push did not go either"
else
  echo "FAIL  the good ref in a refused push reached the remote"; fails=$((fails + 1))
fi
check allowed "after fetch and rebase, a fast-forward"         git push -q origin ai-forward

git checkout -q --detach
check allowed "commit on a detached HEAD"                      git commit --allow-empty -m d

echo
if [ $fails -eq 0 ]; then echo "PASS: every case behaved."; else echo "FAIL: $fails case(s) did not."; exit 1; fi
