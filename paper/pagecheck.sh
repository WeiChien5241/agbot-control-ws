#!/usr/bin/env bash
# PostToolUse hook: after paper.tex is edited, rebuild and report the page count.
#
# The 8-page limit is the only constraint that actually binds on an ICRA
# submission, and it is invisible while writing in a text editor. This reports
# it back automatically so nobody writes 400 words past the limit.
#
# Reads the hook payload as JSON on stdin. Exits silently for every file that
# is not paper.tex. Never fails the tool call -- always exits 0.
set -u

f=$(python3 -c '
import json, sys
try:
    d = json.load(sys.stdin)
except Exception:
    sys.exit(0)
r = d.get("tool_response") or {}
i = d.get("tool_input") or {}
print(r.get("filePath") or i.get("file_path") or "")
' 2>/dev/null)

case "$f" in
  */paper/paper.tex|paper/paper.tex) ;;
  *) exit 0 ;;
esac

d=$(dirname "$f")
if out=$(cd "$d" && make pages 2>&1); then
  printf '%s\n' "$out" | tail -2
else
  # Build failed, or the paper is over the limit (make pages exits 1).
  printf '%s\n' "$out" | grep -E '^!|^l\.[0-9]+|OVER THE LIMIT|^pages:' | head -10
fi
exit 0
