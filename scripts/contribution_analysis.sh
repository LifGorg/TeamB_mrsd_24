#!/bin/bash
# Contribution Analysis Script
# Run from repository root: ./scripts/contribution_analysis.sh

echo "=== Git Contribution Analysis ==="
echo "Repository: $(git remote get-url origin)"
echo "Date: $(date)"
echo ""

echo "=== Commits by Author (all branches) ==="
git shortlog -sne --all
echo ""

echo "=== Lines Added by Author (excluding binaries/generated) ==="
git log --all --numstat --pretty=format:'COMMIT_AUTHOR:%aN' | \
awk '
/^COMMIT_AUTHOR:/ { author = substr($0, 15) }
/^[0-9]+\t[0-9]+\t/ {
    file = $3
    # Skip generated/binary files
    if (file ~ /package-lock\.json|yarn\.lock|\.log$|\.pdf$|\.gif$|\.png$|\.jpg$/) next
    if (file ~ /build\/|install\/|node_modules\/|__pycache__|mavlink|PayloadSdk/) next
    added[author] += $1
}
END {
    for (a in added) printf "%8d  %s\n", added[a], a
}' | sort -rn
echo ""

echo "=== Major File Authors (by first commit) ==="
for pattern in "behavior_executive" "vision_gps" "rtsp_stream" "burst_record" "waypoint_generator"; do
    echo -n "$pattern: "
    git log --all --diff-filter=A --format="%an" -- "**/*${pattern}*" 2>/dev/null | head -1
done
echo ""
