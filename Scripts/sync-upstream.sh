#!/bin/sh
# Re-vendors the upstream engine at a given commit (default: the pinned one in UPSTREAM.md).
# Upstream files are copied unmodified; review the diff, then run the tests and sdf-snapshot.
#
#   Scripts/sync-upstream.sh [commit]
set -eu
root=$(cd "$(dirname "$0")/.." && pwd)
commit=${1:-$(sed -n 's/^Commit: `\(.*\)`$/\1/p' "$root/UPSTREAM.md")}
work=$(mktemp -d)
trap 'rm -rf "$work"' EXIT

git clone --quiet https://github.com/savant117/sdf-dcd-demo "$work/upstream"
git -C "$work/upstream" checkout --quiet "$commit"

for file in maths.h mesh.h mesh.cpp shapes.h shapes.cpp voxel.cpp body.h collide.h collide.cpp \
            optimize.cpp solver.h solver.cpp; do
    cp "$work/upstream/source/$file" "$root/Sources/SDFCore/upstream/$file"
done
cp "$work/upstream/LICENSE" "$root/Sources/SDFCore/upstream/LICENSE"
# Replace the asset set as a whole, so assets removed upstream do not linger
rm -f "$root/Samples/SDFDemoScenes/Resources/assets/"*.obj
cp "$work/upstream/assets/"*.obj "$root/Samples/SDFDemoScenes/Resources/assets/"

sed -i '' "s/^Commit: \`.*\`$/Commit: \`$(git -C "$work/upstream" rev-parse HEAD)\`/" "$root/UPSTREAM.md"
echo "Vendored $(git -C "$work/upstream" rev-parse --short HEAD). Now: git diff, swift test, swift run -c release sdf-snapshot <dir>"
