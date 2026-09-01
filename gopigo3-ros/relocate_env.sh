#!/bin/bash
# Make a full_deploy tree runnable after it has been copied to the robot.
#
# Conan's generated environment scripts reach every dependency through
# "$script_folder/../../../deploy/...", which is relative and survives the trip, but they open
# by pinning script_folder to the absolute path of the machine that generated them. On the
# robot that path does not exist, so LD_LIBRARY_PATH and AMENT_PREFIX_PATH end up empty and the
# node dies looking for its first .so. Recomputing the anchor from each script's own location
# is what actually makes the tree relocatable.
#
# Run it once per transfer, from the folder holding build/ and deploy/:
#
#   bash relocate_env.sh
#
# Invoked through bash rather than directly because a checkout made on Windows carries no
# executable bit, and rsync -a faithfully preserves its absence. It is idempotent: rewriting an
# already-rewritten script leaves it unchanged.
set -euo pipefail

here="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" && pwd)"
generators="${1:-$here/build/Release/generators}"

if [ ! -d "$generators" ]; then
    echo "no generators folder at $generators" >&2
    exit 1
fi

self_locating='script_folder="$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" \&\& pwd)"'

for f in "$generators"/conanrunenv-*.sh "$generators"/conanbuildenv-*.sh; do
    [ -f "$f" ] || continue
    sed -i "1s|^script_folder=.*|$self_locating|" "$f"
done

# The conanrun.sh/conanbuild.sh wrappers hold a second absolute path, the one they source.
for f in "$generators"/conanrun.sh "$generators"/conanbuild.sh; do
    [ -f "$f" ] || continue
    sed -i 's|^\. ".*/\(conan[a-z]*env-[^/"]*\.sh\)"$|. "$(cd "$(dirname "${BASH_SOURCE[0]:-$0}")" \&\& pwd)/\1"|' "$f"
done

echo "rewrote the environment scripts in $generators"
