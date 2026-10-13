#!/bin/bash
# Copy a maize world snapshot with every plant swapped for agbot_nav's tall
# variant (the same meshes at 2x scale, ~1.26 m instead of ~0.63 m).
#
# Usage: make_tall_maize_world.sh <src_snapshot> <dst_snapshot>
#   e.g. make_tall_maize_world.sh gps gps_tall
#        rosrun agbot_bringup switch_maize_world.sh gps_tall
#
# WHY: the stock plants sit below the 0.58 m LiDAR, so the horizontal Velodyne
# cannot see them beyond ~12 m (measured 2026-10-13: zero plant returns from
# the trailer pose 17 m out) and AMCL has nothing to localize on in the
# headland. Real corn is 1-2.5 m. virtual_maize_field hard-codes the plant
# scale in its model files, so the generator cannot do this itself.
#
# ⚠ Taller plants also change what the CAMERA sees. Vision nav was validated on
# the stock plants; treat a vision result from a *_tall world as a new result.
set -e

SRC="$1"
DST="$2"
SNAPSHOTS="${ROS_HOME:-$HOME/.ros}/virtual_maize_field_snapshots"

if [ -z "$SRC" ] || [ -z "$DST" ]; then
    echo "Usage: $(basename "$0") <src_snapshot> <dst_snapshot>"
    ls "$SNAPSHOTS" 2>/dev/null
    exit 1
fi
if [ ! -f "$SNAPSHOTS/$SRC/generated.world" ]; then
    echo "No such snapshot: $SNAPSHOTS/$SRC"
    exit 1
fi

rm -rf "${SNAPSHOTS:?}/$DST"
cp -r "$SNAPSHOTS/$SRC" "$SNAPSHOTS/$DST"
sed -i 's|model://maize_0\([0-9]\)|model://agbot_maize_tall_0\1|g' \
    "$SNAPSHOTS/$DST/generated.world"
n=$(grep -c "model://agbot_maize_tall_" "$SNAPSHOTS/$DST/generated.world")
echo "Wrote $SNAPSHOTS/$DST ($n plants swapped to the tall variant)"
