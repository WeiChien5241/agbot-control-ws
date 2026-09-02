#!/bin/bash
# Generate a virtual_maize_field world from an agbot_bringup config and save it
# as a named snapshot, WITHOUT losing the world that is currently active.
#
# Usage: generate_maize_world.sh <config_basename> <snapshot_name>
#   e.g. generate_maize_world.sh agbot_maize_small small
#        generate_maize_world.sh agbot_maize_long  long
#
# Run on the ROS1 machine with the workspace sourced.
#
# generate_world.py always writes into ~/.ros/virtual_maize_field/ and the
# world SDF references the heightmap by absolute path with a fixed filename,
# so worlds cannot coexist there -- we snapshot each world and swap the
# active one with switch_maize_world.sh.
set -e

CONFIG="$1"
SNAPSHOT="$2"

if [ -z "$CONFIG" ] || [ -z "$SNAPSHOT" ]; then
    echo "Usage: $(basename "$0") <config_basename> <snapshot_name>"
    echo ""
    echo "Configs available in agbot_bringup/config:"
    ls "$(rospack find agbot_bringup)/config"/*.yaml 2>/dev/null \
        | xargs -n1 basename 2>/dev/null | sed 's/\.yaml$/  /;s/^/  /' || echo "  (none)"
    exit 1
fi

CACHE="${ROS_HOME:-$HOME/.ros}/virtual_maize_field"
SNAPSHOTS="${ROS_HOME:-$HOME/.ros}/virtual_maize_field_snapshots"

CONFIG_SRC="$(rospack find agbot_bringup)/config/${CONFIG}.yaml"
if [ ! -f "$CONFIG_SRC" ]; then
    echo "No such config: $CONFIG_SRC"
    exit 1
fi

# 1. First run only: preserve whatever world is already in the cache as "full".
#    Once that snapshot exists it is never touched again, so re-generating a
#    world can never clobber a saved one.
if [ -d "$SNAPSHOTS/full" ]; then
    echo "Snapshot '$SNAPSHOTS/full' already exists -- keeping it."
elif [ -f "$CACHE/generated.world" ]; then
    mkdir -p "$SNAPSHOTS"
    cp -r "$CACHE" "$SNAPSHOTS/full"
    echo "Snapshotted current world to $SNAPSHOTS/full"
else
    echo "No existing world at $CACHE -- nothing to snapshot."
fi

# 2. Generate (the config must live inside virtual_maize_field: generate_world.py
#    only reads configs from its own package).
cp "$CONFIG_SRC" "$(rospack find virtual_maize_field)/config/"
rosrun virtual_maize_field generate_world.py "$CONFIG"

# 3. Snapshot the result under the requested name.
rm -rf "${SNAPSHOTS:?}/$SNAPSHOT"
mkdir -p "$SNAPSHOTS"
cp -r "$CACHE" "$SNAPSHOTS/$SNAPSHOT"
echo "Snapshotted '$CONFIG' world to $SNAPSHOTS/$SNAPSHOT"

# 4. Report what was built. The spawn pose is baked into the generated world and
#    differs per world even at the same seed -- the launch file's defaults are
#    for the SMALL world, so anything else needs these passed explicitly.
echo ""
echo "World '$SNAPSHOT' is now ACTIVE."
if [ -f "$CACHE/gt_map.csv" ]; then
    echo "Plants placed: $(($(wc -l < "$CACHE/gt_map.csv") - 1))"
fi
echo "Spawn pose (pass as x:= y:= z:= yaw:= to agbot_gazebo.launch):"
grep -o '\-x [^ ]* -y [^ ]* -z [^ ]* -R [^ ]* -P [^ ]* -Y [^ ]*' "$CACHE/robot_spawner.launch"
echo ""
echo "Switch worlds anytime with: switch_maize_world.sh $(ls "$SNAPSHOTS" | tr '\n' '|' | sed 's/|$//')"
