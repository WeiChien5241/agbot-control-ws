#!/bin/bash
# Swap the active virtual_maize_field world between saved snapshots.
# Usage: switch_maize_world.sh full|small
#
# Snapshots are created by generate_small_maize_world.sh. The chosen
# snapshot is copied back into the canonical cache folder (the world SDF
# references its heightmap there by absolute path), and Gazebo's terrain
# paging cache is cleared -- both worlds use the same heightmap filename,
# so Gazebo would otherwise render stale cached terrain.
set -e

CACHE="${ROS_HOME:-$HOME/.ros}/virtual_maize_field"
SNAPSHOTS="${ROS_HOME:-$HOME/.ros}/virtual_maize_field_snapshots"

WORLD="$1"
if [ -z "$WORLD" ] || [ ! -d "$SNAPSHOTS/$WORLD" ]; then
    echo "Usage: $(basename "$0") <snapshot>"
    echo "Available snapshots:"
    ls "$SNAPSHOTS" 2>/dev/null || echo "  (none -- run generate_small_maize_world.sh first)"
    exit 1
fi

rm -rf "$CACHE"
cp -r "$SNAPSHOTS/$WORLD" "$CACHE"
rm -rf "$HOME/.gazebo/paging/virtual_maize_field_heightmap"

echo "Active world: $WORLD"
echo "Spawn pose (pass as x:= y:= z:= yaw:= to agbot_gazebo.launch):"
grep -o '\-x [^ ]* -y [^ ]* -z [^ ]* -R [^ ]* -P [^ ]* -Y [^ ]*' "$CACHE/robot_spawner.launch"
