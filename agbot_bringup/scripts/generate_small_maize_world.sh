#!/bin/bash
# Generate the small maize world (agbot_maize_small.yaml) WITHOUT losing the
# current full world. Run on the ROS1 machine with the workspace sourced.
#
# generate_world.py always writes into ~/.ros/virtual_maize_field/ and the
# world SDF references the heightmap by absolute path with a fixed filename,
# so worlds cannot coexist there -- we snapshot each world and swap the
# active one with switch_maize_world.sh.
set -e

CACHE="${ROS_HOME:-$HOME/.ros}/virtual_maize_field"
SNAPSHOTS="${ROS_HOME:-$HOME/.ros}/virtual_maize_field_snapshots"

# 1. Snapshot the CURRENT world as "full" (never overwrite an existing snapshot).
if [ -d "$SNAPSHOTS/full" ]; then
    echo "Snapshot '$SNAPSHOTS/full' already exists -- keeping it."
elif [ -f "$CACHE/generated.world" ]; then
    mkdir -p "$SNAPSHOTS"
    cp -r "$CACHE" "$SNAPSHOTS/full"
    echo "Snapshotted current world to $SNAPSHOTS/full"
else
    echo "No existing world at $CACHE -- nothing to snapshot."
fi

# 2. Generate the small world (config must live inside virtual_maize_field).
cp "$(rospack find agbot_bringup)/config/agbot_maize_small.yaml" \
   "$(rospack find virtual_maize_field)/config/"
rosrun virtual_maize_field generate_world.py agbot_maize_small

# 3. Snapshot the result as "small".
rm -rf "$SNAPSHOTS/small"
mkdir -p "$SNAPSHOTS"
cp -r "$CACHE" "$SNAPSHOTS/small"
echo "Snapshotted small world to $SNAPSHOTS/small"

# 4. Report the spawn pose baked into the generated world.
echo ""
echo "Small world is now ACTIVE. Spawn pose (pass as x:= y:= z:= yaw:= to agbot_gazebo.launch):"
grep -o '\-x [^ ]* -y [^ ]* -z [^ ]* -R [^ ]* -P [^ ]* -Y [^ ]*' "$CACHE/robot_spawner.launch"
echo ""
echo "Switch worlds anytime with: switch_maize_world.sh full|small"
