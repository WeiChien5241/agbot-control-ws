#!/bin/bash
# Save the map being built by mapping.launch, then pad it.
#
# Usage: rosrun agbot_nav save_map.sh <name> [pad_m]
#   -> $(rospack find agbot_nav)/maps/<name>.{pgm,yaml}
#
# WHY PAD: SLAM only maps the area around things the LiDAR actually hit.
# Open ground with nothing on it stays off the map, and a robot standing there
# -- e.g. the start pose, ~0.9 m west of the nearest corn in the field world
# -- is "off the map": AMCL cannot localize it and move_base refuses to plan
# ("The robot's start position is off the global costmap"). pad_map.py adds
# pad_m metres (default 5) of UNKNOWN on every side, which the planner treats
# as drivable.
set -e
NAME="$1"
PAD="${2:-5}"
if [ -z "$NAME" ]; then
    echo "Usage: $(basename "$0") <name> [pad_m]"
    exit 1
fi
OUT="$(rospack find agbot_nav)/maps/$NAME"
rosrun map_server map_saver -f "$OUT"
rosrun agbot_nav pad_map.py "$OUT.yaml" "$PAD"
# map_saver writes the image as an ABSOLUTE path, so a map copied off the
# robot (/home/administrator/...) would not open on the laptop. map_server
# resolves a relative path against the yaml's own folder.
sed -i "s|^image: .*|image: $NAME.pgm|" "$OUT.yaml"
# The map's (0, 0) is where odom started (robot boot), not where mapping
# started; record_start_pose.py (mapping.launch) logged the start spot.
START="$HOME/.ros/agbot_mapping_start_pose"
if [ -f "$START" ]; then
    cp "$START" "$OUT.start_pose"
    read -r SX SY SYAW < "$START"
else
    SX=0; SY=0; SYAW=0
    echo "WARNING: no recorded start pose ($START); 0 0 0 is only right if odom started at the start spot"
fi
echo "Saved $OUT.yaml -- start the robot on the SAME spot it started mapping from, then:"
echo "  roslaunch agbot_nav transit.launch rviz:=true map_file:=$OUT.yaml initial_x:=$SX initial_y:=$SY initial_yaw:=$SYAW"
