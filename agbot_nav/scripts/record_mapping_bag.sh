#!/bin/bash
# rosbag record for mapping.launch record:=true. rosbag does not create the
# output directory, and on the robot ~/bags did not exist: the recorder died
# at startup and a whole mapping lap went unrecorded (2026-10-20).
mkdir -p "$HOME/bags"
exec rosrun rosbag record -o "$HOME/bags/mapping" "$@"
