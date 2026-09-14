#!/usr/bin/env bash
#
# Tear down a simulation session completely: roscore, Gazebo, RViz, the Jackal
# stack and any vision/GPS nav node.
#
# WHY THIS EXISTS -- a session that is only PARTLY killed is worse than one
# left running, and it is very hard to recognise from the symptom.
#
# 2026-09-14: two mission launches from the operator panel died on the line
# after "vision_nav_node ready", the robot never moved, and both metrics CSVs
# had a header and no rows. The cause was a roscore and Gazebo left running 33
# hours earlier. It broke the next session in two separate ways:
#
#   1. A node killed with SIGKILL never unregisters, so /vision_nav_node was
#      still registered on that master with the URI of a long-dead process.
#      Every new node with the same name was evicted ~1.5 s after registering,
#      while it was still loading the model.
#   2. The Jackal's own stack had died piecemeal, so the wheel controller was
#      unloaded and Gazebo was logging "Can't accept new commands. Controller
#      is not running." at 10 Hz. Nothing could have moved the robot.
#
# ⚠ THE TRAP THIS SCRIPT EXISTS TO AVOID: `pkill -f roslaunch` matches the
# FULL COMMAND LINE of every process, INCLUDING the shell running the pkill if
# that shell's own command line contains the word. A one-liner like
#
#     pkill -f roslaunch; pkill -f gzserver; pkill -f rosmaster
#
# kills itself on the first pattern and the rest never run -- which is exactly
# how the 33-hour-old roscore above survived a cleanup that looked fine. This
# script therefore excludes its own process ancestry explicitly.

set -u

DRY_RUN=0
[ "${1:-}" = "--dry-run" ] && DRY_RUN=1

# Matched against the full command line, most specific first.
PATTERNS=(
    "vision_nav_node.py"
    "gps_nav_node.py"
    "mission_supervisor.py"
    "operator_panel.py"
    "roslaunch"
    "gzserver"
    "gzclient"
    "/rviz"
    "mapviz"
    "twist_mux"
    "marker_server"
    "robot_state_publisher"
    "ekf_localization"
    "navsat_transform"
    "controller_spawner"
    "rosout"
    "rosmaster"
)

# Never kill ourselves or anything we are running under (the calling shell, the
# terminal, sshd, ...). Without this the script is the very bug it documents.
protected=" "
pid=$$
while [ "$pid" -gt 1 ]; do
    protected="$protected$pid "
    pid=$(ps -o ppid= -p "$pid" 2>/dev/null | tr -d ' ')
    [ -z "$pid" ] && break
done

killed=0
for pat in "${PATTERNS[@]}"; do
    for target in $(pgrep -f -- "$pat" 2>/dev/null); do
        case "$protected" in *" $target "*) continue ;; esac
        cmd=$(ps -o cmd= -p "$target" 2>/dev/null | cut -c1-70)
        [ -z "$cmd" ] && continue
        if [ "$DRY_RUN" = "1" ]; then
            echo "would kill $target  $cmd"
        else
            echo "kill $target  $cmd"
            kill -9 "$target" 2>/dev/null
            killed=$((killed + 1))
        fi
    done
done

[ "$DRY_RUN" = "1" ] && exit 0

sleep 2
left=$(pgrep -f -- "rosmaster|gzserver" 2>/dev/null | while read -r p; do
    case "$protected" in *" $p "*) continue ;; esac
    echo "$p"
done | wc -l)
echo "killed $killed process(es); rosmaster/gzserver still up: $left"
[ "$left" -gt 0 ] && echo "⚠ something survived -- check 'ps -eo pid,etimes,cmd | grep ros'"
exit 0
