#!/bin/bash
# Runs xacro on the Jackal's base URDF with the environment hooks that
# jackal.urdf.xacro / jackal.gazebo read via $(optenv ...).
#
# Args: $1 = path to a URDF xacro to inject as JACKAL_URDF_EXTRAS,
#            or the literal string "none" for a bare Jackal
#       $2 = path to jackal.urdf.xacro
#       $3 = OPTIONAL path to a datum yaml (agbot_gps_nav/config/gps_datum.yaml).
#            When given, its `datum:` latitude and longitude are exported as
#            GAZEBO_WORLD_LAT / GAZEBO_WORLD_LON, which jackal.gazebo:32-33
#            reads to place the simulated GPS receiver on Earth.
#
# WHY THE ENV VARS ARE SET HERE and not with a roslaunch <env> tag: a <env> tag
# applies to NODES, but the URDF is built by a <param name="robot_description"
# command="..."> subprocess, which a <env> tag does not reliably reach. This
# script IS that subprocess, so exporting here is the one place that works.
set -euo pipefail

extras="$1"
jackal_urdf="$2"
datum_yaml="${3:-}"

if [ "$extras" != "none" ]; then
    export JACKAL_URDF_EXTRAS="$extras"
fi

if [ -n "$datum_yaml" ]; then
    # Parse the two numbers out of the yaml. Kept here rather than duplicated
    # into a launch arg so gps_datum.yaml stays the single definition.
    read -r lat lon < <(python3 -c '
import sys, yaml
d = yaml.safe_load(open(sys.argv[1]))["datum"]
print(d[0], d[1])
' "$datum_yaml")
    export GAZEBO_WORLD_LAT="$lat"
    export GAZEBO_WORLD_LON="$lon"
fi

exec xacro "$jackal_urdf" --inorder
