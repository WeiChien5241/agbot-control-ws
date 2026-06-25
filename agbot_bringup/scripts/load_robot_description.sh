#!/bin/bash
# Sets JACKAL_URDF_EXTRAS so xacro picks up the agbot camera,
# then runs xacro on the Jackal's base URDF.
# Args: $1 = path to agbot_camera.urdf.xacro
#       $2 = path to jackal.urdf.xacro
export JACKAL_URDF_EXTRAS="$1"
exec xacro "$2" --inorder
