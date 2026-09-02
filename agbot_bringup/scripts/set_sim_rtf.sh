#!/bin/bash
# Throttle Gazebo's real-time factor on a RUNNING simulation.
#
# Usage: set_sim_rtf.sh <factor>        e.g. set_sim_rtf.sh 0.3
#        set_sim_rtf.sh 1.0             restore full speed
#
# WHY THIS EXISTS -- it is not a way to hide a slow laptop, it is how a fast
# run becomes a fair test of the CONTROLLER.
#
# The vision pipeline on the dev laptop runs at ~2.3 Hz wall clock (measured
# 2026-09-02: 218 ms inference, 431 ms end-to-end control period). At
# linear_x_cruise 0.5 m/s that is 0.22 m of open-loop travel per decision and a
# command acting on a view 0.27 m out of date -- larger than the ~0.16 m of
# clearance the robot has on each side of a 0.75 m row. The robot drives into
# the corn, and the run has measured the laptop rather than the controller.
#
# use_sim_time is already true for these worlds, so EVERYTHING the node reasons
# with -- max_data_age_sec, blocked_confirm_seconds, mpc_dt, the exit detector's
# odometry distances and every metrics timestamp -- is in SIM seconds. Slowing
# wall-clock time therefore raises the effective inference rate in the robot's
# own time base without changing a single tuning value: at RTF 0.3 that 431 ms
# wall cycle is a 0.13 s cycle in sim time, i.e. ~7.7 Hz, which is the same
# feedback-per-metre as the field-proven 0.15 m/s runs.
#
# ⚠ The alternative -- raising max_data_age_sec so the watchdog stops firing --
# buys blind driving at speed (1.0 s is 0.5 m at 0.5 m/s) and throws away the
# one signal that was correctly reporting the problem. Do not do that.
set -e

FACTOR="$1"
if [ -z "$FACTOR" ]; then
    echo "Usage: $(basename "$0") <factor>   (1.0 = full speed, 0.3 = three times slower)"
    exit 1
fi

# The generated virtual_maize_field worlds use max_step_size 0.005, so the
# update rate that yields a given real-time factor is factor / 0.005.
STEP_SIZE=0.005
RATE=$(awk "BEGIN { printf \"%.0f\", $FACTOR / $STEP_SIZE }")

if ! command -v gz >/dev/null 2>&1; then
    echo "gz not found -- is Gazebo installed and the environment sourced?"
    exit 1
fi

gz physics -u "$RATE"
echo "real_time_update_rate = $RATE  (target RTF $FACTOR at max_step_size $STEP_SIZE)"
echo "Read the ACTUAL factor off Gazebo's status bar -- if it is already below"
echo "$FACTOR the machine, not this setting, is the limit."
