#!/bin/bash

# Set the directory containing rosbag files
BAG_DIR="/home/runrunxin/SF/rosbag/e4/0408/ready" # Change this to your actual directory
source ../devel/setup.bash

# Set default values for use_multi and ring_num
use_multi=${1:-false}  # Default to 'false' if no input is provided
ring_num=${2:-128}       # Default to '128' if no input is provided
use_rviz=${3:-false}       # Default to 'false' if no input is provided


# Iterate over all rosbag_*.bag files
for bagfile in "$BAG_DIR"/rosbag*.bag
do
    # Get the filename without path, e.g., rosbag_2a.bag
    filename=$(basename -- "$bagfile")

    # Extract the trajectory ID, e.g., 2a
    traj=${filename%.bag}          # rosbag_2a
    traj=${traj#rosbag}            # Remove prefix rosbag_

    echo "======== Running for traj: $traj ========"

    # Play rosbag (in background)
    rosbag play "$bagfile" --clock &
    BAG_PID=$!

    # Launch roslaunch file (in background) with use_multi and ring_num arguments
    roslaunch clustering_and_tracking localization.launch \
        arg_traj_num_str:="$traj" \
        arg_bias:=1.30 \
        arg_id_to_track:=-1 \
        arg_auto:=true \
        arg_model_init:=1 \
        arg_use_multi:=$use_multi \
        rviz:=$use_rviz \
        arg_rings:=$ring_num &
    LAUNCH_PID=$!

    echo "Started rosbag (PID=$BAG_PID) and launch file (PID=$LAUNCH_PID)"

    # Wait for roslaunch to finish (make sure roslaunch exits when internal programs finish)
    wait $LAUNCH_PID
    echo "Launch finished for $traj"

    # Kill rosbag playback
    kill $BAG_PID
    wait $BAG_PID 2>/dev/null

    echo "Killed rosbag for $traj"
    echo ""
done

echo "✅ All rosbag executions finished."
