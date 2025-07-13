#!/bin/bash

# Default values
use_multi="false"
ring_num="128"
use_rviz="false"

# Parse named parameters
while [[ $# -gt 0 ]]; do
  case "$1" in
    --use-multi)
      use_multi="$2"
      shift 2
      ;;
    --ring-num)
      ring_num="$2"
      shift 2
      ;;
    --use-rviz)
      use_rviz="$2"
      shift 2
      ;;
    -h|--help)
      echo "Usage: $0 [--use-multi true|false] [--ring-num 128|64|32] [--use-rviz true|false]"
      exit 0
      ;;
    *)
      echo "Unknown option: $1"
      exit 1
      ;;
  esac
done

echo "➤ Configurations:"
echo "  use_multi: $use_multi"
echo "  ring_num:  $ring_num"
echo "  use_rviz:  $use_rviz"
echo ""

# Source ROS setup
source ../devel/setup.bash

# Set the directory containing rosbag files
BAG_DIR="/home/runrunxin/SF/rosbag/e4/0408/ready" # Change this to your actual directory


for bagfile in "$BAG_DIR"/rosbag*.bag; do
    filename=$(basename -- "$bagfile")
    traj=${filename%.bag}
    traj=${traj#rosbag}

    echo "======== Running for traj: $traj ========"

    rosbag play "$bagfile" --clock &
    BAG_PID=$!

    roslaunch clustering_and_tracking localization.launch \
        arg_traj_num_str:="$traj" \
        arg_bias:=1.30 \
        arg_id_to_track:=-1 \
        arg_auto:=true \
        arg_model_init:=1 \
        arg_use_multi:=$use_multi \
        arg_rings:=$ring_num \
        rviz:=$use_rviz &
    LAUNCH_PID=$!

    wait $LAUNCH_PID
    kill $BAG_PID
    wait $BAG_PID 2>/dev/null
    echo "Finished $traj"
done

echo "✅ All rosbag executions finished."
