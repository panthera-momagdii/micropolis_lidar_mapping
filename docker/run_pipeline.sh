#!/usr/bin/env bash
# Variant A (LiDAR-only) + Variant B (GNSS fusion) over ONE folder of .mcap bags,
# treated as a SINGLE continuous session. Bags from $BAGS_DIR; results to $OUT_DIR.
# Session boundaries are NOT decided here — give the container one session's bags.
set -euo pipefail

BAGS_DIR=${BAGS_DIR:-/bags}
OUT_DIR=${OUT_DIR:-/out}

APP=/ws/src/app
PARAMS="$APP/lidar_mapper/params.yaml"
PARAMS_FUSION="$APP/gnss_fusion/params_fusion.yaml"
EVAL_PY="$APP/lidar_mapper/eval/evaluate.py"
PLOT_PY="$APP/lidar_mapper/eval/plot_map_trajectory.py"
PYTHON=/opt/eval-venv/bin/python

# ROS 2's setup.bash (ament template) references $AMENT_TRACE_SETUP_FILES unguarded, which trips
# `set -u`; disable -u only across the two sourced setup files, then restore it.
set +u
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
set -u

if ! compgen -G "$BAGS_DIR/*.mcap" > /dev/null; then
  echo "ERROR: no .mcap files in $BAGS_DIR — mount one session's bags with -v <host>:/bags:ro" >&2
  exit 1
fi
mkdir -p "$OUT_DIR/variant_a" "$OUT_DIR/variant_b" "$OUT_DIR/eval"

# All four tools accept a directory and glob+sort it -> the folder IS the session.
echo ">> ground truth"
ros2 run lidar_mapper export_gt    "$OUT_DIR/gt.tum" "$BAGS_DIR"
echo ">> Variant A (LiDAR-only odometry + map)"
ros2 run lidar_mapper run_odometry "$PARAMS" "$OUT_DIR/variant_a" "$BAGS_DIR"
echo ">> Variant B (GNSS pose-graph fusion + regenerated map)"
ros2 run gnss_fusion  run_fusion   "$BAGS_DIR" "$OUT_DIR/variant_a/trajectory.tum" "$PARAMS_FUSION" "$OUT_DIR/variant_b"
ros2 run gnss_fusion  regen_map    "$BAGS_DIR" "$OUT_DIR/variant_b/fused.tum"       "$PARAMS"        "$OUT_DIR/variant_b"
echo ">> evaluation"
"$PYTHON" "$EVAL_PY" "$OUT_DIR/gt.tum" "$OUT_DIR/variant_a/trajectory.tum" "$OUT_DIR/eval/variant_a"
"$PYTHON" "$EVAL_PY" "$OUT_DIR/gt.tum" "$OUT_DIR/variant_b/fused.tum"       "$OUT_DIR/eval/variant_b"
echo ">> map + trajectory overview plots (alongside each map)"
"$PYTHON" "$PLOT_PY" "$OUT_DIR/variant_a/map.pcd"       "$OUT_DIR/variant_a/trajectory.tum" \
  -o "$OUT_DIR/variant_a/map_overview.png" --label variant_a
"$PYTHON" "$PLOT_PY" "$OUT_DIR/variant_b/fused_map.pcd" "$OUT_DIR/variant_b/fused.tum" \
  -o "$OUT_DIR/variant_b/map_overview.png" --label variant_b

echo ">> DONE. Results under $OUT_DIR : variant_a/, variant_b/, gt.tum, eval/"
