#!/bin/bash
# Save the current SLAM map session and a viewable PGM image.
# Run this while the slam_mapping.launch.py stack is active.
set -e

MAP_DIR="$HOME/f1tenth_maps"
MAP_NAME="track_map"
FULL_PATH="$MAP_DIR/$MAP_NAME"

mkdir -p "$MAP_DIR"

echo "==> Serializing slam_toolbox pose graph (reloadable session)..."
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
    "{filename: '$FULL_PATH'}"

echo ""
echo "==> Saving viewable occupancy grid (subscribes /map with Transient Local QoS)..."
python3 "$(dirname "$0")/save_pgm.py" "$FULL_PATH"

echo ""
echo "Files saved to $MAP_DIR:"
echo "  Reloadable session : ${FULL_PATH}.data"
echo "                       ${FULL_PATH}.posegraph"
echo "  Viewable map       : ${FULL_PATH}.pgm"
echo "                       ${FULL_PATH}.yaml"
