#!/usr/bin/env bash
# Manually save the current slam map (default /map) to my_maps/<name>.
# Usage: save_map.sh <name> [map_topic]
set -e
NAME="${1:-manual_map}"
MAP_TOPIC="${2:-/map}"
OUT="/home/lishu2/2025_ZZU_FLY/my_maps/${NAME}"
rosrun map_server map_saver -f "${OUT}" map:="${MAP_TOPIC}"
echo "Saved ${OUT}.pgm / ${OUT}.yaml"
