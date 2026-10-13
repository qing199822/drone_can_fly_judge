#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PACKAGE_DIR="$(rospack find auto_explore 2>/dev/null || true)"
if [[ -z "${PACKAGE_DIR}" ]]; then
  PACKAGE_DIR="$(cd "${SCRIPT_DIR}/../.." && pwd)"
fi
ROOT_DIR="$(cd "${PACKAGE_DIR}/../.." && pwd)"
CONDA_PREFIX="${YOLO_GPU_PREFIX:-${HOME}/miniconda3/envs/yolo_gpu}"
ROS_PYTHON="/opt/ros/noetic/lib/python3/dist-packages"
DEVEL_PYTHON="${ROOT_DIR}/devel/lib/python3/dist-packages"
NODE_SCRIPT="${SCRIPT_DIR}/road_follow.py"
if [[ ! -f "${NODE_SCRIPT}" ]]; then
  NODE_SCRIPT="${ROOT_DIR}/src/auto_explore/scripts/road_follow.py"
fi

if [[ ! -x "${CONDA_PREFIX}/bin/python" ]]; then
  echo "road_follow_gpu.sh: missing Python interpreter: ${CONDA_PREFIX}/bin/python" >&2
  exit 1
fi
if [[ ! -f "${ROOT_DIR}/src/auto_explore/scripts/road_follow.py" ]]; then
  echo "road_follow_gpu.sh: source road_follow.py not found under ${ROOT_DIR}" >&2
  exit 1
fi

export PYTHONPATH="${DEVEL_PYTHON}:${ROS_PYTHON}:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="${CONDA_PREFIX}/lib:${LD_LIBRARY_PATH:-}"
export PATH="${CONDA_PREFIX}/bin:${PATH}"

exec "${CONDA_PREFIX}/bin/python" "${NODE_SCRIPT}" "$@"
