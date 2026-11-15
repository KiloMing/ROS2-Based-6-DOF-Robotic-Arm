#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/jazzy/setup.bash
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
export LD_LIBRARY_PATH="/home/kiloming/OrbbecSDK_v1.10.37/SDK/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
exec /home/kiloming/pyorbbecsdk/.venv/bin/python collect_handeye.py "$@"
