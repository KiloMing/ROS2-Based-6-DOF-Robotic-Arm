#!/usr/bin/env bash
set -eo pipefail
export ROS_DOMAIN_ID=0
exec bash "$(dirname -- "${BASH_SOURCE[0]}")/calibrate.sh" --mode eye-to-hand --mount-frame wrist_rotation_link --square-mm 10 "$@"
