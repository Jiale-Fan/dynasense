#!/usr/bin/env bash
# Real robot, after `roslaunch dynasense live_wavemap.launch`: estimate the floor
# height from the four feet, then start launch/bps_hardware.launch with it (BPS
# node, flat-floor ground clearance, latched floor height for the bridge).
#
#   rosrun dynasense start_bps_hardware.sh
#   rosrun dynasense start_bps_hardware.sh --floor-z -0.29        # skip the estimate
#   rosrun dynasense start_bps_hardware.sh --duration 3 -- ground_clearance:=false
#
# Options before "--" go to estimate_floor_z.py (--duration, --frame, --max-spread,
# --timeout), except --floor-z; everything after "--" goes to roslaunch.
# The robot must stand still on flat ground while the floor is estimated.
set -o pipefail

floor=""
frame="odom"
estimate_args=()
launch_args=()
while [ $# -gt 0 ]; do
  case "$1" in
    --floor-z) floor="$2"; shift 2 ;;
    --frame) frame="$2"; estimate_args+=("$1" "$2"); shift 2 ;;
    --) shift; launch_args=("$@"); break ;;
    *) estimate_args+=("$1"); shift ;;
  esac
done

if [ -z "$floor" ]; then
  echo "[start_bps_hardware] estimating the floor height (stand still on flat ground) ..." >&2
  floor=$(rosrun dynasense estimate_floor_z.py "${estimate_args[@]}")
  rc=$?
  if [ $rc -ne 0 ] || [ -z "$floor" ]; then
    echo "[start_bps_hardware] floor estimation failed (exit $rc); nothing started" >&2
    exit 1
  fi
fi
echo "[start_bps_hardware] floor_z = $floor m in $frame" >&2
exec roslaunch dynasense bps_hardware.launch floor_z:="$floor" frame:="$frame" "${launch_args[@]}"
