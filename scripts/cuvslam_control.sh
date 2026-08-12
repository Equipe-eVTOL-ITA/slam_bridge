#!/bin/bash
# Runtime control for the Isaac ROS cuVSLAM (isaac_ros_visual_slam) backend
# via ros2 service calls and ros2 param get/set. Must run from a shell that
# can see the container's ROS graph (same DDS domain as cuvslam.launch.py,
# which runs inside the Isaac ROS container with --network host).
#
# UNVERIFIED: isaac_ros_visual_slam's exact service names/types differ across
# Isaac ROS releases, and this pipeline has never been run on the Jetson yet
# (see map/SLAM_TESTING.md). Rather than hardcode names nobody has confirmed,
# save-map/load-map/reset auto-discover the matching service under $CUVSLAM_NS
# and fail loudly with the real service list if nothing matches - run
# 'status' first and adjust CUVSLAM_NS / CUVSLAM_NODE if the names differ.
#
# Usage:
#   bash src/slam_bridge/scripts/cuvslam_control.sh <action> [map_path]
#
# Actions:
#   status              List discovered visual_slam services + mapping param
#   start-mapping       Set enable_localization_n_mapping:=true (may need relaunch)
#   start-localization  Set enable_localization_n_mapping:=false (may need relaunch)
#   toggle-mode         Flip mapping <-> localization based on current param
#   save-map [path]     Call the discovered save_map service (default: map/cuvslam_<timestamp>)
#   load-map <path>     Call the discovered load_map service
#   reset               Call the discovered reset service (destructive)
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"
MAP_DIR="$WORKSPACE_DIR/map"
NS="${CUVSLAM_NS:-/visual_slam}"
PARAM_NODE="${CUVSLAM_NODE:-/visual_slam_node}"

source /opt/ros/humble/setup.bash

action="$1"
if [ -z "$action" ]; then
    echo "Usage: $0 <status|start-mapping|start-localization|toggle-mode|save-map|load-map|reset> [map_path]"
    exit 1
fi

find_service() {
    ros2 service list 2>/dev/null | grep "^$NS" | grep -i "$1" | head -n1
}

require_service() {
    local svc
    svc="$(find_service "$1")"
    if [ -z "$svc" ]; then
        echo "ERROR: no service matching '*$1*' under $NS - is cuvslam.launch.py running inside the Isaac ROS container?" >&2
        echo "Available services under $NS:" >&2
        ros2 service list 2>/dev/null | grep "^$NS" >&2 || echo "  (none found)" >&2
        exit 1
    fi
    echo "$svc"
}

case "$action" in
    status)
        echo "-- Services under $NS --"
        ros2 service list 2>/dev/null | grep "^$NS" || echo "  (none found - is cuvslam running?)"
        echo "-- enable_localization_n_mapping (on $PARAM_NODE) --"
        ros2 param get "$PARAM_NODE" enable_localization_n_mapping 2>/dev/null || echo "  (param node $PARAM_NODE not reachable - set CUVSLAM_NODE if the name differs)"
        ;;
    start-mapping)
        ros2 param set "$PARAM_NODE" enable_localization_n_mapping true
        echo "NOTE: some isaac_ros_visual_slam params only take effect at node startup - relaunch cuvslam.launch.py if this had no effect."
        ;;
    start-localization)
        ros2 param set "$PARAM_NODE" enable_localization_n_mapping false
        echo "NOTE: some isaac_ros_visual_slam params only take effect at node startup - relaunch cuvslam.launch.py if this had no effect."
        ;;
    toggle-mode)
        current=$(ros2 param get "$PARAM_NODE" enable_localization_n_mapping 2>/dev/null | grep -o 'true\|false')
        if [ "$current" == "true" ]; then
            echo "Currently mapping -> switching to localization"
            ros2 param set "$PARAM_NODE" enable_localization_n_mapping false
        else
            echo "Currently localization (or unknown: '$current') -> switching to mapping"
            ros2 param set "$PARAM_NODE" enable_localization_n_mapping true
        fi
        ;;
    save-map)
        svc="$(require_service save_map)"
        mkdir -p "$MAP_DIR"
        dest="${2:-$MAP_DIR/cuvslam_$(date +%d-%m-%Y:%H%M)}"
        ros2 service call "$svc" isaac_ros_visual_slam_interfaces/srv/SaveMap "{map_url: '$dest'}"
        echo "Requested cuVSLAM save map -> $dest"
        ;;
    load-map)
        [ -n "$2" ] || { echo "Usage: $0 load-map <map_path>"; exit 1; }
        svc="$(require_service load_map)"
        ros2 service call "$svc" isaac_ros_visual_slam_interfaces/srv/LoadMap "{map_url: '$2'}"
        ;;
    reset)
        svc="$(require_service reset)"
        read -p "This resets cuVSLAM tracking/mapping state. Continue? [y/N] " confirm
        [ "$confirm" == "y" ] || { echo "Aborted."; exit 1; }
        ros2 service call "$svc" std_srvs/srv/Empty '{}'
        ;;
    *)
        echo "Unknown action: $action"
        exit 1
        ;;
esac
