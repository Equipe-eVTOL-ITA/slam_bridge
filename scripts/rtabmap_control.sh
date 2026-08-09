#!/bin/bash
# Runtime control for the RTAB-Map SLAM backend via ros2 service calls and
# ros2 param get, and a helper to snapshot the live database into map/.
# Assumes launch/rtabmap.launch.py is already running (node name: rtabmap).
#
# Usage:
#   bash src/slam_bridge/scripts/rtabmap_control.sh <action>
#
# Actions:
#   status              Print current mapping/localization mode + node services
#   start-mapping       Switch to mapping mode   (Mem/IncrementalMemory -> true)
#   start-localization  Switch to localization-only mode on the loaded map
#   toggle-mode         Flip mapping <-> localization based on current status
#   new-map             Start a new map in memory without touching the database
#   save-map            Copy the live database into map/ with a timestamped name
#   backup              Ask rtabmap to write a <db>.back copy next to the database
#   reset               Wipe the current map from memory AND database (destructive)
#   pause               Pause mapping/odometry updates
#   resume              Resume mapping/odometry updates
#
# See map/SLAM_TESTING.md - none of this has been exercised on the Jetson yet.
set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"
# Hardcoded rather than derived from SCRIPT_DIR: save-map must land in the
# Jetson's own map/ dir regardless of where this script physically lives.
MAP_DIR="/home/jetson-evtol/evtol/dev/map"
NODE="/rtabmap"
# rtabmap's default database_path (config/rtabmap_slam_params.yaml doesn't
# override it). Point this at a different path with RTABMAP_DB_PATH=... if
# the launch file is ever changed to set database_path explicitly.
DB_PATH="${RTABMAP_DB_PATH:-$HOME/.ros/rtabmap.db}"

source /opt/ros/humble/setup.bash
[ -f "$WORKSPACE_DIR/install/setup.bash" ] && source "$WORKSPACE_DIR/install/setup.bash"

action="$1"
if [ -z "$action" ]; then
    echo "Usage: $0 <status|start-mapping|start-localization|toggle-mode|new-map|save-map|backup|reset|pause|resume>"
    exit 1
fi

require_node() {
    if ! ros2 node list 2>/dev/null | grep -qx "$NODE"; then
        echo "ERROR: $NODE node not found - is rtabmap.launch.py running?"
        exit 1
    fi
}

case "$action" in
    status)
        require_node
        echo "-- Mem/IncrementalMemory (true = mapping, false = localization) --"
        ros2 param get "$NODE" Mem/IncrementalMemory
        ;;
    start-mapping)
        require_node
        ros2 service call "$NODE/set_mode_mapping" std_srvs/srv/Empty '{}'
        ;;
    start-localization)
        require_node
        ros2 service call "$NODE/set_mode_localization" std_srvs/srv/Empty '{}'
        ;;
    toggle-mode)
        require_node
        current=$(ros2 param get "$NODE" Mem/IncrementalMemory 2>/dev/null | grep -o 'true\|false')
        if [ "$current" == "true" ]; then
            echo "Currently mapping -> switching to localization"
            ros2 service call "$NODE/set_mode_localization" std_srvs/srv/Empty '{}'
        else
            echo "Currently localization (or unknown: '$current') -> switching to mapping"
            ros2 service call "$NODE/set_mode_mapping" std_srvs/srv/Empty '{}'
        fi
        ;;
    new-map)
        require_node
        ros2 service call "$NODE/trigger_new_map" std_srvs/srv/Empty '{}'
        ;;
    backup)
        require_node
        ros2 service call "$NODE/backup" std_srvs/srv/Empty '{}'
        echo "rtabmap wrote a .back copy next to $DB_PATH"
        ;;
    reset)
        require_node
        read -p "This WIPES the current map from memory and database. Continue? [y/N] " confirm
        [ "$confirm" == "y" ] || { echo "Aborted."; exit 1; }
        ros2 service call "$NODE/reset" std_srvs/srv/Empty '{}'
        ;;
    pause)
        require_node
        ros2 service call "$NODE/pause" std_srvs/srv/Empty '{}'
        ;;
    resume)
        require_node
        ros2 service call "$NODE/resume" std_srvs/srv/Empty '{}'
        ;;
    *)
        echo "Unknown action: $action"
        exit 1
        ;;
esac
