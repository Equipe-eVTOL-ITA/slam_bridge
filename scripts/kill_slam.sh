#!/bin/bash
# Tear down every part of the VIO pipeline, host side AND Isaac ROS container,
# and prove nothing is left publishing to PX4.
#
# WHY THIS EXISTS
# Stale nodes do not announce themselves - they just keep publishing. Bags
# rosbag2_2026_08_12-19_17_19 and -19_19_23 recorded /fmu/in/vehicle_visual_odometry
# at 154 Hz and 199 Hz (a single source is 30 Hz) because FOUR slam_bridge /
# cuVSLAM instances were alive at once, interleaving four different
# trajectories - two of them hundreds of metres divergent. EKF2 received all
# of them. That is invisible unless you go looking, hence step 4 below.
#
# Usage:
#   bash src/slam_bridge/scripts/kill_slam.sh          # kill + verify
#   bash src/slam_bridge/scripts/kill_slam.sh --check  # verify only, kill nothing

set -uo pipefail
CHECK_ONLY=0
[ "${1:-}" = "--check" ] && CHECK_ONLY=1

EV_TOPIC=/fmu/in/vehicle_visual_odometry

say() { printf '\n\033[1m== %s\033[0m\n' "$*"; }

if [ "$CHECK_ONLY" -eq 0 ]; then

    say "1. Isaac ROS container"
    # run_dev.sh names it isaac_ros_dev-<arch>-container; match loosely so a
    # renamed or hand-started container is still caught.
    mapfile -t CIDS < <(docker ps -q --filter "name=isaac_ros_dev" 2>/dev/null)
    if [ "${#CIDS[@]}" -gt 0 ]; then
        for c in "${CIDS[@]}"; do
            echo "  stopping $(docker inspect --format '{{.Name}}' "$c" 2>/dev/null) ($c)"
            # SIGINT first so ROS nodes shut down cleanly, then stop.
            docker exec "$c" bash -lc 'pkill -INT -f component_container || true' 2>/dev/null
            sleep 2
            docker stop -t 5 "$c" >/dev/null 2>&1
        done
    else
        echo "  (no isaac_ros_dev container running)"
    fi
    # run_dev.sh uses --rm, so a stopped container normally self-removes.
    docker container prune -f >/dev/null 2>&1

    say "2. Container-side processes that outlived the container"
    # --network host means these can still hold DDS ports even if docker ps is empty.
    for pat in component_container visual_slam isaac_ros; do
        pids=$(pgrep -f "$pat" 2>/dev/null | tr '\n' ' ')
        [ -n "$pids" ] && { echo "  SIGINT $pat -> $pids"; pkill -INT -f "$pat"; }
    done
    sleep 2
    for pat in component_container visual_slam isaac_ros; do
        pgrep -f "$pat" >/dev/null 2>&1 && { echo "  SIGKILL $pat (did not exit)"; pkill -KILL -f "$pat"; }
    done

    say "3. Host-side nodes"
    for pat in slam_bridge rect_camera_info_fixer depthai_ros_driver camera_node \
               robot_state_publisher rgbd_odometry rtabmap imu_filter_madgwick; do
        pids=$(pgrep -f "$pat" 2>/dev/null | tr '\n' ' ')
        [ -n "$pids" ] && { echo "  SIGINT $pat -> $pids"; pkill -INT -f "$pat"; }
    done
    sleep 2
    for pat in slam_bridge rect_camera_info_fixer depthai_ros_driver camera_node \
               robot_state_publisher rgbd_odometry rtabmap imu_filter_madgwick; do
        pgrep -f "$pat" >/dev/null 2>&1 && { echo "  SIGKILL $pat"; pkill -KILL -f "$pat"; }
    done

    # Orphaned launch supervisors respawn children if left alive.
    pkill -KILL -f "ros2 launch" 2>/dev/null
    # FastDDS shared-memory segments outlive crashed processes and can block
    # the next run's discovery.
    rm -rf /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null

    sleep 2
fi

say "4. VERIFY - what is still alive"
source /opt/ros/humble/setup.bash 2>/dev/null
[ -f /home/evtol/evtol/dev/install/setup.bash ] && source /home/evtol/evtol/dev/install/setup.bash 2>/dev/null

echo "-- matching processes --"
pgrep -af 'slam_bridge|component_container|visual_slam|depthai|rtabmap|rgbd_odometry|rect_camera_info' \
    || echo "  none"

echo "-- docker --"
docker ps --filter "name=isaac_ros_dev" --format '  {{.Names}} {{.Status}}' 2>/dev/null | grep . \
    || echo "  no isaac container"

echo "-- publishers on $EV_TOPIC (MUST be 0 before relaunch, 1 while running) --"
timeout 8 ros2 topic info -v "$EV_TOPIC" 2>/dev/null \
    | grep -E "Publisher count|Node name|Node namespace" \
    || echo "  (no ROS graph reachable - nothing running, or wrong ROS_DOMAIN_ID)"

echo
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0}  RMW=${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"
echo "(the container must match BOTH of these or it will not see the host's /tf_static)"
