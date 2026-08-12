#!/bin/bash
# Single entry point for both SLAM backends - what the VS Code task
# "launch slam" runs, so picking a backend from the task dropdown is all
# that is needed to bring the pipeline up.
#
#   bash src/slam_bridge/scripts/launch_slam.sh rtabmap
#   bash src/slam_bridge/scripts/launch_slam.sh vslam [enable_imu_fusion:=false ...]
#
# rtabmap runs entirely on the host (one launch file). vslam is split
# across the container boundary and is the reason this script exists:
# ISAAC_ROS_VSLAM_OAKD_PLAN.md documents it as two terminals,
#
#   ros2 launch slam_bridge vslam.launch.py                        # host
#   cd src/isaac_ros_common && ./scripts/run_dev.sh -d <workspace>  # container
#   ros2 launch /workspaces/isaac_ros-dev/src/slam_bridge/launch/cuvslam.launch.py
#
# and this script drives both halves from one. Extra arguments are passed
# through to the backend's launch file (cuvslam.launch.py for vslam).
#
# Ctrl+C stops everything, including the container it started.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"
ISAAC_SCRIPTS_DIR="$WORKSPACE_DIR/src/isaac_ros_common/scripts"

PLATFORM="$(uname -m)"
CONTAINER_IMAGE="isaac_ros_dev-$PLATFORM"
CONTAINER_NAME="$CONTAINER_IMAGE-container"
# Where the workspace is mounted inside the container by run_dev.sh.
CONTAINER_WS="/workspaces/isaac_ros-dev"

BACKEND="${1:-}"
shift || true

usage() {
    echo "usage: $(basename "$0") <rtabmap|vslam> [extra ros2 launch args]"
}

setup_host_ros() {
    if [ ! -f "$WORKSPACE_DIR/install/setup.bash" ]; then
        echo "ERROR: $WORKSPACE_DIR/install is missing - build the workspace first:"
        echo "         colcon build --packages-up-to slam_bridge"
        exit 1
    fi
    # ROS/ament setup files read unset variables (AMENT_TRACE_SETUP_FILES and
    # friends), so `set -u` has to stand down while they are sourced.
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/humble/setup.bash
    # shellcheck disable=SC1091
    source "$WORKSPACE_DIR/install/setup.bash"
    set -u
}

# ---------------------------------------------------------------- rtabmap

run_rtabmap() {
    setup_host_ros
    echo "== RTAB-Map: camera + rgbd_odometry + rtabmap + slam_bridge (host only)"
    exec ros2 launch slam_bridge rtabmap.launch.py "$@"
}

# ------------------------------------------------------------------ vslam

HOST_PID=""      # ros2 launch vslam.launch.py
KEEPER_PID=""    # holds the run_dev.sh container session open (see below)
KEEPER_LOG=""
KEEPER_FIFO=""
KEEPER_FD=""

cleanup() {
    trap - EXIT INT TERM
    # An exit handler must never abort half way through and strand a
    # container or a temp file, so `set -e` stands down for the rest of it.
    set +e
    echo
    echo "== shutting down"

    if [ -n "$HOST_PID" ] && kill -0 "$HOST_PID" 2>/dev/null; then
        kill -INT "$HOST_PID" 2>/dev/null || true
        wait "$HOST_PID" 2>/dev/null || true
    fi

    if [ -n "$KEEPER_PID" ] && kill -0 "$KEEPER_PID" 2>/dev/null; then
        # Stopping the container makes the held-open shell exit on its own;
        # docker run --rm then removes it.
        docker stop -t 5 "$CONTAINER_NAME" >/dev/null 2>&1 || true
        kill "$KEEPER_PID" 2>/dev/null || true
        wait "$KEEPER_PID" 2>/dev/null || true
    fi

    if [ -n "$KEEPER_FD" ]; then
        eval "exec $KEEPER_FD>&-"
    fi
    rm -f "$KEEPER_FIFO" "$KEEPER_LOG"
}

container_running() {
    [ -n "$(docker ps --quiet --filter status=running --filter "name=^/$CONTAINER_NAME$")" ]
}

start_container() {
    if container_running; then
        echo "== reusing running container $CONTAINER_NAME"
        return
    fi

    if [ ! -x "$ISAAC_SCRIPTS_DIR/run_dev.sh" ]; then
        echo "ERROR: $ISAAC_SCRIPTS_DIR/run_dev.sh not found - is isaac_ros_common cloned into src/?"
        exit 1
    fi
    if [ -z "$(docker images --quiet "$CONTAINER_IMAGE")" ]; then
        echo "ERROR: docker image $CONTAINER_IMAGE does not exist. Build it once with:"
        echo "         bash src/slam_bridge/scripts/setup_isaac_container.sh --build"
        exit 1
    fi

    # run_dev.sh only ever ends in `docker run -it --rm ... /bin/bash`: it
    # cannot start detached and cannot be handed a command (the trailing
    # "$@" it forwards only applies when attaching to an ALREADY running
    # container). So start it as a background session instead of
    # reimplementing its ~30 docker arguments here:
    #   - `script` gives it the PTY that `docker run -it` demands;
    #   - its stdin is a fifo this script holds open, so the container's shell
    #     never reads EOF and the container stays up until cleanup() closes
    #     the fifo / stops it. (A `sleep infinity | ...` pipe does the same
    #     job but leaks the sleep: it never writes, so it never gets SIGPIPE.)
    # The real work then goes in through `docker exec` below.
    # TERM is forced: run_dev.sh colours its output with `tput setaf`, which
    # fails when TERM is unset or "dumb" (what a VS Code task shell gets) and
    # takes the whole script down silently through its `set -e`. `script`
    # gives it a real PTY, so claiming xterm here is honest.
    echo "== starting container $CONTAINER_NAME (-b: image built by setup_isaac_container.sh)"
    KEEPER_LOG="$(mktemp -t launch_slam_container.XXXXXX.log)"
    KEEPER_FIFO="$(mktemp -u -t launch_slam_stdin.XXXXXX)"
    mkfifo "$KEEPER_FIFO"
    # <> (read-write) rather than >: opening a fifo write-only blocks until a
    # reader shows up, and the reader here is the child started below.
    exec {KEEPER_FD}<>"$KEEPER_FIFO"
    TERM=xterm \
    script -qec "$ISAAC_SCRIPTS_DIR/run_dev.sh -d $WORKSPACE_DIR -b" /dev/null \
        <"$KEEPER_FIFO" >"$KEEPER_LOG" 2>&1 &
    KEEPER_PID=$!

    for _ in $(seq 60); do
        container_running && break
        if ! kill -0 "$KEEPER_PID" 2>/dev/null; then
            echo "ERROR: run_dev.sh exited before the container came up:"
            cat "$KEEPER_LOG"
            exit 1
        fi
        sleep 1
    done

    if ! container_running; then
        echo "ERROR: container $CONTAINER_NAME did not start within 60s:"
        cat "$KEEPER_LOG"
        exit 1
    fi
}

run_vslam() {
    trap cleanup EXIT INT TERM

    setup_host_ros

    # Host half: OAK-D driver (vslam camera config) + rect_camera_info_fixer
    # + slam_bridge. Started first so the oak TF tree exists before cuVSLAM
    # asks for the camera extrinsics.
    echo "== host: camera + rect_camera_info_fixer + slam_bridge"
    ros2 launch slam_bridge vslam.launch.py &
    HOST_PID=$!

    start_container

    # Everything in the container runs as admin (uid 1000): FastDDS SHM
    # segments are per-user, so a root node here would see the host's topics
    # but never receive data from them.
    local tty_flag=()
    [ -t 1 ] && tty_flag=(-t)

    echo "== container: cuVSLAM"
    docker exec -i "${tty_flag[@]}" -u admin --workdir "$CONTAINER_WS" "$CONTAINER_NAME" \
        bash -c "source /opt/ros/humble/setup.bash && \
                 ros2 launch $CONTAINER_WS/src/slam_bridge/launch/cuvslam.launch.py $*"
}

# ------------------------------------------------------------------- main

case "$BACKEND" in
    rtabmap) run_rtabmap "$@" ;;
    vslam)   run_vslam "$@" ;;
    "")      usage; exit 1 ;;
    *)       echo "ERROR: unknown SLAM backend '$BACKEND'"; usage; exit 1 ;;
esac
