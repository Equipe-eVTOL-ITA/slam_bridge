# SLAM Bridge - VIO on PX4 1.15

A ROS 2 (C++) node plus launch files that feed visual/RGBD odometry to PX4's
EKF2 as vision-based position estimation, over `/fmu/in/vehicle_visual_odometry`.
Two SLAM backends are supported on an OAK-D Pro camera:

- **RTAB-Map** (`rtabmap_odom` + `rtabmap_slam`) — RGB-D visual odometry, runs
  entirely in plain ROS 2 (host or PC).
- **cuVSLAM** (NVIDIA Isaac ROS `isaac_ros_visual_slam`) — stereo VIO, runs
  inside the Isaac ROS container (see `docker/Dockerfile.evtol`).

Full pipeline status, run commands, and hardware gotchas: `PX4_FUSION_CHECK.md`
(verifying PX4 is actually fusing the data) and `ISAAC_ROS_VSLAM_OAKD_PLAN.md`
at the workspace root (cuVSLAM setup/validation notes).

## The `slam_bridge` node

Subscribes to a `nav_msgs/Odometry` topic from whichever SLAM backend is
running and republishes it to PX4 as `px4_msgs/VehicleOdometry`.

**Subscribed:**
- SLAM odometry topic (`nav_msgs/msg/Odometry`), e.g. `/odom_local` (RTAB-Map)
  or `/visual_slam/tracking/odometry` (cuVSLAM) — configurable via param.
- `/fmu/out/timesync_status` (`px4_msgs/msg/TimesyncStatus`, BEST_EFFORT) —
  used to convert ROS timestamps into PX4's clock.

**Published:**
- `/fmu/in/vehicle_visual_odometry` (`px4_msgs/msg/VehicleOdometry`)

**Parameters:**
| Param | Default | Purpose |
|---|---|---|
| `slam_odometry_topic` | `/slam/odometry` | Input odometry topic |
| `px4_odometry_output_topic` | `/fmu/in/vehicle_visual_odometry` | Output topic to PX4 |
| `timesync_topic` | `/fmu/out/timesync_status` | PX4 timesync source |
| `variance_floor` | `0.1` | Minimum stddev (m or rad) enforced on published covariances |

**What it does:**
- Converts pose from ENU/FLU (ROS) to NED/FRD (PX4), including the
  world-frame *and* body-frame quaternion rotation.
- Converts body-frame twist from FLU to FRD (velocity is expressed in the
  body frame, not world ENU, so it's a different rotation than position).
- Applies `_timestamp_offset` from timesync so PX4 receives its own clock's
  timestamps, not ROS time.
- Detects RTAB-Map's "lost tracking" signal (covariance diagonal ≥ 9998) and
  pauses forwarding instead of feeding garbage to EKF2, bumping
  `reset_counter` on recovery so PX4 treats the next sample as a
  discontinuity rather than a jump.
- Enforces `variance_floor` on all published covariances so an
  overconfident SLAM estimate can't make EKF2 over-trust it.
- Warns (throttled) if timesync hasn't been received yet, since the offset
  would silently stay 0.

## Launch files

| File | Runs where | Pipeline |
|---|---|---|
| `launch/camera_jetson.launch.py` | Jetson | OAK-D Pro driver only, rectified stereo + IMU for cuVSLAM. |
| `launch/rtabmap.launch.py` | Jetson (all-in-one) | Camera + IMU filter + RTAB-Map odometry/SLAM + `slam_bridge`, all on one machine. |
| `launch/vslam.launch.py` | Jetson | `camera_jetson.launch.py` + `slam_bridge` pointed at cuVSLAM's odometry topic. cuVSLAM itself must be started separately inside the Isaac ROS container. |
| `launch/cuvslam.launch.py` | Inside Isaac ROS container | The `isaac_ros_visual_slam` composable node. Not built by `colcon` — launch by absolute path (see file header). `enable_imu_fusion:=true` to opt into the (uncalibrated) BNO086 IMU. |

Splitting the camera (Jetson) from the SLAM backend (a separate PC over the
network) was tried for RTAB-Map and dropped — raw RGB+depth is too heavy for
the link. RTAB-Map now always runs all-in-one via `rtabmap.launch.py`. The
Jetson/container split for cuVSLAM is different (shared DDS via
`--network host`, not a network hop) and still requires: matching
`ROS_DOMAIN_ID`, the same RMW implementation everywhere, and synced clocks
(chrony/NTP) — see comments in `camera_jetson.launch.py`.

## Config

`config/camera_params.yaml` / `camera_params_vslam.yaml` — OAK-D Pro driver
params per backend (RGB+depth vs. rectified stereo+IMU).
`config/rtabmap_odometry_params.yaml` / `rtabmap_slam_params.yaml` — RTAB-Map
tuning.

## Isaac ROS container setup (cuVSLAM only)

`isaac_ros_visual_slam` only exists inside the Isaac ROS dev container, and
`run_dev.sh` uses `docker run --rm`, so anything apt-installed by hand is
lost when the container stops. `docker/Dockerfile.evtol` bakes the package
into a custom image layer instead. Since `isaac_ros_common`'s
`.isaac_ros_common-config` is gitignored upstream, wire it up once per
machine (or after re-cloning `isaac_ros_common`):

```bash
bash src/slam_bridge/scripts/setup_isaac_container.sh          # write config
bash src/slam_bridge/scripts/setup_isaac_container.sh --build  # + build image
```

## Usage

```bash
ros2 launch slam_bridge rtabmap.launch.py     # RTAB-Map, all-in-one on Jetson
ros2 launch slam_bridge vslam.launch.py       # cuVSLAM camera+bridge half (Jetson)
# then, inside the Isaac ROS container:
ros2 launch /workspaces/isaac_ros-dev/src/slam_bridge/launch/cuvslam.launch.py

ros2 run slam_bridge slam_bridge --ros-args -p slam_odometry_topic:=/odom_local
```

To verify PX4 is actually fusing the published odometry, follow the
checklist in `PX4_FUSION_CHECK.md`.

## Dependencies

- ROS 2 (rclcpp, geometry_msgs, nav_msgs, px4_msgs, tf2_ros, launch_ros)
- Eigen3
- `depthai_ros_driver` (OAK-D Pro)
- `imu_filter_madgwick`
- `rtabmap_odom`, `rtabmap_slam`
- `isaac_ros_visual_slam` (cuVSLAM backend only, container-side)
