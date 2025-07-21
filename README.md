# SLAM Bridge - VIO on PX4 1.15

A ROS 2 node that bridges SLAM odometry data from SpectacularAI to PX4 autopilot v1.15 for position estimation.

## Overview

This node subscribes to SLAM pose data and forwards it to PX4 using the LocalPositionMeasurementInterface. It performs coordinate frame transformation from ENU (East-North-Up) to NED (North-East-Down) as required by PX4.

## Topics

**Subscribed:**
- `/slam/odometry` (geometry_msgs/PoseStamped) - SLAM pose estimates in ENU frame

## Features

- ENU to NED coordinate transformation
- Position and attitude variance configuration
- Error handling with throttled logging
- 5cm position accuracy, 3° attitude accuracy assumptions

## Usage

```bash
ros2 run slam_bridge slam_bridge
```

## Dependencies

- ROS 2
- px4_ros2
- Eigen3
- geometry_msgs