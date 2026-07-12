# Checking that PX4 is fusing the visual odometry (and how well)

Ordered checklist — each step has a measurable pass/fail. Run the MAVLink
console commands from QGC (**Analyze Tools → MAVLink Console**) over the USB-C
link while the uXRCE-DDS agent runs over ethernet.

## 0. Required PX4 parameters (one-time setup)

| Param | Value | Why |
|---|---|---|
| `EKF2_EV_CTRL` | `11` (bits 0+1+3: hpos + vpos + yaw) | Which EV measurements EKF2 fuses. Add bit 2 (+4 = 15) to also fuse velocity. |
| `EKF2_HGT_REF` | `3` (Vision) | Use vision as the height reference (instead of baro). |
| `EKF2_EV_DELAY` | start at `50` ms | Camera→EKF latency; tune later from log innovations. |
| `EKF2_GPS_CTRL` | `0` (if flying indoor/GPS-denied) | Don't let a bad GPS fight the vision estimate. |
| `MAV_ODOM_LOM` | `1` (optional) | Streams `ODOMETRY` back over MAVLink so QGC/logs show what PX4 received. |

## 1. Is the data arriving at PX4? (transport layer)

```
uxrce_dds_client status        # MAVLink console: agent connected, num topics
listener vehicle_visual_odometry
```

**Pass:** `listener vehicle_visual_odometry` prints messages at the rate
rgbd_odometry publishes (~20–30 Hz) with sane values (position in meters,
`pose_frame: 1` = NED). If it prints "never published", the bridge/agent
chain is broken — check `ros2 topic hz /fmu/in/vehicle_visual_odometry` on
the companion side.

**Also check timesync is alive** (the bridge now warns if it isn't):

```bash
ros2 topic hz /fmu/out/timesync_status     # should tick at ~1-10 Hz
```

## 2. Is EKF2 accepting/fusing it? (estimator layer)

```
listener estimator_status_flags
```

**Pass:** `cs_ev_pos: True`, `cs_ev_hgt: True` (and `cs_ev_yaw` /
`cs_ev_vel` if enabled). These flags are the definitive "PX4 is fusing
vision" signal. If they stay `False`, look at the `reject_*` and
`fs_*` (fault) flags in the same message.

## 3. How good is the fusion? (quality layer)

PX4 publishes per-source aid status with innovations and test ratios:

```
listener estimator_aid_src_ev_pos
listener estimator_aid_src_ev_yaw
listener estimator_aid_src_ev_vel      # if velocity fusion enabled
```

Measurable quality criteria per message:

- `fused: True` and `time_last_fuse` advancing → samples actually used.
- `test_ratio < 1.0` — above 1.0 the sample is **rejected** (innovation
  gate). Healthy VIO sits at **< 0.3** in steady hover.
- `innovation` [m] — steady-state position innovation should be < 0.1–0.2 m.
  A growing trend = drift or a latency (`EKF2_EV_DELAY`) mismatch.
- `innovation_rejected: True` appearing repeatedly = covariance from the
  bridge too optimistic, or timestamps wrong.

## 4. Closed-loop sanity check (ROS side)

Compare what PX4 estimates against what SLAM says (both ~NED after the
bridge). With the drone in hand, walk a 2 m square and back:

```bash
ros2 topic echo /fmu/out/vehicle_local_position --field x,y,z
ros2 topic echo /odom_local --field pose.pose.position
```

**Pass:** `vehicle_local_position` tracks the walked path and returns to
< 0.2 m of the start. If PX4's estimate lags or overshoots the SLAM path,
tune `EKF2_EV_DELAY` (log analysis below gives the exact value).

QGC check: with vision fused, the position estimate on the map should hold
still indoors (no drift walk) and the "Ready to Fly" / local position valid
status should appear without GPS.

## 5. Log-based tuning (most precise)

1. Record a log (arm, or set `SDLOG_MODE 1` to log from boot).
2. Upload the `.ulg` to https://logs.px4.io (Flight Review).
3. Look at **"Estimator Innovations"** plots: EV position/yaw innovations
   should be zero-mean noise, not sawtooth (sawtooth = timestamp offset →
   adjust `EKF2_EV_DELAY` in 10 ms steps until minimized).
4. `estimator_status.reset_count_*` — resets during flight mean fusion
   dropouts; correlate timestamps with rgbd_odometry "lost" warnings.

## 6. What the bridge itself now reports

- Warns `No timesync from PX4 yet` → agent down or QoS mismatch.
- Warns `SLAM tracking lost - pausing odometry to PX4` → it stops feeding
  EKF2 during RTAB-Map dropouts and increments `reset_counter` on recovery
  (PX4 then treats the next sample as a discontinuity instead of a jump).

Count dropouts per session: `grep -c "tracking lost"` on the node log.
