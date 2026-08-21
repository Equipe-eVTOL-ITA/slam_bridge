#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <algorithm>
#include <vector>
#include <deque>
#include <array>

using namespace std::chrono_literals;

class SlamBridge : public rclcpp::Node
{
public:
  SlamBridge() : Node("slam_bridge")
  {
    // Declare parameters
    declare_parameter<std::string>("slam_odometry_topic", "/slam/odometry");
    declare_parameter<std::string>("px4_odometry_output_topic", "/fmu/in/vehicle_visual_odometry");
    declare_parameter<std::string>("timesync_topic", "/fmu/out/timesync_status");
    declare_parameter<double>("variance_floor", 0.1);
    // Camera position relative to the flight controller, in BODY FLU metres
    // (x forward, y left, z up). Measured: camera sits 15 cm forward of the FC.
    declare_parameter<std::vector<double>>("camera_offset_body", {0.15, 0.0, 0.0});
    // cuVSLAM leaves nav_msgs/Odometry.pose.covariance all zeros and publishes
    // the real per-frame covariance on this separate topic instead. Empty
    // string disables it (RTAB-Map puts its covariance in the odometry msg).
    declare_parameter<std::string>("covariance_topic", "/visual_slam/tracking/vo_pose_covariance");
    declare_parameter<int>("covariance_max_age_ms", 100);

    // Get parameters
    std::string slam_topic = get_parameter("slam_odometry_topic").as_string();
    std::string px4_topic = get_parameter("px4_odometry_output_topic").as_string();
    std::string timesync_topic = get_parameter("timesync_topic").as_string();
    _variance_floor = get_parameter("variance_floor").as_double();

    _cov_topic = get_parameter("covariance_topic").as_string();
    _cov_max_age_us = get_parameter("covariance_max_age_ms").as_int() * 1000;

    const auto off = get_parameter("camera_offset_body").as_double_array();
    if (off.size() != 3) {
      RCLCPP_ERROR(get_logger(), "camera_offset_body needs exactly 3 elements, got %zu - using zero",
                   off.size());
    } else {
      _r_cam_body = Eigen::Vector3f(off[0], off[1], off[2]);
    }
    RCLCPP_INFO(get_logger(), "camera_offset_body (FLU): [%.3f, %.3f, %.3f] m",
                _r_cam_body.x(), _r_cam_body.y(), _r_cam_body.z());

    RCLCPP_INFO(get_logger(), "slam_odometry_topic: %s", slam_topic.c_str());
    RCLCPP_INFO(get_logger(), "px4_odometry_output_topic: %s", px4_topic.c_str());
    RCLCPP_INFO(get_logger(), "timesync_topic: %s", timesync_topic.c_str());
    RCLCPP_INFO(get_logger(), "variance_floor: %.3f", _variance_floor);
    
    _pub = create_publisher<px4_msgs::msg::VehicleOdometry>(px4_topic, 20);

    // Subscribe to SLAM odometry
    _sub = create_subscription<nav_msgs::msg::Odometry>(
            slam_topic, 20,
            std::bind(&SlamBridge::odom_cb, this, std::placeholders::_1));

    // PX4 /fmu/out/* topics are published BEST_EFFORT; a RELIABLE
    // subscription never matches and the offset silently stays 0.
    auto px4_qos = rclcpp::QoS(rclcpp::KeepLast(10)).best_effort();
    _timesync_sub = create_subscription<px4_msgs::msg::TimesyncStatus>(
            timesync_topic, px4_qos,
            std::bind(&SlamBridge::timesync_cb, this, std::placeholders::_1));
    
    if (!_cov_topic.empty()) {
      _cov_sub = create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
              _cov_topic, 20,
              std::bind(&SlamBridge::covariance_cb, this, std::placeholders::_1));
      RCLCPP_INFO(get_logger(), "external covariance: %s (max age %ld ms)",
                  _cov_topic.c_str(), _cov_max_age_us / 1000);
    } else {
      RCLCPP_INFO(get_logger(), "external covariance disabled - using odometry msg covariance");
    }

    RCLCPP_INFO(get_logger(), "SLAM Bridge initialized - converting ENU -> NED with Covariance & Velocity");
  }

private:
  int64_t _timestamp_offset = 0;  // Must be signed!
  bool _timesync_received = false;
  bool _tracking_lost = false;
  uint8_t _reset_counter = 0;
  double _variance_floor = 0.1;
  Eigen::Vector3f _r_cam_body{0.0f, 0.0f, 0.0f};  // camera offset from FC, body FLU

  // Recent external covariances, keyed by header stamp. cuVSLAM stamps the
  // covariance identically to the matching odometry sample (verified: max
  // stamp delta 0 us over a 180 s run) but the two callbacks can fire in
  // either order, so keep a short history rather than just "the latest".
  std::string _cov_topic;
  int64_t _cov_max_age_us = 100000;
  struct CovSample { int64_t stamp_us; std::array<double, 6> var; };
  std::deque<CovSample> _cov_hist;
  static constexpr size_t COV_HIST_MAX = 90;  // 3 s at 30 Hz
  bool _cov_warned_stale = false;

  void timesync_cb(const px4_msgs::msg::TimesyncStatus::SharedPtr msg)
  {
    int64_t ros_now_us = this->get_clock()->now().nanoseconds() / 1000;
    _timestamp_offset = (int64_t)msg->timestamp - ros_now_us;
    _timesync_received = true;
    RCLCPP_DEBUG(get_logger(), "Updated timestamp offset: %ld us", _timestamp_offset);
  }

  double apply_variance_floor(double variance)
  {
    return std::max(variance, _variance_floor * _variance_floor); // Apply floor squared for variance
  }

  void covariance_cb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
  {
    CovSample s;
    s.stamp_us = (int64_t)msg->header.stamp.sec * 1000000 + msg->header.stamp.nanosec / 1000;
    // Diagonal only: VehicleOdometry has no off-diagonal fields. (Measured
    // max |off-diagonal| 0.045 on real data - discarding it is a small
    // optimism, which the variance floor absorbs.)
    for (int i = 0; i < 6; ++i) {
      const double v = msg->pose.covariance[i * 6 + i];
      // A negative or non-finite variance would poison EKF2; drop the sample.
      if (!std::isfinite(v) || v < 0.0) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
          "discarding covariance sample with bad diagonal [%d] = %f", i, v);
        return;
      }
      s.var[i] = v;
    }
    _cov_hist.push_back(s);
    while (_cov_hist.size() > COV_HIST_MAX) {
      _cov_hist.pop_front();
    }
  }

  // Find the covariance whose stamp best matches this odometry sample.
  // Returns false if nothing is close enough, so the caller can fall back.
  bool lookup_covariance(int64_t stamp_us, std::array<double, 6> & out)
  {
    int64_t best_dt = _cov_max_age_us + 1;
    const CovSample * best = nullptr;
    for (const auto & c : _cov_hist) {
      const int64_t dt = std::llabs(c.stamp_us - stamp_us);
      if (dt < best_dt) {
        best_dt = dt;
        best = &c;
      }
    }
    if (!best || best_dt > _cov_max_age_us) {
      return false;
    }
    out = best->var;
    return true;
  }

  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // RTAB-Map signals lost tracking with a huge covariance (9999).
    // Feeding that pose to the EKF would inject garbage; skip instead,
    // and bump reset_counter on recovery so PX4 knows the odometry
    // frame may have jumped (Odom/ResetCountdown re-zeroes the pose).
    if (msg->pose.covariance[0] >= 9998.0) {
      if (!_tracking_lost) {
        RCLCPP_WARN(get_logger(), "SLAM tracking lost - pausing odometry to PX4");
        _tracking_lost = true;
      }
      return;
    }
    if (_tracking_lost) {
      _tracking_lost = false;
      _reset_counter++;
      RCLCPP_WARN(get_logger(), "SLAM tracking recovered - reset_counter=%u", _reset_counter);
    }

    if (!_timesync_received) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 5000,
        "No timesync from PX4 yet (%s) - is the uXRCE-DDS agent running?",
        "/fmu/out/timesync_status");
    }

    px4_msgs::msg::VehicleOdometry px4_odom;

    // 1. Timestamp: Convert ROS time to PX4 time using offset.
    // The uXRCE-DDS client does NOT fill timestamp==0; set both fields.
    uint64_t ros_time_us = msg->header.stamp.sec * 1000000ULL + msg->header.stamp.nanosec / 1000ULL;
    px4_odom.timestamp_sample = ros_time_us + _timestamp_offset;
    px4_odom.timestamp = (uint64_t)(get_clock()->now().nanoseconds() / 1000 + _timestamp_offset);

    /* ==============================================
       POSITION & ORIENTATION (ENU/FLU -> NED/FRD)
       ============================================== */
    // POSE_FRAME_FRD, not NED: the SLAM world frame is gravity-aligned but its
    // heading origin is wherever the camera pointed at init, which is NOT
    // North. PX4 v1.15 ev_pos_control.cpp handles that explicitly - with
    // ev_yaw fusion off it rotates our position into the EKF frame via
    // _ev_q_error_filt (an alpha filter on q_state * q_ev^-1), so it
    // self-calibrates the frame offset instead of believing a false North.
    px4_odom.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_FRD;

    // Orientation first: the lever-arm correction below needs it.
    // q is body(FLU)-in-ENU; PX4 wants body(FRD)-in-FRD-world.
    // Both the world frame AND the body frame must be converted:
    //   q_out = q_ned<-enu * q_enu_flu * q_flu<-frd
    const auto &q_in = msg->pose.pose.orientation;
    Eigen::Quaternionf q_enu_flu(q_in.w, q_in.x, q_in.y, q_in.z);
    q_enu_flu.normalize();
    static const Eigen::Quaternionf Q_NED_ENU(0.0f, 0.7071068f, 0.7071068f, 0.0f);
    static const Eigen::Quaternionf Q_FLU_FRD(0.0f, 1.0f, 0.0f, 0.0f);
    Eigen::Quaternionf q_ned = (Q_NED_ENU * q_enu_flu * Q_FLU_FRD).normalized();

    px4_odom.q[0] = q_ned.w();
    px4_odom.q[1] = q_ned.x();
    px4_odom.q[2] = q_ned.y();
    px4_odom.q[3] = q_ned.z();

    /* ---------------- LEVER ARM (camera -> flight controller) -------------
       The SLAM backend tracks the CAMERA (cuVSLAM base_frame 'oak', so the
       odometry carries child_frame_id 'oak'), but EKF2 needs the pose of the
       FC. With the camera at offset r from the FC, any vehicle rotation
       sweeps the camera through an arc:

           p_fc = p_cam - R(q_world<-body) * r

       Without this, yawing in place fabricates |r| of translation that the
       position controller then chases - a self-exciting oscillation that is
       invisible handheld (no closed loop, slow rotation) and continuously
       excited in flight. r = 15 cm forward means a 90 deg yaw invents ~21 cm
       of motion that never happened.

       Done here in ENU/FLU, before the axis swap, so the rotation and the
       offset are in the same frame convention.

       NOTE: PX4 can do this itself via EKF2_EV_POS_X/Y/Z (ev_pos_control.cpp
       subtracts _R_to_earth * (ev_pos_body - imu_pos_body)). Those params are
       currently 0 - LEAVE THEM AT 0, or the offset gets applied twice.
       --------------------------------------------------------------------- */
    Eigen::Vector3f p_cam_enu(msg->pose.pose.position.x,
                              msg->pose.pose.position.y,
                              msg->pose.pose.position.z);
    Eigen::Vector3f p_fc_enu = p_cam_enu - q_enu_flu * _r_cam_body;

    // Position: ENU (X, Y, Z) -> FRD world (Y, X, -Z)
    px4_odom.position[0] =  p_fc_enu.y();  // Forward/North-ish
    px4_odom.position[1] =  p_fc_enu.x();  // Right/East-ish
    px4_odom.position[2] = -p_fc_enu.z();  // Down

    /* ==============================================
       VELOCITY (body FLU -> body FRD)
       ============================================== */
    // nav_msgs/Odometry twist is expressed in child_frame_id (the body
    // frame), NOT in ENU. Converting it with the world ENU->NED swap is
    // wrong; the correct body-frame conversion is FLU -> FRD (x, -y, -z).
    px4_odom.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD;

    // Same lever arm, differentiated: the camera's velocity includes the
    // tangential term from the vehicle's rotation, so
    //     v_fc = v_cam - omega x r
    // Angular velocity is unchanged (rigid body, same omega everywhere).
    // For r = (0.15, 0, 0): omega x r = (0, wz*0.15, -wy*0.15), i.e. a 1 rad/s
    // yaw rate alone shows up as 15 cm/s of phantom sideways velocity.
    Eigen::Vector3f v_cam_flu(msg->twist.twist.linear.x,
                              msg->twist.twist.linear.y,
                              msg->twist.twist.linear.z);
    Eigen::Vector3f w_flu(msg->twist.twist.angular.x,
                          msg->twist.twist.angular.y,
                          msg->twist.twist.angular.z);
    Eigen::Vector3f v_fc_flu = v_cam_flu - w_flu.cross(_r_cam_body);

    // body FLU -> body FRD (x, -y, -z)
    px4_odom.velocity[0] =  v_fc_flu.x();
    px4_odom.velocity[1] = -v_fc_flu.y();
    px4_odom.velocity[2] = -v_fc_flu.z();

    // Angular velocity is always body FRD in VehicleOdometry
    px4_odom.angular_velocity[0] =  w_flu.x();
    px4_odom.angular_velocity[1] = -w_flu.y();
    px4_odom.angular_velocity[2] = -w_flu.z();

    /* ==============================================
       COVARIANCE / UNCERTAINTY
       ==============================================
       Source of truth, in order of preference:
         1. the external covariance topic (cuVSLAM's real per-frame estimate)
         2. the odometry message's own covariance (RTAB-Map fills this in)
       cuVSLAM leaves nav_msgs/Odometry.pose.covariance ALL ZEROS, so on that
       backend option 2 collapses to the variance floor and EKF2 is told a
       flat "sigma = variance_floor" no matter how well tracking is going.
       Measured on a 180 s handheld run, the real values span:
         position stddev  p50 [0.078, 0.113, 0.161] m  ->  p99 [0.219, 0.244, 0.280] m
         worst-case variance 2.80 m^2 (sigma = 1.67 m)
       i.e. the flat 0.1 floor was simultaneously too pessimistic in x and too
       OPTIMISTIC in z and during every tracking wobble. Passing the real
       numbers is what lets EKF2's innovation gate do its job.

       ROS covariance is a 6x6 row-major [x y z rx ry rz]; index (i,i) = i*6+i,
       so the diagonal is 0, 7, 14, 21, 28, 35.
       ============================================== */
    std::array<double, 6> cov = {
      msg->pose.covariance[0],  msg->pose.covariance[7],  msg->pose.covariance[14],
      msg->pose.covariance[21], msg->pose.covariance[28], msg->pose.covariance[35]
    };

    if (!_cov_topic.empty()) {
      std::array<double, 6> ext;
      if (lookup_covariance((int64_t)ros_time_us, ext)) {
        cov = ext;
        _cov_warned_stale = false;
      } else if (!_cov_warned_stale) {
        _cov_warned_stale = true;
        RCLCPP_WARN(get_logger(),
          "no covariance within %ld ms of the odometry stamp on %s - falling back "
          "to the odometry message covariance (all zeros on cuVSLAM, so the "
          "variance floor is all EKF2 will see)",
          _cov_max_age_us / 1000, _cov_topic.c_str());
      }
    }

    // Position: world ENU X/Y/Z -> N/E/D, so swap X and Y.
    px4_odom.position_variance[0] = apply_variance_floor(cov[1]);  // North <- ENU Y
    px4_odom.position_variance[1] = apply_variance_floor(cov[0]);  // East  <- ENU X
    px4_odom.position_variance[2] = apply_variance_floor(cov[2]);  // Down

    // Orientation: the pose covariance is expressed in the WORLD frame, so
    // cov[3..5] are rotations about ENU X (East), Y (North), Z (Up). PX4 wants
    // [roll, pitch, yaw] about NED/FRD axes - roll is about North and pitch
    // about East, so these swap exactly like the position block above.
    // (The previous mapping passed them straight through, which had rx and ry
    // crossed.) Sign flips do not change a variance, so only the swap matters.
    px4_odom.orientation_variance[0] = apply_variance_floor(cov[4]);  // roll  <- about ENU Y
    px4_odom.orientation_variance[1] = apply_variance_floor(cov[3]);  // pitch <- about ENU X
    px4_odom.orientation_variance[2] = apply_variance_floor(cov[5]);  // yaw

    // Velocity: vo_pose_covariance is POSE-only (6x6, no velocity block), so
    // this still comes from the odometry twist - which cuVSLAM also leaves at
    // zero, i.e. the floor. Harmless while EKF2_EV_CTRL bit 2 is clear and
    // velocity is not fused; revisit if it is ever enabled.
    // Body-frame axis flips do not change a variance, so map directly.
    px4_odom.velocity_variance[0] = apply_variance_floor(msg->twist.covariance[0]);
    px4_odom.velocity_variance[1] = apply_variance_floor(msg->twist.covariance[7]);
    px4_odom.velocity_variance[2] = apply_variance_floor(msg->twist.covariance[14]);

    // Loud when the estimate degrades badly - this is the signal that used to
    // be invisible because everything was pinned at the floor.
    const double worst = std::max({cov[0], cov[1], cov[2]});
    if (worst > 1.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
        "VO position uncertainty high: sigma = %.2f m - EKF2 should be "
        "down-weighting or gating these samples", std::sqrt(worst));
    }

    px4_odom.reset_counter = _reset_counter;
    px4_odom.quality = 0;  // unknown

    _pub->publish(px4_odom);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _sub;
  rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr _timesync_sub;
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr _cov_sub;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr _pub;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SlamBridge>());
  rclcpp::shutdown();
  return 0;
}