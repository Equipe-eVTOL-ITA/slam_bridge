#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <px4_msgs/msg/timesync_status.hpp>
#include <eigen3/Eigen/Dense>
#include <cmath>
#include <algorithm>

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
    
    // Get parameters
    std::string slam_topic = get_parameter("slam_odometry_topic").as_string();
    std::string px4_topic = get_parameter("px4_odometry_output_topic").as_string();
    std::string timesync_topic = get_parameter("timesync_topic").as_string();
    _variance_floor = get_parameter("variance_floor").as_double();
    
    RCLCPP_INFO(get_logger(), "slam_odometry_topic: %s", slam_topic.c_str());
    RCLCPP_INFO(get_logger(), "px4_odometry_output_topic: %s", px4_topic.c_str());
    RCLCPP_INFO(get_logger(), "timesync_topic: %s", timesync_topic.c_str());
    RCLCPP_INFO(get_logger(), "variance_floor: %.3f", _variance_floor);
    
    _pub = create_publisher<px4_msgs::msg::VehicleOdometry>(px4_topic, 20);
    
    // Subscribe to SLAM odometry
    _sub = create_subscription<nav_msgs::msg::Odometry>(
            slam_topic, 20,
            std::bind(&SlamBridge::odom_cb, this, std::placeholders::_1));
    
    // Subscribe to timesync for timestamp correction
    _timesync_sub = create_subscription<px4_msgs::msg::TimesyncStatus>(
            timesync_topic, 10,
            std::bind(&SlamBridge::timesync_cb, this, std::placeholders::_1));
    
    RCLCPP_INFO(get_logger(), "SLAM Bridge initialized - converting ENU -> NED with Covariance & Velocity");
  }

private:
  int64_t _timestamp_offset = 0;  // Must be signed!
  double _variance_floor = 0.1;

  void timesync_cb(const px4_msgs::msg::TimesyncStatus::SharedPtr msg)
  {
    int64_t ros_now_us = this->get_clock()->now().nanoseconds() / 1000;
    _timestamp_offset = (int64_t)msg->timestamp - ros_now_us;
    RCLCPP_DEBUG(get_logger(), "Updated timestamp offset: %ld us", _timestamp_offset);
  }

  double apply_variance_floor(double variance)
  {
    return std::max(variance, _variance_floor * _variance_floor); // Apply floor squared for variance
  }

  void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    px4_msgs::msg::VehicleOdometry px4_odom;

    // 1. Timestamp: Convert ROS time to PX4 time using offset
    uint64_t ros_time_us = msg->header.stamp.sec * 1000000ULL + msg->header.stamp.nanosec / 1000ULL;
    px4_odom.timestamp_sample = ros_time_us + _timestamp_offset;
    px4_odom.timestamp = 0;  // Server will fill this
    
    /* ==============================================
       POSITION & ORIENTATION (ENU -> NED)
       ============================================== */
    px4_odom.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;

    // Position: ENU (X, Y, Z) -> NED (Y, X, -Z)
    px4_odom.position[0] =  msg->pose.pose.position.y;  // North
    px4_odom.position[1] =  msg->pose.pose.position.x;  // East
    px4_odom.position[2] = -msg->pose.pose.position.z;  // Down

    // Orientation: Rotate ENU Quaternion to NED
    const auto &q_enu = msg->pose.pose.orientation;
    Eigen::Quaternionf q_enu_ros(q_enu.w, q_enu.x, q_enu.y, q_enu.z);
    const Eigen::Quaternionf R_ENU_NED(0.0f, 0.7071068f, 0.7071068f, 0.0f);
    Eigen::Quaternionf q_ned = R_ENU_NED * q_enu_ros;

    px4_odom.q[0] = q_ned.w();
    px4_odom.q[1] = q_ned.x();
    px4_odom.q[2] = q_ned.y();
    px4_odom.q[3] = q_ned.z();

    /* ==============================================
       VELOCITY (ENU -> NED)
       ============================================== */
    px4_odom.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;

    // Linear Velocity: ENU -> NED
    px4_odom.velocity[0] =  msg->twist.twist.linear.y;
    px4_odom.velocity[1] =  -msg->twist.twist.linear.x;
    px4_odom.velocity[2] = -msg->twist.twist.linear.z;

    // Angular Velocity (Roll, Pitch, Yaw rates): ENU -> NED
    px4_odom.angular_velocity[0] =  msg->twist.twist.angular.y;
    px4_odom.angular_velocity[1] =  -msg->twist.twist.angular.x;
    px4_odom.angular_velocity[2] = -msg->twist.twist.angular.z;

    /* ==============================================
       COVARIANCE / UNCERTAINTY (ENU -> NED)
       ============================================== */
    // ROS covariance is a 36-element array (6x6 matrix).
    // The diagonals [0], [7], [14] represent X, Y, Z variances.
    
    // Position Variances (Mapping ENU X/Y/Z to NED N/E/D)
    float pos_var_x_enu = msg->pose.covariance[0];
    float pos_var_y_enu = msg->pose.covariance[7];
    float pos_var_z_enu = msg->pose.covariance[14];
    
    px4_odom.position_variance[0] = apply_variance_floor(pos_var_y_enu); // North
    px4_odom.position_variance[1] = apply_variance_floor(pos_var_x_enu); // East
    px4_odom.position_variance[2] = apply_variance_floor(pos_var_z_enu); // Down

    // Velocity Variances
    float vel_var_x_enu = msg->twist.covariance[0];
    float vel_var_y_enu = msg->twist.covariance[7];
    float vel_var_z_enu = msg->twist.covariance[14];

    px4_odom.velocity_variance[0] = apply_variance_floor(vel_var_y_enu);
    px4_odom.velocity_variance[1] = apply_variance_floor(vel_var_x_enu);
    px4_odom.velocity_variance[2] = apply_variance_floor(vel_var_z_enu);

    // Orientation Variances (Roll, Pitch, Yaw)
    float rot_var_roll_enu  = msg->pose.covariance[21];
    float rot_var_pitch_enu = msg->pose.covariance[28];
    float rot_var_yaw_enu   = msg->pose.covariance[35];

    px4_odom.orientation_variance[0] = apply_variance_floor(rot_var_pitch_enu);
    px4_odom.orientation_variance[1] = apply_variance_floor(rot_var_roll_enu);
    px4_odom.orientation_variance[2] = apply_variance_floor(rot_var_yaw_enu);

    _pub->publish(px4_odom);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr _sub;
  rclcpp::Subscription<px4_msgs::msg::TimesyncStatus>::SharedPtr _timesync_sub;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr _pub;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SlamBridge>());
  rclcpp::shutdown();
  return 0;
}