#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_ros2/navigation/experimental/local_position_measurement_interface.hpp>  // library API
#include <Eigen/Eigen>

using namespace std::chrono_literals;

class SlamBridge : public rclcpp::Node
{
public:
  SlamBridge() : Node("sai_px4_bridge")
  {
    using px4_ros2::PoseFrame;
    using px4_ros2::VelocityFrame;
    _lp_iface = std::make_shared<px4_ros2::LocalPositionMeasurementInterface>(
                  *this, PoseFrame::LocalNED, VelocityFrame::Unknown);

    _sub = create_subscription<geometry_msgs::msg::PoseStamped>(
            "/slam/odometry", 20,
            std::bind(&SlamBridge::odom_cb, this, std::placeholders::_1));
  }

private:
void odom_cb(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
  /* ---------- ENU → NED -------------- */
  Eigen::Vector3f enu_pos(msg->pose.position.x,
                          msg->pose.position.y,
                          msg->pose.position.z);
  Eigen::Vector3f ned_pos( enu_pos.y(),          // N
                           enu_pos.x(),          // E
                          -enu_pos.z());         // D

  const auto &q_enu = msg->pose.orientation;
  Eigen::Quaternionf q_enu_ros(q_enu.w, q_enu.x, q_enu.y, q_enu.z);

  /* Rotate 180° about X to change handedness ENU→NED */
  const Eigen::Quaternionf R_ENU_NED(0, 1, 0, 0);
  Eigen::Quaternionf q_ned = R_ENU_NED * q_enu_ros;

  /* ---------- LocalPositionMeasurement -------------- */
  px4_ros2::LocalPositionMeasurement meas{};
  meas.timestamp_sample = msg->header.stamp;

  meas.position_xy         = Eigen::Vector2f(ned_pos.x(), ned_pos.y());
  meas.position_xy_variance= Eigen::Vector2f::Constant(0.05f*0.05f);   // 5 cm ²

  meas.position_z          = ned_pos.z();
  meas.position_z_variance = 0.05f*0.05f;

  meas.attitude_quaternion = q_ned;
  meas.attitude_variance   = Eigen::Vector3f::Constant(
                                powf(3.0f*M_PI/180.0f, 2));            // (3 deg)²

  try {
     _lp_iface->update(meas);
  } catch (const std::exception &e) {
     RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000,
                           "LPI update failed: %s", e.what());
  }
}

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr _sub;
  std::shared_ptr<px4_ros2::LocalPositionMeasurementInterface> _lp_iface;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SlamBridge>());
  rclcpp::shutdown();
  return 0;
}
