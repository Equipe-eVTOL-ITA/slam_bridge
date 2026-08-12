// Republishes the OAK-D's rectified-pair CameraInfo with a single, consistent
// rectified camera model, because depthai_ros_driver does not.
//
// THE BUG THIS EXISTS FOR
// -----------------------
// The driver's /oak/{left,right}/image_rect are rectified ON-DEVICE by
// StereoDepth into ONE common pinhole model. But the CameraInfo it publishes
// alongside them comes from ImageConverter::calibrationToCameraInfo(), which
// simply copies each sensor's RAW (unrectified) intrinsics into K and P - it
// never computes a common rectified model. On this OAK-D Pro at 640x400:
//
//     left  K: fx=399.2785  cx=319.9380  cy=207.5984
//     right K: fx=396.8882  cx=333.3717  cy=206.2037
//
// cuVSLAM takes its intrinsics straight from CameraInfo `k` (see
// visual_slam_impl.cpp / FillIntrinsics), so it built a stereo rig whose two
// halves disagree by 13.4 px in cx. That is a constant disparity bias: a
// feature at true disparity d is triangulated as if it were at d + 13.4 px, so
// EVERY depth collapses toward fx*B/13.4 = 2.2 m no matter how far away it
// really is. Measured on a live pair: a point at a true 3.7 m was being
// reconstructed at 1.4 m. Hence cuVSLAM's trajectory came out ~2.4x too short
// and geometrically warped, while RTAB-Map (which uses the on-device depth,
// not these CameraInfos) was fine.
//
// Measured on hardware to establish what the rectified images really are:
//   - matched features across the rect pair give median dy = 0.00 px and
//     far-field disparity -> 0, i.e. the pair IS a canonical rectified pair
//     sharing one model (per-camera models would give dy = +1.4, dx = -13.4);
//   - re-rectifying the raw frames ourselves with cv2.initUndistortRectifyMap
//     reproduces the driver's image_rect with NCC 0.99 when the new camera
//     matrix is the RIGHT sensor's K, and only 0.73 with the left's.
// So: both rectified images live in the RIGHT sensor's intrinsic model.
//
// WHAT THIS NODE PUBLISHES
// ------------------------
//   K, P  <- the RIGHT camera_info's K (the true shared rectified model)
//   D     <- zeros (the images are already undistorted)
//   R     <- identity, deliberately. cuVSLAM composes the TF pose with
//            inverse(R): base_T_rect = base_T_optical * inverse(R). The OAK
//            URDF's optical frames are already the nominal PARALLEL pair, so
//            feeding it the driver's rectification rotations would inject a
//            spurious ~1.8 deg relative rotation between the two cameras -
//            worth another ~12 px of disparity bias. Identity R + the URDF's
//            parallel frames describes the rectified rig correctly.
//
// Residual known error: the URDF baseline is a nominal 75.0 mm while this
// device's real one is 74.76 mm (logged at startup), so metric scale is ~0.3%
// long. Fix that by correcting the TF, not here - cuVSLAM reads extrinsics
// from TF and ignores P entirely.

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <cmath>
#include <string>

class RectCameraInfoFixer : public rclcpp::Node
{
public:
  RectCameraInfoFixer() : Node("rect_camera_info_fixer")
  {
    declare_parameter<std::string>("left_input_topic", "/oak/left/camera_info");
    declare_parameter<std::string>("right_input_topic", "/oak/right/camera_info");
    declare_parameter<std::string>("left_output_topic", "/vslam/left/camera_info");
    declare_parameter<std::string>("right_output_topic", "/vslam/right/camera_info");

    const std::string left_in = get_parameter("left_input_topic").as_string();
    const std::string right_in = get_parameter("right_input_topic").as_string();
    const std::string left_out = get_parameter("left_output_topic").as_string();
    const std::string right_out = get_parameter("right_output_topic").as_string();

    // cuVSLAM subscribes to camera_info with SENSOR_DATA QoS.
    auto qos = rclcpp::SensorDataQoS(); //possible qos missmatch?

    _left_pub = create_publisher<sensor_msgs::msg::CameraInfo>(left_out, qos);
    _right_pub = create_publisher<sensor_msgs::msg::CameraInfo>(right_out, qos);

    _right_sub = create_subscription<sensor_msgs::msg::CameraInfo>(
            right_in, qos,
            [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
              _reference = *msg;              // the shared rectified model
              publish(msg, _right_pub, false);
            });

    _left_sub = create_subscription<sensor_msgs::msg::CameraInfo>(
            left_in, qos,
            [this](const sensor_msgs::msg::CameraInfo::SharedPtr msg) {
              publish(msg, _left_pub, true);
            });

    RCLCPP_INFO(get_logger(), "rect camera_info fix: %s + %s -> %s + %s",
                left_in.c_str(), right_in.c_str(), left_out.c_str(), right_out.c_str());
  }

private:
  sensor_msgs::msg::CameraInfo _reference;  // k[0] == 0 until the first right msg
  bool _logged = false;
  double _baseline = 0.0;

  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr _left_pub, _right_pub;
  rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr _left_sub, _right_sub;

  void publish(const sensor_msgs::msg::CameraInfo::SharedPtr in,
               const rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr & pub,
               bool is_left)
  {
    if (_reference.k[0] == 0.0) {
      return;  // no right camera_info seen yet
    }

    // The driver puts the stereo Tx on whichever socket it calls "first",
    // which on this driver is the LEFT one - the opposite of the ROS
    // convention. Take the baseline from whichever message actually carries
    // it, so this survives i_reverse_stereo_socket_order flipping.
    if (_baseline == 0.0 && in->p[3] != 0.0 && in->k[0] != 0.0) {
      _baseline = std::fabs(in->p[3]) / in->k[0];
    }

    sensor_msgs::msg::CameraInfo out = *in;
    out.k = _reference.k;
    out.distortion_model = "plumb_bob";
    out.d.assign(5, 0.0);

    // R = identity: the images are already in the common rectified frame and
    // the URDF optical frames are already parallel (see header).
    out.r = {1.0, 0.0, 0.0,
             0.0, 1.0, 0.0,
             0.0, 0.0, 1.0};

    // P = [K | Tx], ROS convention: left is the reference (Tx = 0), the right
    // camera sits at +baseline so its Tx = -fx * baseline.
    out.p = {_reference.k[0], 0.0, _reference.k[2], 0.0,
             0.0, _reference.k[4], _reference.k[5], 0.0,
             0.0, 0.0, 1.0, 0.0};
    if (!is_left) {
      out.p[3] = -_reference.k[0] * _baseline;
    }

    pub->publish(out);
    log_once(in, is_left);
  }

  void log_once(const sensor_msgs::msg::CameraInfo::SharedPtr in, bool is_left)
  {
    if (_logged || !is_left || _baseline == 0.0) {
      return;
    }
    _logged = true;
    // RCLCPP_INFO(get_logger(),
    //             "driver published left K (fx=%.4f cx=%.4f cy=%.4f), overriding with "
    //             "the shared rectified model from right (fx=%.4f cx=%.4f cy=%.4f)",
    //             in->k[0], in->k[2], in->k[5],
    //             _reference.k[0], _reference.k[2], _reference.k[5]);
    // RCLCPP_INFO(get_logger(),
    //             "cx correction = %.2f px (that was the disparity bias); "
    //             "device baseline %.5f m vs URDF/TF nominal 0.075 m",
    //             _reference.k[2] - in->k[2], _baseline);
    if (std::fabs(_reference.k[2] - in->k[2]) < 0.01) {
      RCLCPP_WARN(get_logger(),
                  "left and right K already agree - the driver may have been fixed "
                  "upstream, in which case this node is a no-op");
    }
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RectCameraInfoFixer>());
  rclcpp::shutdown();
  return 0;
}
