#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <limits>

class OdomToVisualOdom : public rclcpp::Node
{
public:
  OdomToVisualOdom() : Node("odom_to_visual_odom")
  {
    // Declare parameters
    this->declare_parameter<bool>("use_vrpn", false);
    this->declare_parameter<std::string>("odom_topic", "/d2vins/odometry");
    this->declare_parameter<std::string>("vrpn_topic", "/vrpn_client_node/pose");
    this->declare_parameter<std::string>("visual_odom_topic", "fmu/in/vehicle_visual_odometry");
    this->declare_parameter<std::string>("input_world_frame", "ENU");
    this->declare_parameter<std::string>("output_world_frame", "NED");
    this->declare_parameter<std::string>("input_body_frame", "FLU");
    this->declare_parameter<std::string>("output_body_frame", "FRD");

    // Get parameters
    use_vrpn_ = this->get_parameter("use_vrpn").as_bool();
    auto odom_topic = this->get_parameter("odom_topic").as_string();
    auto vrpn_topic = this->get_parameter("vrpn_topic").as_string();
    auto visual_odom_topic = this->get_parameter("visual_odom_topic").as_string();
    input_world_frame_ = this->get_parameter("input_world_frame").as_string();
    output_world_frame_ = this->get_parameter("output_world_frame").as_string();
    input_body_frame_ = this->get_parameter("input_body_frame").as_string();
    output_body_frame_ = this->get_parameter("output_body_frame").as_string();

    RCLCPP_INFO(this->get_logger(), "=== OdomToVisualOdom Configuration ===");
    RCLCPP_INFO(this->get_logger(), "use_vrpn: %s", use_vrpn_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "World frame: %s -> %s",
                input_world_frame_.c_str(), output_world_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "Body frame: %s -> %s",
                input_body_frame_.c_str(), output_body_frame_.c_str());

    // Validate frame types
    std::vector<std::string> valid_frames = {"ENU", "FLU", "NED", "FRD"};
    auto validate = [&](const std::string& frame, const std::string& name) {
      if (std::find(valid_frames.begin(), valid_frames.end(), frame) == valid_frames.end()) {
        RCLCPP_ERROR(this->get_logger(), "Invalid %s: %s", name.c_str(), frame.c_str());
        throw std::runtime_error("Invalid frame: " + name);
      }
    };
    validate(input_world_frame_, "input_world_frame");
    validate(output_world_frame_, "output_world_frame");
    validate(input_body_frame_, "input_body_frame");
    validate(output_body_frame_, "output_body_frame");

    // Calculate transformation matrices and quaternions
    R_world_transform_ = getTransformationMatrix(input_world_frame_, output_world_frame_);
    q_world_transform_ = Eigen::Quaterniond(R_world_transform_);

    R_body_transform_ = getTransformationMatrix(input_body_frame_, output_body_frame_);
    q_body_transform_ = Eigen::Quaterniond(R_body_transform_);

    RCLCPP_INFO(this->get_logger(), "World transformation matrix (%s -> %s):",
                input_world_frame_.c_str(), output_world_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "[%.2f, %.2f, %.2f]",
                R_world_transform_(0,0), R_world_transform_(0,1), R_world_transform_(0,2));
    RCLCPP_INFO(this->get_logger(), "[%.2f, %.2f, %.2f]",
                R_world_transform_(1,0), R_world_transform_(1,1), R_world_transform_(1,2));
    RCLCPP_INFO(this->get_logger(), "[%.2f, %.2f, %.2f]",
                R_world_transform_(2,0), R_world_transform_(2,1), R_world_transform_(2,2));

    RCLCPP_INFO(this->get_logger(), "Body transformation matrix (%s -> %s):",
                input_body_frame_.c_str(), output_body_frame_.c_str());
    RCLCPP_INFO(this->get_logger(), "[%.2f, %.2f, %.2f]",
                R_body_transform_(0,0), R_body_transform_(0,1), R_body_transform_(0,2));
    RCLCPP_INFO(this->get_logger(), "[%.2f, %.2f, %.2f]",
                R_body_transform_(1,0), R_body_transform_(1,1), R_body_transform_(1,2));
    RCLCPP_INFO(this->get_logger(), "[%.2f, %.2f, %.2f]",
                R_body_transform_(2,0), R_body_transform_(2,1), R_body_transform_(2,2));

    // QoS profile compatible with PX4 micro-XRCE-DDS bridge
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, qos,
      std::bind(&OdomToVisualOdom::odom_callback, this, std::placeholders::_1));

    vrpn_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      vrpn_topic, qos,
      std::bind(&OdomToVisualOdom::vrpn_callback, this, std::placeholders::_1));

    visual_odom_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>(visual_odom_topic, 10);

    RCLCPP_INFO(this->get_logger(), "=== OdomToVisualOdom Ready ===");
  }

private:
  Eigen::Matrix3d getTransformationMatrix(const std::string& from_frame,
                                          const std::string& to_frame)
  {
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity();

    if (from_frame == to_frame) return R;

    if (from_frame == "ENU" && to_frame == "NED") {
      R << 0,  1,  0,
           1,  0,  0,
           0,  0, -1;
    } else if (from_frame == "NED" && to_frame == "ENU") {
      R << 0,  1,  0,
           1,  0,  0,
           0,  0, -1;
    } else if (from_frame == "FLU" && to_frame == "FRD") {
      R << 1,  0,  0,
           0, -1,  0,
           0,  0, -1;
    } else if (from_frame == "FRD" && to_frame == "FLU") {
      R << 1,  0,  0,
           0, -1,  0,
           0,  0, -1;
    } else if (from_frame == "FLU" && to_frame == "NED") {
      R << 1,  0,  0,
           0, -1,  0,
           0,  0, -1;
    } else if (from_frame == "NED" && to_frame == "FLU") {
      R << 1,  0,  0,
           0, -1,  0,
           0,  0, -1;
    } else if (from_frame == "ENU" && to_frame == "FRD") {
      Eigen::Matrix3d R_enu_to_ned;
      R_enu_to_ned << 0,  1,  0,
                      1,  0,  0,
                      0,  0, -1;
      Eigen::Matrix3d R_ned_to_frd = Eigen::Matrix3d::Identity();
      R = R_ned_to_frd * R_enu_to_ned;
    } else if (from_frame == "FRD" && to_frame == "ENU") {
      R = getTransformationMatrix("ENU", "FRD").transpose();
    } else if (from_frame == "ENU" && to_frame == "FLU") {
      Eigen::Matrix3d R_enu_to_ned = getTransformationMatrix("ENU", "NED");
      Eigen::Matrix3d R_ned_to_flu = getTransformationMatrix("NED", "FLU");
      R = R_ned_to_flu * R_enu_to_ned;
    } else if (from_frame == "FLU" && to_frame == "ENU") {
      R = getTransformationMatrix("ENU", "FLU").transpose();
    } else {
      RCLCPP_WARN(this->get_logger(),
                  "Unknown frame pair: %s -> %s, using identity",
                  from_frame.c_str(), to_frame.c_str());
    }

    return R;
  }

  void convertPose(const geometry_msgs::msg::Pose& pose_in,
                   geometry_msgs::msg::Pose& pose_out)
  {
    // Position: p_out = R_world * p_in
    Eigen::Vector3d pos_in(pose_in.position.x, pose_in.position.y, pose_in.position.z);
    Eigen::Vector3d pos_out = R_world_transform_ * pos_in;

    pose_out.position.x = pos_out.x();
    pose_out.position.y = pos_out.y();
    pose_out.position.z = pos_out.z();

    // Orientation: q_out = q_world * q_in * q_body
    Eigen::Quaterniond q_in(pose_in.orientation.w,
                            pose_in.orientation.x,
                            pose_in.orientation.y,
                            pose_in.orientation.z);

    Eigen::Quaterniond q_out = q_world_transform_ * q_in * q_body_transform_;
    q_out.normalize();

    pose_out.orientation.w = q_out.w();
    pose_out.orientation.x = q_out.x();
    pose_out.orientation.y = q_out.y();
    pose_out.orientation.z = q_out.z();
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (use_vrpn_) return;

    px4_msgs::msg::VehicleOdometry visual_odom_msg;

    visual_odom_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    visual_odom_msg.timestamp_sample = msg->header.stamp.sec * 1000000ULL + msg->header.stamp.nanosec / 1000;
    visual_odom_msg.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;

    // Convert pose (position + orientation)
    geometry_msgs::msg::Pose pose_out;
    convertPose(msg->pose.pose, pose_out);

    visual_odom_msg.position = {static_cast<float>(pose_out.position.x),
                                static_cast<float>(pose_out.position.y),
                                static_cast<float>(pose_out.position.z)};

    visual_odom_msg.q = {static_cast<float>(pose_out.orientation.w),
                         static_cast<float>(pose_out.orientation.x),
                         static_cast<float>(pose_out.orientation.y),
                         static_cast<float>(pose_out.orientation.z)};

    // Convert body-frame velocity using body frame transformation
    Eigen::Vector3d vel_in(msg->twist.twist.linear.x,
                           msg->twist.twist.linear.y,
                           msg->twist.twist.linear.z);
    Eigen::Vector3d vel_out = R_body_transform_ * vel_in;

    visual_odom_msg.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_BODY_FRD;
    visual_odom_msg.velocity = {static_cast<float>(vel_out.x()),
                                static_cast<float>(vel_out.y()),
                                static_cast<float>(vel_out.z())};

    // Convert body-frame angular velocity
    Eigen::Vector3d ang_vel_in(msg->twist.twist.angular.x,
                               msg->twist.twist.angular.y,
                               msg->twist.twist.angular.z);
    Eigen::Vector3d ang_vel_out = R_body_transform_ * ang_vel_in;

    visual_odom_msg.angular_velocity = {static_cast<float>(ang_vel_out.x()),
                                        static_cast<float>(ang_vel_out.y()),
                                        static_cast<float>(ang_vel_out.z())};

    // Convert position variance (world frame)
    Eigen::Vector3d pos_var_in(msg->pose.covariance[0],
                               msg->pose.covariance[7],
                               msg->pose.covariance[14]);
    Eigen::Vector3d pos_var_out = (R_world_transform_.cwiseAbs() * pos_var_in);

    visual_odom_msg.position_variance = {static_cast<float>(pos_var_out.x()),
                                         static_cast<float>(pos_var_out.y()),
                                         static_cast<float>(pos_var_out.z())};

    // Convert orientation variance (world frame)
    Eigen::Vector3d ori_var_in(msg->pose.covariance[21],
                               msg->pose.covariance[28],
                               msg->pose.covariance[35]);
    Eigen::Vector3d ori_var_out = (R_world_transform_.cwiseAbs() * ori_var_in);

    visual_odom_msg.orientation_variance = {static_cast<float>(ori_var_out.x()),
                                            static_cast<float>(ori_var_out.y()),
                                            static_cast<float>(ori_var_out.z())};

    // Convert velocity variance (body frame)
    Eigen::Vector3d vel_var_in(msg->twist.covariance[0],
                               msg->twist.covariance[7],
                               msg->twist.covariance[14]);
    Eigen::Vector3d vel_var_out = (R_body_transform_.cwiseAbs() * vel_var_in);

    visual_odom_msg.velocity_variance = {static_cast<float>(vel_var_out.x()),
                                         static_cast<float>(vel_var_out.y()),
                                         static_cast<float>(vel_var_out.z())};

    visual_odom_pub_->publish(visual_odom_msg);
  }

  void vrpn_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!use_vrpn_) return;

    constexpr float nan = std::numeric_limits<float>::quiet_NaN();

    px4_msgs::msg::VehicleOdometry visual_odom_msg;

    visual_odom_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    visual_odom_msg.timestamp_sample = msg->header.stamp.sec * 1000000ULL + msg->header.stamp.nanosec / 1000;
    visual_odom_msg.pose_frame = px4_msgs::msg::VehicleOdometry::POSE_FRAME_NED;

    // Convert pose (position + orientation)
    geometry_msgs::msg::Pose pose_out;
    convertPose(msg->pose, pose_out);

    visual_odom_msg.position = {static_cast<float>(pose_out.position.x),
                                static_cast<float>(pose_out.position.y),
                                static_cast<float>(pose_out.position.z)};

    visual_odom_msg.q = {static_cast<float>(pose_out.orientation.w),
                         static_cast<float>(pose_out.orientation.x),
                         static_cast<float>(pose_out.orientation.y),
                         static_cast<float>(pose_out.orientation.z)};

    // No velocity available from VRPN pose
    visual_odom_msg.velocity_frame = px4_msgs::msg::VehicleOdometry::VELOCITY_FRAME_NED;
    visual_odom_msg.velocity = {nan, nan, nan};
    visual_odom_msg.angular_velocity = {nan, nan, nan};
    visual_odom_msg.position_variance = {0.001f, 0.001f, 0.001f};
    visual_odom_msg.orientation_variance = {0.001f, 0.001f, 0.001f};
    visual_odom_msg.velocity_variance = {nan, nan, nan};

    visual_odom_pub_->publish(visual_odom_msg);
  }

  // Parameters
  bool use_vrpn_;
  std::string input_world_frame_;
  std::string output_world_frame_;
  std::string input_body_frame_;
  std::string output_body_frame_;

  // Transformation matrices and quaternions
  Eigen::Matrix3d R_world_transform_;
  Eigen::Matrix3d R_body_transform_;
  Eigen::Quaterniond q_world_transform_;
  Eigen::Quaterniond q_body_transform_;

  // ROS2 publishers and subscribers
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vrpn_sub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr visual_odom_pub_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<OdomToVisualOdom>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("odom_to_visual_odom"),
                 "Exception: %s", e.what());
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}