#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>
#include <limits>

class OdomToVisualOdom : public rclcpp::Node
{
public:
  OdomToVisualOdom() : Node("odom_to_visual_odom")
  {
    this->declare_parameter<bool>("use_vrpn", false);
    this->declare_parameter<std::string>("odom_topic", "/d2vins/odometry");
    this->declare_parameter<std::string>("vrpn_topic", "/vrpn_client_node/pose");
    this->declare_parameter<std::string>("visual_odom_topic", "fmu/in/vehicle_visual_odometry");
    this->declare_parameter<std::string>("realsense_odom_topic", "/camera/odom/sample");

    // realsense t265
    this->declare_parameter<bool>("use_realsense", false);
    use_realsense_ = this->get_parameter("use_realsense").as_bool();

    use_vrpn_ = this->get_parameter("use_vrpn").as_bool();
    auto odom_topic = use_realsense_ ? this->get_parameter("realsense_odom_topic").as_string() : this->get_parameter("odom_topic").as_string();
    auto vrpn_topic = this->get_parameter("vrpn_topic").as_string();
    auto visual_odom_topic = this->get_parameter("visual_odom_topic").as_string();


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

    RCLCPP_INFO(this->get_logger(), "use_vrpn: %s", use_vrpn_ ? "true" : "false");
    RCLCPP_INFO(this->get_logger(), "use_realsense: %s", use_realsense_ ? "true" : "false");
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    if (use_vrpn_) return;

    // T265 mounting correction: lens UP, USB RIGHT — applied before ENU->NED
    if (use_realsense_) {
        // constexpr float qrx=0.0f, qry=0.7071f, qrz=0.0f, qrw=0.7071f;
        constexpr float qrx=0.5f, qry=0.5f, qrz=-0.5f, qrw=0.5f;

        // Rotate position
        // float px = msg->pose.pose.position.x;
        // float py = msg->pose.pose.position.y;
        // float pz = msg->pose.pose.position.z;
        // msg->pose.pose.position.x =  py;
        // msg->pose.pose.position.y = -px;
        // msg->pose.pose.position.z = -pz;

        // Rotate orientation: q_body = q_rot * q_cam
        float qcx = msg->pose.pose.orientation.x;
        float qcy = msg->pose.pose.orientation.y;
        float qcz = msg->pose.pose.orientation.z;
        float qcw = msg->pose.pose.orientation.w;
        msg->pose.pose.orientation.x = qrw*qcx + qrx*qcw + qry*qcz - qrz*qcy;
        msg->pose.pose.orientation.y = qrw*qcy - qrx*qcz + qry*qcw + qrz*qcx;
        msg->pose.pose.orientation.z = qrw*qcz + qrx*qcy - qry*qcx + qrz*qcw;
        msg->pose.pose.orientation.w = qrw*qcw - qrx*qcx - qry*qcy - qrz*qcz;

        // Rotate velocity
        // NOTE: Not used
        // float vx = msg->twist.twist.linear.x;
        // float vy = msg->twist.twist.linear.y;
        // float vz = msg->twist.twist.linear.z;
        // msg->twist.twist.linear.x =  vy;
        // msg->twist.twist.linear.y = -vx;
        // msg->twist.twist.linear.z = -vz;
    }

    px4_msgs::msg::VehicleOdometry visual_odom_msg;

    visual_odom_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    visual_odom_msg.timestamp_sample = msg->header.stamp.sec * 1000000ULL + msg->header.stamp.nanosec / 1000;
    visual_odom_msg.pose_frame = 1; // MAV_FRAME_LOCAL_NED
    visual_odom_msg.position = {static_cast<float>(msg->pose.pose.position.y),  // ENU y -> NED x
                                static_cast<float>(msg->pose.pose.position.x),  // ENU x -> NED y
                                static_cast<float>(-msg->pose.pose.position.z)}; // ENU z -> NED -z
    // ENU-FLU to NED-FRD quaternion
    // (1/sqrt(2)) * {w+z, x+y, x-y, w-z}
    {
      constexpr float s = 0.7071067811865476f; // 1/sqrt(2)
      float qw = static_cast<float>(msg->pose.pose.orientation.w);
      float qx = static_cast<float>(msg->pose.pose.orientation.x);
      float qy = static_cast<float>(msg->pose.pose.orientation.y);
      float qz = static_cast<float>(msg->pose.pose.orientation.z);
      visual_odom_msg.q = {s*(qw+qz), s*(qx+qy), s*(qx-qy), s*(qw-qz)};
    }
    visual_odom_msg.velocity_frame = 3; // VELOCITY_FRAME_BODY_FRD
    visual_odom_msg.velocity = {static_cast<float>(msg->twist.twist.linear.x),   // FLU x -> FRD x
                                static_cast<float>(-msg->twist.twist.linear.y),  // FLU y -> FRD -y
                                static_cast<float>(-msg->twist.twist.linear.z)}; // FLU z -> FRD -z
    // Angular velocity is body-frame: FLU -> FRD: {x, -y, -z}
    visual_odom_msg.angular_velocity = {static_cast<float>(msg->twist.twist.angular.x),
                                       static_cast<float>(-msg->twist.twist.angular.y),
                                       static_cast<float>(-msg->twist.twist.angular.z)};
    visual_odom_msg.position_variance = {static_cast<float>(msg->pose.covariance[7]),   // ENU y -> NED x
                                        static_cast<float>(msg->pose.covariance[0]),   // ENU x -> NED y
                                        static_cast<float>(msg->pose.covariance[14])}; // ENU z -> NED z
    visual_odom_msg.orientation_variance = {static_cast<float>(msg->pose.covariance[28]), // ENU pitch -> NED roll
                                            static_cast<float>(msg->pose.covariance[21]), // ENU roll -> NED pitch
                                            static_cast<float>(msg->pose.covariance[35])}; // yaw
    visual_odom_msg.velocity_variance = {static_cast<float>(msg->twist.covariance[0]),   // FLU/FRD x
                                         static_cast<float>(msg->twist.covariance[7]),   // FLU/FRD y
                                         static_cast<float>(msg->twist.covariance[14])}; // FLU/FRD z

    visual_odom_pub_->publish(visual_odom_msg);
  }

  void vrpn_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    if (!use_vrpn_) return;

    constexpr float nan = std::numeric_limits<float>::quiet_NaN();

    px4_msgs::msg::VehicleOdometry visual_odom_msg;

    visual_odom_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    visual_odom_msg.timestamp_sample = msg->header.stamp.sec * 1000000ULL + msg->header.stamp.nanosec / 1000;
    visual_odom_msg.pose_frame = 1; // MAV_FRAME_LOCAL_NED
    visual_odom_msg.position = {static_cast<float>(msg->pose.position.y),  // ENU y -> NED x
                                static_cast<float>(msg->pose.position.x),  // ENU x -> NED y
                                static_cast<float>(-msg->pose.position.z)}; // ENU z -> NED -z
    // ENU-FLU to NED-FRD quaternion: (1/sqrt(2)) * {w+z, x+y, x-y, w-z}
    {
      constexpr float s = 0.7071067811865476f; // 1/sqrt(2)
      float qw = static_cast<float>(msg->pose.orientation.w);
      float qx = static_cast<float>(msg->pose.orientation.x);
      float qy = static_cast<float>(msg->pose.orientation.y);
      float qz = static_cast<float>(msg->pose.orientation.z);
      visual_odom_msg.q = {s*(qw+qz), s*(qx+qy), s*(qx-qy), s*(qw-qz)};
    }
    // no velocity
    visual_odom_msg.velocity_frame = 1; // MAV_FRAME_LOCAL_NED
    visual_odom_msg.velocity = {nan, nan, nan};
    visual_odom_msg.angular_velocity = {nan, nan, nan};
    visual_odom_msg.position_variance = {0.001f, 0.001f, 0.001f};
    visual_odom_msg.orientation_variance = {0.001f, 0.001f, 0.001f};
    visual_odom_msg.velocity_variance = {nan, nan, nan};

    visual_odom_pub_->publish(visual_odom_msg);
  }

  bool use_vrpn_;
  bool use_realsense_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr vrpn_sub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr visual_odom_pub_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomToVisualOdom>());
  rclcpp::shutdown();
  return 0;
}
