#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <px4_msgs/msg/vehicle_local_position.hpp>
#include <cmath>

class LocalPoseConversion : public rclcpp::Node
{
public:
  LocalPoseConversion() : Node("local_pose_conversion")
  {
    // QoS profile compatible with PX4 micro-XRCE-DDS bridge
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    this->declare_parameter("tracking_ros.vehicle_local_position_topic", "/fmu/out/vehicle_local_position");
    local_pose_topic_ros2_ = this->get_parameter("tracking_ros.vehicle_local_position_topic").as_string();
    local_pose_topic_ros1_ = "mavros/local_position/pose";
    local_vel_topic_ros1_  = "mavros/local_position/velocity";

    local_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
      local_pose_topic_ros1_, qos,
      std::bind(&LocalPoseConversion::local_pose_callback, this, std::placeholders::_1));

    local_vel_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
      local_vel_topic_ros1_, qos,
      std::bind(&LocalPoseConversion::local_vel_callback, this, std::placeholders::_1));

    local_pose_pub_ = this->create_publisher<px4_msgs::msg::VehicleLocalPosition>(local_pose_topic_ros2_, 10);
  }

private:
  void local_vel_callback(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
  {
    last_velocity_ = *msg;
    has_velocity_ = true;
  }

  void local_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
  {
    px4_msgs::msg::VehicleLocalPosition px4_msg;

    px4_msg.timestamp        = this->get_clock()->now().nanoseconds() / 1000;
    px4_msg.timestamp_sample = px4_msg.timestamp;

    // ENU (x=East, y=North, z=Up) -> NED (x=North, y=East, z=Down)
    px4_msg.x =  static_cast<float>(msg->pose.position.y);
    px4_msg.y =  static_cast<float>(msg->pose.position.x);
    px4_msg.z = -static_cast<float>(msg->pose.position.z);

    // Extract yaw from ENU quaternion (ROS: CCW from East)
    // and convert to NED heading (CW from North): heading_NED = pi/2 - yaw_ENU
    const auto &q = msg->pose.orientation;
    double yaw_enu = std::atan2(2.0 * (q.w * q.z + q.x * q.y),
                                1.0 - 2.0 * (q.y * q.y + q.z * q.z));
    double heading = M_PI / 2.0 - yaw_enu;
    // Normalize to [-pi, pi]
    while (heading >  M_PI) heading -= 2.0 * M_PI;
    while (heading < -M_PI) heading += 2.0 * M_PI;
    px4_msg.heading = static_cast<float>(heading);

    px4_msg.xy_valid = true;
    px4_msg.z_valid  = true;

    if (has_velocity_) {
      const auto &v = last_velocity_.twist.linear;
      // ENU -> NED velocity: vx_NED = vy_ENU, vy_NED = vx_ENU, vz_NED = -vz_ENU
      px4_msg.vx = static_cast<float>( v.y);
      px4_msg.vy = static_cast<float>( v.x);
      px4_msg.vz = static_cast<float>(-v.z);
      px4_msg.v_xy_valid = true;
      px4_msg.v_z_valid  = true;
    } else {
      px4_msg.v_xy_valid = false;
      px4_msg.v_z_valid  = false;
    }

    local_pose_pub_->publish(px4_msg);
  }

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr local_pose_sub_;
  rclcpp::Subscription<geometry_msgs::msg::TwistStamped>::SharedPtr local_vel_sub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleLocalPosition>::SharedPtr local_pose_pub_;

  std::string local_pose_topic_ros2_;
  std::string local_pose_topic_ros1_;
  std::string local_vel_topic_ros1_;

  geometry_msgs::msg::TwistStamped last_velocity_;
  bool has_velocity_ = false;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<LocalPoseConversion>());
  rclcpp::shutdown();
  return 0;
}
