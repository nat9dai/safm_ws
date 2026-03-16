#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <px4_msgs/msg/vehicle_odometry.hpp>

class OdomToVisualOdom : public rclcpp::Node
{
public:
  OdomToVisualOdom() : Node("odom_to_visual_odom")
  {
    // QoS profile compatible with PX4 micro-XRCE-DDS bridge
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
      "/d2vins/odometry", qos,
      std::bind(&OdomToVisualOdom::odom_callback, this, std::placeholders::_1));

    visual_odom_pub_ = this->create_publisher<px4_msgs::msg::VehicleOdometry>("fmu/in/vehicle_visual_odometry", 10);
  }

private:
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    px4_msgs::msg::VehicleOdometry visual_odom_msg;
    
    visual_odom_msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    visual_odom_msg.timestamp_sample = msg->header.stamp.sec * 1000000ULL + msg->header.stamp.nanosec / 1000;
    visual_odom_msg.pose_frame = 1; // MAV_FRAME_LOCAL_NED
    visual_odom_msg.position = {static_cast<float>(msg->pose.pose.position.y),  // ENU y -> NED x
                                static_cast<float>(msg->pose.pose.position.x),  // ENU x -> NED y
                                static_cast<float>(-msg->pose.pose.position.z)}; // ENU z -> NED -z
    // ENU-FLU to NED-FRD quaternion: {w, y, x, -z}
    visual_odom_msg.q = {static_cast<float>(msg->pose.pose.orientation.w),
                          static_cast<float>(msg->pose.pose.orientation.y),
                          static_cast<float>(msg->pose.pose.orientation.x),
                          static_cast<float>(-msg->pose.pose.orientation.z)};
    visual_odom_msg.velocity_frame = 1; // MAV_FRAME_LOCAL_NED
    visual_odom_msg.velocity = {static_cast<float>(msg->twist.twist.linear.y),  // ENU y -> NED x
                                static_cast<float>(msg->twist.twist.linear.x),  // ENU x -> NED y
                                static_cast<float>(-msg->twist.twist.linear.z)}; // ENU z -> NED -z
    // Angular velocity is body-frame: FLU -> FRD: {x, -y, -z}
    visual_odom_msg.angular_velocity = {static_cast<float>(msg->twist.twist.angular.x),
                                       static_cast<float>(-msg->twist.twist.angular.y),
                                       static_cast<float>(-msg->twist.twist.angular.z)};
    visual_odom_msg.position_variance = {static_cast<float>(msg->pose.covariance[0]),  // x variance
                                        static_cast<float>(msg->pose.covariance[7]),  // y variance
                                        static_cast<float>(msg->pose.covariance[14])}; // z variance
    visual_odom_msg.orientation_variance = {static_cast<float>(msg->pose.covariance[21]), // roll variance
                                         static_cast<float>(msg->pose.covariance[28]), // pitch variance
                                         static_cast<float>(msg->pose.covariance[35])}; // yaw variance
    visual_odom_msg.velocity_variance = {static_cast<float>(msg->twist.covariance[0]),  // x variance
                                        static_cast<float>(msg->twist.covariance[7]),  // y variance
                                        static_cast<float>(msg->twist.covariance[14])}; // z variance

    visual_odom_pub_->publish(visual_odom_msg);
  }

  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Publisher<px4_msgs::msg::VehicleOdometry>::SharedPtr visual_odom_pub_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OdomToVisualOdom>());
  rclcpp::shutdown();
  return 0;
}