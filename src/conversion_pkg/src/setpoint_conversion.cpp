#include <rclcpp/rclcpp.hpp>
#include <mavros_msgs/msg/attitude_target.hpp>
#include <px4_msgs/msg/vehicle_rates_setpoint.hpp>

class SetpointConversion : public rclcpp::Node
{
public:
  SetpointConversion() : Node("setpoint_conversion")
  {
    // QoS profile compatible with PX4 micro-XRCE-DDS bridge
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    this->declare_parameter("tracking_ros.vehicle_rates_setpoint_topic", "/fmu/in/vehicle_rates_setpoint");
    setpoint_topic_ros2_ = this->get_parameter("tracking_ros.vehicle_rates_setpoint_topic").as_string();
    setpoint_topic_ros1_ = "mavros/setpoint_raw/attitude";

    setpoint_sub_ = this->create_subscription<px4_msgs::msg::VehicleRatesSetpoint>(
      setpoint_topic_ros2_, qos,
      std::bind(&SetpointConversion::setpoint_callback, this, std::placeholders::_1));

    setpoint_pub_ = this->create_publisher<mavros_msgs::msg::AttitudeTarget>(setpoint_topic_ros1_, 10);
  }

private:
  void setpoint_callback(const px4_msgs::msg::VehicleRatesSetpoint::SharedPtr msg)
  {
    mavros_msgs::msg::AttitudeTarget ctbr_msg;
    ctbr_msg.header.stamp = this->now();
    ctbr_msg.header.frame_id = "base_link";

    // Ignore orientation, use body rates + thrust
    ctbr_msg.type_mask = mavros_msgs::msg::AttitudeTarget::IGNORE_ATTITUDE;

    // PX4 VehicleRatesSetpoint uses FRD
    // MAVROS AttitudeTarget expects FLU
    // FRD -> FLU: x = x, y = -y, z = -z
    ctbr_msg.body_rate.x =  msg->roll;
    ctbr_msg.body_rate.y = -msg->pitch;
    ctbr_msg.body_rate.z = -msg->yaw;

    // thrust_body[2] is FRD z-axis (negative = upward); negate to get positive [0,1] thrust
    ctbr_msg.thrust = -msg->thrust_body[2];

    setpoint_pub_->publish(ctbr_msg);
  }

  rclcpp::Subscription<px4_msgs::msg::VehicleRatesSetpoint>::SharedPtr setpoint_sub_;
  rclcpp::Publisher<mavros_msgs::msg::AttitudeTarget>::SharedPtr setpoint_pub_;

  std::string setpoint_topic_ros2_;
  std::string setpoint_topic_ros1_;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SetpointConversion>());
  rclcpp::shutdown();
  return 0;
}
