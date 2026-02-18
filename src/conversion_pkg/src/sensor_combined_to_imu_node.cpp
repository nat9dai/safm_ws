#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <sensor_msgs/msg/imu.hpp>

class SensorCombinedToImu : public rclcpp::Node
{
public:
  SensorCombinedToImu() : Node("sensor_combined_to_imu")
  {
    // QoS profile compatible with PX4 micro-XRCE-DDS bridge
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    sensor_sub_ = this->create_subscription<px4_msgs::msg::SensorCombined>(
      "fmu/out/sensor_combined", qos,
      std::bind(&SensorCombinedToImu::sensor_callback, this, std::placeholders::_1));

    attitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
      "fmu/out/vehicle_attitude", qos,
      std::bind(&SensorCombinedToImu::attitude_callback, this, std::placeholders::_1));

    pub_ = this->create_publisher<sensor_msgs::msg::Imu>("imu/data_raw", 10);
  }

private:
  void attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
  {
    last_attitude_ = *msg;
    has_attitude_ = true;
  }

  void sensor_callback(const px4_msgs::msg::SensorCombined::SharedPtr msg)
  {
    sensor_msgs::msg::Imu imu_msg;

    imu_msg.header.stamp = this->now();
    imu_msg.header.frame_id = "imu_link";

    // PX4 SensorCombined uses FRD
    // sensor_msgs/Imu expects FLU
    // FRD -> FLU: x = x, y = -y, z = -z
    imu_msg.angular_velocity.x =  msg->gyro_rad[0];
    imu_msg.angular_velocity.y = -msg->gyro_rad[1];
    imu_msg.angular_velocity.z = -msg->gyro_rad[2];

    imu_msg.linear_acceleration.x =  msg->accelerometer_m_s2[0];
    imu_msg.linear_acceleration.y = -msg->accelerometer_m_s2[1];
    imu_msg.linear_acceleration.z = -msg->accelerometer_m_s2[2];

    if (has_attitude_) {
      // VehicleAttitude.q is Hamilton [w, x, y, z], FRD body -> NED earth.
      // ROS expects FLU body -> ENU earth.
      // Conversion: q_enu_flu = R * q_ned_frd * R^-1, where R rotates NED->ENU / FRD->FLU.
      // This simplifies to: q_enu_flu = (w, x, -y, -z) of q_ned_frd
      // then swap x,y for NED->ENU on the earth frame:
      // Full transform: w'=w, x'=y, y'=x, z'=-z  (from q_ned_frd)
      const auto &q = last_attitude_.q;  // [w, x, y, z] in NED-FRD
      imu_msg.orientation.w =  q[0];
      imu_msg.orientation.x =  q[2];  // NED y -> ENU x
      imu_msg.orientation.y =  q[1];  // NED x -> ENU y
      imu_msg.orientation.z = -q[3];  // NED -z -> ENU z
      imu_msg.orientation_covariance = {0};
    } else {
      // No attitude yet
      imu_msg.orientation.x = 0.0;
      imu_msg.orientation.y = 0.0;
      imu_msg.orientation.z = 0.0;
      imu_msg.orientation.w = 0.0;
      imu_msg.orientation_covariance[0] = -1.0;
    }

    imu_msg.angular_velocity_covariance = {0};
    imu_msg.linear_acceleration_covariance = {0};

    pub_->publish(imu_msg);
  }

  rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr sensor_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;

  px4_msgs::msg::VehicleAttitude last_attitude_;
  bool has_attitude_ = false;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorCombinedToImu>());
  rclcpp::shutdown();
  return 0;
}