#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <sensor_msgs/msg/imu.hpp>

#include <cstdint>

class SensorCombinedToImu : public rclcpp::Node
{
public:
  SensorCombinedToImu() : Node("sensor_combined_to_imu")
  {
    this->declare_parameter<std::string>("sensor_combined_topic", "fmu/out/sensor_combined");
    this->declare_parameter<std::string>("vehicle_attitude_topic", "fmu/out/vehicle_attitude");
    this->declare_parameter<std::string>("imu_topic", "imu/data");
    this->declare_parameter<bool>("publish_orientation", false);
    this->declare_parameter<int64_t>("max_attitude_age_us", 5000);

    auto sensor_combined_topic = this->get_parameter("sensor_combined_topic").as_string();
    auto vehicle_attitude_topic = this->get_parameter("vehicle_attitude_topic").as_string();
    auto imu_topic = this->get_parameter("imu_topic").as_string();
    publish_orientation_ = this->get_parameter("publish_orientation").as_bool();
    max_attitude_age_us_ = this->get_parameter("max_attitude_age_us").as_long();

    // QoS profile compatible with PX4 micro-XRCE-DDS bridge
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 1), qos_profile);

    sensor_sub_ = this->create_subscription<px4_msgs::msg::SensorCombined>(
      sensor_combined_topic, qos,
      std::bind(&SensorCombinedToImu::sensor_callback, this, std::placeholders::_1));

    if (publish_orientation_) {
      attitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
        vehicle_attitude_topic, qos,
        std::bind(&SensorCombinedToImu::attitude_callback, this, std::placeholders::_1));
    }

    pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic, qos);
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

    imu_msg.header.stamp = px4_timestamp_to_ros_time(msg->timestamp);
    imu_msg.header.frame_id = "base_link";

    // PX4 SensorCombined uses FRD
    // sensor_msgs/Imu expects FLU
    // FRD -> FLU: x = x, y = -y, z = -z
    imu_msg.angular_velocity.x =  msg->gyro_rad[0];
    imu_msg.angular_velocity.y = -msg->gyro_rad[1];
    imu_msg.angular_velocity.z = -msg->gyro_rad[2];

    imu_msg.linear_acceleration.x =  msg->accelerometer_m_s2[0];
    imu_msg.linear_acceleration.y = -msg->accelerometer_m_s2[1];
    imu_msg.linear_acceleration.z = -msg->accelerometer_m_s2[2];

    if (publish_orientation_ && has_attitude_ && is_attitude_fresh(msg->timestamp)) {
      // VehicleAttitude.q is [w, x, y, z], FRD body -> NED earth.
      // ROS IMU expects FLU body -> ENU earth.
      // NED-FRD to ENU-FLU
      // (1/sqrt(2)) * {W+Z, X+Y, X-Y, W-Z}
      const auto &q = last_attitude_.q;  // [w, x, y, z] in NED-FRD
      constexpr float s = 0.7071067811865476f; // 1/sqrt(2)
      imu_msg.orientation.w =  s*(q[0]+q[3]);
      imu_msg.orientation.x =  s*(q[1]+q[2]);
      imu_msg.orientation.y =  s*(q[1]-q[2]);
      imu_msg.orientation.z =  s*(q[0]-q[3]);
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

  bool is_attitude_fresh(uint64_t sensor_timestamp_us) const
  {
    const int64_t attitude_timestamp_us =
      static_cast<int64_t>(last_attitude_.timestamp_sample != 0 ?
      last_attitude_.timestamp_sample : last_attitude_.timestamp);
    const int64_t sensor_timestamp_us_signed = static_cast<int64_t>(sensor_timestamp_us);
    const int64_t diff_us = sensor_timestamp_us_signed - attitude_timestamp_us;
    return diff_us <= max_attitude_age_us_ && diff_us >= -max_attitude_age_us_;
  }

  rclcpp::Time px4_timestamp_to_ros_time(uint64_t px4_timestamp_us) const
  {
    if (px4_timestamp_us > 0) {
      return rclcpp::Time(static_cast<int64_t>(px4_timestamp_us) * 1000, RCL_SYSTEM_TIME);
    }
    return this->now();
  }

  rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr sensor_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;

  px4_msgs::msg::VehicleAttitude last_attitude_;
  bool has_attitude_ = false;
  bool publish_orientation_ = false;
  int64_t max_attitude_age_us_ = 5000;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SensorCombinedToImu>());
  rclcpp::shutdown();
  return 0;
}
