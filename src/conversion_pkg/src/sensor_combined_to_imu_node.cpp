#include <rclcpp/rclcpp.hpp>
#include <px4_msgs/msg/sensor_combined.hpp>
#include <px4_msgs/msg/vehicle_attitude.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class SensorCombinedToImu : public rclcpp::Node
{
public:
  SensorCombinedToImu() : Node("sensor_combined_to_imu")
  {
    // Declare parameters
    this->declare_parameter<std::string>("sensor_combined_topic", "fmu/out/sensor_combined");
    this->declare_parameter<std::string>("vehicle_attitude_topic", "fmu/out/vehicle_attitude");
    this->declare_parameter<std::string>("imu_topic", "imu/data");
    this->declare_parameter<std::string>("input_world_frame", "NED");
    this->declare_parameter<std::string>("output_world_frame", "ENU");
    this->declare_parameter<std::string>("input_body_frame", "FRD");
    this->declare_parameter<std::string>("output_body_frame", "FLU");

    // Get parameters
    auto sensor_combined_topic = this->get_parameter("sensor_combined_topic").as_string();
    auto vehicle_attitude_topic = this->get_parameter("vehicle_attitude_topic").as_string();
    auto imu_topic = this->get_parameter("imu_topic").as_string();
    input_world_frame_ = this->get_parameter("input_world_frame").as_string();
    output_world_frame_ = this->get_parameter("output_world_frame").as_string();
    input_body_frame_ = this->get_parameter("input_body_frame").as_string();
    output_body_frame_ = this->get_parameter("output_body_frame").as_string();

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

    // QoS profile compatible with PX4 micro-XRCE-DDS bridge
    rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
    auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);

    sensor_sub_ = this->create_subscription<px4_msgs::msg::SensorCombined>(
      sensor_combined_topic, qos,
      std::bind(&SensorCombinedToImu::sensor_callback, this, std::placeholders::_1));

    attitude_sub_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
      vehicle_attitude_topic, qos,
      std::bind(&SensorCombinedToImu::attitude_callback, this, std::placeholders::_1));

    pub_ = this->create_publisher<sensor_msgs::msg::Imu>(imu_topic, 10);
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

  void attitude_callback(const px4_msgs::msg::VehicleAttitude::SharedPtr msg)
  {
    last_attitude_ = *msg;
    has_attitude_ = true;
  }

  void sensor_callback(const px4_msgs::msg::SensorCombined::SharedPtr msg)
  {
    sensor_msgs::msg::Imu imu_msg;

    imu_msg.header.stamp = this->now();
    imu_msg.header.frame_id = "base_link";

    // Convert gyro from input body frame to output body frame
    Eigen::Vector3d gyro_in(msg->gyro_rad[0], msg->gyro_rad[1], msg->gyro_rad[2]);
    Eigen::Vector3d gyro_out = R_body_transform_ * gyro_in;

    imu_msg.angular_velocity.x = gyro_out.x();
    imu_msg.angular_velocity.y = gyro_out.y();
    imu_msg.angular_velocity.z = gyro_out.z();

    // Convert accelerometer from input body frame to output body frame
    Eigen::Vector3d accel_in(msg->accelerometer_m_s2[0], msg->accelerometer_m_s2[1], msg->accelerometer_m_s2[2]);
    Eigen::Vector3d accel_out = R_body_transform_ * accel_in;

    imu_msg.linear_acceleration.x = accel_out.x();
    imu_msg.linear_acceleration.y = accel_out.y();
    imu_msg.linear_acceleration.z = accel_out.z();

    if (has_attitude_) {
      // VehicleAttitude.q is [w, x, y, z] in input world/body frame convention
      // Convert using: q_out = q_world * q_in * q_body
      Eigen::Quaterniond q_in(last_attitude_.q[0],
                              last_attitude_.q[1],
                              last_attitude_.q[2],
                              last_attitude_.q[3]);

      Eigen::Quaterniond q_out = q_world_transform_ * q_in * q_body_transform_;
      q_out.normalize();

      imu_msg.orientation.w = q_out.w();
      imu_msg.orientation.x = q_out.x();
      imu_msg.orientation.y = q_out.y();
      imu_msg.orientation.z = q_out.z();
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

  // Parameters
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
  rclcpp::Subscription<px4_msgs::msg::SensorCombined>::SharedPtr sensor_sub_;
  rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr attitude_sub_;
  rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_;

  px4_msgs::msg::VehicleAttitude last_attitude_;
  bool has_attitude_ = false;
};

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<SensorCombinedToImu>();
    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(rclcpp::get_logger("sensor_combined_to_imu"),
                 "Exception: %s", e.what());
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}