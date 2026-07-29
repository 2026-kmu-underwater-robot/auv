#include <array>
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

#include "auv/depth_sample_gate.hpp"
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/fluid_pressure.hpp>

class PressureToDepthPose : public rclcpp::Node
{
public:
  PressureToDepthPose()
  : Node("pressure_to_depth_pose")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/mavros/imu/static_pressure");
    input_mode_ = declare_parameter<std::string>("input_mode", "pressure_pa");
    output_topic_ = declare_parameter<std::string>("output_topic", "/depth/pose");
    world_frame_ = declare_parameter<std::string>("world_frame", "odom");
    fluid_density_ = declare_parameter<double>("fluid_density", 1000.0);
    gravity_ = declare_parameter<double>("gravity", 9.80665);
    surface_pressure_pa_ = declare_parameter<double>("surface_pressure_pa", 101325.0);
    zero_at_start_ = declare_parameter<bool>("zero_at_start", true);
    depth_offset_m_ = declare_parameter<double>("depth_offset_m", 0.0);
    z_variance_ = declare_parameter<double>("z_variance", 0.05);
    enable_depth_gate_ = declare_parameter<bool>("enable_depth_gate", true);
    max_vertical_speed_mps_ =
      declare_parameter<double>("max_vertical_speed_mps", 2.0);
    jump_tolerance_m_ = declare_parameter<double>("jump_tolerance_m", 0.10);
    max_rate_dt_s_ = declare_parameter<double>("max_rate_dt_s", 0.5);
    const auto configured_reacquire_good_samples =
      declare_parameter<std::int64_t>("reacquire_good_samples", 3);
    reacquire_good_samples_ = static_cast<int>(
      std::max<std::int64_t>(1, configured_reacquire_good_samples));
    const auto configured_reacquire_variance_samples =
      declare_parameter<std::int64_t>("reacquire_variance_samples", 3);
    reacquire_variance_samples_ = static_cast<int>(
      std::max<std::int64_t>(0, configured_reacquire_variance_samples));
    reacquire_variance_scale_ = std::max(
      1.0, declare_parameter<double>("reacquire_variance_scale", 10.0));

    depth_gate_ = std::make_unique<auv::DepthSampleGate>(
      max_vertical_speed_mps_,
      jump_tolerance_m_,
      max_rate_dt_s_,
      static_cast<std::size_t>(reacquire_good_samples_));

    const auto sensor_qos = rclcpp::SensorDataQoS();

    publisher_ = create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(output_topic_, 10);
    subscription_ = create_subscription<sensor_msgs::msg::FluidPressure>(
      input_topic_, sensor_qos,
      std::bind(&PressureToDepthPose::handle_msg, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Bridging %s -> %s; depth gate %s "
      "(max speed %.2f m/s, jump %.2f m, max dt %.2f s, reacquire %d samples, "
      "variance ramp %d samples at %.1fx)",
      input_topic_.c_str(), output_topic_.c_str(),
      enable_depth_gate_ ? "enabled" : "disabled",
      max_vertical_speed_mps_, jump_tolerance_m_, max_rate_dt_s_,
      reacquire_good_samples_, reacquire_variance_samples_, reacquire_variance_scale_);
  }

private:
  void handle_msg(const sensor_msgs::msg::FluidPressure::SharedPtr msg)
  {
    if (!std::isfinite(msg->fluid_pressure)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skipping non-finite pressure sample.");
      return;
    }

    double depth_m = 0.0;
    if (input_mode_ == "pressure_pa") {
      depth_m = (msg->fluid_pressure - surface_pressure_pa_) / (fluid_density_ * gravity_);
    } else {
      if (input_mode_ != "depth_m") {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 5000,
          "Unknown input_mode '%s'. Falling back to depth_m.",
          input_mode_.c_str());
      }
      depth_m = msg->fluid_pressure;
    }

    if (enable_depth_gate_) {
      auto sample_time = rclcpp::Time(msg->header.stamp);
      if (sample_time.nanoseconds() <= 0) {
        sample_time = now();
      }

      const auto gate_result = depth_gate_->update(depth_m, sample_time.nanoseconds());
      if (gate_result.decision != auv::DepthSampleGate::Decision::ACCEPT) {
        if (gate_result.decision == auv::DepthSampleGate::Decision::HOLD_FOR_RECOVERY) {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Depth gate recovery: holding normal sample %zu/%d; "
            "raw depth %.3f m, last accepted %.3f m.",
            gate_result.consecutive_good_samples, reacquire_good_samples_,
            depth_m, depth_gate_->last_accepted_depth_m());
        } else {
          RCLCPP_WARN_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "Depth gate rejected sample: raw depth %.3f m, "
            "last accepted %.3f m, delta %.3f m > %.3f m.",
            depth_m, depth_gate_->last_accepted_depth_m(),
            gate_result.delta_m, gate_result.allowed_delta_m);
        }
        return;
      }

      if (gate_result.reacquired) {
        variance_ramp_remaining_ = reacquire_variance_samples_;
        RCLCPP_INFO(
          get_logger(),
          "Depth gate reacquired after %d normal samples; applying covariance ramp.",
          reacquire_good_samples_);
      }
    }

    if (!have_reference_) {
      reference_depth_m_ = zero_at_start_ ? depth_m : 0.0;
      have_reference_ = true;
    }

    geometry_msgs::msg::PoseWithCovarianceStamped out;
    out.header = msg->header;
    out.header.frame_id = world_frame_;
    out.pose.pose.position.z = -(depth_m - reference_depth_m_) + depth_offset_m_;
    out.pose.covariance.fill(0.0);
    double output_z_variance = z_variance_;
    if (variance_ramp_remaining_ > 0 && reacquire_variance_samples_ > 0) {
      const double ramp_fraction =
        static_cast<double>(variance_ramp_remaining_) /
        static_cast<double>(reacquire_variance_samples_);
      const double variance_scale =
        1.0 + (reacquire_variance_scale_ - 1.0) * ramp_fraction;
      output_z_variance *= variance_scale;
      --variance_ramp_remaining_;
    }
    out.pose.covariance[14] = output_z_variance;
    publisher_->publish(out);
  }

  std::string input_topic_;
  std::string input_mode_;
  std::string output_topic_;
  std::string world_frame_;
  double fluid_density_;
  double gravity_;
  double surface_pressure_pa_;
  bool zero_at_start_;
  double depth_offset_m_;
  double z_variance_;
  bool enable_depth_gate_;
  double max_vertical_speed_mps_;
  double jump_tolerance_m_;
  double max_rate_dt_s_;
  int reacquire_good_samples_;
  int reacquire_variance_samples_;
  double reacquire_variance_scale_;
  int variance_ramp_remaining_{0};
  bool have_reference_{false};
  double reference_depth_m_{0.0};
  std::unique_ptr<auv::DepthSampleGate> depth_gate_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr publisher_;
  rclcpp::Subscription<sensor_msgs::msg::FluidPressure>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PressureToDepthPose>());
  rclcpp::shutdown();
  return 0;
}
