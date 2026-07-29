#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <memory>
#include <string>

#include "auv/covariance_recovery_ramp.hpp"
#include "auv/velocity_sample_gate.hpp"
#include <auv_dvl_a50_msg/msg/dvl.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>

class DvlToTwistBridge : public rclcpp::Node
{
public:
  DvlToTwistBridge()
  : Node("dvl_to_twist_bridge")
  {
    input_topic_ = declare_parameter<std::string>("input_topic", "/dvl/data");
    output_topic_ = declare_parameter<std::string>("output_topic", "/dvl/twist");
    output_frame_id_ = declare_parameter<std::string>("output_frame_id", "dvl_link");
    default_linear_variance_ = declare_parameter<double>("default_linear_variance", 0.02);
    min_linear_variance_ = declare_parameter<double>("min_linear_variance", 0.0);
    max_linear_variance_ = declare_parameter<double>("max_linear_variance", 1.0);
    covariance_scale_ = declare_parameter<double>("covariance_scale", 1.0);
    max_fom_ = declare_parameter<double>("max_fom", 0.05);
    min_altitude_ = declare_parameter<double>("min_altitude", 0.05);
    min_valid_beams_ = declare_parameter<int>("min_valid_beams", 3);
    use_dvl_covariance_ = declare_parameter<bool>("use_dvl_covariance", true);
    require_valid_velocity_ = declare_parameter<bool>("require_valid_velocity", true);
    reacquire_good_samples_ = declare_parameter<int>("reacquire_good_samples", 1);
    reacquire_duration_s_ = declare_parameter<double>("reacquire_duration_s", 0.0);
    max_velocity_mps_ = declare_parameter<double>("max_velocity_mps", 0.8);
    max_acceleration_mps2_ = declare_parameter<double>("max_acceleration_mps2", 1.0);
    velocity_jump_tolerance_mps_ =
      declare_parameter<double>("velocity_jump_tolerance_mps", 0.03);
    max_rate_dt_s_ = declare_parameter<double>("max_rate_dt_s", 0.5);
    recovery_trigger_gap_s_ = declare_parameter<double>("recovery_trigger_gap_s", 0.25);
    recovery_initial_variance_ =
      declare_parameter<double>("recovery_initial_variance", 1.0);
    recovery_variance_decay_ =
      declare_parameter<double>("recovery_variance_decay", 0.8);
    recovery_variance_samples_ =
      declare_parameter<int>("recovery_variance_samples", 56);
    velocity_gate_ = std::make_unique<auv::VelocitySampleGate>(
      max_velocity_mps_,
      max_acceleration_mps2_,
      velocity_jump_tolerance_mps_,
      max_rate_dt_s_);
    covariance_recovery_ = std::make_unique<auv::CovarianceRecoveryRamp>(
      recovery_initial_variance_, recovery_variance_decay_, recovery_variance_samples_);

    const auto sensor_qos = rclcpp::SensorDataQoS();

    publisher_ =
      create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>(output_topic_, 10);
    subscription_ = create_subscription<auv_dvl_a50_msg::msg::DVL>(
      input_topic_, sensor_qos,
      std::bind(&DvlToTwistBridge::handle_msg, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Bridging %s -> %s",
      input_topic_.c_str(), output_topic_.c_str());
    RCLCPP_INFO(
      get_logger(),
      "DVL gates: min_var=%.4g max_var=%.4g cov_scale=%.3f max_fom=%.4g min_alt=%.3f min_beams=%d",
      min_linear_variance_, max_linear_variance_, covariance_scale_, max_fom_,
      min_altitude_, min_valid_beams_);
    RCLCPP_INFO(
      get_logger(),
      "DVL reacquire gate: good_samples=%d duration=%.2fs",
      reacquire_good_samples_, reacquire_duration_s_);
    RCLCPP_INFO(
      get_logger(),
      "DVL physical gate: max_velocity=%.2f m/s max_acceleration=%.2f m/s^2 "
      "jump_tolerance=%.3f m/s max_dt=%.2f s",
      max_velocity_mps_, max_acceleration_mps2_,
      velocity_jump_tolerance_mps_, max_rate_dt_s_);
    RCLCPP_INFO(
      get_logger(),
      "DVL covariance recovery: gap=%.2f s initial_variance=%.3g decay=%.3g "
      "samples=%d -> sensor",
      recovery_trigger_gap_s_, recovery_initial_variance_, recovery_variance_decay_,
      recovery_variance_samples_);
  }

private:
  void handle_msg(const auv_dvl_a50_msg::msg::DVL::SharedPtr msg)
  {
    if (require_valid_velocity_ && !msg->velocity_valid) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skipping DVL sample marked invalid.");
      reset_reacquisition();
      trigger_covariance_recovery();
      return;
    }
    if (!is_valid_measurement(*msg)) {
      reset_reacquisition();
      trigger_covariance_recovery();
      return;
    }

    auto sample_time = rclcpp::Time(msg->header.stamp);
    if (sample_time.nanoseconds() <= 0) {
      sample_time = now();
    }
    const auto velocity_gate_result = velocity_gate_->update(
      msg->velocity.x, msg->velocity.y, msg->velocity.z,
      sample_time.nanoseconds());
    if (velocity_gate_result.decision != auv::VelocitySampleGate::Decision::ACCEPT) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Skipping DVL sample rejected by physical gate: "
        "speed=%.3f m/s, delta=%.3f m/s, allowed=%.3f m/s, reason=%d.",
        velocity_gate_result.speed_mps,
        velocity_gate_result.velocity_delta_mps,
        velocity_gate_result.allowed_delta_mps,
        static_cast<int>(velocity_gate_result.decision));
      reset_reacquisition();
      trigger_covariance_recovery();
      return;
    }

    if (
      last_published_sample_time_ns_ > 0 &&
      recovery_trigger_gap_s_ > 0.0 &&
      static_cast<double>(
        sample_time.nanoseconds() - last_published_sample_time_ns_) * 1.0e-9 >
      recovery_trigger_gap_s_)
    {
      trigger_covariance_recovery();
    }

    geometry_msgs::msg::TwistWithCovarianceStamped out;
    out.header = msg->header;
    if (!output_frame_id_.empty()) {
      out.header.frame_id = output_frame_id_;
    }

    out.twist.twist.linear.x = msg->velocity.x;
    out.twist.twist.linear.y = msg->velocity.y;
    out.twist.twist.linear.z = msg->velocity.z;

    auto & cov = out.twist.covariance;
    cov.fill(0.0);

    if (use_dvl_covariance_ && msg->covariance.size() == 9) {
      for (size_t row = 0; row < 3; ++row) {
        for (size_t col = 0; col < 3; ++col) {
          cov[row * 6 + col] = msg->covariance[row * 3 + col] * covariance_scale_;
        }
      }
    } else if (use_dvl_covariance_ && msg->covariance.size() == 36) {
      for (size_t index = 0; index < cov.size(); ++index) {
        cov[index] = msg->covariance[index] * covariance_scale_;
      }
    } else {
      cov[0] = default_linear_variance_;
      cov[7] = default_linear_variance_;
      cov[14] = default_linear_variance_;
    }

    if (has_rejected_covariance(cov)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skipping DVL sample with excessive covariance.");
      reset_reacquisition();
      trigger_covariance_recovery();
      return;
    }

    cov[0] = sanitize_variance(cov[0]);
    cov[7] = sanitize_variance(cov[7]);
    cov[14] = sanitize_variance(cov[14]);

    if (!is_reacquired()) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Holding DVL twist during reacquire: %d/%d good samples.",
        consecutive_good_samples_, reacquire_good_samples_);
      return;
    }

    if (covariance_recovery_->active()) {
      const int recovery_stage = covariance_recovery_->stage();
      cov[0] = covariance_recovery_->apply(cov[0]);
      cov[7] = covariance_recovery_->apply(cov[7]);
      cov[14] = covariance_recovery_->apply(cov[14]);
      RCLCPP_INFO_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "DVL covariance recovery stage %d: linear variance %.6g.",
        recovery_stage, cov[0]);
      covariance_recovery_->advance();
    }

    publisher_->publish(out);
    last_published_sample_time_ns_ = sample_time.nanoseconds();
  }

  bool is_valid_measurement(const auv_dvl_a50_msg::msg::DVL & msg)
  {
    const auto & velocity = msg.velocity;
    if (!std::isfinite(velocity.x) || !std::isfinite(velocity.y) || !std::isfinite(velocity.z)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skipping DVL sample with non-finite velocity.");
      return false;
    }

    if (min_altitude_ > 0.0 && (!std::isfinite(msg.altitude) || msg.altitude < min_altitude_)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skipping DVL sample with low/invalid altitude: %.3f m.", msg.altitude);
      return false;
    }

    if (max_fom_ > 0.0 && (!std::isfinite(msg.fom) || msg.fom > max_fom_)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Skipping DVL sample with high FOM: %.6f.", msg.fom);
      return false;
    }

    if (min_valid_beams_ > 0) {
      const auto valid_beams = static_cast<int>(std::count_if(
          msg.beams.begin(), msg.beams.end(),
          [](const auto & beam) {return beam.valid;}));
      if (valid_beams < min_valid_beams_) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Skipping DVL sample with %d valid beams.", valid_beams);
        return false;
      }
    }

    return true;
  }

  bool has_rejected_covariance(const std::array<double, 36> & cov) const
  {
    if (max_linear_variance_ <= 0.0) {
      return false;
    }
    return exceeds_max_variance(cov[0]) || exceeds_max_variance(cov[7]) ||
           exceeds_max_variance(cov[14]);
  }

  bool exceeds_max_variance(double value) const
  {
    return !std::isfinite(value) || value > max_linear_variance_;
  }

  double sanitize_variance(double value) const
  {
    if (min_linear_variance_ <= 0.0) {
      return value;
    }
    if (!std::isfinite(value) || value <= 0.0) {
      return min_linear_variance_;
    }
    return std::max(value, min_linear_variance_);
  }

  void reset_reacquisition()
  {
    reacquired_ = false;
    consecutive_good_samples_ = 0;
    first_good_time_ = rclcpp::Time(0, 0, get_clock()->get_clock_type());
  }

  void trigger_covariance_recovery()
  {
    covariance_recovery_->trigger();
  }

  bool is_reacquired()
  {
    if (reacquire_good_samples_ <= 1 && reacquire_duration_s_ <= 0.0) {
      reacquired_ = true;
      return true;
    }

    const auto now = get_clock()->now();
    if (consecutive_good_samples_ == 0) {
      first_good_time_ = now;
    }
    consecutive_good_samples_++;

    const bool samples_ok = consecutive_good_samples_ >= std::max(1, reacquire_good_samples_);
    const bool duration_ok =
      reacquire_duration_s_ <= 0.0 || (now - first_good_time_).seconds() >= reacquire_duration_s_;
    reacquired_ = samples_ok && duration_ok;
    return reacquired_;
  }

  std::string input_topic_;
  std::string output_topic_;
  std::string output_frame_id_;
  double default_linear_variance_;
  double min_linear_variance_;
  double max_linear_variance_;
  double covariance_scale_;
  double max_fom_;
  double min_altitude_;
  int min_valid_beams_;
  bool use_dvl_covariance_;
  bool require_valid_velocity_;
  int reacquire_good_samples_;
  double reacquire_duration_s_;
  double max_velocity_mps_;
  double max_acceleration_mps2_;
  double velocity_jump_tolerance_mps_;
  double max_rate_dt_s_;
  double recovery_trigger_gap_s_;
  double recovery_initial_variance_;
  double recovery_variance_decay_;
  int recovery_variance_samples_;
  bool reacquired_{false};
  int consecutive_good_samples_{0};
  std::int64_t last_published_sample_time_ns_{0};
  rclcpp::Time first_good_time_{0, 0, RCL_ROS_TIME};
  std::unique_ptr<auv::VelocitySampleGate> velocity_gate_;
  std::unique_ptr<auv::CovarianceRecoveryRamp> covariance_recovery_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr publisher_;
  rclcpp::Subscription<auv_dvl_a50_msg::msg::DVL>::SharedPtr subscription_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DvlToTwistBridge>());
  rclcpp::shutdown();
  return 0;
}
