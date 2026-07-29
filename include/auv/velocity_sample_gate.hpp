#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace auv
{

class VelocitySampleGate
{
public:
  enum class Decision
  {
    ACCEPT,
    REJECT_NONFINITE,
    REJECT_SPEED,
    REJECT_RATE,
    REJECT_TIME
  };

  struct Result
  {
    Decision decision{Decision::REJECT_NONFINITE};
    double speed_mps{0.0};
    double velocity_delta_mps{0.0};
    double allowed_delta_mps{0.0};
  };

  VelocitySampleGate(
    double max_velocity_mps,
    double max_acceleration_mps2,
    double jump_tolerance_mps,
    double max_rate_dt_s)
  : max_velocity_mps_(std::max(0.0, max_velocity_mps)),
    max_acceleration_mps2_(std::max(0.0, max_acceleration_mps2)),
    jump_tolerance_mps_(std::max(0.0, jump_tolerance_mps)),
    max_rate_dt_s_(std::max(0.001, max_rate_dt_s))
  {
  }

  Result update(double vx, double vy, double vz, std::int64_t sample_time_ns)
  {
    Result result;
    if (!std::isfinite(vx) || !std::isfinite(vy) || !std::isfinite(vz)) {
      return result;
    }

    result.speed_mps = vector_norm(vx, vy, vz);
    if (max_velocity_mps_ > 0.0 && result.speed_mps > max_velocity_mps_) {
      result.decision = Decision::REJECT_SPEED;
      return result;
    }

    if (sample_time_ns <= 0) {
      result.decision = Decision::REJECT_TIME;
      return result;
    }

    if (!have_accepted_sample_) {
      accept(vx, vy, vz, sample_time_ns);
      result.decision = Decision::ACCEPT;
      return result;
    }

    if (sample_time_ns <= last_accepted_time_ns_) {
      result.decision = Decision::REJECT_TIME;
      return result;
    }

    const double dt_s =
      static_cast<double>(sample_time_ns - last_accepted_time_ns_) * 1.0e-9;
    result.velocity_delta_mps = vector_norm(
      vx - last_accepted_vx_,
      vy - last_accepted_vy_,
      vz - last_accepted_vz_);
    result.allowed_delta_mps =
      jump_tolerance_mps_ +
      max_acceleration_mps2_ * std::min(dt_s, max_rate_dt_s_);

    if (
      max_acceleration_mps2_ > 0.0 &&
      result.velocity_delta_mps > result.allowed_delta_mps)
    {
      result.decision = Decision::REJECT_RATE;
      return result;
    }

    accept(vx, vy, vz, sample_time_ns);
    result.decision = Decision::ACCEPT;
    return result;
  }

private:
  static double vector_norm(double x, double y, double z)
  {
    return std::sqrt(x * x + y * y + z * z);
  }

  void accept(double vx, double vy, double vz, std::int64_t sample_time_ns)
  {
    have_accepted_sample_ = true;
    last_accepted_vx_ = vx;
    last_accepted_vy_ = vy;
    last_accepted_vz_ = vz;
    last_accepted_time_ns_ = sample_time_ns;
  }

  double max_velocity_mps_;
  double max_acceleration_mps2_;
  double jump_tolerance_mps_;
  double max_rate_dt_s_;
  bool have_accepted_sample_{false};
  double last_accepted_vx_{0.0};
  double last_accepted_vy_{0.0};
  double last_accepted_vz_{0.0};
  std::int64_t last_accepted_time_ns_{0};
};

}  // namespace auv
