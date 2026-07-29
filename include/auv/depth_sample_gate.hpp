#pragma once

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>

namespace auv
{

class DepthSampleGate
{
public:
  enum class Decision
  {
    ACCEPT,
    REJECT,
    HOLD_FOR_RECOVERY
  };

  struct Result
  {
    Decision decision{Decision::REJECT};
    double delta_m{0.0};
    double allowed_delta_m{0.0};
    std::size_t consecutive_good_samples{0};
    bool reacquired{false};
  };

  DepthSampleGate(
    double max_vertical_speed_mps,
    double jump_tolerance_m,
    double max_rate_dt_s,
    std::size_t reacquire_good_samples)
  : max_vertical_speed_mps_(std::max(0.0, max_vertical_speed_mps)),
    jump_tolerance_m_(std::max(0.0, jump_tolerance_m)),
    max_rate_dt_s_(std::max(0.001, max_rate_dt_s)),
    reacquire_good_samples_(std::max<std::size_t>(1, reacquire_good_samples))
  {
  }

  Result update(double depth_m, std::int64_t sample_time_ns)
  {
    Result result;

    if (!std::isfinite(depth_m) || sample_time_ns <= 0) {
      enter_recovery();
      return result;
    }

    if (!have_accepted_sample_) {
      accept(depth_m, sample_time_ns);
      result.decision = Decision::ACCEPT;
      return result;
    }

    if (sample_time_ns <= last_observed_time_ns_) {
      enter_recovery();
      result.delta_m = std::abs(depth_m - last_accepted_depth_m_);
      return result;
    }

    const double raw_dt_s =
      static_cast<double>(sample_time_ns - last_observed_time_ns_) * 1.0e-9;
    last_observed_time_ns_ = sample_time_ns;

    const double accepted_dt_s =
      static_cast<double>(sample_time_ns - last_accepted_time_ns_) * 1.0e-9;
    result.delta_m = std::abs(depth_m - last_accepted_depth_m_);
    result.allowed_delta_m =
      allowed_delta(std::min(accepted_dt_s, max_rate_dt_s_));

    bool plausible = accepted_dt_s > 0.0 &&
      result.delta_m <= result.allowed_delta_m;

    if (recovering_ && have_recovery_candidate_) {
      const double candidate_delta_m = std::abs(depth_m - recovery_candidate_depth_m_);
      const double candidate_allowed_delta_m =
        allowed_delta(std::min(raw_dt_s, max_rate_dt_s_));
      plausible = plausible && candidate_delta_m <= candidate_allowed_delta_m;
    }

    if (!plausible) {
      enter_recovery();
      return result;
    }

    if (recovering_) {
      ++consecutive_good_samples_;
      recovery_candidate_depth_m_ = depth_m;
      have_recovery_candidate_ = true;
      result.consecutive_good_samples = consecutive_good_samples_;

      if (consecutive_good_samples_ < reacquire_good_samples_) {
        result.decision = Decision::HOLD_FOR_RECOVERY;
        return result;
      }

      recovering_ = false;
      consecutive_good_samples_ = 0;
      have_recovery_candidate_ = false;
      result.reacquired = true;
    }

    accept(depth_m, sample_time_ns);
    result.decision = Decision::ACCEPT;
    return result;
  }

  bool has_accepted_sample() const
  {
    return have_accepted_sample_;
  }

  double last_accepted_depth_m() const
  {
    return last_accepted_depth_m_;
  }

  std::size_t reacquire_good_samples() const
  {
    return reacquire_good_samples_;
  }

private:
  double allowed_delta(double dt_s) const
  {
    return jump_tolerance_m_ + max_vertical_speed_mps_ * std::max(0.0, dt_s);
  }

  void accept(double depth_m, std::int64_t sample_time_ns)
  {
    have_accepted_sample_ = true;
    last_accepted_depth_m_ = depth_m;
    last_accepted_time_ns_ = sample_time_ns;
    last_observed_time_ns_ = sample_time_ns;
  }

  void enter_recovery()
  {
    if (have_accepted_sample_) {
      recovering_ = true;
    }
    consecutive_good_samples_ = 0;
    have_recovery_candidate_ = false;
  }

  double max_vertical_speed_mps_;
  double jump_tolerance_m_;
  double max_rate_dt_s_;
  std::size_t reacquire_good_samples_;
  bool have_accepted_sample_{false};
  bool recovering_{false};
  bool have_recovery_candidate_{false};
  std::size_t consecutive_good_samples_{0};
  double last_accepted_depth_m_{0.0};
  double recovery_candidate_depth_m_{0.0};
  std::int64_t last_accepted_time_ns_{0};
  std::int64_t last_observed_time_ns_{0};
};

}  // namespace auv
