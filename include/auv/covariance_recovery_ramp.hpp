#pragma once

#include <algorithm>

namespace auv
{

class CovarianceRecoveryRamp
{
public:
  CovarianceRecoveryRamp(double initial_variance, double variance_decay, int sample_count)
  : initial_variance_(std::max(0.0, initial_variance)),
    variance_decay_(std::clamp(variance_decay, 0.0, 1.0)),
    sample_count_(std::max(0, sample_count))
  {
  }

  void trigger()
  {
    stage_ = sample_count_ > 0 ? 1 : 0;
    current_variance_ = initial_variance_;
  }

  bool active() const
  {
    return stage_ != 0;
  }

  int stage() const
  {
    return stage_;
  }

  double apply(double sensor_variance) const
  {
    if (active()) {
      return std::max(sensor_variance, current_variance_);
    }
    return sensor_variance;
  }

  void advance()
  {
    if (!active()) {
      return;
    }
    if (stage_ >= sample_count_) {
      stage_ = 0;
      return;
    }
    stage_++;
    current_variance_ *= variance_decay_;
  }

private:
  double initial_variance_;
  double variance_decay_;
  int sample_count_;
  double current_variance_{0.0};
  int stage_{0};
};

}  // namespace auv
