#include <cmath>

#include <gtest/gtest.h>

#include "auv/covariance_recovery_ramp.hpp"

TEST(CovarianceRecoveryRamp, LeavesNormalCovarianceUnchanged)
{
  auv::CovarianceRecoveryRamp ramp(1.0, 0.1, 6);

  EXPECT_FALSE(ramp.active());
  EXPECT_DOUBLE_EQ(ramp.apply(4.0e-8), 4.0e-8);
}

TEST(CovarianceRecoveryRamp, AppliesGeometricRecovery)
{
  auv::CovarianceRecoveryRamp ramp(1.0, 0.1, 6);
  ramp.trigger();

  const double expected_variances[] = {1.0, 0.1, 0.01, 0.001, 0.0001, 0.00001};
  for (int stage = 1; stage <= 6; ++stage) {
    EXPECT_EQ(ramp.stage(), stage);
    EXPECT_NEAR(ramp.apply(4.0e-8), expected_variances[stage - 1], 1.0e-12);
    ramp.advance();
  }

  EXPECT_FALSE(ramp.active());
  EXPECT_DOUBLE_EQ(ramp.apply(4.0e-8), 4.0e-8);
}

TEST(CovarianceRecoveryRamp, RejectionRestartsFirstStage)
{
  auv::CovarianceRecoveryRamp ramp(1.0, 0.1, 6);
  ramp.trigger();
  ramp.advance();
  EXPECT_EQ(ramp.stage(), 2);

  ramp.trigger();
  EXPECT_EQ(ramp.stage(), 1);
  EXPECT_DOUBLE_EQ(ramp.apply(4.0e-8), 1.0);
}

TEST(CovarianceRecoveryRamp, DoesNotReduceSensorVariance)
{
  auv::CovarianceRecoveryRamp ramp(1.0, 0.1, 6);
  ramp.trigger();

  EXPECT_DOUBLE_EQ(ramp.apply(2.0), 2.0);
}

TEST(CovarianceRecoveryRamp, ZeroSamplesDisablesRecovery)
{
  auv::CovarianceRecoveryRamp ramp(1.0, 0.1, 0);
  ramp.trigger();

  EXPECT_FALSE(ramp.active());
  EXPECT_DOUBLE_EQ(ramp.apply(4.0e-8), 4.0e-8);
}

TEST(CovarianceRecoveryRamp, AppliesTunedLongRecovery)
{
  auv::CovarianceRecoveryRamp ramp(1.0, 0.8, 56);
  ramp.trigger();

  for (int stage = 1; stage <= 56; ++stage) {
    EXPECT_EQ(ramp.stage(), stage);
    EXPECT_NEAR(ramp.apply(4.0e-8), std::pow(0.8, stage - 1), 1.0e-12);
    ramp.advance();
  }

  EXPECT_FALSE(ramp.active());
  EXPECT_DOUBLE_EQ(ramp.apply(4.0e-8), 4.0e-8);
}
