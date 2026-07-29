#include <gtest/gtest.h>

#include "auv/depth_sample_gate.hpp"

namespace
{

constexpr std::int64_t kOneSecondNs = 1000000000LL;

TEST(DepthSampleGate, AcceptsNormalMotion)
{
  auv::DepthSampleGate gate(2.0, 0.10, 0.5, 5);

  EXPECT_EQ(
    gate.update(5.0, kOneSecondNs).decision,
    auv::DepthSampleGate::Decision::ACCEPT);
  EXPECT_EQ(
    gate.update(5.2, kOneSecondNs + 100000000LL).decision,
    auv::DepthSampleGate::Decision::ACCEPT);
}

TEST(DepthSampleGate, RejectsJumpAndRequiresThreeGoodSamples)
{
  auv::DepthSampleGate gate(2.0, 0.10, 0.5, 3);

  EXPECT_EQ(
    gate.update(5.0, kOneSecondNs).decision,
    auv::DepthSampleGate::Decision::ACCEPT);
  EXPECT_EQ(
    gate.update(25.0, kOneSecondNs + 100000000LL).decision,
    auv::DepthSampleGate::Decision::REJECT);

  for (std::int64_t index = 1; index <= 2; ++index) {
    const auto result = gate.update(
      5.0 + 0.02 * static_cast<double>(index),
      kOneSecondNs + (index + 1) * 100000000LL);
    EXPECT_EQ(result.decision, auv::DepthSampleGate::Decision::HOLD_FOR_RECOVERY);
    EXPECT_EQ(result.consecutive_good_samples, static_cast<std::size_t>(index));
  }

  const auto reacquired = gate.update(5.06, kOneSecondNs + 400000000LL);
  EXPECT_EQ(reacquired.decision, auv::DepthSampleGate::Decision::ACCEPT);
  EXPECT_TRUE(reacquired.reacquired);
  EXPECT_DOUBLE_EQ(gate.last_accepted_depth_m(), 5.06);
}

TEST(DepthSampleGate, BadRecoverySampleRestartsCounter)
{
  auv::DepthSampleGate gate(2.0, 0.10, 0.5, 3);

  EXPECT_EQ(
    gate.update(5.0, kOneSecondNs).decision,
    auv::DepthSampleGate::Decision::ACCEPT);
  EXPECT_EQ(
    gate.update(20.0, kOneSecondNs + 100000000LL).decision,
    auv::DepthSampleGate::Decision::REJECT);
  EXPECT_EQ(
    gate.update(5.1, kOneSecondNs + 200000000LL).decision,
    auv::DepthSampleGate::Decision::HOLD_FOR_RECOVERY);
  EXPECT_EQ(
    gate.update(20.0, kOneSecondNs + 300000000LL).decision,
    auv::DepthSampleGate::Decision::REJECT);

  EXPECT_EQ(
    gate.update(5.1, kOneSecondNs + 400000000LL).consecutive_good_samples,
    1U);
}

TEST(DepthSampleGate, PersistentStepCannotBecomeValidAsTimePasses)
{
  auv::DepthSampleGate gate(2.0, 0.10, 0.5, 5);

  EXPECT_EQ(
    gate.update(5.0, kOneSecondNs).decision,
    auv::DepthSampleGate::Decision::ACCEPT);

  for (std::int64_t index = 1; index <= 100; ++index) {
    EXPECT_EQ(
      gate.update(25.0, kOneSecondNs + index * 100000000LL).decision,
      auv::DepthSampleGate::Decision::REJECT);
  }
  EXPECT_DOUBLE_EQ(gate.last_accepted_depth_m(), 5.0);
}

TEST(DepthSampleGate, RejectsNonMonotonicTimestamp)
{
  auv::DepthSampleGate gate(2.0, 0.10, 0.5, 2);

  EXPECT_EQ(
    gate.update(5.0, kOneSecondNs).decision,
    auv::DepthSampleGate::Decision::ACCEPT);
  EXPECT_EQ(
    gate.update(5.0, kOneSecondNs).decision,
    auv::DepthSampleGate::Decision::REJECT);
}

}  // namespace
