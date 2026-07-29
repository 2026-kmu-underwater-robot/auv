#include <gtest/gtest.h>

#include "auv/velocity_sample_gate.hpp"

namespace
{

constexpr std::int64_t kOneSecondNs = 1000000000LL;

TEST(VelocitySampleGate, AcceptsNormalAcceleration)
{
  auv::VelocitySampleGate gate(0.8, 1.0, 0.03, 0.5);

  EXPECT_EQ(
    gate.update(0.0, 0.0, 0.0, kOneSecondNs).decision,
    auv::VelocitySampleGate::Decision::ACCEPT);
  EXPECT_EQ(
    gate.update(0.10, 0.0, 0.0, kOneSecondNs + 100000000LL).decision,
    auv::VelocitySampleGate::Decision::ACCEPT);
}

TEST(VelocitySampleGate, RejectsSuddenVelocityChange)
{
  auv::VelocitySampleGate gate(0.8, 1.0, 0.03, 0.5);

  EXPECT_EQ(
    gate.update(0.0, 0.0, 0.0, kOneSecondNs).decision,
    auv::VelocitySampleGate::Decision::ACCEPT);
  const auto result =
    gate.update(0.20, 0.20, 0.0, kOneSecondNs + 100000000LL);
  EXPECT_EQ(result.decision, auv::VelocitySampleGate::Decision::REJECT_RATE);
  EXPECT_NEAR(result.velocity_delta_mps, 0.2828427, 1.0e-6);
  EXPECT_NEAR(result.allowed_delta_mps, 0.13, 1.0e-9);
}

TEST(VelocitySampleGate, NextNormalSamplePassesWithoutRecoveryDelay)
{
  auv::VelocitySampleGate gate(0.8, 1.0, 0.03, 0.5);

  EXPECT_EQ(
    gate.update(0.0, 0.0, 0.0, kOneSecondNs).decision,
    auv::VelocitySampleGate::Decision::ACCEPT);
  EXPECT_EQ(
    gate.update(0.20, 0.20, 0.0, kOneSecondNs + 100000000LL).decision,
    auv::VelocitySampleGate::Decision::REJECT_RATE);
  EXPECT_EQ(
    gate.update(0.01, 0.0, 0.0, kOneSecondNs + 200000000LL).decision,
    auv::VelocitySampleGate::Decision::ACCEPT);
}

TEST(VelocitySampleGate, RejectsExcessiveAbsoluteSpeed)
{
  auv::VelocitySampleGate gate(0.8, 1.0, 0.03, 0.5);

  const auto result = gate.update(0.81, 0.0, 0.0, kOneSecondNs);
  EXPECT_EQ(result.decision, auv::VelocitySampleGate::Decision::REJECT_SPEED);
}

TEST(VelocitySampleGate, RejectedSampleDoesNotMoveAcceptedReference)
{
  auv::VelocitySampleGate gate(0.8, 1.0, 0.03, 0.5);

  EXPECT_EQ(
    gate.update(0.0, 0.0, 0.0, kOneSecondNs).decision,
    auv::VelocitySampleGate::Decision::ACCEPT);
  EXPECT_EQ(
    gate.update(0.20, 0.20, 0.0, kOneSecondNs + 100000000LL).decision,
    auv::VelocitySampleGate::Decision::REJECT_RATE);

  const auto second_bad =
    gate.update(0.20, 0.20, 0.0, kOneSecondNs + 200000000LL);
  EXPECT_EQ(second_bad.decision, auv::VelocitySampleGate::Decision::REJECT_RATE);
  EXPECT_NEAR(second_bad.velocity_delta_mps, 0.2828427, 1.0e-6);
}

}  // namespace
