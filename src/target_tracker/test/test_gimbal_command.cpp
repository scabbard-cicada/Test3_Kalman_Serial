// 云台纯算法单元测试：不依赖 ROS 节点、串口或下位机。

#include <cmath>
#include <limits>

#include <gtest/gtest.h>

#include "target_tracker/gimbal_command.hpp"

namespace target_tracker
{
namespace
{

KalmanFilterCA::Vec6 state(double px, double py, double vx = 0.0, double vy = 0.0)
{
  KalmanFilterCA::Vec6 x = KalmanFilterCA::Vec6::Zero();
  x << px, py, vx, vy, 0.0, 0.0;
  return x;
}

constexpr double kPi = 3.14159265358979323846;

}  // namespace

TEST(GimbalCommand, ZeroYawPointsAlongHeading)
{
  // station 的 yaw=0 朝世界 -x；目标正好在前方。
  const auto ref = makeGimbalReference(state(5.0, 0.0), KalmanFilterCA::Vec2(10.0, 0.0), kPi, true);
  EXPECT_NEAR(ref.yaw, 0.0, 1e-12);
  EXPECT_NEAR(ref.yaw_vel, 0.0, 1e-12);
}

TEST(GimbalCommand, ConvertsWorldBearingToGimbalYaw)
{
  // 世界 +y 相对 heading=pi 是 -pi/2。
  const auto ref = makeGimbalReference(state(10.0, 2.0), KalmanFilterCA::Vec2(10.0, 0.0), kPi, true);
  EXPECT_NEAR(ref.yaw, -kPi / 2.0, 1e-12);
}

TEST(GimbalCommand, WrapsAcrossPiBoundary)
{
  const double bearing = -kPi + 0.1;
  const auto ref = makeGimbalReference(
    state(5.0 * std::cos(bearing), 5.0 * std::sin(bearing)),
    KalmanFilterCA::Vec2(0.0, 0.0), kPi, true);
  EXPECT_NEAR(ref.yaw, 0.1, 1e-12);
}

TEST(GimbalCommand, CalculatesAngularVelocityFeedforward)
{
  // 目标在 +x，向 +y 横向运动：角速度 = x*vy/r^2 = 5*2/25 = 0.4 rad/s。
  const auto ref = makeGimbalReference(
    state(5.0, 0.0, 0.0, 2.0), KalmanFilterCA::Vec2(0.0, 0.0), 0.0, true);
  EXPECT_NEAR(ref.yaw, 0.0, 1e-12);
  EXPECT_NEAR(ref.yaw_vel, 0.4, 1e-12);
}

TEST(GimbalCommand, CanDisableFeedforward)
{
  const auto ref = makeGimbalReference(
    state(5.0, 0.0, 0.0, 2.0), KalmanFilterCA::Vec2(0.0, 0.0), 0.0, false);
  EXPECT_NEAR(ref.yaw_vel, 0.0, 1e-12);
}

TEST(GimbalCommand, TargetAtStationGetsSafeZeroReference)
{
  const auto ref = makeGimbalReference(
    state(10.0, 0.0, 1.0, 2.0), KalmanFilterCA::Vec2(10.0, 0.0), kPi, true);
  EXPECT_NEAR(ref.yaw, 0.0, 1e-12);
  EXPECT_NEAR(ref.yaw_vel, 0.0, 1e-12);
}

TEST(GimbalCommand, CommandPhaseStateMachine)
{
  constexpr double max_coast = 0.4;
  constexpr double home_delay = 1.0;
  EXPECT_EQ(chooseCommandPhase(false, 0.0, max_coast, home_delay), CommandPhase::kRelax);
  EXPECT_EQ(
    chooseCommandPhase(true, std::numeric_limits<double>::quiet_NaN(), max_coast, home_delay),
    CommandPhase::kRelax);
  EXPECT_EQ(chooseCommandPhase(true, 0.0, max_coast, home_delay), CommandPhase::kTrack);
  EXPECT_EQ(chooseCommandPhase(true, max_coast, max_coast, home_delay), CommandPhase::kTrack);
  EXPECT_EQ(chooseCommandPhase(true, 0.5, max_coast, home_delay), CommandPhase::kHold);
  EXPECT_EQ(chooseCommandPhase(true, home_delay, max_coast, home_delay), CommandPhase::kHold);
  EXPECT_EQ(chooseCommandPhase(true, 1.01, max_coast, home_delay), CommandPhase::kReturnHome);
  EXPECT_EQ(chooseCommandPhase(true, -0.01, max_coast, home_delay), CommandPhase::kHold);
}

}  // namespace target_tracker
