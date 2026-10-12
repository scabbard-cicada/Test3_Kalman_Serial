// Copyright 2026 Target Tracker Exam
//
// EKF 单元测试：colcon test --packages-select target_tracker
//   colcon test-result --verbose

#include <gtest/gtest.h>

#include <cmath>

#include "target_tracker/kalman_filter.hpp"

using target_tracker::KalmanFilterCA;
using Vec2 = KalmanFilterCA::Vec2;
using Vec6 = KalmanFilterCA::Vec6;
using Mat6 = KalmanFilterCA::Mat6;
using Mat26 = KalmanFilterCA::Mat26;

namespace
{
Vec6 state(double px, double py, double vx = 0, double vy = 0, double ax = 0, double ay = 0)
{
  Vec6 x;
  x << px, py, vx, vy, ax, ay;
  return x;
}
}  // namespace

TEST(Ekf, MakeFMatchesKinematics)
{
  const double dt = 0.1;
  const Vec6 x = state(1, 2, 3, 4, 5, 6);
  const Vec6 y = KalmanFilterCA::makeF(dt) * x;
  EXPECT_NEAR(y(0), 1 + 3 * dt + 0.5 * 5 * dt * dt, 1e-12);
  EXPECT_NEAR(y(1), 2 + 4 * dt + 0.5 * 6 * dt * dt, 1e-12);
  EXPECT_NEAR(y(2), 3 + 5 * dt, 1e-12);
  EXPECT_NEAR(y(3), 4 + 6 * dt, 1e-12);
  EXPECT_NEAR(y(4), 5, 1e-12);
  EXPECT_NEAR(y(5), 6, 1e-12);
}

TEST(Ekf, MakeQIsSymmetricPsdAndVanishesAtZero)
{
  EXPECT_TRUE(KalmanFilterCA::makeQ(0.0, 5.0).isZero(1e-15));
  const Mat6 Q = KalmanFilterCA::makeQ(0.02, 5.0);
  EXPECT_TRUE(Q.isApprox(Q.transpose()));
  Eigen::SelfAdjointEigenSolver<Mat6> es(Q);
  EXPECT_GE(es.eigenvalues().minCoeff(), -1e-18);
  // x、y 两轴无相关
  EXPECT_EQ(Q(0, 1), 0.0);
  EXPECT_EQ(Q(2, 3), 0.0);
  // dt 的幂次：Q_pp ∝ dt^5，Q_aa ∝ dt
  EXPECT_NEAR(Q(0, 0), 25.0 * std::pow(0.02, 5) / 20.0, 1e-18);
  EXPECT_NEAR(Q(4, 4), 25.0 * 0.02, 1e-12);
}

TEST(Ekf, ObservationModel)
{
  const Vec2 s(10, 0);
  const Vec2 z = KalmanFilterCA::h(state(7, 4), s);  // dx=-3, dy=4
  EXPECT_NEAR(z(0), 5.0, 1e-12);
  EXPECT_NEAR(z(1), std::atan2(4.0, -3.0), 1e-12);
}

TEST(Ekf, JacobianMatchesNumericalDerivative)
{
  const Vec2 s(10, 0);
  for (const auto & x : {state(7, 4, 1, 2, 3, 4), state(-3, -2), state(9.5, 0.3), state(2, -6)}) {
    const Mat26 H = KalmanFilterCA::jacobianH(x, s);
    for (int j = 0; j < 6; ++j) {
      const double eps = 1e-6;
      Vec6 xp = x, xm = x;
      xp(j) += eps;
      xm(j) -= eps;
      Vec2 d = KalmanFilterCA::h(xp, s) - KalmanFilterCA::h(xm, s);
      d(1) = KalmanFilterCA::wrapAngle(d(1));
      d /= 2 * eps;
      EXPECT_NEAR(H(0, j), d(0), 1e-6) << "row r, col " << j;
      EXPECT_NEAR(H(1, j), d(1), 1e-6) << "row theta, col " << j;
    }
    // 速度、加速度不出现在观测里
    EXPECT_TRUE(H.rightCols<4>().isZero());
  }
}

TEST(Ekf, WrapAngle)
{
  EXPECT_NEAR(KalmanFilterCA::wrapAngle(3 * M_PI / 2), -M_PI / 2, 1e-12);
  EXPECT_NEAR(KalmanFilterCA::wrapAngle(-3 * M_PI / 2), M_PI / 2, 1e-12);
  EXPECT_NEAR(KalmanFilterCA::wrapAngle(6.26), 6.26 - 2 * M_PI, 1e-12);
  EXPECT_LE(std::fabs(KalmanFilterCA::wrapAngle(M_PI)), M_PI + 1e-12);
}

TEST(Ekf, InitConvertsPolarToCartesianWithShapedCovariance)
{
  KalmanFilterCA kf;
  kf.setStation(Vec2(10, 0));
  kf.setMeasurementNoise(0.1, 0.003);
  kf.init(Vec2(8.0, M_PI), 3.0, 5.0);  // 正西 8 m
  EXPECT_NEAR(kf.state()(0), 2.0, 1e-9);
  EXPECT_NEAR(kf.state()(1), 0.0, 1e-9);
  // 视线沿 x：x 方向方差 = σ_r²，y 方向方差 = (r σ_θ)²
  EXPECT_NEAR(kf.covariance()(0, 0), 0.01, 1e-9);
  EXPECT_NEAR(kf.covariance()(1, 1), std::pow(8.0 * 0.003, 2), 1e-9);
  EXPECT_NEAR(kf.covariance()(2, 2), 9.0, 1e-12);
  EXPECT_NEAR(kf.covariance()(4, 4), 25.0, 1e-12);
}

TEST(Ekf, InnovationIsWrappedAcrossPi)
{
  // 目标在站点正西方，预测方位 ≈ -π + 0.001，观测 = +π - 0.001：真实差 -0.002 rad
  KalmanFilterCA kf;
  kf.setStation(Vec2(10, 0));
  kf.setMeasurementNoise(0.1, 0.003);
  kf.setGateThreshold(13.8);
  kf.init(Vec2(8.0, -M_PI + 0.001), 1.0, 1.0);
  const bool ok = kf.update(Vec2(8.0, M_PI - 0.001));
  EXPECT_TRUE(ok);
  EXPECT_LT(kf.lastNis(), 1.0);
  EXPECT_NEAR(kf.state()(0), 2.0, 0.05);

  // 关掉 wrap：同一帧 NIS 爆炸，被门控丢弃
  KalmanFilterCA bad;
  bad.setStation(Vec2(10, 0));
  bad.setMeasurementNoise(0.1, 0.003);
  bad.setWrapInnovation(false);
  bad.init(Vec2(8.0, -M_PI + 0.001), 1.0, 1.0);
  EXPECT_FALSE(bad.update(Vec2(8.0, M_PI - 0.001)));
  EXPECT_GT(bad.lastNis(), 1e5);
}

TEST(Ekf, GateRejectsOutlierAndKeepsState)
{
  KalmanFilterCA kf;
  kf.setStation(Vec2(10, 0));
  kf.setMeasurementNoise(0.1, 0.003);
  kf.init(Vec2(8.0, M_PI), 0.5, 0.5);
  const Vec6 before = kf.state();
  EXPECT_FALSE(kf.update(Vec2(9.5, M_PI)));  // 距离跳 1.5 m
  EXPECT_GT(kf.lastNis(), 13.8);
  EXPECT_TRUE(kf.state().isApprox(before));

  // inflate：仍然更新，但幅度很小
  kf.setOutlierPolicy(KalmanFilterCA::OutlierPolicy::kInflate);
  kf.setInflateFactor(50.0);
  EXPECT_FALSE(kf.update(Vec2(9.5, M_PI)));
  EXPECT_FALSE(kf.state().isApprox(before));
  EXPECT_LT(std::fabs(kf.state()(0) - before(0)), 0.5);
}

TEST(Ekf, SkipsUpdateNearStation)
{
  KalmanFilterCA kf;
  kf.setStation(Vec2(0, 0));
  kf.init(Vec2(0.05, 0.0), 1.0, 1.0);
  const Vec6 before = kf.state();
  EXPECT_FALSE(kf.update(Vec2(0.05, 0.0)));
  EXPECT_TRUE(kf.state().allFinite());
  EXPECT_TRUE(kf.state().isApprox(before));
}

TEST(Ekf, PredictStateAheadIsConst)
{
  KalmanFilterCA kf;
  kf.setStation(Vec2(10, 0));
  kf.init(Vec2(8.0, M_PI), 1.0, 1.0);
  kf.predict(0.02);
  const Vec6 before = kf.state();
  const Vec6 xs = kf.predictStateAhead(0.1);
  EXPECT_TRUE(kf.state() == before);
  EXPECT_TRUE(xs.allFinite());
}

TEST(Ekf, PredictIgnoresNonPositiveDt)
{
  KalmanFilterCA kf;
  kf.setStation(Vec2(10, 0));
  kf.init(Vec2(8.0, M_PI), 1.0, 1.0);
  const Vec6 x = kf.state();
  const Mat6 P = kf.covariance();
  kf.predict(0.0);
  kf.predict(-0.1);
  EXPECT_TRUE(kf.state() == x);
  EXPECT_TRUE(kf.covariance() == P);
}

TEST(Ekf, TracksConstantVelocityTargetAndStaysConsistent)
{
  // 目标匀速穿过 ±π 方向（站点正西），带噪声观测。检查收敛、协方差对称正定、NIS 均值合理
  KalmanFilterCA kf;
  const Vec2 s(10, 0);
  kf.setStation(s);
  kf.setProcessNoise(0.5);
  kf.setMeasurementNoise(0.1, 0.003);
  std::srand(7);
  const auto gauss = []() {
      double u = (std::rand() + 1.0) / (RAND_MAX + 2.0);
      double v = (std::rand() + 1.0) / (RAND_MAX + 2.0);
      return std::sqrt(-2 * std::log(u)) * std::cos(2 * M_PI * v);
    };
  double nis_sum = 0;
  int n = 0;
  for (int k = 0; k < 500; ++k) {
    const double t = k * 0.02;
    const double px = 2.0;
    const double py = -3.0 + 0.6 * t;  // 从 y=-3 走到 y=+3，方位角跨过 ±π
    Vec2 z = KalmanFilterCA::h(state(px, py), s);
    z(0) += 0.1 * gauss();
    z(1) = KalmanFilterCA::wrapAngle(z(1) + 0.003 * gauss());
    if (k == 0) {
      kf.init(z, 2.0, 2.0);
      continue;
    }
    kf.predict(0.02);
    kf.update(z);
    if (k > 50) {
      nis_sum += kf.lastNis();
      ++n;
    }
  }
  EXPECT_NEAR(kf.state()(0), 2.0, 0.1);
  EXPECT_NEAR(kf.state()(1), -3.0 + 0.6 * 499 * 0.02, 0.1);
  EXPECT_NEAR(kf.state()(3), 0.6, 0.1);
  const Mat6 P = kf.covariance();
  EXPECT_TRUE(P.isApprox(P.transpose()));
  Eigen::SelfAdjointEigenSolver<Mat6> es(P);
  EXPECT_GT(es.eigenvalues().minCoeff(), 0.0);
  // NIS 服从 χ²(2)，均值应接近 2
  EXPECT_NEAR(nis_sum / n, 2.0, 0.6);
}
