// Copyright 2026 Target Tracker Exam
//
// ============================================================================
//  CA 运动模型 + 极坐标观测的 EKF。
//
//  状态索引布局（x、y 两轴交错排列）：
//      0 = px    1 = py
//      2 = vx    3 = vy
//      4 = ax    5 = ay
//
//  本文件的 TODO 是本题 EKF 核心。
// ============================================================================

#include "target_tracker/kalman_filter.hpp"

#include <cmath>

namespace target_tracker
{

KalmanFilterCA::KalmanFilterCA()
{
  x_.setZero();
  P_.setIdentity();
  R_ = Mat2::Zero();
  R_(0, 0) = 0.10 * 0.10;
  R_(1, 1) = 0.003 * 0.003;
}

double KalmanFilterCA::wrapAngle(double a)
{
  // std::remainder 的结果落在 [-π, π]，比 fmod 少一次判断
  return std::remainder(a, 2.0 * M_PI);
}

void KalmanFilterCA::setMeasurementNoise(double sigma_range, double sigma_bearing)
{
  R_.setZero();
  R_(0, 0) = sigma_range * sigma_range;
  R_(1, 1) = sigma_bearing * sigma_bearing;
}

// ---------------------------------------------------------------- 观测模型
KalmanFilterCA::Vec2 KalmanFilterCA::h(const Vec6 & x, const Vec2 & s)
{
  // TODO: 实现极坐标观测模型。
  (void)x;
  (void)s;
  return Vec2::Zero();
}

KalmanFilterCA::Mat26 KalmanFilterCA::jacobianH(const Vec6 & x, const Vec2 & s)
{
  // TODO: 实现观测模型的雅可比。
  (void)x;
  (void)s;
  return Mat26::Zero();
}

// ---------------------------------------------------------------- 初始化
void KalmanFilterCA::init(const Vec2 & z, double p0_vel, double p0_acc)
{
  // TODO: 用首帧极坐标观测初始化状态与协方差。
  (void)z;
  (void)p0_vel;
  (void)p0_acc;
  x_.setZero();
  P_.setIdentity();
  last_nis_ = 0.0;
  initialized_ = true;
}

// ---------------------------------------------------------------- 运动模型
KalmanFilterCA::Mat6 KalmanFilterCA::makeF(double dt)
{
  // TODO: 实现 CA 状态转移矩阵。
  (void)dt;
  return Mat6::Identity();
}

KalmanFilterCA::Mat6 KalmanFilterCA::makeQ(double dt, double sigma_jerk)
{
  // TODO: 实现 jerk 过程噪声协方差。
  (void)dt;
  (void)sigma_jerk;
  return Mat6::Zero();
}

void KalmanFilterCA::predict(double dt)
{
  if (!initialized_ || !(dt > 0.0)) {
    return;
  }
  // TODO: 实现 EKF 预测步。
}

// ---------------------------------------------------------------- 更新
bool KalmanFilterCA::update(const Vec2 & z)
{
  if (!initialized_) {
    return false;
  }
  // TODO: 实现 EKF 更新、角度处理与门控。
  (void)z;
  last_nis_ = 0.0;
  return false;
}

// ---------------------------------------------------------------- 外推
KalmanFilterCA::Vec6 KalmanFilterCA::predictStateAhead(double dt) const
{
  return makeF(dt) * x_;
}

}  // namespace target_tracker
