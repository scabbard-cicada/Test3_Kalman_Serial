// Copyright 2026 Target Tracker Exam

#include "target_tracker/gimbal_command.hpp"

#include <cmath>

namespace target_tracker
{

GimbalReference makeGimbalReference(
  const KalmanFilterCA::Vec6 & future_state,
  const KalmanFilterCA::Vec2 & station,
  double gimbal_heading,
  bool use_feedforward)
{
  // TODO: 由未来状态生成云台 yaw 与角速度前馈。
  (void)future_state;
  (void)station;
  (void)gimbal_heading;
  (void)use_feedforward;
  return {};
}

CommandPhase chooseCommandPhase(
  bool filter_initialized,
  double observation_age,
  double max_coast,
  double home_delay)
{
  // TODO: 根据滤波状态和观测新鲜度选择命令阶段。
  (void)filter_initialized;
  (void)observation_age;
  (void)max_coast;
  (void)home_delay;
  return CommandPhase::kRelax;
}

}  // namespace target_tracker
