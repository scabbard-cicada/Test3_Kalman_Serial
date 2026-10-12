// Copyright 2026 Target Tracker Exam
#ifndef TARGET_TRACKER__GIMBAL_COMMAND_HPP_
#define TARGET_TRACKER__GIMBAL_COMMAND_HPP_

#include "target_tracker/kalman_filter.hpp"

namespace target_tracker
{

/// tracker_node 根据观测新鲜度选择的云台命令阶段。
enum class CommandPhase
{
  kRelax,       // 还没有可用滤波状态，电机卸力
  kTrack,       // 有新鲜观测，跟踪预测目标
  kHold,        // 短暂断流，保持最后角度
  kReturnHome,  // 一个回放周期结束，缓慢回到 yaw = 0
};

/// 未来目标状态换算出的云台参考。
struct GimbalReference
{
  double yaw{0.0};      // rad，云台系
  double yaw_vel{0.0};  // rad/s，角速度前馈
};

/// 将未来状态换算为云台系目标 yaw 和角速度前馈。
/// future_state = [px, py, vx, vy, ax, ay]，位置/速度均在世界系。
/// station 是云台安装站点在世界系的位置；gimbal_heading 是 yaw=0 时云台的世界朝向。
GimbalReference makeGimbalReference(
  const KalmanFilterCA::Vec6 & future_state,
  const KalmanFilterCA::Vec2 & station,
  double gimbal_heading,
  bool use_feedforward);

/// 根据滤波状态与距离上次有效观测的时间，选择云台命令阶段。
/// max_coast 内继续预测跟踪；之后先 HOLD；超过 home_delay 后回零。
CommandPhase chooseCommandPhase(
  bool filter_initialized,
  double observation_age,
  double max_coast,
  double home_delay);

}  // namespace target_tracker

#endif  // TARGET_TRACKER__GIMBAL_COMMAND_HPP_
