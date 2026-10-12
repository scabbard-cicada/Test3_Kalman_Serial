// Copyright 2026 Target Tracker Exam
//
// 跟踪节点：订阅极坐标观测，跑 EKF，并以固定频率给云台发指令。
//
//   订阅  target/measurement   PolarMeasurement   50 Hz（有断流）
//   发布  target/filtered      PointStamped       滤波后位置，每帧观测一次
//         target/nis           Float64            每帧 NIS
//         gimbal/cmd           GimbalCmd          云台指令，cmd_rate（默认 100 Hz）
//
// 两条路径分开：
//   - 观测到达时：predict + update，发布 filtered / nis
//   - 定时器触发时：从"上次更新时刻"外推到"现在 + predict_dt"，换算成云台角和角速度发出去
// 云台指令的频率因此和观测频率无关；短暂断流时先 coast、再 HOLD，
// 一个回放周期结束后的长断流则命令云台归零。
//
// 所有参数在 config/params.yaml，改完重启即可，不用重新编译。

#include <chrono>
#include <cmath>
#include <memory>
#include <string>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <std_msgs/msg/float64.hpp>

#include "target_tracker/gimbal_command.hpp"
#include "target_tracker/kalman_filter.hpp"
#include "interfaces/msg/gimbal_cmd.hpp"
#include "interfaces/msg/polar_measurement.hpp"

namespace target_tracker
{

using interfaces::msg::GimbalCmd;
using interfaces::msg::PolarMeasurement;

class TrackerNode : public rclcpp::Node
{
public:
  explicit TrackerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("tracker_node", options)
  {
    // ---- 滤波参数 ----
    sigma_jerk_ = declare_parameter<double>("sigma_jerk", 2.0);
    sigma_range_ = declare_parameter<double>("sigma_range", 0.11);
    sigma_bearing_ = declare_parameter<double>("sigma_bearing", 0.0035);
    p0_vel_ = declare_parameter<double>("p0_vel", 3.0);
    p0_acc_ = declare_parameter<double>("p0_acc", 5.0);

    // ---- 离群点门控 ----
    gate_threshold_ = declare_parameter<double>("gate_threshold", 13.8);
    const auto policy = declare_parameter<std::string>("outlier_policy", "reject");
    inflate_factor_ = declare_parameter<double>("inflate_factor", 50.0);
    wrap_innovation_ = declare_parameter<bool>("wrap_innovation", true);

    // ---- 预测与云台 ----
    predict_dt_ = declare_parameter<double>("predict_dt", 0.08);
    cmd_rate_ = declare_parameter<double>("cmd_rate", 100.0);
    max_coast_ = declare_parameter<double>("max_coast", 0.4);
    home_delay_ = declare_parameter<double>("home_delay", 1.0);
    gimbal_heading_ = declare_parameter<double>("gimbal_heading", M_PI);
    use_feedforward_ = declare_parameter<bool>("use_feedforward", true);
    laser_on_ = declare_parameter<bool>("laser_on", false);

    // ---- 其它 ----
    timeout_reset_ = declare_parameter<double>("timeout_reset", 0.5);
    frame_id_ = declare_parameter<std::string>("frame_id", "map");

    kf_.setProcessNoise(sigma_jerk_);
    kf_.setMeasurementNoise(sigma_range_, sigma_bearing_);
    kf_.setGateThreshold(gate_threshold_);
    kf_.setInflateFactor(inflate_factor_);
    kf_.setWrapInnovation(wrap_innovation_);
    kf_.setOutlierPolicy(
      policy == "inflate" ? KalmanFilterCA::OutlierPolicy::kInflate :
      KalmanFilterCA::OutlierPolicy::kReject);

    sub_ = create_subscription<PolarMeasurement>(
      "target/measurement", 20,
      std::bind(&TrackerNode::onMeasurement, this, std::placeholders::_1));

    pub_filtered_ = create_publisher<geometry_msgs::msg::PointStamped>("target/filtered", 20);
    pub_nis_ = create_publisher<std_msgs::msg::Float64>("target/nis", 20);
    pub_cmd_ = create_publisher<GimbalCmd>("gimbal/cmd", rclcpp::QoS(10));

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, cmd_rate_));
    cmd_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&TrackerNode::onCmdTimer, this));

    RCLCPP_INFO(
      get_logger(),
      "EKF tracker | sigma_jerk=%.2f sigma_range=%.3f sigma_bearing=%.4f gate=%.1f policy=%s "
      "wrap=%s | predict_dt=%.3f cmd_rate=%.0f heading=%.1fdeg ff=%s",
      sigma_jerk_, sigma_range_, sigma_bearing_, gate_threshold_, policy.c_str(),
      wrap_innovation_ ? "on" : "OFF", predict_dt_, cmd_rate_, gimbal_heading_ * 180.0 / M_PI,
      use_feedforward_ ? "on" : "off");
    if (!wrap_innovation_) {
      RCLCPP_WARN(get_logger(), "wrap_innovation is OFF: 仅用于复现角度环绕错误，正常运行必须打开");
    }
  }

private:
  // ------------------------------------------------------------------ 观测
  void onMeasurement(const PolarMeasurement::SharedPtr msg)
  {
    const rclcpp::Time stamp(msg->header.stamp);
    KalmanFilterCA::Vec2 z;
    z << msg->range, msg->bearing;
    kf_.setStation(KalmanFilterCA::Vec2(msg->station_x, msg->station_y));

    if (!std::isfinite(z(0)) || !std::isfinite(z(1)) || z(0) <= 0.0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "invalid measurement, skipped");
      return;
    }

    // TODO: 完成一帧观测的 EKF 生命周期。
    // - 首帧调用 initialize()，不要直接 predict / update。
    // - 用本帧与 last_stamp_ 的差得到 dt；dt 非正时跳过。
    // - dt 超过 timeout_reset_ 时重新初始化，避免把上一周期状态带入下一周期。
    // - 正常帧按 predict(dt) 再 update(z) 的顺序处理。
    // - 更新 last_stamp_、last_outlier_、total_、rejected_，并调用 publishEstimate(stamp)。
    (void)stamp;
  }

  void initialize(const KalmanFilterCA::Vec2 & z, const rclcpp::Time & stamp)
  {
    kf_.init(z, p0_vel_, p0_acc_);
    last_stamp_ = stamp;
    last_outlier_ = false;
    publishEstimate(stamp);
  }

  void publishEstimate(const rclcpp::Time & stamp)
  {
    const KalmanFilterCA::Vec2 f = kf_.position();

    geometry_msgs::msg::PointStamped m;
    m.header.stamp = stamp;
    m.header.frame_id = frame_id_;
    m.point.x = f(0);
    m.point.y = f(1);
    pub_filtered_->publish(m);

    std_msgs::msg::Float64 nis;
    nis.data = kf_.lastNis();
    pub_nis_->publish(nis);
  }

  // ------------------------------------------------------------------ 云台指令
  void onCmdTimer()
  {
    GimbalCmd cmd;
    cmd.stamp = now();
    // 距离上次更新已经过去多久。观测时间戳和本地时钟同源（回放节点用 now() 作起点）
    const double age = (rclcpp::Time(cmd.stamp) - last_stamp_).seconds();
    // TODO: 根据当前滤波状态与观测新鲜度选择命令阶段。
    (void)age;
    const CommandPhase phase = CommandPhase::kRelax;

    switch (phase) {
      case CommandPhase::kRelax:
        // TODO: 填 RELAX 命令。电机应卸力，target_valid 应为 false。
        break;

      case CommandPhase::kHold:
        // TODO: 填 HOLD 命令，保持 last_yaw_。
        break;

      case CommandPhase::kReturnHome:
        // TODO: 填归零命令。使用 TRACK 驱动到 yaw=0，且不把它标成有效目标。
        break;

      case CommandPhase::kTrack: {
        // TODO: 前推到 age + predict_dt_，调用 makeGimbalReference()，并填 TRACK 命令。
        // 同时传递 target_valid、laser_on、outlier，并更新 last_yaw_。
        break;
      }
    }
    pub_cmd_->publish(cmd);
  }

  KalmanFilterCA kf_;

  double sigma_jerk_, sigma_range_, sigma_bearing_, p0_vel_, p0_acc_;
  double gate_threshold_, inflate_factor_;
  bool wrap_innovation_;
  double predict_dt_, cmd_rate_, max_coast_, home_delay_, gimbal_heading_;
  bool use_feedforward_, laser_on_;
  double timeout_reset_;
  std::string frame_id_;

  rclcpp::Time last_stamp_{0, 0, RCL_ROS_TIME};
  bool last_outlier_{false};
  float last_yaw_{0.0f};
  size_t total_{0};
  size_t rejected_{0};

  rclcpp::Subscription<PolarMeasurement>::SharedPtr sub_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr pub_filtered_;
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pub_nis_;
  rclcpp::Publisher<GimbalCmd>::SharedPtr pub_cmd_;
  rclcpp::TimerBase::SharedPtr cmd_timer_;
};

}  // namespace target_tracker

RCLCPP_COMPONENTS_REGISTER_NODE(target_tracker::TrackerNode)
