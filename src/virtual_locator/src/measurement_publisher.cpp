// Copyright 2026 Target Tracker Exam
//
// virtual_locator：虚拟定位/观测节点。读 data/measurements.json，
// 按原始时间间隔发布极坐标观测，充当雷达/相机这一路"传感器"。
// 这是下发给考生的输入源，考生不需要修改本文件。
//
// 时间戳：header.stamp = 回放起点 + 数据里的 t。数据里有几段观测中断（遮挡），
// 中断期间不发消息，时间戳照样连续推进 —— 跟踪器看到的就是一段真实的"断流"。

#include <chrono>
#include <fstream>
#include <memory>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include "interfaces/msg/polar_measurement.hpp"

namespace virtual_locator
{

class MeasurementPublisher : public rclcpp::Node
{
public:
  explicit MeasurementPublisher(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("virtual_locator", options)
  {
    json_path_ = declare_parameter<std::string>("json_path", "");
    loop_ = declare_parameter<bool>("loop", false);
    loop_gap_ = declare_parameter<double>("loop_gap", 4.0);
    start_delay_ = declare_parameter<double>("start_delay", 1.0);
    frame_id_ = declare_parameter<std::string>("frame_id", "map");

    if (!load(json_path_)) {
      RCLCPP_FATAL(get_logger(), "failed to load measurements: %s", json_path_.c_str());
      throw std::runtime_error("measurements load failed");
    }

    pub_ = create_publisher<interfaces::msg::PolarMeasurement>("target/measurement", 20);

    // 1 ms 定时器检查是否到了下一帧的发布时刻；观测按 50 Hz 采样，1 ms 分辨率足够
    timer_ = create_wall_timer(std::chrono::milliseconds(1), [this]() {onTimer();});

    RCLCPP_INFO(
      get_logger(),
      "loaded %zu measurements (%.1f s) from station (%.2f, %.2f), start in %.1f s, loop=%s",
      points_.size(), points_.empty() ? 0.0 : points_.back().t, station_x_, station_y_,
      start_delay_, loop_ ? "true" : "false");
  }

private:
  struct Point
  {
    double t;
    double range;
    double bearing;
  };

  bool load(const std::string & path)
  {
    std::ifstream ifs(path);
    if (!ifs.is_open()) {
      return false;
    }
    nlohmann::json j;
    try {
      ifs >> j;
      const auto & st = j.at("meta").at("station");
      station_x_ = st.at("x").get<double>();
      station_y_ = st.at("y").get<double>();
      for (const auto & it : j.at("points")) {
        points_.push_back({it.at("t").get<double>(), it.at("range").get<double>(),
            it.at("bearing").get<double>()});
      }
    } catch (const std::exception & e) {
      RCLCPP_ERROR(get_logger(), "json parse error: %s", e.what());
      return false;
    }
    return !points_.empty();
  }

  void onTimer()
  {
    const rclcpp::Time now = this->now();
    if (!started_) {
      if (!t0_set_) {
        t0_ = now + rclcpp::Duration::from_seconds(start_delay_);
        t0_set_ = true;
      }
      if (now < t0_) {
        return;
      }
      started_ = true;
    }

    while (idx_ < points_.size()) {
      const auto & p = points_[idx_];
      const rclcpp::Time due = t0_ + rclcpp::Duration::from_seconds(p.t);
      if (now < due) {
        return;
      }
      interfaces::msg::PolarMeasurement m;
      m.header.stamp = due;
      m.header.frame_id = frame_id_;
      m.range = p.range;
      m.bearing = p.bearing;
      m.station_x = station_x_;
      m.station_y = station_y_;
      pub_->publish(m);
      ++idx_;
    }

    // 播完
    if (!loop_) {
      if (!finished_logged_) {
        RCLCPP_INFO(
          get_logger(),
          "playback finished: %zu measurements. Final image retained, Ctrl-C to exit.",
          points_.size());
        finished_logged_ = true;
        timer_->cancel();
      }
      return;
    }
    // 循环：下一轮的时间轴接在这一轮末尾之后 loop_gap_ 秒。
    // 这段空档是留给"电机归零 + 滤波器重置"的周期间停顿。
    t0_ = t0_ + rclcpp::Duration::from_seconds(points_.back().t + loop_gap_);
    idx_ = 0;
    RCLCPP_INFO(
      get_logger(), "cycle done, next starts in %.1f s (target jumps back to the start)",
      loop_gap_);
  }

  std::string json_path_;
  std::string frame_id_;
  bool loop_{false};
  double loop_gap_{4.0};
  double start_delay_{1.0};
  double station_x_{0.0};
  double station_y_{0.0};

  std::vector<Point> points_;
  size_t idx_{0};
  bool started_{false};
  bool t0_set_{false};
  bool finished_logged_{false};
  rclcpp::Time t0_;

  rclcpp::Publisher<interfaces::msg::PolarMeasurement>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace virtual_locator

RCLCPP_COMPONENTS_REGISTER_NODE(virtual_locator::MeasurementPublisher)
