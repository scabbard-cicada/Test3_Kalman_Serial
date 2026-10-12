// Copyright 2026 Target Tracker Exam
//
// 可视化节点：把跟踪结果发布成 ROS 标准的可视化消息，交给 Foxglove / RViz 的
// 3D 面板（切到 2D 视图即为俯视坐标图）显示。
//
// 订阅（只读，不改动算法与下位机接口）：
//   target/measurement   PolarMeasurement   原始极坐标观测
//   target/filtered      PointStamped        EKF 滤波位置
//   gimbal/cmd           GimbalCmd           云台指令
//   gimbal/feedback      GimbalFeedback      下位机回传
//
// 发布：
//   viz/raw_path         Marker(LINE_STRIP, 红, 细)  原始观测连成的线
//   viz/filtered_path    Marker(LINE_STRIP, 绿, 细)  滤波轨迹
//   viz/gimbal           MarkerArray (station 系)    指令/反馈射线、限位、站点、状态
//   /tf_static           map -> station
//   /tf                  station -> gimbal
//
// 周期间检测到长断流时清空轨迹，避免上一周期末尾和下一周期开头被一条线连起来。

#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#if __has_include(<tf2_ros/static_transform_broadcaster.hpp>)
#include <tf2_ros/static_transform_broadcaster.hpp>
#else
#include <tf2_ros/static_transform_broadcaster.h>
#endif
#if __has_include(<tf2_ros/transform_broadcaster.hpp>)
#include <tf2_ros/transform_broadcaster.hpp>
#else
#include <tf2_ros/transform_broadcaster.h>
#endif
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "interfaces/msg/gimbal_cmd.hpp"
#include "interfaces/msg/gimbal_feedback.hpp"
#include "interfaces/msg/polar_measurement.hpp"

namespace target_tracker_visualizer
{

using interfaces::msg::GimbalCmd;
using interfaces::msg::GimbalFeedback;
using interfaces::msg::PolarMeasurement;
using visualization_msgs::msg::Marker;
using visualization_msgs::msg::MarkerArray;

class VisualizerNode : public rclcpp::Node
{
public:
  explicit VisualizerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("visualizer_node", options)
  {
    map_frame_ = declare_parameter<std::string>("map_frame", "map");
    station_frame_ = declare_parameter<std::string>("station_frame", "station");
    gimbal_frame_ = declare_parameter<std::string>("gimbal_frame", "gimbal");
    publish_rate_ = declare_parameter<double>("publish_rate", 15.0);
    trail_length_ = static_cast<size_t>(declare_parameter<int>("trail_length", 6000));
    clear_gap_ = declare_parameter<double>("clear_gap", 1.5);
    ray_length_ = declare_parameter<double>("ray_length", 15.0);
    gimbal_heading_ = declare_parameter<double>("gimbal_heading", M_PI);
    yaw_min_ = declare_parameter<double>("yaw_min_deg", -35.0) * M_PI / 180.0;
    yaw_max_ = declare_parameter<double>("yaw_max_deg", 35.0) * M_PI / 180.0;
    station_x_ = declare_parameter<double>("station_x", 10.0);
    station_y_ = declare_parameter<double>("station_y", 0.0);
    raw_width_ = declare_parameter<double>("raw_width", 0.03);
    filtered_width_ = declare_parameter<double>("filtered_width", 0.05);
    draw_raw_ = declare_parameter<bool>("draw_raw", true);
    draw_filtered_ = declare_parameter<bool>("draw_filtered", true);
    draw_gimbal_ = declare_parameter<bool>("draw_gimbal", true);

    sub_meas_ = create_subscription<PolarMeasurement>(
      "target/measurement", 20, [this](PolarMeasurement::SharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_);
        const rclcpp::Time stamp(m->header.stamp);
        if (newCycle(stamp)) {
          raw_.clear();
          filtered_.clear();
        }
        station_x_ = m->station_x;
        station_y_ = m->station_y;
        const double x = station_x_ + m->range * std::cos(m->bearing);
        const double y = station_y_ + m->range * std::sin(m->bearing);
        push(raw_, x, y);
        publishStationTf();
      });
    sub_filtered_ = create_subscription<geometry_msgs::msg::PointStamped>(
      "target/filtered", 20, [this](geometry_msgs::msg::PointStamped::SharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_);
        push(filtered_, m->point.x, m->point.y);
      });
    sub_cmd_ = create_subscription<GimbalCmd>(
      "gimbal/cmd", 10, [this](GimbalCmd::SharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_);
        cmd_ = *m;
        have_cmd_ = true;
      });
    sub_fb_ = create_subscription<GimbalFeedback>(
      "gimbal/feedback", rclcpp::SensorDataQoS(), [this](GimbalFeedback::SharedPtr m) {
        std::lock_guard<std::mutex> lk(mtx_);
        fb_ = *m;
        have_fb_ = true;
        fb_rx_time_ = now();
      });

    pub_raw_ = create_publisher<Marker>("viz/raw_path", 5);
    pub_filtered_ = create_publisher<Marker>("viz/filtered_path", 5);
    pub_gimbal_ = create_publisher<MarkerArray>("viz/gimbal", 5);

    tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
    publishStationTf();

    const auto period = std::chrono::duration<double>(1.0 / std::max(1.0, publish_rate_));
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      [this]() {onRender();});

    RCLCPP_INFO(
      get_logger(), "visualizer started | frames %s/%s/%s, publish %.1f Hz, trail %zu",
      map_frame_.c_str(), station_frame_.c_str(), gimbal_frame_.c_str(), publish_rate_,
      trail_length_);
  }

private:
  /// 观测时间戳出现大跳变时视为新周期，清空旧轨迹
  bool newCycle(const rclcpp::Time & stamp)
  {
    const bool boundary = last_meas_stamp_.nanoseconds() != 0 &&
      (stamp - last_meas_stamp_).seconds() > clear_gap_;
    last_meas_stamp_ = stamp;
    return boundary;
  }

  void push(std::deque<geometry_msgs::msg::Point> & buf, double x, double y)
  {
    geometry_msgs::msg::Point p;
    p.x = x;
    p.y = y;
    p.z = 0.0;
    buf.push_back(p);
    while (buf.size() > trail_length_) {
      buf.pop_front();
    }
  }

  static void yawToQuat(double yaw, geometry_msgs::msg::Quaternion & q)
  {
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw * 0.5);
    q.w = std::cos(yaw * 0.5);
  }

  void publishStationTf()
  {
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now();
    t.header.frame_id = map_frame_;
    t.child_frame_id = station_frame_;
    t.transform.translation.x = station_x_;
    t.transform.translation.y = station_y_;
    t.transform.translation.z = 0.0;
    yawToQuat(gimbal_heading_, t.transform.rotation);
    tf_static_broadcaster_->sendTransform(t);
  }

  static Marker lineStrip(const std::string & frame, int id, const std::string & ns,
    double width, float r, float g, float b)
  {
    Marker m;
    m.header.frame_id = frame;
    m.header.stamp = rclcpp::Clock().now();
    m.ns = ns;
    m.id = id;
    m.type = Marker::LINE_STRIP;
    m.action = Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = width;
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = 1.0f;
    return m;
  }

  static Marker baseMarker(int id, const std::string & frame, const std::string & ns, int type)
  {
    Marker mk;
    mk.header.frame_id = frame;
    mk.ns = ns;
    mk.id = id;
    mk.type = type;
    mk.action = Marker::ADD;
    mk.pose.orientation.w = 1.0;
    return mk;
  }

  static void setColor(Marker & m, float r, float g, float b, float a = 1.0f)
  {
    m.color.r = r;
    m.color.g = g;
    m.color.b = b;
    m.color.a = a;
  }

  Marker makeRay(int id, double yaw, float r, float g, float b, double width) const
  {
    Marker m = baseMarker(id, station_frame_, "gimbal", Marker::ARROW);
    m.header.stamp = now();
    geometry_msgs::msg::Point tail;
    tail.x = 0.0;
    geometry_msgs::msg::Point head;
    head.x = ray_length_ * std::cos(yaw);
    head.y = ray_length_ * std::sin(yaw);
    m.points.push_back(tail);
    m.points.push_back(head);
    m.scale.x = width;
    m.scale.y = width * 2.0;
    setColor(m, r, g, b);
    return m;
  }

  void publishTrail(
    rclcpp::Publisher<Marker>::SharedPtr pub, const std::deque<geometry_msgs::msg::Point> & buf,
    const std::string & ns, double width, float r, float g, float b)
  {
    if (buf.size() < 2) {
      return;
    }
    Marker m = lineStrip(map_frame_, 0, ns, width, r, g, b);
    m.header.stamp = now();
    m.points.assign(buf.begin(), buf.end());
    pub->publish(m);
  }

  void onRender()
  {
    const rclcpp::Time stamp = now();
    std::deque<geometry_msgs::msg::Point> raw, filt;
    GimbalCmd cmd;
    GimbalFeedback fb;
    bool have_cmd, have_fb;
    rclcpp::Time fb_time;
    {
      std::lock_guard<std::mutex> lk(mtx_);
      raw = raw_;
      filt = filtered_;
      cmd = cmd_;
      fb = fb_;
      have_cmd = have_cmd_;
      have_fb = have_fb_;
      fb_time = fb_rx_time_;
    }

    if (draw_raw_) {
      publishTrail(pub_raw_, raw, "raw", raw_width_, 0.95f, 0.25f, 0.25f);
    }
    if (draw_filtered_) {
      publishTrail(pub_filtered_, filt, "filtered", filtered_width_, 0.15f, 0.9f, 0.2f);
    }

    MarkerArray arr;
    if (draw_gimbal_) {
      Marker site = baseMarker(10, station_frame_, "site", Marker::SPHERE);
      site.header.stamp = stamp;
      site.scale.x = site.scale.y = site.scale.z = 0.4;
      setColor(site, 0.1f, 0.9f, 0.1f);
      arr.markers.push_back(site);

      Marker txt = baseMarker(11, station_frame_, "site", Marker::TEXT_VIEW_FACING);
      txt.header.stamp = stamp;
      txt.pose.position.y = 0.6;
      txt.scale.z = 0.8;
      txt.text = "S";
      setColor(txt, 0.95f, 0.95f, 0.95f);
      arr.markers.push_back(txt);

      for (int i = 0; i < 2; ++i) {
        const double yaw = (i == 0) ? yaw_min_ : yaw_max_;
        Marker lim = baseMarker(20 + i, station_frame_, "limit", Marker::LINE_STRIP);
        lim.header.stamp = stamp;
        geometry_msgs::msg::Point a;
        a.x = 0.0;
        geometry_msgs::msg::Point b;
        b.x = ray_length_ * std::cos(yaw);
        b.y = ray_length_ * std::sin(yaw);
        lim.points.push_back(a);
        lim.points.push_back(b);
        lim.scale.x = 0.04;
        setColor(lim, 0.5f, 0.5f, 0.5f, 0.8f);
        arr.markers.push_back(lim);
      }

      if (have_cmd && cmd.mode != GimbalCmd::MODE_RELAX) {
        arr.markers.push_back(makeRay(30, cmd.yaw, 0.95f, 0.75f, 0.1f, 0.05));
      }
      const bool fb_fresh = have_fb && (stamp - fb_time).seconds() < 0.5;
      if (fb_fresh) {
        arr.markers.push_back(makeRay(31, fb.yaw, 0.1f, 0.9f, 0.9f, 0.09));
        Marker st = baseMarker(12, station_frame_, "site", Marker::TEXT_VIEW_FACING);
        st.header.stamp = stamp;
        st.pose.position.y = -0.8;
        st.scale.z = 0.5;
        const char * name = "?";
        switch (fb.state) {
          case GimbalFeedback::STATE_SAFE: name = "SAFE"; break;
          case GimbalFeedback::STATE_ARMED: name = "ARMED"; break;
          case GimbalFeedback::STATE_TIMEOUT: name = "TIMEOUT"; break;
          case GimbalFeedback::STATE_FAULT: name = "FAULT"; break;
          default: break;
        }
        st.text = std::string("MCU ") + name + "  " + std::to_string(fb.temperature) + "C";
        setColor(st, 0.85f, 0.85f, 0.85f);
        arr.markers.push_back(st);
      }
    }
    pub_gimbal_->publish(arr);

    if (have_fb) {
      geometry_msgs::msg::TransformStamped t;
      t.header.stamp = stamp;
      t.header.frame_id = station_frame_;
      t.child_frame_id = gimbal_frame_;
      yawToQuat(fb.yaw, t.transform.rotation);
      tf_broadcaster_->sendTransform(t);
    }
  }

  std::string map_frame_, station_frame_, gimbal_frame_;
  double publish_rate_{15.0}, clear_gap_{1.5}, ray_length_{15.0}, gimbal_heading_{M_PI};
  double yaw_min_{0.0}, yaw_max_{0.0}, station_x_{10.0}, station_y_{0.0};
  double raw_width_{0.03}, filtered_width_{0.05};
  size_t trail_length_{6000};
  bool draw_raw_{true}, draw_filtered_{true}, draw_gimbal_{true};

  std::mutex mtx_;
  std::deque<geometry_msgs::msg::Point> raw_, filtered_;
  rclcpp::Time last_meas_stamp_{0, 0, RCL_ROS_TIME};
  GimbalCmd cmd_;
  GimbalFeedback fb_;
  bool have_cmd_{false}, have_fb_{false};
  rclcpp::Time fb_rx_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<PolarMeasurement>::SharedPtr sub_meas_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_filtered_;
  rclcpp::Subscription<GimbalCmd>::SharedPtr sub_cmd_;
  rclcpp::Subscription<GimbalFeedback>::SharedPtr sub_fb_;
  rclcpp::Publisher<Marker>::SharedPtr pub_raw_;
  rclcpp::Publisher<Marker>::SharedPtr pub_filtered_;
  rclcpp::Publisher<MarkerArray>::SharedPtr pub_gimbal_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace target_tracker_visualizer

RCLCPP_COMPONENTS_REGISTER_NODE(target_tracker_visualizer::VisualizerNode)
