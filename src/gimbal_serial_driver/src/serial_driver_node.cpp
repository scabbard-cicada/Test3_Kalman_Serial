// 串口驱动节点：ROS 话题 <-> C1/C2 串口帧
//
//   订阅  gimbal/cmd        GimbalCmd       → 编码成 C2 发给下位机
//   发布  gimbal/feedback   GimbalFeedback  ← 下位机回传的 C1
//         serial/status     SerialStatus    1 Hz 链路统计
//
// 结构沿用 autoaim_serial_driver：AsioSerial 负责收发和重连，节点只做编解码和统计。
// 时延测量：C2 里带上本机时钟 host_time_ms，下位机在 C1 里原样回传最近一次采纳的值，
//   rtt = now - echo_host_time_ms
// 这是"上位机发出 → 下位机解析 → 下一次回传 → 上位机收到"的往返时间，
// 包含下位机最多 10 ms 的回传等待（C1 是 100 Hz），所以是实际单程延迟的上界。

#include <chrono>
#include <cmath>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>

#include "gimbal_serial_driver/asio_serial.hpp"
#include "gimbal_serial_driver/protocol.hpp"
#include "interfaces/msg/gimbal_cmd.hpp"
#include "interfaces/msg/gimbal_feedback.hpp"
#include "interfaces/msg/serial_status.hpp"

namespace gimbal_serial_driver
{

using interfaces::msg::GimbalCmd;
using interfaces::msg::GimbalFeedback;
using interfaces::msg::SerialStatus;

class SerialDriverNode : public rclcpp::Node
{
public:
  explicit SerialDriverNode(const rclcpp::NodeOptions & options)
  : Node("gimbal_serial_driver", options),
    parser_(proto::kHeaderFb, proto::kFbLen)
  {
    AsioSerial::Options opt;
    opt.device_name = declare_parameter<std::string>("device_name", "/dev/ttyUSB0");
    opt.baud_rate = static_cast<uint32_t>(declare_parameter<int>("baud_rate", 115200));
    opt.reconnect_initial_delay =
      std::chrono::milliseconds(declare_parameter<int>("reconnect_initial_delay_ms", 5));
    opt.reconnect_max_delay =
      std::chrono::milliseconds(declare_parameter<int>("reconnect_max_delay_ms", 500));
    opt.reconnect_backoff_multiplier =
      declare_parameter<double>("reconnect_backoff_multiplier", 1.5);
    opt.read_buffer_size = static_cast<size_t>(declare_parameter<int>("read_buffer_size", 1024));
    opt.io_thread_realtime = declare_parameter<bool>("io_thread_realtime", false);
    opt.io_thread_priority = static_cast<int>(declare_parameter<int>("io_thread_priority", 20));
    log_frames_ = declare_parameter<bool>("log_frames", false);

    pub_fb_ = create_publisher<GimbalFeedback>("gimbal/feedback", rclcpp::SensorDataQoS());
    pub_status_ = create_publisher<SerialStatus>("serial/status", 10);
    sub_cmd_ = create_subscription<GimbalCmd>(
      "gimbal/cmd", rclcpp::QoS(10),
      std::bind(&SerialDriverNode::onCmd, this, std::placeholders::_1));

    serial_ = std::make_shared<AsioSerial>(get_logger(), opt);
    serial_->on_receive([this](const uint8_t * d, size_t n) {onBytes(d, n);});
    serial_->on_connect([this](bool up) {
        std::lock_guard<std::mutex> lk(mtx_);
        // 断线前后的字节不能拼成一帧：重连时清空解析器
        parser_.reset();
        have_last_echo_ = false;
        if (!up) {
          RCLCPP_WARN(get_logger(), "serial link down");
        }
      });
    serial_->start();

    status_timer_ = create_wall_timer(
      std::chrono::seconds(1), std::bind(&SerialDriverNode::publishStatus, this));

    RCLCPP_INFO(
      get_logger(), "gimbal serial driver: %s @ %u", opt.device_name.c_str(), opt.baud_rate);
  }

  ~SerialDriverNode() override
  {
    if (serial_) {
      serial_->stop();
    }
  }

private:
  static uint32_t steadyMs()
  {
    return static_cast<uint32_t>(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::steady_clock::now().time_since_epoch())
      .count());
  }

  // ------------------------------------------------------------------ 发送
  void onCmd(const GimbalCmd::SharedPtr msg)
  {
    proto::Cmd c;
    c.yaw = msg->yaw;
    c.yaw_vel = msg->yaw_vel;
    c.mode = msg->mode;
    c.flags = static_cast<uint8_t>(
      (msg->target_valid ? proto::kFlagTargetValid : 0) |
      (msg->laser_on ? proto::kFlagLaserOn : 0) | (msg->outlier ? proto::kFlagOutlier : 0));
    c.host_time_ms = steadyMs();
    {
      std::lock_guard<std::mutex> lk(mtx_);
      c.seq = tx_seq_++;
    }
    // 非有限值不发：下位机会拒绝并计故障，这里先拦下来，方便定位是上位机的问题
    if (!std::isfinite(c.yaw) || !std::isfinite(c.yaw_vel)) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 1000, "gimbal/cmd contains NaN/Inf, frame dropped");
      return;
    }
    const auto frame = proto::encodeCommandC2(c);
    if (serial_->async_send(frame)) {
      std::lock_guard<std::mutex> lk(mtx_);
      ++tx_frames_;
    }
    if (log_frames_) {
      logHex("TX", frame.data(), frame.size());
    }
  }

  // ------------------------------------------------------------------ 接收
  void onBytes(const uint8_t * data, size_t n)
  {
    std::lock_guard<std::mutex> lk(mtx_);
    parser_.feed(data, n, [this](const uint8_t * f, size_t len) {onFrame(f, len);});
  }

  void onFrame(const uint8_t * f, size_t len)  // 已持有 mtx_
  {
    proto::Feedback fb;
    if (!proto::decodeFeedbackC1(f, len, fb)) {
      return;
    }
    if (log_frames_) {
      logHex("RX", f, len);
    }
    const uint32_t now_ms = steadyMs();

    GimbalFeedback m;
    m.stamp = now();
    m.yaw = fb.yaw;
    m.yaw_vel = fb.yaw_vel;
    m.yaw_ref = fb.yaw_ref;
    m.current = fb.current;
    m.temperature = fb.temperature;
    m.state = fb.state;
    m.error_flags = fb.error_flags;
    m.echo_seq = fb.echo_seq;
    m.rtt_ms = -1.0f;

    // 往返时延：只在回显的指令是新的时候算一次，否则同一条指令会被重复统计
    if (fb.echo_host_time_ms != 0 &&
      (!have_last_echo_ || fb.echo_host_time_ms != last_echo_time_))
    {
      const uint32_t rtt = now_ms - fb.echo_host_time_ms;
      if (rtt < 5000) {
        m.rtt_ms = static_cast<float>(rtt);
        rtt_window_.push_back(m.rtt_ms);
      }
      // 下行丢包：下位机回显的序号跳了几帧。回传 100 Hz、指令 100 Hz，
      // 两次回传之间偶尔会夹两条指令，所以只有跳过 >= 2 才算真的丢了
      if (have_last_echo_) {
        const uint8_t gap = static_cast<uint8_t>(fb.echo_seq - last_echo_seq_ - 1u);
        if (gap >= 2 && gap < 128) {
          echo_gaps_ += gap - 1u;
        }
      }
      last_echo_time_ = fb.echo_host_time_ms;
      last_echo_seq_ = fb.echo_seq;
      have_last_echo_ = true;
    }
    ++rx_window_;
    pub_fb_->publish(m);
  }

  void publishStatus()
  {
    SerialStatus s;
    s.stamp = now();
    s.connected = serial_->is_open();
    s.reconnects = serial_->reconnect_count();
    {
      std::lock_guard<std::mutex> lk(mtx_);
      s.tx_frames = static_cast<uint32_t>(tx_frames_);
      s.rx_frames = static_cast<uint32_t>(parser_.frames_ok());
      s.crc_errors = static_cast<uint32_t>(parser_.crc_errors());
      s.len_errors = static_cast<uint32_t>(parser_.len_errors());
      s.echo_gaps = echo_gaps_;
      s.rx_rate_hz = static_cast<float>(rx_window_);
      rx_window_ = 0;
      if (!rtt_window_.empty()) {
        float sum = 0, mx = 0;
        for (float v : rtt_window_) {
          sum += v;
          mx = std::max(mx, v);
        }
        s.rtt_ms_mean = sum / static_cast<float>(rtt_window_.size());
        s.rtt_ms_max = mx;
        rtt_window_.clear();
      } else {
        s.rtt_ms_mean = s.rtt_ms_max = -1.0f;
      }
    }
    pub_status_->publish(s);
    if (s.connected && s.rx_rate_hz < 1.0f) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 5000,
        "serial open but no valid C1 frames (check TX/RX crossover, baud rate, MCU firmware)");
    }
  }

  void logHex(const char * dir, const uint8_t * d, size_t n)
  {
    std::string s;
    char b[4];
    for (size_t i = 0; i < n; ++i) {
      std::snprintf(b, sizeof(b), "%02X ", d[i]);
      s += b;
    }
    RCLCPP_INFO(get_logger(), "%s %s", dir, s.c_str());
  }

  std::shared_ptr<AsioSerial> serial_;
  std::mutex mtx_;
  proto::StreamParser parser_;
  bool log_frames_{false};

  uint8_t tx_seq_{0};
  uint64_t tx_frames_{0};
  uint32_t rx_window_{0};
  uint32_t echo_gaps_{0};
  std::deque<float> rtt_window_;
  bool have_last_echo_{false};
  uint32_t last_echo_time_{0};
  uint8_t last_echo_seq_{0};

  rclcpp::Publisher<GimbalFeedback>::SharedPtr pub_fb_;
  rclcpp::Publisher<SerialStatus>::SharedPtr pub_status_;
  rclcpp::Subscription<GimbalCmd>::SharedPtr sub_cmd_;
  rclcpp::TimerBase::SharedPtr status_timer_;
};

}  // namespace gimbal_serial_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(gimbal_serial_driver::SerialDriverNode)
