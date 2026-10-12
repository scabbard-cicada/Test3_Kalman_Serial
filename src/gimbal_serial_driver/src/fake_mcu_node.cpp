// 软件下位机：没有 C 板时代替它。
//
// 用 openpty() 创建一对伪终端，在 link_path（默认 /tmp/gimbal_fake_mcu）建立指向从端的
// 符号链接。串口驱动打开这个路径，走的是和打开真实 /dev/ttyUSB0 完全相同的代码。
//
// 内部直接编译下位机固件的 Shared/ 目录（协议解析、串级 PID、安全状态机、GM6020 模型），
// 以 1 kHz 运行。所以 fake_mcu 的行为就是固件的行为，只是电机换成了仿真模型：
//   - 上电进入 SAFE，需要"按键"才解锁。按键用 ROS 服务模拟：
//       ros2 service call /fake_mcu/press_key std_srvs/srv/Trigger
//     或者把参数 auto_arm 设为 true（launch 默认），收到指令 0.5 s 后自动按一次
//   - 指令超时 100 ms 进 TIMEOUT 保持角度，500 ms 回到 SAFE
//   - 100 Hz 回传 C1
//
// 故障注入（用来检验上位机解析器和统计）：
//   drop_rate        每帧 C1 以这个概率不发
//   bit_error_rate   每个发出的比特以这个概率翻转
//   extra_latency_ms C1 额外延迟
//   motor_offline    模拟电机掉线

#include <fcntl.h>
#include <pty.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <deque>
#include <memory>
#include <mutex>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

extern "C" {
#include "gimbal_ctrl.h"
#include "gimbal_proto.h"
#include "gm6020.h"
}

namespace gimbal_serial_driver
{

class FakeMcuNode : public rclcpp::Node
{
public:
  FakeMcuNode()
  : Node("fake_mcu")
  {
    link_path_ = declare_parameter<std::string>("link_path", "/tmp/gimbal_fake_mcu");
    auto_arm_ = declare_parameter<bool>("auto_arm", true);
    auto_arm_delay_ms_ = static_cast<uint32_t>(declare_parameter<int>("auto_arm_delay_ms", 500));
    drop_rate_ = declare_parameter<double>("drop_rate", 0.0);
    bit_error_rate_ = declare_parameter<double>("bit_error_rate", 0.0);
    extra_latency_ms_ = static_cast<uint32_t>(declare_parameter<int>("extra_latency_ms", 0));
    baud_ = declare_parameter<int>("baud_rate", 115200);
    initial_yaw_deg_ = declare_parameter<double>("initial_yaw_deg", 0.0);
    output_scale_ = declare_parameter<double>("output_scale", 1.0);

    // 电机掉线可以运行时切换：ros2 param set /fake_mcu motor_offline true
    declare_parameter<bool>("motor_offline", false);
    param_cb_ = add_on_set_parameters_callback(
      [this](const std::vector<rclcpp::Parameter> & ps) {
        for (const auto & p : ps) {
          if (p.get_name() == "motor_offline") {
            motor_offline_.store(p.as_bool());
          } else if (p.get_name() == "drop_rate") {
            std::lock_guard<std::mutex> lk(cfg_mtx_);
            drop_rate_ = p.as_double();
          } else if (p.get_name() == "bit_error_rate") {
            std::lock_guard<std::mutex> lk(cfg_mtx_);
            bit_error_rate_ = p.as_double();
          }
        }
        rcl_interfaces::msg::SetParametersResult r;
        r.successful = true;
        return r;
      });

    openPty();

    srv_key_ = create_service<std_srvs::srv::Trigger>(
      "~/press_key", [this](
        const std::shared_ptr<std_srvs::srv::Trigger::Request>,
        std::shared_ptr<std_srvs::srv::Trigger::Response> res) {
        key_press_until_ms_.store(ms_.load() + 60);
        res->success = true;
        res->message = "KEY pressed";
      });

    running_ = true;
    thread_ = std::thread([this]() {run();});

    RCLCPP_INFO(
      get_logger(),
      "fake MCU on %s -> %s | auto_arm=%s drop=%.3f ber=%.1e latency=%u ms",
      link_path_.c_str(), slave_name_.c_str(), auto_arm_ ? "true" : "false", drop_rate_,
      bit_error_rate_, extra_latency_ms_);
  }

  ~FakeMcuNode() override
  {
    running_ = false;
    if (thread_.joinable()) {
      thread_.join();
    }
    if (master_fd_ >= 0) {
      close(master_fd_);
    }
    if (slave_fd_ >= 0) {
      close(slave_fd_);
    }
    unlink(link_path_.c_str());
  }

private:
  void openPty()
  {
    char name[256] = {0};
    if (openpty(&master_fd_, &slave_fd_, name, nullptr, nullptr) != 0) {
      throw std::runtime_error(std::string("openpty failed: ") + std::strerror(errno));
    }
    slave_name_ = name;
    // 从端设成 raw，否则行规程会把 0x0D/0x0A 等字节翻译掉、回显发出的数据
    termios t{};
    tcgetattr(slave_fd_, &t);
    cfmakeraw(&t);
    tcsetattr(slave_fd_, TCSANOW, &t);
    // 主端非阻塞
    const int fl = fcntl(master_fd_, F_GETFL);
    fcntl(master_fd_, F_SETFL, fl | O_NONBLOCK);

    unlink(link_path_.c_str());
    if (symlink(slave_name_.c_str(), link_path_.c_str()) != 0) {
      throw std::runtime_error(
              "symlink " + link_path_ + " failed: " + std::strerror(errno));
    }
    // 从端保持打开：否则串口驱动还没连上时主端 read 会返回 EIO
  }

  struct PendingTx
  {
    uint32_t due_ms;
    std::vector<uint8_t> bytes;
  };

  static void onCmdFrame(const uint8_t * frame, size_t len, void * user)
  {
    auto * self = static_cast<FakeMcuNode *>(user);
    gp_cmd_t c;
    if (gp_decode_cmd(frame, len, &c)) {
      self->pending_cmd_ = c;
      self->have_pending_cmd_ = true;
    }
  }

  void run()
  {
    gimbal_config_t cfg;
    gimbal_ctrl_default_config(&cfg);
    cfg.output_scale = static_cast<float>(output_scale_);
    gimbal_ctrl_t ctrl;
    gimbal_ctrl_init(&ctrl, &cfg, 0);

    gm6020_model_params_t mp;
    gm6020_model_default_params(&mp);
    gm6020_model_t motor;
    const float yaw0 = static_cast<float>(initial_yaw_deg_ * M_PI / 180.0);
    gm6020_model_init(&motor, &mp, yaw0);
    gm6020_unwrap_t unwrap;
    gm6020_unwrap_init(&unwrap, 0, 1);

    gp_parser_t parser;
    gp_parser_init(&parser, GP_HEADER_CMD);

    gimbal_input_t in;
    std::memset(&in, 0, sizeof(in));
    gimbal_output_t out;
    std::memset(&out, 0, sizeof(out));

    std::mt19937 rng(42);
    std::uniform_real_distribution<double> uni(0.0, 1.0);
    std::deque<PendingTx> tx;
    // 按真实波特率节流：115200 8N1 每毫秒约 11.5 字节
    const double bytes_per_ms = static_cast<double>(baud_) / 10.0 / 1000.0;
    double tx_budget = 0.0;

    bool armed_once = false;
    uint32_t first_cmd_ms = 0;
    uint8_t last_state = GP_STATE_SAFE;

    auto next = std::chrono::steady_clock::now();
    uint8_t rx[512];
    while (running_) {
      next += std::chrono::milliseconds(1);
      const uint32_t now = ms_.fetch_add(1) + 1;

      // ---- 收 ----
      have_pending_cmd_ = false;
      for (;;) {
        const ssize_t n = read(master_fd_, rx, sizeof(rx));
        if (n <= 0) {
          break;
        }
        gp_parser_feed(&parser, rx, static_cast<size_t>(n), &FakeMcuNode::onCmdFrame, this);
      }
      in.new_cmd = have_pending_cmd_;
      if (in.new_cmd) {
        in.cmd = pending_cmd_;
        if (first_cmd_ms == 0) {
          first_cmd_ms = now;
        }
      }

      // ---- 电机 ----
      gm6020_raw_t raw;
      gm6020_model_raw(&motor, &raw);
      in.motor_fresh = !motor_offline_.load();
      if (in.motor_fresh) {
        in.yaw_fb = gm6020_unwrap_update(&unwrap, raw.ecd);
        in.yaw_vel_fb = gm6020_rpm_to_rads(raw.rpm, 1);
        in.current_raw = raw.current;
        in.temperature = raw.temperature;
      }

      // ---- 按键 ----
      bool key = now < key_press_until_ms_.load();
      if (auto_arm_ && !armed_once && first_cmd_ms != 0 &&
        now - first_cmd_ms >= auto_arm_delay_ms_ && now - first_cmd_ms < auto_arm_delay_ms_ + 60)
      {
        key = true;
      }
      in.key_pressed = key;
      in.now_ms = now;

      // ---- 控制 ----
      gimbal_ctrl_step(&ctrl, &in, &out);
      gm6020_model_step(&motor, out.motor_enable ? out.voltage : 0, 0.001f);
      if (out.state == GP_STATE_ARMED) {
        armed_once = true;
      }
      if (out.state != last_state) {
        static const char * names[] = {"SAFE", "ARMED", "TIMEOUT", "FAULT"};
        RCLCPP_INFO(
          get_logger(), "state %s -> %s (err 0x%02X)", names[last_state & 3], names[out.state & 3],
          out.error_flags);
        last_state = static_cast<uint8_t>(out.state);
      }

      // ---- 回传 ----
      if (now % 10 == 0) {
        double drop, ber;
        {
          std::lock_guard<std::mutex> lk(cfg_mtx_);
          drop = drop_rate_;
          ber = bit_error_rate_;
        }
        if (uni(rng) >= drop) {
          gp_fb_t fb;
          gimbal_ctrl_fill_feedback(&ctrl, &in, &out, &fb);
          std::vector<uint8_t> f(GP_FB_LEN);
          gp_encode_fb(&fb, f.data(), f.size());
          if (ber > 0.0) {
            for (auto & b : f) {
              for (int k = 0; k < 8; ++k) {
                if (uni(rng) < ber) {
                  b ^= static_cast<uint8_t>(1u << k);
                }
              }
            }
          }
          tx.push_back({now + extra_latency_ms_, std::move(f)});
        }
      }
      tx_budget = std::min(tx_budget + bytes_per_ms, 64.0);
      while (!tx.empty() && tx.front().due_ms <= now && tx_budget >= 1.0) {
        auto & p = tx.front();
        const size_t n = std::min(p.bytes.size(), static_cast<size_t>(tx_budget));
        const ssize_t w = write(master_fd_, p.bytes.data(), n);
        if (w <= 0) {
          break;  // 没人读：缓冲满了，丢掉
        }
        tx_budget -= static_cast<double>(w);
        p.bytes.erase(p.bytes.begin(), p.bytes.begin() + w);
        if (p.bytes.empty()) {
          tx.pop_front();
        }
      }
      if (tx.size() > 50) {
        tx.pop_front();
      }

      std::this_thread::sleep_until(next);
    }
  }

  std::string link_path_;
  std::string slave_name_;
  int master_fd_{-1};
  int slave_fd_{-1};

  bool auto_arm_{true};
  uint32_t auto_arm_delay_ms_{500};
  double drop_rate_{0.0};
  double bit_error_rate_{0.0};
  uint32_t extra_latency_ms_{0};
  int64_t baud_{115200};
  double initial_yaw_deg_{0.0};
  double output_scale_{1.0};
  std::mutex cfg_mtx_;
  std::atomic_bool motor_offline_{false};

  std::atomic<uint32_t> ms_{0};
  std::atomic<uint32_t> key_press_until_ms_{0};
  gp_cmd_t pending_cmd_{};
  bool have_pending_cmd_{false};

  std::atomic_bool running_{false};
  std::thread thread_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr srv_key_;
  OnSetParametersCallbackHandle::SharedPtr param_cb_;
};

}  // namespace gimbal_serial_driver

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<gimbal_serial_driver::FakeMcuNode>());
  rclcpp::shutdown();
  return 0;
}
