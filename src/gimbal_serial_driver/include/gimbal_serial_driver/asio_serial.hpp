#pragma once
//
// Boost.Asio 串口：异步收发 + 自动重连。
//
// 改编自 autoaim2026/src/autoaim_serial_driver 的 AsioSerial：
//   - 所有对 port_ 的操作都在同一个 strand 上串行执行，读写回调之间不需要加锁
//   - 打开失败或读写出错后，按指数退避（initial → ×multiplier → max）自动重连
//   - 可选把 IO 线程设为 SCHED_RR 实时优先级（需要权限，失败只告警）
// 本版本另外：
//   - 记录重连次数，供 SerialStatus 发布
//   - 连接状态变化时回调（驱动用它清空解析器，避免拼接断线前后的半帧）

#include <atomic>
#include <chrono>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <boost/asio.hpp>
#include <rclcpp/logger.hpp>

namespace gimbal_serial_driver
{

class AsioSerial final : public std::enable_shared_from_this<AsioSerial>
{
public:
  using ReceiveCallback = std::function<void(const uint8_t * data, size_t size)>;
  using ConnectCallback = std::function<void(bool connected)>;

  struct Options
  {
    std::string device_name;
    uint32_t baud_rate{115200};
    std::chrono::milliseconds reconnect_initial_delay{5};
    std::chrono::milliseconds reconnect_max_delay{500};
    double reconnect_backoff_multiplier{1.5};
    size_t read_buffer_size{1024};
    bool io_thread_realtime{false};
    int io_thread_priority{20};
  };

  AsioSerial(rclcpp::Logger logger, Options options);
  ~AsioSerial();

  AsioSerial(const AsioSerial &) = delete;
  AsioSerial & operator=(const AsioSerial &) = delete;

  /// 设置回调要在 start() 之前
  void on_receive(ReceiveCallback cb) {receive_cb_ = std::move(cb);}
  void on_connect(ConnectCallback cb) {connect_cb_ = std::move(cb);}

  void start();
  void stop();

  /// 投递一段数据；当前未连接时返回 false（调用方可以丢弃这一帧，下一帧会是新数据）
  bool async_send(std::vector<uint8_t> data);

  bool is_open() const noexcept {return open_.load(std::memory_order_relaxed);}
  uint32_t reconnect_count() const noexcept {return reconnects_.load(std::memory_order_relaxed);}

private:
  void configure_io_thread();
  void run_io();
  void do_open();
  void close_port(const char * reason);
  void handle_io_error(const boost::system::error_code & ec, const char * where);
  void start_read();
  void schedule_reconnect();
  void start_write();
  void set_open(bool open);

  rclcpp::Logger logger_;
  const Options options_;

  boost::asio::io_context io_;
  boost::asio::strand<boost::asio::io_context::executor_type> strand_;
  boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_guard_;
  boost::asio::serial_port port_;
  boost::asio::steady_timer reconnect_timer_;

  std::thread io_thread_;
  std::atomic_bool stopped_{false};
  std::atomic_bool open_{false};
  std::atomic<uint32_t> reconnects_{0};
  bool ever_opened_{false};
  uint32_t open_failures_{0};

  ReceiveCallback receive_cb_;
  ConnectCallback connect_cb_;
  std::vector<uint8_t> read_buf_;
  bool read_in_progress_{false};

  std::deque<std::vector<uint8_t>> write_queue_;
  bool write_in_progress_{false};

  std::chrono::milliseconds reconnect_delay_;
  bool reconnect_scheduled_{false};
};

}  // namespace gimbal_serial_driver
