#include "gimbal_serial_driver/asio_serial.hpp"

#include <algorithm>
#include <cstring>
#include <future>

#if defined(__linux__)
#include <pthread.h>
#include <sched.h>
#endif

#include <rclcpp/logging.hpp>

namespace gimbal_serial_driver
{

AsioSerial::AsioSerial(rclcpp::Logger logger, Options options)
: logger_(std::move(logger)),
  options_(std::move(options)),
  strand_(boost::asio::make_strand(io_)),
  work_guard_(boost::asio::make_work_guard(io_)),
  port_(io_),
  reconnect_timer_(io_),
  read_buf_(std::max<size_t>(64, options_.read_buffer_size), 0),
  reconnect_delay_(options_.reconnect_initial_delay)
{
}

AsioSerial::~AsioSerial() {stop();}

void AsioSerial::start()
{
  if (stopped_.load() || io_thread_.joinable()) {
    return;
  }
  io_thread_ = std::thread([this]() {run_io();});
  auto self = shared_from_this();
  boost::asio::post(strand_, [self]() {self->do_open();});
}

void AsioSerial::stop()
{
  if (stopped_.exchange(true)) {
    return;
  }
  if (io_thread_.joinable()) {
    // 在 strand 上关闭端口，保证不会和正在进行的读写回调交错
    auto done = std::make_shared<std::promise<void>>();
    auto fut = done->get_future();
    boost::asio::post(strand_, [this, done]() {
        reconnect_timer_.cancel();
        close_port("stop");
        write_queue_.clear();
        done->set_value();
      });
    fut.wait_for(std::chrono::seconds(2));
    work_guard_.reset();
    io_.stop();
    io_thread_.join();
  } else {
    close_port("stop");
  }
}

bool AsioSerial::async_send(std::vector<uint8_t> data)
{
  if (!open_.load(std::memory_order_relaxed)) {
    return false;
  }
  auto self = shared_from_this();
  boost::asio::post(strand_, [self, data = std::move(data)]() mutable {
      if (!self->open_.load(std::memory_order_relaxed)) {
        return;
      }
      // 写队列有上限：串口慢于发送速度时丢最旧的，不让延迟无限累积
      if (self->write_queue_.size() >= 32) {
        self->write_queue_.pop_front();
      }
      self->write_queue_.push_back(std::move(data));
      if (!self->write_in_progress_) {
        self->start_write();
      }
    });
  return true;
}

void AsioSerial::configure_io_thread()
{
#if defined(__linux__)
  pthread_setname_np(pthread_self(), "gimbal_serial");
  if (!options_.io_thread_realtime) {
    return;
  }
  sched_param param{};
  param.sched_priority = options_.io_thread_priority;
  const int rc = pthread_setschedparam(pthread_self(), SCHED_RR, &param);
  if (rc != 0) {
    RCLCPP_WARN(
      logger_, "cannot set serial IO thread to SCHED_RR %d: %s (continuing with default)",
      options_.io_thread_priority, std::strerror(rc));
  } else {
    RCLCPP_INFO(logger_, "serial IO thread running SCHED_RR %d", options_.io_thread_priority);
  }
#endif
}

void AsioSerial::run_io()
{
  configure_io_thread();
  while (!stopped_.load()) {
    try {
      io_.run();
      break;
    } catch (const std::exception & e) {
      RCLCPP_ERROR(logger_, "asio io_context exception: %s", e.what());
    }
  }
}

void AsioSerial::set_open(bool open)
{
  const bool was = open_.exchange(open);
  if (was != open && connect_cb_) {
    connect_cb_(open);
  }
}

void AsioSerial::do_open()
{
  if (stopped_.load()) {
    return;
  }
  reconnect_scheduled_ = false;

  boost::system::error_code ec;
  if (port_.is_open()) {
    port_.close(ec);
  }
  port_.open(options_.device_name, ec);
  if (ec) {
    // 重连期间每次失败都打印会刷屏：只打印第一次和之后每 20 次
    if (open_failures_++ % 20 == 0) {
      RCLCPP_WARN(
        logger_, "serial open failed (%s): %s — retrying", options_.device_name.c_str(),
        ec.message().c_str());
    }
    schedule_reconnect();
    return;
  }
  open_failures_ = 0;

  using B = boost::asio::serial_port_base;
  port_.set_option(B::baud_rate(options_.baud_rate), ec);
  if (!ec) {port_.set_option(B::character_size(8), ec);}
  if (!ec) {port_.set_option(B::parity(B::parity::none), ec);}
  if (!ec) {port_.set_option(B::stop_bits(B::stop_bits::one), ec);}
  if (!ec) {port_.set_option(B::flow_control(B::flow_control::none), ec);}
  if (ec) {
    handle_io_error(ec, "configure");
    return;
  }

  if (ever_opened_) {
    reconnects_.fetch_add(1);
  }
  ever_opened_ = true;
  reconnect_delay_ = options_.reconnect_initial_delay;
  RCLCPP_INFO(
    logger_, "serial connected: %s @ %u 8N1", options_.device_name.c_str(), options_.baud_rate);
  set_open(true);
  start_read();
  if (!write_queue_.empty()) {
    start_write();
  }
}

void AsioSerial::close_port(const char * reason)
{
  boost::system::error_code ec;
  if (port_.is_open()) {
    port_.cancel(ec);
    port_.close(ec);
  }
  read_in_progress_ = false;
  write_in_progress_ = false;
  write_queue_.clear();
  (void)reason;
  set_open(false);
}

void AsioSerial::handle_io_error(const boost::system::error_code & ec, const char * where)
{
  if (stopped_.load() || ec == boost::asio::error::operation_aborted) {
    return;
  }
  RCLCPP_ERROR(logger_, "serial %s error: %s", where, ec.message().c_str());
  close_port(where);
  schedule_reconnect();
}

void AsioSerial::start_read()
{
  if (!open_.load() || read_in_progress_) {
    return;
  }
  read_in_progress_ = true;
  auto self = shared_from_this();
  port_.async_read_some(
    boost::asio::buffer(read_buf_),
    boost::asio::bind_executor(
      strand_, [self](const boost::system::error_code & ec, size_t n) {
        self->read_in_progress_ = false;
        if (ec) {
          self->handle_io_error(ec, "read");
          return;
        }
        if (n > 0 && self->receive_cb_) {
          self->receive_cb_(self->read_buf_.data(), n);
        }
        self->start_read();
      }));
}

void AsioSerial::schedule_reconnect()
{
  if (stopped_.load() || reconnect_scheduled_) {
    return;
  }
  reconnect_scheduled_ = true;
  const auto delay = reconnect_delay_;
  reconnect_delay_ = std::min(
    options_.reconnect_max_delay,
    std::chrono::milliseconds(
      static_cast<int64_t>(
        std::max(1.0, reconnect_delay_.count() * options_.reconnect_backoff_multiplier))));
  reconnect_timer_.expires_after(delay);
  auto self = shared_from_this();
  reconnect_timer_.async_wait(
    boost::asio::bind_executor(
      strand_, [self](const boost::system::error_code & ec) {
        self->reconnect_scheduled_ = false;
        if (!ec) {
          self->do_open();
        }
      }));
}

void AsioSerial::start_write()
{
  if (!open_.load() || write_in_progress_ || write_queue_.empty()) {
    return;
  }
  write_in_progress_ = true;
  auto self = shared_from_this();
  boost::asio::async_write(
    port_, boost::asio::buffer(write_queue_.front()),
    boost::asio::bind_executor(
      strand_, [self](const boost::system::error_code & ec, size_t) {
        self->write_in_progress_ = false;
        if (ec) {
          self->handle_io_error(ec, "write");
          return;
        }
        if (!self->write_queue_.empty()) {
          self->write_queue_.pop_front();
        }
        self->start_write();
      }));
}

}  // namespace gimbal_serial_driver
