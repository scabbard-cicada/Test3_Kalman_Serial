#pragma once
//
// 上位机一侧的 C1/C2 协议实现（C++，独立于下位机的 C 实现）。
// 两边是否一致，由 test/test_protocol.cpp 里的黄金字节向量和交叉编解码测试保证。
//
// 帧格式（与队内 autoaim / Radaraim 一致）：
//
//   | Header(1) | Length(1) | Payload(N) | CRC8(1) |     Length = N + 3 = 整帧长度
//
//   CRC8：多项式 0x07，初值 0x00，覆盖 Header ~ Payload
//   多字节字段小端，float 为 IEEE-754 单精度
//
//   0xC2  PC → MCU  18 B   yaw f32 | yaw_vel f32 | mode u8 | flags u8 | seq u8 | host_time_ms u32
//   0xC1  MCU → PC  25 B   yaw f32 | yaw_vel f32 | yaw_ref f32 | current i16 | temperature u8 |
//                          state u8 | error_flags u8 | echo_seq u8 | echo_host_time_ms u32
//
// 详见仓库根目录 PROTOCOL.md。

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <functional>
#include <vector>

namespace gimbal_serial_driver::proto
{

constexpr uint8_t kHeaderCmd = 0xC2;  // PC -> MCU
constexpr uint8_t kHeaderFb = 0xC1;   // MCU -> PC
constexpr uint8_t kCmdLen = 18;
constexpr uint8_t kFbLen = 25;

constexpr uint8_t kFlagTargetValid = 0x01;
constexpr uint8_t kFlagLaserOn = 0x02;
constexpr uint8_t kFlagOutlier = 0x04;

struct Cmd
{
  float yaw{0};
  float yaw_vel{0};
  uint8_t mode{0};
  uint8_t flags{0};
  uint8_t seq{0};
  uint32_t host_time_ms{0};
};

struct Feedback
{
  float yaw{0};
  float yaw_vel{0};
  float yaw_ref{0};
  int16_t current{0};
  uint8_t temperature{0};
  uint8_t state{0};
  uint8_t error_flags{0};
  uint8_t echo_seq{0};
  uint32_t echo_host_time_ms{0};
};

uint8_t crc8(const uint8_t * data, size_t len);

/// C2：上位机命令帧，PC -> MCU。
std::vector<uint8_t> encodeCommandC2(const Cmd & cmd);
/// C1：下位机反馈帧，MCU -> PC。给 fake MCU 和交叉测试使用。
std::vector<uint8_t> encodeFeedbackC1(const Feedback & feedback);
/// C2：下位机（或 fake MCU）解析来自 PC 的命令。
bool decodeCommandC2(const uint8_t * frame, size_t len, Cmd & out);
/// C1：上位机解析来自 MCU 的反馈。
bool decodeFeedbackC1(const uint8_t * frame, size_t len, Feedback & out);

/**
 * 流式解析器：一次可以喂任意长度的字节，帧可能被切成任意多块，也可能多帧粘在一起。
 *
 * 只认一种包 ID（上位机只收 C1），期望长度由包 ID 唯一确定。
 * 长度字节不对或 CRC 不过时，只丢掉当前候选帧头这一个字节，从下一个字节重新找头，
 * 因为候选帧头可能只是上一帧载荷里碰巧出现的 0xC1，真正的帧头就在后面。
 */
class StreamParser
{
public:
  using FrameCallback = std::function<void(const uint8_t * frame, size_t len)>;

  StreamParser(uint8_t header, uint8_t frame_len)
  : header_(header), frame_len_(frame_len) {}

  void feed(const uint8_t * data, size_t len, const FrameCallback & on_frame);
  void reset() {buf_.clear();}

  uint64_t frames_ok() const {return frames_ok_;}
  uint64_t crc_errors() const {return crc_errors_;}
  uint64_t len_errors() const {return len_errors_;}
  uint64_t skipped_bytes() const {return skipped_;}

private:
  uint8_t header_;
  uint8_t frame_len_;
  std::vector<uint8_t> buf_;
  uint64_t frames_ok_{0}, crc_errors_{0}, len_errors_{0}, skipped_{0};
};

}  // namespace gimbal_serial_driver::proto
