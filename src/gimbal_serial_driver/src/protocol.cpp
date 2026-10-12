#include "gimbal_serial_driver/protocol.hpp"

#include <algorithm>
#include <array>

namespace gimbal_serial_driver::proto
{

namespace
{

// 多项式 0x07 的查找表在编译期生成，和下位机 Shared/gimbal_proto.c 里手写的表应当完全一致
constexpr std::array<uint8_t, 256> makeTable()
{
  std::array<uint8_t, 256> t{};
  for (int i = 0; i < 256; ++i) {
    uint8_t c = static_cast<uint8_t>(i);
    for (int b = 0; b < 8; ++b) {
      c = (c & 0x80) ? static_cast<uint8_t>((c << 1) ^ 0x07) : static_cast<uint8_t>(c << 1);
    }
    t[static_cast<size_t>(i)] = c;
  }
  return t;
}
constexpr auto kTable = makeTable();

// 逐字节按小端写入；float 经 memcpy 取位模式，不依赖主机字节序和结构体打包
class Writer
{
public:
  explicit Writer(std::vector<uint8_t> & out)
  : out_(out) {}
  void u8(uint8_t v) {out_.push_back(v);}
  void u16(uint16_t v)
  {
    u8(static_cast<uint8_t>(v & 0xFF));
    u8(static_cast<uint8_t>(v >> 8));
  }
  void u32(uint32_t v)
  {
    for (int i = 0; i < 4; ++i) {
      u8(static_cast<uint8_t>((v >> (8 * i)) & 0xFF));
    }
  }
  void f32(float v)
  {
    uint32_t u;
    std::memcpy(&u, &v, sizeof(u));
    u32(u);
  }

private:
  std::vector<uint8_t> & out_;
};

class Reader
{
public:
  explicit Reader(const uint8_t * p)
  : p_(p) {}

  uint8_t u8() {return *p_++;}
  uint16_t u16()
  {
    const uint16_t v = static_cast<uint16_t>(p_[0] | (p_[1] << 8));
    p_ += 2;
    return v;
  }
  uint32_t u32()
  {
    const uint32_t v = static_cast<uint32_t>(p_[0]) | (static_cast<uint32_t>(p_[1]) << 8) |
      (static_cast<uint32_t>(p_[2]) << 16) | (static_cast<uint32_t>(p_[3]) << 24);
    p_ += 4;
    return v;
  }
  float f32()
  {
    const uint32_t u = u32();
    float v;
    std::memcpy(&v, &u, sizeof(v));
    return v;
  }

private:
  const uint8_t * p_;
};

void finish(std::vector<uint8_t> & f)
{
  f.push_back(crc8(f.data(), f.size()));
}

bool check(const uint8_t * frame, size_t len, uint8_t header, uint8_t expect_len)
{
  return frame != nullptr && len == expect_len && frame[0] == header && frame[1] == expect_len &&
         crc8(frame, expect_len - 1u) == frame[expect_len - 1u];
}

}  // namespace

uint8_t crc8(const uint8_t * data, size_t len)
{
  // TODO: 实现 CRC8。
  (void)data;
  (void)len;
  return 0;
}

std::vector<uint8_t> encodeCommandC2(const Cmd & c)
{
  // TODO: 编码 PC -> MCU 的 C2 命令帧。
  (void)c;
  return {};
}

std::vector<uint8_t> encodeFeedbackC1(const Feedback & fb)
{
  std::vector<uint8_t> f;
  f.reserve(kFbLen);
  Writer w(f);
  w.u8(kHeaderFb);
  w.u8(kFbLen);
  w.f32(fb.yaw);
  w.f32(fb.yaw_vel);
  w.f32(fb.yaw_ref);
  w.u16(static_cast<uint16_t>(fb.current));
  w.u8(fb.temperature);
  w.u8(fb.state);
  w.u8(fb.error_flags);
  w.u8(fb.echo_seq);
  w.u32(fb.echo_host_time_ms);
  finish(f);
  return f;
}

bool decodeCommandC2(const uint8_t * frame, size_t len, Cmd & out)
{
  if (!check(frame, len, kHeaderCmd, kCmdLen)) {
    return false;
  }
  Reader r(frame + 2);
  out.yaw = r.f32();
  out.yaw_vel = r.f32();
  out.mode = r.u8();
  out.flags = r.u8();
  out.seq = r.u8();
  out.host_time_ms = r.u32();
  return true;
}

bool decodeFeedbackC1(const uint8_t * frame, size_t len, Feedback & out)
{
  // TODO: 解码 MCU -> PC 的 C1 反馈帧。
  (void)frame;
  (void)len;
  (void)out;
  return false;
}

void StreamParser::feed(const uint8_t * data, size_t len, const FrameCallback & on_frame)
{
  // TODO: 实现 C1 帧的流式解析与重同步。
  (void)data;
  (void)len;
  (void)on_frame;
}

}  // namespace gimbal_serial_driver::proto
