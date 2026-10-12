// 协议一致性测试：上位机 C++ 实现（protocol.hpp）与下位机 C 实现（Shared/gimbal_proto.h）
// 必须逐字节一致。下位机是"写好的被通信对象"，上位机的实现以它为准。
//
//   colcon test --packages-select gimbal_serial_driver && colcon test-result --verbose

#include <gtest/gtest.h>

#include <cmath>
#include <cstring>
#include <random>
#include <vector>

#include "gimbal_serial_driver/protocol.hpp"

extern "C" {
#include "gimbal_proto.h"
}

namespace proto = gimbal_serial_driver::proto;

TEST(Protocol, LengthsMatchFirmware)
{
  EXPECT_EQ(proto::kCmdLen, GP_CMD_LEN);
  EXPECT_EQ(proto::kFbLen, GP_FB_LEN);
  EXPECT_EQ(proto::kHeaderCmd, GP_HEADER_CMD);
  EXPECT_EQ(proto::kHeaderFb, GP_HEADER_FB);
}

TEST(Protocol, Crc8CheckValue)
{
  const uint8_t s[] = {'1', '2', '3', '4', '5', '6', '7', '8', '9'};
  EXPECT_EQ(proto::crc8(s, 9), 0xF4);
  EXPECT_EQ(gp_crc8(s, 9), 0xF4);
  // 两份查找表逐项一致
  for (int i = 0; i < 256; ++i) {
    const uint8_t b = static_cast<uint8_t>(i);
    EXPECT_EQ(proto::crc8(&b, 1), gp_crc8(&b, 1)) << i;
  }
}

TEST(Protocol, GoldenCmdFrame)
{
  // 与 PROTOCOL.md 第 4 节、lower_computer/gimbal_fw/tests/test_shared.c 中的向量相同
  const std::vector<uint8_t> golden = {0xC2, 0x12, 0x00, 0x00, 0x80, 0x3E, 0x00, 0x00, 0xC0,
    0xBF, 0x01, 0x03, 0x07, 0x78, 0x56, 0x34, 0x12, 0x8C};
  proto::Cmd c;
  c.yaw = 0.25f;
  c.yaw_vel = -1.5f;
  c.mode = 1;
  c.flags = proto::kFlagTargetValid | proto::kFlagLaserOn;
  c.seq = 7;
  c.host_time_ms = 0x12345678u;
  EXPECT_EQ(proto::encodeCommandC2(c), golden);
}

TEST(Protocol, GoldenFeedbackFrame)
{
  const std::vector<uint8_t> golden = {0xC1, 0x19, 0xCD, 0xCC, 0xCC, 0x3D, 0x00, 0x00, 0x00,
    0x40, 0x9A, 0x99, 0x99, 0xBE, 0x2E, 0xFB, 0x2D, 0x01, 0x04, 0x63, 0xBE, 0xBA, 0xFE, 0xCA,
    0x8D};
  proto::Feedback f;
  ASSERT_TRUE(proto::decodeFeedbackC1(golden.data(), golden.size(), f));
  EXPECT_FLOAT_EQ(f.yaw, 0.1f);
  EXPECT_FLOAT_EQ(f.yaw_vel, 2.0f);
  EXPECT_FLOAT_EQ(f.yaw_ref, -0.3f);
  EXPECT_EQ(f.current, -1234);
  EXPECT_EQ(f.temperature, 45);
  EXPECT_EQ(f.state, 1);
  EXPECT_EQ(f.error_flags, 4);
  EXPECT_EQ(f.echo_seq, 99);
  EXPECT_EQ(f.echo_host_time_ms, 0xCAFEBABEu);
  EXPECT_EQ(proto::encodeFeedbackC1(f), golden);
}

TEST(Protocol, HostEncodeFirmwareDecode)
{
  std::mt19937 rng(1);
  std::uniform_real_distribution<float> u(-10.0f, 10.0f);
  for (int k = 0; k < 500; ++k) {
    proto::Cmd c;
    c.yaw = u(rng);
    c.yaw_vel = u(rng);
    c.mode = static_cast<uint8_t>(k % 3);
    c.flags = static_cast<uint8_t>(k & 7);
    c.seq = static_cast<uint8_t>(k);
    c.host_time_ms = static_cast<uint32_t>(rng());
    const auto f = proto::encodeCommandC2(c);
    gp_cmd_t d;
    ASSERT_TRUE(gp_decode_cmd(f.data(), f.size(), &d));
    EXPECT_EQ(d.yaw, c.yaw);
    EXPECT_EQ(d.yaw_vel, c.yaw_vel);
    EXPECT_EQ(d.mode, c.mode);
    EXPECT_EQ(d.flags, c.flags);
    EXPECT_EQ(d.seq, c.seq);
    EXPECT_EQ(d.host_time_ms, c.host_time_ms);
  }
}

TEST(Protocol, FirmwareEncodeHostDecode)
{
  std::mt19937 rng(2);
  std::uniform_real_distribution<float> u(-10.0f, 10.0f);
  for (int k = 0; k < 500; ++k) {
    gp_fb_t fb;
    fb.yaw = u(rng);
    fb.yaw_vel = u(rng);
    fb.yaw_ref = u(rng);
    fb.current = static_cast<int16_t>(rng());
    fb.temperature = static_cast<uint8_t>(rng());
    fb.state = static_cast<uint8_t>(k % 4);
    fb.error_flags = static_cast<uint8_t>(rng());
    fb.echo_seq = static_cast<uint8_t>(k);
    fb.echo_host_time_ms = static_cast<uint32_t>(rng());
    uint8_t buf[GP_FB_LEN];
    ASSERT_EQ(gp_encode_fb(&fb, buf, sizeof(buf)), GP_FB_LEN);
    proto::Feedback d;
    ASSERT_TRUE(proto::decodeFeedbackC1(buf, sizeof(buf), d));
    EXPECT_EQ(d.yaw, fb.yaw);
    EXPECT_EQ(d.yaw_vel, fb.yaw_vel);
    EXPECT_EQ(d.yaw_ref, fb.yaw_ref);
    EXPECT_EQ(d.current, fb.current);
    EXPECT_EQ(d.temperature, fb.temperature);
    EXPECT_EQ(d.state, fb.state);
    EXPECT_EQ(d.error_flags, fb.error_flags);
    EXPECT_EQ(d.echo_seq, fb.echo_seq);
    EXPECT_EQ(d.echo_host_time_ms, fb.echo_host_time_ms);
  }
}

TEST(Protocol, StreamParserSplitsAndResyncs)
{
  proto::StreamParser p(proto::kHeaderFb, proto::kFbLen);
  std::vector<std::vector<uint8_t>> got;
  const auto sink = [&](const uint8_t * f, size_t n) {got.emplace_back(f, f + n);};

  proto::Feedback a, b;
  a.yaw = 1.0f;
  a.echo_seq = 1;
  b.yaw = 2.0f;
  b.echo_seq = 2;
  const auto fa = proto::encodeFeedbackC1(a);
  const auto fb = proto::encodeFeedbackC1(b);

  // 坏帧头 + 好帧：坏帧头后面紧跟好帧，凑满 25 字节时 CRC 必然不过，必须回退找回好帧
  std::vector<uint8_t> s = {0x00, 0xC1, proto::kFbLen, 0x11, 0x22};
  s.insert(s.end(), fa.begin(), fa.end());
  // 半包
  s.insert(s.end(), fb.begin(), fb.begin() + 10);
  p.feed(s.data(), s.size(), sink);
  ASSERT_EQ(got.size(), 1u);
  EXPECT_EQ(got[0], fa);
  EXPECT_EQ(p.crc_errors(), 1u);
  // 剩下半包到达
  p.feed(fb.data() + 10, fb.size() - 10, sink);
  ASSERT_EQ(got.size(), 2u);
  EXPECT_EQ(got[1], fb);

  // C2 帧不应被上位机解析器收下
  proto::Cmd c;
  const auto fc = proto::encodeCommandC2(c);
  p.feed(fc.data(), fc.size(), sink);
  EXPECT_EQ(got.size(), 2u);
}

TEST(Protocol, StreamParserRandomChunksWithNoise)
{
  // 200 帧好帧穿插随机垃圾（大量 0xC1），按随机块长喂入：好帧一帧不少
  std::mt19937 rng(3);
  std::vector<uint8_t> stream;
  for (int k = 0; k < 200; ++k) {
    const int junk = static_cast<int>(rng() % 6);
    for (int j = 0; j < junk; ++j) {
      stream.push_back(rng() % 3 == 0 ? 0xC1 : static_cast<uint8_t>(rng()));
    }
    proto::Feedback f;
    f.yaw = static_cast<float>(k);
    f.echo_seq = static_cast<uint8_t>(k);
    const auto e = proto::encodeFeedbackC1(f);
    stream.insert(stream.end(), e.begin(), e.end());
  }
  proto::StreamParser p(proto::kHeaderFb, proto::kFbLen);
  int good = 0;
  size_t i = 0;
  while (i < stream.size()) {
    const size_t n = std::min<size_t>(1 + rng() % 40, stream.size() - i);
    p.feed(stream.data() + i, n, [&](const uint8_t * f, size_t len) {
        proto::Feedback d;
        if (proto::decodeFeedbackC1(f, len, d) && d.yaw == static_cast<float>(d.echo_seq)) {
          ++good;
        }
      });
    i += n;
  }
  EXPECT_EQ(good, 200);
}
