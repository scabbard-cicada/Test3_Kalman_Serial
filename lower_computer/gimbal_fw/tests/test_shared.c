/**
 * 主机端单元测试：覆盖 Shared/ 下全部模块。不依赖 HAL，也不依赖任何测试框架。
 *
 *   cmake -S tests -B build/host-tests && cmake --build build/host-tests
 *   ctest --test-dir build/host-tests --output-on-failure
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "gimbal_ctrl.h"
#include "gimbal_proto.h"
#include "gm6020.h"
#include "pid.h"

static int g_fail = 0;
static int g_pass = 0;

#define CHECK(cond)                                                    \
  do {                                                                 \
    if (cond) {                                                        \
      ++g_pass;                                                        \
    } else {                                                           \
      ++g_fail;                                                        \
      fprintf(stderr, "  FAIL %s:%d  %s\n", __FILE__, __LINE__, #cond); \
    }                                                                  \
  } while (0)

#define CHECK_NEAR(a, b, tol) CHECK(fabs((double)(a) - (double)(b)) <= (tol))

#define DEG2RAD(d) ((float)((d)*3.14159265358979323846 / 180.0))

/* ================================================================ 协议 */
typedef struct {
  int n;
  uint8_t frames[16][GP_MAX_LEN];
} sink_t;

static void sink_cb(const uint8_t *frame, size_t len, void *user) {
  sink_t *s = (sink_t *)user;
  if (s->n < 16) {
    memcpy(s->frames[s->n], frame, len);
  }
  s->n++;
}

static gp_cmd_t sample_cmd(uint8_t seq) {
  gp_cmd_t c;
  c.yaw = 0.25f;
  c.yaw_vel = -1.5f;
  c.mode = GP_MODE_TRACK;
  c.flags = GP_FLAG_TARGET_VALID | GP_FLAG_LASER_ON;
  c.seq = seq;
  c.host_time_ms = 0x12345678u;
  return c;
}

static void test_crc8(void) {
  /* CRC-8/SMBUS（多项式 0x07，初值 0）的标准校验值 */
  const uint8_t s[] = "123456789";
  CHECK(gp_crc8(s, 9) == 0xF4);
  CHECK(gp_crc8(NULL, 0) == 0x00);
}

static void test_golden_cmd(void) {
  /* 黄金向量：协议文档 PROTOCOL.md 里列出的同一帧，上位机实现必须逐字节一致 */
  /* 由独立的 Python 实现（struct.pack('<BBffBBBI', ...) + CRC8/0x07）算出 */
  const uint8_t expect[GP_CMD_LEN] = {0xC2, 0x12, 0x00, 0x00, 0x80, 0x3E, 0x00, 0x00, 0xC0,
                                      0xBF, 0x01, 0x03, 0x07, 0x78, 0x56, 0x34, 0x12, 0x8C};
  gp_cmd_t c = sample_cmd(7);
  uint8_t buf[GP_CMD_LEN];
  CHECK(gp_encode_cmd(&c, buf, sizeof(buf)) == GP_CMD_LEN);
  CHECK(memcmp(buf, expect, GP_CMD_LEN) == 0);
  printf("  golden C2: ");
  for (unsigned i = 0; i < GP_CMD_LEN; ++i) {
    printf("%02X ", buf[i]);
  }
  printf("\n");
}

static void test_roundtrip(void) {
  gp_cmd_t c = sample_cmd(200), d;
  uint8_t buf[GP_CMD_LEN];
  gp_encode_cmd(&c, buf, sizeof(buf));
  CHECK(gp_decode_cmd(buf, sizeof(buf), &d));
  CHECK(d.yaw == c.yaw && d.yaw_vel == c.yaw_vel && d.mode == c.mode && d.flags == c.flags &&
        d.seq == c.seq && d.host_time_ms == c.host_time_ms);

  gp_fb_t f = {0.1f, 2.0f, -0.3f, -1234, 45, GP_STATE_ARMED, GP_ERR_CMD_CLAMPED, 99, 0xCAFEBABEu};
  gp_fb_t g;
  uint8_t fbuf[GP_FB_LEN];
  CHECK(gp_encode_fb(&f, fbuf, sizeof(fbuf)) == GP_FB_LEN);
  /* C1 黄金向量，同样由 Python struct.pack('<BBfffhBBBBI', ...) 独立算出 */
  const uint8_t expect_fb[GP_FB_LEN] = {0xC1, 0x19, 0xCD, 0xCC, 0xCC, 0x3D, 0x00, 0x00, 0x00,
                                        0x40, 0x9A, 0x99, 0x99, 0xBE, 0x2E, 0xFB, 0x2D, 0x01,
                                        0x04, 0x63, 0xBE, 0xBA, 0xFE, 0xCA, 0x8D};
  CHECK(memcmp(fbuf, expect_fb, GP_FB_LEN) == 0);
  CHECK(gp_decode_fb(fbuf, sizeof(fbuf), &g));
  CHECK(g.yaw == f.yaw && g.yaw_vel == f.yaw_vel && g.yaw_ref == f.yaw_ref &&
        g.current == f.current && g.temperature == f.temperature && g.state == f.state &&
        g.error_flags == f.error_flags && g.echo_seq == f.echo_seq &&
        g.echo_host_time_ms == f.echo_host_time_ms);

  /* 缓冲区不够 */
  CHECK(gp_encode_cmd(&c, buf, GP_CMD_LEN - 1) == 0);
  /* 篡改一个字节 */
  buf[5] ^= 0x01;
  CHECK(!gp_decode_cmd(buf, sizeof(buf), &d));
}

static void test_parser_basic(void) {
  gp_parser_t p;
  sink_t s = {0};
  gp_parser_init(&p, GP_HEADER_CMD);

  uint8_t f1[GP_CMD_LEN], f2[GP_CMD_LEN];
  gp_cmd_t c1 = sample_cmd(1), c2 = sample_cmd(2);
  gp_encode_cmd(&c1, f1, sizeof(f1));
  gp_encode_cmd(&c2, f2, sizeof(f2));

  /* 逐字节喂 */
  for (unsigned i = 0; i < GP_CMD_LEN; ++i) {
    gp_parser_feed(&p, &f1[i], 1, sink_cb, &s);
  }
  CHECK(s.n == 1);
  CHECK(memcmp(s.frames[0], f1, GP_CMD_LEN) == 0);

  /* 粘包：两帧一起到 */
  uint8_t both[2 * GP_CMD_LEN];
  memcpy(both, f1, GP_CMD_LEN);
  memcpy(both + GP_CMD_LEN, f2, GP_CMD_LEN);
  gp_parser_feed(&p, both, sizeof(both), sink_cb, &s);
  CHECK(s.n == 3);
  CHECK(memcmp(s.frames[2], f2, GP_CMD_LEN) == 0);

  /* 半包：先到 7 字节，再到剩下的 */
  gp_parser_feed(&p, f1, 7, sink_cb, &s);
  CHECK(s.n == 3);
  gp_parser_feed(&p, f1 + 7, GP_CMD_LEN - 7, sink_cb, &s);
  CHECK(s.n == 4);
  CHECK(p.frames_ok == 4);
}

static void test_parser_garbage(void) {
  gp_parser_t p;
  sink_t s = {0};
  gp_parser_init(&p, GP_HEADER_CMD);
  uint8_t f[GP_CMD_LEN];
  gp_cmd_t c = sample_cmd(9);
  gp_encode_cmd(&c, f, sizeof(f));

  /* 帧前垃圾字节，其中混有假帧头 0xC2 + 错误长度 */
  const uint8_t junk[] = {0x00, 0xFF, 0xC2, 0x05, 0x11, 0xC2, 0xC2, 0x99};
  gp_parser_feed(&p, junk, sizeof(junk), sink_cb, &s);
  gp_parser_feed(&p, f, sizeof(f), sink_cb, &s);
  CHECK(s.n == 1);
  CHECK(p.len_errors >= 3);

  /* 收方向不对的包：解析器只收 C2，C1 帧应被当作垃圾 */
  gp_fb_t fb = {0};
  uint8_t fbuf[GP_FB_LEN];
  gp_encode_fb(&fb, fbuf, sizeof(fbuf));
  gp_parser_feed(&p, fbuf, sizeof(fbuf), sink_cb, &s);
  CHECK(s.n == 1);
}

static void test_parser_crc_resync(void) {
  /*
   * 关键用例：坏帧的载荷里藏着一个真帧头。
   *
   *   [C2 13 ...前 5 字节...] [完整的好帧 19 B] ...
   *
   * 坏帧头 C2 13 后面紧跟的是好帧，坏"帧"凑满 19 字节时 CRC 一定不过。
   * 正确的解析器必须回退到坏帧头的下一个字节重新找头，从而找回好帧；
   * 如果整段丢弃，好帧就丢了。
   */
  gp_parser_t p;
  sink_t s = {0};
  gp_parser_init(&p, GP_HEADER_CMD);
  uint8_t good[GP_CMD_LEN];
  gp_cmd_t c = sample_cmd(42);
  gp_encode_cmd(&c, good, sizeof(good));

  uint8_t stream[64];
  size_t n = 0;
  stream[n++] = 0xC2;
  stream[n++] = GP_CMD_LEN;
  stream[n++] = 0x01;
  stream[n++] = 0x02;
  stream[n++] = 0x03;
  memcpy(stream + n, good, GP_CMD_LEN);
  n += GP_CMD_LEN;

  gp_parser_feed(&p, stream, n, sink_cb, &s);
  CHECK(s.n == 1);
  CHECK(p.crc_errors == 1);
  CHECK(s.n >= 1 && memcmp(s.frames[0], good, GP_CMD_LEN) == 0);

  /* 单比特翻转：本帧丢，紧随其后的正常帧必须能解出 */
  sink_t s2 = {0};
  gp_parser_init(&p, GP_HEADER_CMD);
  uint8_t bad[GP_CMD_LEN];
  memcpy(bad, good, GP_CMD_LEN);
  bad[8] ^= 0x10;
  gp_parser_feed(&p, bad, sizeof(bad), sink_cb, &s2);
  gp_parser_feed(&p, good, sizeof(good), sink_cb, &s2);
  CHECK(s2.n == 1);
  CHECK(p.crc_errors == 1);
}

static void test_parser_payload_contains_header(void) {
  /* 载荷里本来就有 0xC2：好帧之间互不干扰 */
  gp_parser_t p;
  sink_t s = {0};
  gp_parser_init(&p, GP_HEADER_CMD);
  gp_cmd_t c = sample_cmd(0xC2);
  c.host_time_ms = 0xC2C2C2C2u;
  c.flags = 0xC2;
  uint8_t f[GP_CMD_LEN];
  gp_encode_cmd(&c, f, sizeof(f));
  for (int k = 0; k < 5; ++k) {
    gp_parser_feed(&p, f, sizeof(f), sink_cb, &s);
  }
  CHECK(s.n == 5);
  CHECK(p.crc_errors == 0 && p.len_errors == 0);
}

static void test_parser_fuzz(void) {
  /* 随机字节流里穿插 200 帧好帧，随机切块喂入：好帧一帧不丢，且不会误收 */
  srand(12345);
  gp_parser_t p;
  sink_t s = {0};
  gp_parser_init(&p, GP_HEADER_CMD);
  static uint8_t stream[200 * (GP_CMD_LEN + 8)];
  size_t n = 0;
  int good_frames = 0;
  for (int k = 0; k < 200; ++k) {
    int junk = rand() % 8;
    for (int j = 0; j < junk; ++j) {
      /* 垃圾里故意放很多 0xC2 */
      stream[n++] = (rand() % 3 == 0) ? 0xC2 : (uint8_t)rand();
    }
    gp_cmd_t c = sample_cmd((uint8_t)k);
    c.yaw = (float)k * 0.01f;
    n += gp_encode_cmd(&c, stream + n, sizeof(stream) - n);
    good_frames++;
  }
  size_t i = 0;
  while (i < n) {
    size_t chunk = 1 + (size_t)(rand() % 40);
    if (i + chunk > n) {
      chunk = n - i;
    }
    gp_parser_feed(&p, stream + i, chunk, sink_cb, &s);
    i += chunk;
  }
  /* 垃圾字节里可能偶然拼出一帧 CRC 正确的假帧（概率约 1/256 × 长度匹配），
     所以只要求好帧全部收到，误收的帧数极少 */
  CHECK(s.n >= good_frames);
  CHECK(s.n <= good_frames + 2);
}

static void test_seq_gap(void) {
  CHECK(gp_seq_gap(5, 6) == 0);
  CHECK(gp_seq_gap(5, 8) == 2);
  CHECK(gp_seq_gap(255, 0) == 0);
  CHECK(gp_seq_gap(254, 1) == 2);
}

/* ================================================================ GM6020 */
static void test_gm6020_codec(void) {
  uint8_t d[8] = {0};
  gm6020_raw_t r = {4000, -123, 2000, 41}, q;
  gm6020_encode_feedback(d, &r);
  CHECK(d[0] == 0x0F && d[1] == 0xA0); /* 4000 = 0x0FA0，大端 */
  CHECK(gm6020_decode(d, 8, &q));
  CHECK(q.ecd == 4000 && q.rpm == -123 && q.current == 2000 && q.temperature == 41);
  CHECK(!gm6020_decode(d, 7, &q));

  uint8_t c[8] = {0};
  gm6020_encode_voltage(c, 1, -1000);
  CHECK(c[0] == 0xFC && c[1] == 0x18); /* -1000 = 0xFC18 */
  gm6020_encode_voltage(c, 1, 30000);  /* 超限被钳到 25000 = 0x61A8 */
  CHECK(c[0] == 0x61 && c[1] == 0xA8);
  gm6020_encode_voltage(c, 3, 1);
  CHECK(c[4] == 0x00 && c[5] == 0x01);
}

static void test_gm6020_unwrap(void) {
  gm6020_unwrap_t u;
  gm6020_unwrap_init(&u, 1000, 1);
  CHECK_NEAR(gm6020_unwrap_update(&u, 1000), 0.0, 1e-6);
  /* 正转跨过 8191 -> 0 */
  gm6020_unwrap_init(&u, 0, 1);
  float a = gm6020_unwrap_update(&u, 8100);
  float b = gm6020_unwrap_update(&u, 50);
  CHECK(b > a);
  CHECK_NEAR(b - a, (50 + 8192 - 8100) * 2.0 * M_PI / 8192.0, 1e-4);
  /* 反向安装 */
  gm6020_unwrap_init(&u, 0, -1);
  CHECK_NEAR(gm6020_unwrap_update(&u, 2048), -M_PI / 2, 1e-4);
  /* 首帧选最近的一圈：零位 100，读到 8000 → 应是小负角，不是接近 2π */
  gm6020_unwrap_init(&u, 100, 1);
  float c0 = gm6020_unwrap_update(&u, 8000);
  CHECK(c0 < 0.0f && c0 > -0.5f);
}

/* ================================================================ PID */
static void test_pid(void) {
  pid_params_t pp = {0};
  pp.kp = 2.0f;
  pp.ki = 10.0f;
  pp.i_max = 1.0f;
  pp.out_max = 5.0f;
  pp.d_lpf_alpha = 1.0f;
  pid_ctrl_t pid;
  pid_init(&pid, &pp);
  /* 纯比例 + 前馈 */
  float o = pid_calc(&pid, 1.0f, 0.0f, 0.5f, 0.001f);
  CHECK_NEAR(o, 0.5 + 2.0 + 10.0 * 1.0 * 0.001, 1e-5);
  /* 积分限幅 */
  for (int k = 0; k < 10000; ++k) {
    pid_calc(&pid, 1.0f, 0.0f, 0.0f, 0.001f);
  }
  CHECK(pid.integral <= 1.0f + 1e-6f);
  /* 输出限幅 */
  CHECK(pid_calc(&pid, 100.0f, 0.0f, 0.0f, 0.001f) <= 5.0f);
  /* 非法 dt 不改变积分 */
  float before = pid.integral;
  pid_calc(&pid, 1.0f, 0.0f, 0.0f, 0.0f);
  CHECK(pid.integral == before);

  /* 积分分离 */
  pp.i_sep = 0.1f;
  pid_init(&pid, &pp);
  pid_calc(&pid, 1.0f, 0.0f, 0.0f, 0.001f);
  CHECK(pid.integral == 0.0f);
  pid_calc(&pid, 0.05f, 0.0f, 0.0f, 0.001f);
  CHECK(pid.integral > 0.0f);

  /* 角度 wrap：179° 到 -179° 的误差是 +2°，不是 -358° */
  pid_params_t pa = {0};
  pa.kp = 1.0f;
  pa.angle_wrap = true;
  pid_init(&pid, &pa);
  float e = pid_calc(&pid, DEG2RAD(-179.0), DEG2RAD(179.0), 0.0f, 0.001f);
  CHECK_NEAR(e, DEG2RAD(2.0), 1e-4);

  CHECK_NEAR(pid_wrap_pi(DEG2RAD(540.0)), DEG2RAD(180.0) - 2 * M_PI * 0 /* ±π 等价 */, 1e-4 + 2 * M_PI);
  CHECK_NEAR(fabsf(pid_wrap_pi(DEG2RAD(540.0))), M_PI, 1e-4);
  CHECK_NEAR(pid_wrap_pi(DEG2RAD(-190.0)), DEG2RAD(170.0), 1e-4);
}

/* ================================================================ 云台状态机 */
typedef struct {
  gimbal_ctrl_t g;
  gimbal_input_t in;
  gimbal_output_t out;
  gm6020_model_t m;
} rig_t;

static void rig_init(rig_t *r) {
  gimbal_config_t cfg;
  gimbal_ctrl_default_config(&cfg);
  gimbal_ctrl_init(&r->g, &cfg, 0);
  memset(&r->in, 0, sizeof(r->in));
  memset(&r->out, 0, sizeof(r->out));
  gm6020_model_params_t mp;
  gm6020_model_default_params(&mp);
  gm6020_model_init(&r->m, &mp, 0.0f);
}

/* 跑 ms 毫秒；cmd_every_ms > 0 时按该周期发 TRACK 指令（yaw 由 yaw_fn(t) 给出） */
static void rig_run(rig_t *r, uint32_t ms, uint32_t cmd_every_ms, float yaw, float vel,
                    uint8_t mode, bool key, bool motor_alive) {
  for (uint32_t k = 0; k < ms; ++k) {
    r->in.now_ms++;
    r->in.new_cmd = false;
    if (cmd_every_ms > 0 && (r->in.now_ms % cmd_every_ms) == 0) {
      r->in.new_cmd = true;
      r->in.cmd.yaw = yaw;
      r->in.cmd.yaw_vel = vel;
      r->in.cmd.mode = mode;
      r->in.cmd.flags = GP_FLAG_TARGET_VALID;
      r->in.cmd.seq++;
    }
    r->in.key_pressed = key;
    r->in.motor_fresh = motor_alive;
    r->in.yaw_fb = r->m.theta;
    r->in.yaw_vel_fb = r->m.omega;
    r->in.temperature = 35;
    gimbal_ctrl_step(&r->g, &r->in, &r->out);
    gm6020_model_step(&r->m, r->out.motor_enable ? r->out.voltage : 0, 0.001f);
  }
}

static void press_key(rig_t *r, uint32_t cmd_every_ms, float yaw, uint8_t mode) {
  rig_run(r, 50, cmd_every_ms, yaw, 0.0f, mode, true, true);
  rig_run(r, 50, cmd_every_ms, yaw, 0.0f, mode, false, true);
}

static void test_state_safe_until_key(void) {
  rig_t r;
  rig_init(&r);
  rig_run(&r, 300, 10, DEG2RAD(10), 0, GP_MODE_TRACK, false, true);
  CHECK(r.out.state == GP_STATE_SAFE);
  CHECK(!r.out.motor_enable);
  CHECK(r.out.voltage == 0);
}

static void test_state_arm_requires_fresh_cmd(void) {
  rig_t r;
  rig_init(&r);
  /* 没有指令时按键：不能解锁 */
  press_key(&r, 0, 0, GP_MODE_TRACK);
  CHECK(r.out.state == GP_STATE_SAFE);
  /* 有指令时按键：解锁 */
  rig_run(&r, 50, 10, 0, 0, GP_MODE_TRACK, false, true);
  press_key(&r, 10, 0, GP_MODE_TRACK);
  CHECK(r.out.state == GP_STATE_ARMED);
}

static void test_track_step(void) {
  rig_t r;
  rig_init(&r);
  rig_run(&r, 50, 10, 0, 0, GP_MODE_TRACK, false, true);
  press_key(&r, 10, 0, GP_MODE_TRACK);
  CHECK(r.out.state == GP_STATE_ARMED);
  rig_run(&r, 1500, 10, DEG2RAD(10), 0, GP_MODE_TRACK, false, true);
  CHECK_NEAR(r.m.theta, DEG2RAD(10), DEG2RAD(0.2));
}

static void test_clamp(void) {
  rig_t r;
  rig_init(&r);
  rig_run(&r, 50, 10, 0, 0, GP_MODE_TRACK, false, true);
  press_key(&r, 10, 0, GP_MODE_TRACK);
  rig_run(&r, 3000, 10, DEG2RAD(90), 0, GP_MODE_TRACK, false, true);
  CHECK(r.out.error_flags & GP_ERR_CMD_CLAMPED);
  CHECK(r.m.theta <= r.g.cfg.yaw_max + DEG2RAD(0.5));
  CHECK(r.out.state == GP_STATE_ARMED);
}

static void test_timeout_hold_then_safe(void) {
  rig_t r;
  rig_init(&r);
  rig_run(&r, 50, 10, 0, 0, GP_MODE_TRACK, false, true);
  press_key(&r, 10, 0, GP_MODE_TRACK);
  rig_run(&r, 1000, 10, DEG2RAD(5), 0, GP_MODE_TRACK, false, true);
  /* 停发指令 150 ms：进入 TIMEOUT，仍然保持角度 */
  rig_run(&r, 150, 0, 0, 0, GP_MODE_TRACK, false, true);
  CHECK(r.out.state == GP_STATE_TIMEOUT);
  CHECK(r.out.motor_enable);
  CHECK(r.out.error_flags & GP_ERR_CMD_TIMEOUT);
  CHECK_NEAR(r.m.theta, DEG2RAD(5), DEG2RAD(0.5));
  /* 恢复指令：回到 ARMED */
  rig_run(&r, 30, 10, DEG2RAD(5), 0, GP_MODE_TRACK, false, true);
  CHECK(r.out.state == GP_STATE_ARMED);
  /* 再断 600 ms：回到 SAFE 卸力 */
  rig_run(&r, 600, 0, 0, 0, GP_MODE_TRACK, false, true);
  CHECK(r.out.state == GP_STATE_SAFE);
  CHECK(!r.out.motor_enable);
}

static void test_interp_limited(void) {
  /* 只发一帧带速度的指令然后断流：外推最多 30 ms，不会一直转下去 */
  rig_t r;
  rig_init(&r);
  rig_run(&r, 50, 10, 0, 0, GP_MODE_TRACK, false, true);
  press_key(&r, 10, 0, GP_MODE_TRACK);
  r.in.now_ms++;
  r.in.new_cmd = true;
  r.in.cmd.yaw = 0.0f;
  r.in.cmd.yaw_vel = 1.0f; /* 1 rad/s */
  r.in.cmd.mode = GP_MODE_TRACK;
  gimbal_ctrl_step(&r.g, &r.in, &r.out);
  rig_run(&r, 90, 0, 0, 0, GP_MODE_TRACK, false, true);
  /* 30 ms × 1 rad/s = 0.03 rad */
  CHECK(r.g.yaw_ref <= 0.03f + 1e-3f);
}

static void test_bad_float_fault(void) {
  rig_t r;
  rig_init(&r);
  rig_run(&r, 50, 10, 0, 0, GP_MODE_TRACK, false, true);
  press_key(&r, 10, 0, GP_MODE_TRACK);
  rig_run(&r, 100, 10, NAN, 0, GP_MODE_TRACK, false, true);
  CHECK(r.out.state == GP_STATE_FAULT);
  CHECK(!r.out.motor_enable);
  CHECK(r.out.error_flags & GP_ERR_BAD_FLOAT);
  /* 故障消失后按键：回到 SAFE（不是直接 ARMED） */
  rig_run(&r, 50, 10, 0, 0, GP_MODE_TRACK, false, true);
  press_key(&r, 10, 0, GP_MODE_TRACK);
  CHECK(r.out.state == GP_STATE_SAFE);
  press_key(&r, 10, 0, GP_MODE_TRACK);
  CHECK(r.out.state == GP_STATE_ARMED);
}

static void test_motor_offline_fault(void) {
  rig_t r;
  rig_init(&r);
  /* SAFE 下电机离线：只报错不进 FAULT（只有 C 板也能看串口） */
  rig_run(&r, 200, 10, 0, 0, GP_MODE_TRACK, false, false);
  CHECK(r.out.state == GP_STATE_SAFE);
  CHECK(r.out.error_flags & GP_ERR_MOTOR_OFFLINE);
  /* 离线时按键不能解锁（按键期间电机也保持离线） */
  rig_run(&r, 50, 10, 0, 0, GP_MODE_TRACK, true, false);
  rig_run(&r, 50, 10, 0, 0, GP_MODE_TRACK, false, false);
  CHECK(r.out.state == GP_STATE_SAFE);
  /* 解锁后电机掉线：进 FAULT */
  rig_run(&r, 100, 10, 0, 0, GP_MODE_TRACK, false, true);
  press_key(&r, 10, 0, GP_MODE_TRACK);
  CHECK(r.out.state == GP_STATE_ARMED);
  rig_run(&r, 100, 10, 0, 0, GP_MODE_TRACK, false, false);
  CHECK(r.out.state == GP_STATE_FAULT);
}

static void test_relax_and_hold_modes(void) {
  rig_t r;
  rig_init(&r);
  rig_run(&r, 50, 10, 0, 0, GP_MODE_RELAX, false, true);
  press_key(&r, 10, 0, GP_MODE_RELAX);
  CHECK(r.out.state == GP_STATE_ARMED);
  rig_run(&r, 100, 10, DEG2RAD(10), 0, GP_MODE_RELAX, false, true);
  CHECK(!r.out.motor_enable);
  /* 先跟到 8°，再切 HOLD 并把指令改到 -10°：应停在 8° */
  rig_run(&r, 1500, 10, DEG2RAD(8), 0, GP_MODE_TRACK, false, true);
  rig_run(&r, 1500, 10, DEG2RAD(-10), 0, GP_MODE_HOLD, false, true);
  CHECK(r.out.motor_enable);
  CHECK_NEAR(r.m.theta, DEG2RAD(8), DEG2RAD(0.5));
}

static void test_feedforward_reduces_lag(void) {
  /* 匀速 40°/s 斜坡：有前馈时稳态跟踪误差明显小于无前馈 */
  float err_ff = 0, err_noff = 0;
  for (int use_ff = 0; use_ff < 2; ++use_ff) {
    rig_t r;
    rig_init(&r);
    r.m.theta = DEG2RAD(-20);
    rig_run(&r, 50, 10, DEG2RAD(-20), 0, GP_MODE_TRACK, false, true);
    press_key(&r, 10, DEG2RAD(-20), GP_MODE_TRACK);
    const float w = DEG2RAD(40);
    float yaw = DEG2RAD(-20);
    double acc = 0;
    int n = 0;
    for (int k = 0; k < 1000; ++k) {
      r.in.now_ms++;
      r.in.new_cmd = (r.in.now_ms % 10) == 0;
      yaw += w * 0.001f;
      if (r.in.new_cmd) {
        r.in.cmd.yaw = yaw;
        r.in.cmd.yaw_vel = use_ff ? w : 0.0f;
        r.in.cmd.mode = GP_MODE_TRACK;
        r.in.cmd.flags = GP_FLAG_TARGET_VALID;
      }
      r.in.motor_fresh = true;
      r.in.key_pressed = false;
      r.in.yaw_fb = r.m.theta;
      r.in.yaw_vel_fb = r.m.omega;
      gimbal_ctrl_step(&r.g, &r.in, &r.out);
      gm6020_model_step(&r.m, r.out.voltage, 0.001f);
      if (k > 500) {
        acc += fabs(yaw - r.m.theta);
        n++;
      }
    }
    if (use_ff) {
      err_ff = (float)(acc / n);
    } else {
      err_noff = (float)(acc / n);
    }
  }
  printf("  ramp 40deg/s mean |err|: no-ff %.3f deg, ff %.3f deg\n", err_noff * 57.2958,
         err_ff * 57.2958);
  CHECK(err_ff < err_noff * 0.5f);
}

static void test_feedback_fill(void) {
  rig_t r;
  rig_init(&r);
  rig_run(&r, 50, 10, 0, 0, GP_MODE_TRACK, false, true);
  r.in.cmd.host_time_ms = 777;
  rig_run(&r, 10, 10, 0, 0, GP_MODE_TRACK, false, true);
  gp_fb_t fb;
  gimbal_ctrl_fill_feedback(&r.g, &r.in, &r.out, &fb);
  CHECK(fb.echo_seq == r.in.cmd.seq);
  CHECK(fb.echo_host_time_ms == 777);
  CHECK(fb.state == GP_STATE_SAFE);
}

int main(void) {
  struct {
    const char *name;
    void (*fn)(void);
  } tests[] = {
      {"crc8", test_crc8},
      {"golden_cmd", test_golden_cmd},
      {"roundtrip", test_roundtrip},
      {"parser_basic", test_parser_basic},
      {"parser_garbage", test_parser_garbage},
      {"parser_crc_resync", test_parser_crc_resync},
      {"parser_payload_header", test_parser_payload_contains_header},
      {"parser_fuzz", test_parser_fuzz},
      {"seq_gap", test_seq_gap},
      {"gm6020_codec", test_gm6020_codec},
      {"gm6020_unwrap", test_gm6020_unwrap},
      {"pid", test_pid},
      {"state_safe_until_key", test_state_safe_until_key},
      {"state_arm_requires_cmd", test_state_arm_requires_fresh_cmd},
      {"track_step", test_track_step},
      {"clamp", test_clamp},
      {"timeout_hold_then_safe", test_timeout_hold_then_safe},
      {"interp_limited", test_interp_limited},
      {"bad_float_fault", test_bad_float_fault},
      {"motor_offline_fault", test_motor_offline_fault},
      {"relax_and_hold", test_relax_and_hold_modes},
      {"feedforward", test_feedforward_reduces_lag},
      {"feedback_fill", test_feedback_fill},
  };
  for (size_t i = 0; i < sizeof(tests) / sizeof(tests[0]); ++i) {
    const int before = g_fail;
    printf("[ RUN  ] %s\n", tests[i].name);
    tests[i].fn();
    printf("[ %s ] %s\n", g_fail == before ? " OK " : "FAIL", tests[i].name);
  }
  printf("\n%d checks passed, %d failed\n", g_pass, g_fail);
  return g_fail == 0 ? 0 : 1;
}
