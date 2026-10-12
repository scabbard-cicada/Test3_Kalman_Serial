/**
 * @file    gimbal_proto.c
 * @brief   C1/C2 协议实现，见 gimbal_proto.h
 */
#include "gimbal_proto.h"

#include <string.h>

/* ---------------------------------------------------------------- CRC8 */
/* 多项式 0x07，初值 0x00。与 autoaim / Radaraim 现有 C1/C2 协议使用的表完全一致。 */
static const uint8_t kCrc8Table[256] = {
    0x00, 0x07, 0x0e, 0x09, 0x1c, 0x1b, 0x12, 0x15, 0x38, 0x3f, 0x36, 0x31, 0x24, 0x23, 0x2a, 0x2d,
    0x70, 0x77, 0x7e, 0x79, 0x6c, 0x6b, 0x62, 0x65, 0x48, 0x4f, 0x46, 0x41, 0x54, 0x53, 0x5a, 0x5d,
    0xe0, 0xe7, 0xee, 0xe9, 0xfc, 0xfb, 0xf2, 0xf5, 0xd8, 0xdf, 0xd6, 0xd1, 0xc4, 0xc3, 0xca, 0xcd,
    0x90, 0x97, 0x9e, 0x99, 0x8c, 0x8b, 0x82, 0x85, 0xa8, 0xaf, 0xa6, 0xa1, 0xb4, 0xb3, 0xba, 0xbd,
    0xc7, 0xc0, 0xc9, 0xce, 0xdb, 0xdc, 0xd5, 0xd2, 0xff, 0xf8, 0xf1, 0xf6, 0xe3, 0xe4, 0xed, 0xea,
    0xb7, 0xb0, 0xb9, 0xbe, 0xab, 0xac, 0xa5, 0xa2, 0x8f, 0x88, 0x81, 0x86, 0x93, 0x94, 0x9d, 0x9a,
    0x27, 0x20, 0x29, 0x2e, 0x3b, 0x3c, 0x35, 0x32, 0x1f, 0x18, 0x11, 0x16, 0x03, 0x04, 0x0d, 0x0a,
    0x57, 0x50, 0x59, 0x5e, 0x4b, 0x4c, 0x45, 0x42, 0x6f, 0x68, 0x61, 0x66, 0x73, 0x74, 0x7d, 0x7a,
    0x89, 0x8e, 0x87, 0x80, 0x95, 0x92, 0x9b, 0x9c, 0xb1, 0xb6, 0xbf, 0xb8, 0xad, 0xaa, 0xa3, 0xa4,
    0xf9, 0xfe, 0xf7, 0xf0, 0xe5, 0xe2, 0xeb, 0xec, 0xc1, 0xc6, 0xcf, 0xc8, 0xdd, 0xda, 0xd3, 0xd4,
    0x69, 0x6e, 0x67, 0x60, 0x75, 0x72, 0x7b, 0x7c, 0x51, 0x56, 0x5f, 0x58, 0x4d, 0x4a, 0x43, 0x44,
    0x19, 0x1e, 0x17, 0x10, 0x05, 0x02, 0x0b, 0x0c, 0x21, 0x26, 0x2f, 0x28, 0x3d, 0x3a, 0x33, 0x34,
    0x4e, 0x49, 0x40, 0x47, 0x52, 0x55, 0x5c, 0x5b, 0x76, 0x71, 0x78, 0x7f, 0x6a, 0x6d, 0x64, 0x63,
    0x3e, 0x39, 0x30, 0x37, 0x22, 0x25, 0x2c, 0x2b, 0x06, 0x01, 0x08, 0x0f, 0x1a, 0x1d, 0x14, 0x13,
    0xae, 0xa9, 0xa0, 0xa7, 0xb2, 0xb5, 0xbc, 0xbb, 0x96, 0x91, 0x98, 0x9f, 0x8a, 0x8d, 0x84, 0x83,
    0xde, 0xd9, 0xd0, 0xd7, 0xc2, 0xc5, 0xcc, 0xcb, 0xe6, 0xe1, 0xe8, 0xef, 0xfa, 0xfd, 0xf4, 0xf3,
};

uint8_t gp_crc8(const uint8_t *data, size_t len) {
  uint8_t crc = 0x00u;
  for (size_t i = 0; i < len; ++i) {
    crc = kCrc8Table[crc ^ data[i]];
  }
  return crc;
}

/* ---------------------------------------------------------------- 逐字段读写
 * 逐字节写入，float 用 memcpy 转换，不用 packed 结构体强转：
 *   1. Cortex-M4 的 VLDR/VSTR 不支持非对齐访问，packed 结构体里的 float 在某些编译选项下
 *      会触发 UsageFault；
 *   2. 不依赖编译器的打包行为和主机字节序（下面显式按小端拆字节）。
 */
static uint8_t *put_u8(uint8_t *p, uint8_t v) {
  *p = v;
  return p + 1;
}
static uint8_t *put_u16(uint8_t *p, uint16_t v) {
  p[0] = (uint8_t)(v & 0xFFu);
  p[1] = (uint8_t)(v >> 8);
  return p + 2;
}
static uint8_t *put_u32(uint8_t *p, uint32_t v) {
  p[0] = (uint8_t)(v & 0xFFu);
  p[1] = (uint8_t)((v >> 8) & 0xFFu);
  p[2] = (uint8_t)((v >> 16) & 0xFFu);
  p[3] = (uint8_t)(v >> 24);
  return p + 4;
}
static uint8_t *put_f32(uint8_t *p, float v) {
  uint32_t u;
  memcpy(&u, &v, sizeof(u));
  return put_u32(p, u);
}
static const uint8_t *get_u8(const uint8_t *p, uint8_t *v) {
  *v = p[0];
  return p + 1;
}
static const uint8_t *get_u16(const uint8_t *p, uint16_t *v) {
  *v = (uint16_t)(p[0] | ((uint16_t)p[1] << 8));
  return p + 2;
}
static const uint8_t *get_u32(const uint8_t *p, uint32_t *v) {
  *v = (uint32_t)p[0] | ((uint32_t)p[1] << 8) | ((uint32_t)p[2] << 16) | ((uint32_t)p[3] << 24);
  return p + 4;
}
static const uint8_t *get_f32(const uint8_t *p, float *v) {
  uint32_t u;
  p = get_u32(p, &u);
  memcpy(v, &u, sizeof(u));
  return p;
}

/* ---------------------------------------------------------------- 编码 */
size_t gp_encode_cmd(const gp_cmd_t *cmd, uint8_t *buf, size_t buf_len) {
  if (cmd == NULL || buf == NULL || buf_len < GP_CMD_LEN) {
    return 0;
  }
  uint8_t *p = buf;
  p = put_u8(p, GP_HEADER_CMD);
  p = put_u8(p, GP_CMD_LEN);
  p = put_f32(p, cmd->yaw);
  p = put_f32(p, cmd->yaw_vel);
  p = put_u8(p, cmd->mode);
  p = put_u8(p, cmd->flags);
  p = put_u8(p, cmd->seq);
  p = put_u32(p, cmd->host_time_ms);
  *p = gp_crc8(buf, GP_CMD_LEN - 1u);
  return GP_CMD_LEN;
}

size_t gp_encode_fb(const gp_fb_t *fb, uint8_t *buf, size_t buf_len) {
  if (fb == NULL || buf == NULL || buf_len < GP_FB_LEN) {
    return 0;
  }
  uint8_t *p = buf;
  p = put_u8(p, GP_HEADER_FB);
  p = put_u8(p, GP_FB_LEN);
  p = put_f32(p, fb->yaw);
  p = put_f32(p, fb->yaw_vel);
  p = put_f32(p, fb->yaw_ref);
  p = put_u16(p, (uint16_t)fb->current);
  p = put_u8(p, fb->temperature);
  p = put_u8(p, fb->state);
  p = put_u8(p, fb->error_flags);
  p = put_u8(p, fb->echo_seq);
  p = put_u32(p, fb->echo_host_time_ms);
  *p = gp_crc8(buf, GP_FB_LEN - 1u);
  return GP_FB_LEN;
}

/* ---------------------------------------------------------------- 解码 */
static bool frame_ok(const uint8_t *frame, size_t len, uint8_t header, uint8_t expect_len) {
  return frame != NULL && len == expect_len && frame[0] == header && frame[1] == expect_len &&
         gp_crc8(frame, expect_len - 1u) == frame[expect_len - 1u];
}

bool gp_decode_cmd(const uint8_t *frame, size_t len, gp_cmd_t *out) {
  if (out == NULL || !frame_ok(frame, len, GP_HEADER_CMD, GP_CMD_LEN)) {
    return false;
  }
  const uint8_t *p = frame + 2;
  p = get_f32(p, &out->yaw);
  p = get_f32(p, &out->yaw_vel);
  p = get_u8(p, &out->mode);
  p = get_u8(p, &out->flags);
  p = get_u8(p, &out->seq);
  (void)get_u32(p, &out->host_time_ms);
  return true;
}

bool gp_decode_fb(const uint8_t *frame, size_t len, gp_fb_t *out) {
  if (out == NULL || !frame_ok(frame, len, GP_HEADER_FB, GP_FB_LEN)) {
    return false;
  }
  const uint8_t *p = frame + 2;
  uint16_t cur;
  p = get_f32(p, &out->yaw);
  p = get_f32(p, &out->yaw_vel);
  p = get_f32(p, &out->yaw_ref);
  p = get_u16(p, &cur);
  out->current = (int16_t)cur;
  p = get_u8(p, &out->temperature);
  p = get_u8(p, &out->state);
  p = get_u8(p, &out->error_flags);
  p = get_u8(p, &out->echo_seq);
  (void)get_u32(p, &out->echo_host_time_ms);
  return true;
}

/* ---------------------------------------------------------------- 流式解析 */
static uint8_t expected_len_of(uint8_t header) {
  switch (header) {
    case GP_HEADER_CMD:
      return GP_CMD_LEN;
    case GP_HEADER_FB:
      return GP_FB_LEN;
    default:
      return 0;
  }
}

void gp_parser_init(gp_parser_t *p, uint8_t accept_header) {
  memset(p, 0, sizeof(*p));
  p->accept_header = accept_header;
  p->expect_len = expected_len_of(accept_header);
}

/* 丢掉 buf 开头 n 个字节 */
static void drop_front(gp_parser_t *p, uint8_t n) {
  if (n >= p->pos) {
    p->pos = 0;
    return;
  }
  memmove(p->buf, p->buf + n, (size_t)(p->pos - n));
  p->pos = (uint8_t)(p->pos - n);
}

/*
 * 尽可能从 buf 中解出帧。返回时满足 pos < 2 或 pos < expect_len，
 * 所以再追加一个字节也不会超过 GP_MAX_LEN。
 *
 * 失败（长度不对 / CRC 不过）时只丢掉 buf[0] 这一个候选帧头，然后从 buf[1] 开始
 * 重新找帧头 —— 已读入但还没确认的字节都会被重新检查一遍。
 */
static void try_parse(gp_parser_t *p, gp_frame_cb_t on_frame, void *user) {
  for (;;) {
    /* 1. 对齐到帧头 */
    uint8_t i = 0;
    while (i < p->pos && p->buf[i] != p->accept_header) {
      ++i;
    }
    if (i > 0) {
      p->skipped_bytes += i;
      drop_front(p, i);
    }
    if (p->pos < 2) {
      return; /* 还没有长度字节 */
    }

    /* 2. 长度字节必须与包 ID 对应的固定长度一致 */
    if (p->buf[1] != p->expect_len) {
      p->len_errors++;
      drop_front(p, 1);
      continue;
    }
    if (p->pos < p->expect_len) {
      return; /* 等后续字节 */
    }

    /* 3. 整帧到齐，校验 CRC */
    if (gp_crc8(p->buf, p->expect_len - 1u) == p->buf[p->expect_len - 1u]) {
      p->frames_ok++;
      if (on_frame != NULL) {
        on_frame(p->buf, p->expect_len, user);
      }
      drop_front(p, p->expect_len);
    } else {
      p->crc_errors++;
      drop_front(p, 1);
    }
  }
}

void gp_parser_feed(gp_parser_t *p, const uint8_t *data, size_t len, gp_frame_cb_t on_frame,
                    void *user) {
  if (p->expect_len == 0) {
    return; /* 未用合法包 ID 初始化 */
  }
  for (size_t k = 0; k < len; ++k) {
    p->buf[p->pos++] = data[k];
    try_parse(p, on_frame, user);
  }
}
