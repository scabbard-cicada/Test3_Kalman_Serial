/**
 * @file    gimbal_proto.h
 * @brief   上下位机串口协议（沿用队内 C1/C2 帧格式），单轴 yaw 版本
 *
 * 帧结构（所有包相同）：
 *
 *   | 0      | 1      | 2 .. N+1 | N+2  |
 *   | Header | Length | Payload  | CRC8 |
 *
 *   - Header：包 ID，同时充当帧头
 *   - Length：整帧字节数（Header + Length + Payload + CRC），用来做长度校验
 *   - CRC8  ：覆盖 byte 0 ~ N+1；多项式 0x07，初值 0x00，不反转，不异或
 *             校验值：crc8("123456789") == 0xF4
 *   - 多字节字段一律小端，float 为 IEEE-754 单精度
 *
 * 包定义：
 *
 *   0xC2  PC -> MCU  云台指令  18 B
 *         yaw(f32,rad)  yaw_vel(f32,rad/s)  mode(u8)  flags(u8)
 *         seq(u8)  host_time_ms(u32)
 *
 *   0xC1  MCU -> PC  云台反馈  25 B
 *         yaw(f32,rad)  yaw_vel(f32,rad/s)  yaw_ref(f32,rad)  current(i16,原始值)
 *         temperature(u8,°C)  state(u8)  error_flags(u8)  echo_seq(u8)
 *         echo_host_time_ms(u32)
 *
 * 本文件是协议的唯一参考实现：固件、软件下位机 fake_mcu 与主机单元测试都直接编译它。
 * 只依赖 C99 标准库，不依赖 HAL / ROS，也不使用动态内存。
 */
#ifndef GIMBAL_PROTO_H_
#define GIMBAL_PROTO_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ---------------------------------------------------------------- 包 ID / 长度 */
#define GP_HEADER_CMD 0xC2u /* PC -> MCU */
#define GP_HEADER_FB 0xC1u  /* MCU -> PC */

#define GP_CMD_LEN 18u /* 2 + (4+4+1+1+1+4 = 15) + 1 */
#define GP_FB_LEN 25u  /* 2 + (4+4+4+2+1+1+1+1+4 = 22) + 1 */
#define GP_MAX_LEN 25u

/* ---------------------------------------------------------------- 枚举 */
/** 上位机请求的云台模式 */
typedef enum {
  GP_MODE_RELAX = 0, /* 电机卸力 */
  GP_MODE_TRACK = 1, /* 跟随 yaw 指令 */
  GP_MODE_HOLD = 2,  /* 保持当前角度 */
} gp_mode_t;

/** flags 位定义（C2） */
#define GP_FLAG_TARGET_VALID 0x01u /* 跟踪器认为目标有效 */
#define GP_FLAG_LASER_ON 0x02u     /* 请求打开激光 */
#define GP_FLAG_OUTLIER 0x04u      /* 最近一次观测被判为离群点，只用于指示灯 */

/** 下位机安全状态（C1.state），见 gimbal_ctrl.h */
typedef enum {
  GP_STATE_SAFE = 0,    /* 上电默认：卸力，等待按键解锁 */
  GP_STATE_ARMED = 1,   /* 已解锁，按 mode 执行 */
  GP_STATE_TIMEOUT = 2, /* 指令中断：保持最后角度 */
  GP_STATE_FAULT = 3,   /* 故障：卸力，需按键清除 */
} gp_state_t;

/** error_flags 位定义（C1） */
#define GP_ERR_MOTOR_OFFLINE 0x01u /* 电机反馈超时 */
#define GP_ERR_OVER_TEMP 0x02u     /* 电机过温 */
#define GP_ERR_CMD_CLAMPED 0x04u   /* 指令超出限位被钳位 */
#define GP_ERR_CMD_TIMEOUT 0x08u   /* 指令超时 */
#define GP_ERR_BAD_FLOAT 0x10u     /* 收到 NaN / Inf */
#define GP_ERR_CAN 0x20u           /* CAN 发送失败或 bus-off */
#define GP_ERR_UART 0x40u          /* 串口溢出 / 帧错误 */
#define GP_ERR_WDG_RESET 0x80u     /* 上次复位来自看门狗 */

/* ---------------------------------------------------------------- 载荷结构 */
/** 0xC2 云台指令（主机侧结构，不是线上布局） */
typedef struct {
  float yaw;             /* rad，云台系 */
  float yaw_vel;         /* rad/s，前馈 */
  uint8_t mode;          /* gp_mode_t */
  uint8_t flags;         /* GP_FLAG_* */
  uint8_t seq;           /* 逐帧 +1，255 后回到 0 */
  uint32_t host_time_ms; /* 上位机时钟，下位机原样回传，用于测往返时延 */
} gp_cmd_t;

/** 0xC1 云台反馈 */
typedef struct {
  float yaw;                  /* rad，云台系，实测 */
  float yaw_vel;              /* rad/s，实测 */
  float yaw_ref;              /* rad，插值/限幅后真正送进控制环的参考 */
  int16_t current;            /* GM6020 转矩电流原始值 */
  uint8_t temperature;        /* °C */
  uint8_t state;              /* gp_state_t */
  uint8_t error_flags;        /* GP_ERR_* */
  uint8_t echo_seq;           /* 最近一次被采纳的 C2.seq */
  uint32_t echo_host_time_ms; /* 最近一次被采纳的 C2.host_time_ms */
} gp_fb_t;

/* ---------------------------------------------------------------- CRC8 */
uint8_t gp_crc8(const uint8_t *data, size_t len);

/* ---------------------------------------------------------------- 编码 */
/** 编码 C2，返回写入字节数（GP_CMD_LEN）；buf_len 不足返回 0 */
size_t gp_encode_cmd(const gp_cmd_t *cmd, uint8_t *buf, size_t buf_len);
/** 编码 C1，返回写入字节数（GP_FB_LEN）；buf_len 不足返回 0 */
size_t gp_encode_fb(const gp_fb_t *fb, uint8_t *buf, size_t buf_len);

/* ---------------------------------------------------------------- 解码（整帧） */
/** 输入一帧完整的 C2（含 header 与 CRC），校验通过返回 true */
bool gp_decode_cmd(const uint8_t *frame, size_t len, gp_cmd_t *out);
/** 输入一帧完整的 C1（含 header 与 CRC），校验通过返回 true */
bool gp_decode_fb(const uint8_t *frame, size_t len, gp_fb_t *out);

/* ---------------------------------------------------------------- 流式解析 */
/**
 * 字节流解析器。一次喂一个字节，和串口读到的块大小无关。
 *
 * 只接受 accept_header 指定的那一种包（MCU 只收 C2，PC 只收 C1），
 * 期望长度由包 ID 唯一确定。
 *
 * 失败重同步：长度字节不对或 CRC 不过时，不整段丢弃，而是从这次"帧头"的下一个字节
 * 开始重新找帧头。原因是这次的帧头可能是上一帧载荷里碰巧出现的 0xC1/0xC2，
 * 真正的帧头就在已读入的字节里。
 */
typedef struct {
  uint8_t accept_header;
  uint8_t expect_len;
  uint8_t buf[GP_MAX_LEN];
  uint8_t pos;
  /* 统计 */
  uint32_t frames_ok;
  uint32_t crc_errors;
  uint32_t len_errors;
  uint32_t skipped_bytes; /* 等帧头时丢掉的字节 */
} gp_parser_t;

void gp_parser_init(gp_parser_t *p, uint8_t accept_header);

/**
 * 喂一段字节。每解出一帧调用一次 on_frame(frame, len, user)。
 * frame 指向解析器内部缓冲，只在回调期间有效；回调里不能再对同一个解析器调用 feed。
 */
typedef void (*gp_frame_cb_t)(const uint8_t *frame, size_t len, void *user);
void gp_parser_feed(gp_parser_t *p, const uint8_t *data, size_t len, gp_frame_cb_t on_frame,
                    void *user);

/** 序号差：期望 last+1，实际 seq；返回中间丢掉的帧数（0~255） */
static inline uint8_t gp_seq_gap(uint8_t last, uint8_t seq) { return (uint8_t)(seq - last - 1u); }

#ifdef __cplusplus
}
#endif

#endif /* GIMBAL_PROTO_H_ */
