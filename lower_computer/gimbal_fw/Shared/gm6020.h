/**
 * @file    gm6020.h
 * @brief   GM6020 CAN 报文编解码（与 HAL 无关），以及一个用于仿真的电机模型
 *
 * 报文（标准帧，DLC = 8，大端）：
 *
 *   反馈  ID = 0x204 + motor_id（motor_id = 1 时为 0x205），1 kHz
 *     byte 0-1  转子机械角度  0 ~ 8191 对应 0 ~ 360°
 *     byte 2-3  转速          rpm，有符号
 *     byte 4-5  实际转矩电流  有符号，原始值
 *     byte 6    温度          °C
 *
 *   控制  电压模式 ID = 0x1FF（motor_id 1~4），byte[2(id-1) .. 2(id-1)+1] 为电压给定，
 *         有符号，范围 ±GM6020_VOLTAGE_MAX
 *
 * 电压给定的上限：队内组件库按 ±25000 处理（HW-Components motor_GM6020.cpp）；
 * 部分固件版本标称 ±30000。这里取 25000，两种固件都安全。
 */
#ifndef GIMBAL_GM6020_H_
#define GIMBAL_GM6020_H_

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define GM6020_ENCODER_RANGE 8192
#define GM6020_FB_ID_BASE 0x204u
#define GM6020_CTRL_ID_1_4 0x1FFu
#define GM6020_VOLTAGE_MAX 25000
#define GM6020_VOLTAGE_MAX_F 25000.0f

/** 反馈帧解码结果（原始量） */
typedef struct {
  uint16_t ecd;    /* 0 ~ 8191 */
  int16_t rpm;     /* 转速 */
  int16_t current; /* 转矩电流原始值 */
  uint8_t temperature;
} gm6020_raw_t;

/** 解码反馈帧，len 必须为 8 */
bool gm6020_decode(const uint8_t data[8], uint8_t len, gm6020_raw_t *out);
/** 编码控制帧中第 motor_id 个电机的电压（motor_id 1~4），其余槽位不动 */
void gm6020_encode_voltage(uint8_t data[8], uint8_t motor_id, int16_t voltage);
/** 编码反馈帧（给仿真电机用） */
void gm6020_encode_feedback(uint8_t data[8], const gm6020_raw_t *raw);

/**
 * 角度展开：把 0~8191 的编码器值换算成连续角（rad），跨过 0/8191 时自动加减一圈。
 * offset_ecd：机械零位对应的编码器值；dir：+1 或 -1，用于和云台正方向对齐。
 */
typedef struct {
  uint16_t offset_ecd;
  int8_t dir;
  bool has_last;
  uint16_t last_ecd;
  int32_t turns;
} gm6020_unwrap_t;

void gm6020_unwrap_init(gm6020_unwrap_t *u, uint16_t offset_ecd, int8_t dir);
/** 返回云台系连续角（rad） */
float gm6020_unwrap_update(gm6020_unwrap_t *u, uint16_t ecd);
/** rpm -> rad/s，并乘方向 */
float gm6020_rpm_to_rads(int16_t rpm, int8_t dir);

/* ---------------------------------------------------------------- 仿真电机
 * 电压驱动的直流电机二阶模型：
 *
 *   L·di/dt = u·V_bus/25000 - R·i - Ke·ω
 *   J·dω/dt = Kt·i - b·ω - τ_c·sign(ω)
 *
 * 默认参数按 GM6020 手册量级取（空载转速约 320 rpm @24V、额定转矩 1.2 N·m），
 * 并在转子上挂了一个小惯量负载。用于 fake_mcu 和固件的 VIRTUAL 后端：
 * 不追求和实物一致，只要求"量级合理、PID 调得动、能看出前馈的作用"。
 */
typedef struct {
  float J;      /* kg·m²，转子 + 负载 */
  float b;      /* N·m·s/rad，粘滞摩擦 */
  float tau_c;  /* N·m，库仑摩擦 */
  float R;      /* Ω */
  float L;      /* H */
  float Kt;     /* N·m/A */
  float Ke;     /* V·s/rad */
  float V_bus;  /* V，电压给定 25000 对应的电压 */
  float i_max;  /* A，电流饱和 */
} gm6020_model_params_t;

typedef struct {
  gm6020_model_params_t p;
  float theta; /* rad，转子角（连续） */
  float omega; /* rad/s */
  float i;     /* A */
  float temperature;
} gm6020_model_t;

void gm6020_model_default_params(gm6020_model_params_t *p);
void gm6020_model_init(gm6020_model_t *m, const gm6020_model_params_t *p, float theta0);
/** 推进 dt 秒，voltage_cmd 为电压给定原始值 */
void gm6020_model_step(gm6020_model_t *m, int16_t voltage_cmd, float dt);
/** 按真实电机的格式输出一帧反馈原始量 */
void gm6020_model_raw(const gm6020_model_t *m, gm6020_raw_t *out);

#ifdef __cplusplus
}
#endif

#endif /* GIMBAL_GM6020_H_ */
