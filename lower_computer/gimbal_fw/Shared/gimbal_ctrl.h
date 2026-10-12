/**
 * @file    gimbal_ctrl.h
 * @brief   单轴云台控制：安全状态机 + 参考生成 + 串级 PID
 *
 * 每 1 ms 调用一次 gimbal_ctrl_step()，输入是"这一刻知道的一切"（最近一条上位机指令、
 * 电机反馈、按键），输出是电机电压给定和要回传的状态。函数本身不碰硬件，固件、
 * fake_mcu 和单元测试调的是同一份代码。
 *
 *   上位机 C2 ──► 指令新鲜度/合法性检查 ──► 安全状态机 ──► 参考生成 ──► 串级 PID ──► 电压
 *                                                │            │
 *                                    SAFE/ARMED/TIMEOUT/FAULT  插值 + 限位 + 限速
 *
 * 串级 PID（外环角度、内环速度），参考队内 Radar-2026 的 MultiNodesPid 级联结构：
 *
 *   ω_ref = PID_angle(yaw_ref, yaw_fb) + yaw_vel_cmd     ← 上位机给的角速度作为前馈
 *   u     = PID_speed(ω_ref, ω_fb)                       ← 输出 GM6020 电压给定
 *
 * 只靠角度环时，目标匀速转动也需要先产生角度误差才能让电机跟着转，必然滞后；
 * 前馈直接把"目标正以多快速度转"告诉速度环，误差只需修正剩余部分。
 */
#ifndef GIMBAL_CTRL_H_
#define GIMBAL_CTRL_H_

#include <stdbool.h>
#include <stdint.h>

#include "gimbal_proto.h"
#include "gm6020.h"
#include "pid.h"

#ifdef __cplusplus
extern "C" {
#endif

/** 可配置参数，默认值见 gimbal_ctrl_default_config() */
typedef struct {
  /* 机械限位（云台系，rad）。指令超出时钳位，并置 GP_ERR_CMD_CLAMPED */
  float yaw_min;
  float yaw_max;
  /* 参考角速度上限（rad/s），同时限制前馈和参考变化率 */
  float yaw_vel_max;

  /* 两帧指令之间按 yaw_vel 线性外推的最长时间（ms）。上位机 100 Hz 时两帧相隔 10 ms，
     超过这个时间说明丢帧了，外推就停住，防止断流后一直往一个方向转 */
  uint32_t interp_max_ms;

  /* 指令超时（ms）：超过 cmd_timeout_ms 没收到有效指令进入 TIMEOUT，保持最后角度；
     超过 relax_timeout_ms 回到 SAFE 卸力 */
  uint32_t cmd_timeout_ms;
  uint32_t relax_timeout_ms;

  /* 解锁条件：按键按下时，最近 arm_fresh_ms 内必须收到过有效指令 */
  uint32_t arm_fresh_ms;
  /* 按键消抖（ms） */
  uint32_t key_debounce_ms;

  /* 故障判定 */
  uint32_t motor_offline_ms; /* 电机反馈超时 */
  uint8_t over_temp_c;       /* 电机过温 */
  uint8_t bad_float_limit;   /* 连续多少帧非有限浮点数判故障 */

  /* 输出缩放 (0,1]：首次上电测试建议 0.2，确认方向和零位后再改成 1 */
  float output_scale;

  pid_params_t angle_pid; /* 输出 rad/s */
  pid_params_t speed_pid; /* 输出 GM6020 电压给定 */
} gimbal_config_t;

/** 每个控制周期的输入 */
typedef struct {
  uint32_t now_ms;

  /* 最近一次解析成功的 C2，new_cmd = true 表示是本周期新到的 */
  bool new_cmd;
  gp_cmd_t cmd;

  /* 电机反馈，motor_fresh = true 表示本周期收到过新反馈 */
  bool motor_fresh;
  float yaw_fb;     /* rad，云台系连续角 */
  float yaw_vel_fb; /* rad/s */
  int16_t current_raw;
  uint8_t temperature;

  bool key_pressed; /* 物理按键当前电平（按下为 true） */
  bool can_error;   /* CAN 发送失败 / bus-off */
  bool uart_error;  /* 串口溢出 / 帧错误 */
} gimbal_input_t;

/** 每个控制周期的输出 */
typedef struct {
  int16_t voltage; /* 发给 GM6020 的电压给定 */
  bool motor_enable;
  bool laser_on;
  gp_state_t state;
  uint8_t error_flags;
  float yaw_ref; /* 真正送进角度环的参考 */
} gimbal_output_t;

typedef struct {
  gimbal_config_t cfg;
  pid_ctrl_t angle_pid;
  pid_ctrl_t speed_pid;

  gp_state_t state;
  uint8_t sticky_errors; /* 本次运行累积过的错误位，直到按键清除 */

  /* 指令 */
  bool has_cmd;
  gp_cmd_t last_cmd;
  uint32_t last_cmd_ms;
  uint8_t bad_float_count;
  bool cmd_clamped;

  /* 参考 */
  float yaw_ref;
  bool ref_init;
  float hold_yaw; /* HOLD / TIMEOUT 时保持的角度 */

  /* 电机 */
  uint32_t last_motor_ms;
  bool has_motor;

  /* 按键（消抖后的上升沿） */
  bool key_stable;
  bool key_raw_last;
  uint32_t key_change_ms;
  bool key_edge;

  bool wdg_reset; /* 上电时由固件设置，只用于回传 */
} gimbal_ctrl_t;

void gimbal_ctrl_default_config(gimbal_config_t *cfg);
void gimbal_ctrl_init(gimbal_ctrl_t *g, const gimbal_config_t *cfg, uint32_t now_ms);
void gimbal_ctrl_step(gimbal_ctrl_t *g, const gimbal_input_t *in, gimbal_output_t *out);
/** 用当前状态填一帧 C1 */
void gimbal_ctrl_fill_feedback(const gimbal_ctrl_t *g, const gimbal_input_t *in,
                               const gimbal_output_t *out, gp_fb_t *fb);

#ifdef __cplusplus
}
#endif

#endif /* GIMBAL_CTRL_H_ */
