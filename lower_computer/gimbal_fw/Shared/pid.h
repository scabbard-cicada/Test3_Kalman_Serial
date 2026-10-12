/**
 * @file    pid.h
 * @brief   位置式 PID，参考队内 hello_world::pid::BasicPid 的结构，改写为纯 C
 *
 * 输出：
 *
 *   out = ffd + kp·e + I + D            然后限幅到 [-out_max, out_max]
 *
 *   e = ref - fdb（若 angle_wrap = true，则先把误差 wrap 到 [-π, π]）
 *
 *   I += ki · e · dt                     积分项，带两道保护：
 *        - 积分限幅（抗饱和）：|I| <= i_max
 *        - 积分分离：|e| > i_sep 时本周期不累积（大误差时只靠 P 拉回来，避免超调）
 *
 *   D  = -kd · d(fdb)/dt                 微分作用在反馈上而不是误差上（微分先行），
 *                                         这样 ref 突变时 D 不会产生尖峰；再过一阶低通
 *
 * ffd 是前馈：上位机发来的角速度指令直接加到速度环参考上，不需要误差先积累起来。
 *
 * 只依赖 <math.h>，不依赖 HAL；dt 由调用者传入（控制周期固定 1 ms）。
 */
#ifndef GIMBAL_PID_H_
#define GIMBAL_PID_H_

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
  float kp;
  float ki;
  float kd;
  float i_max;       /* 积分项绝对值上限，<= 0 表示不限 */
  float i_sep;       /* 积分分离阈值，|e| 超过它时不积分；<= 0 表示不分离 */
  float out_max;     /* 输出绝对值上限，<= 0 表示不限 */
  float d_lpf_alpha; /* 微分低通系数 (0,1]，1 表示不滤波 */
  bool angle_wrap;   /* 误差是否按角度 wrap 到 [-π, π] */
} pid_params_t;

typedef struct {
  pid_params_t p;
  float integral;
  float d_filtered;
  float last_fdb;
  bool has_last;
  /* 最近一次计算的分量，便于调试和回传 */
  float err;
  float p_out;
  float i_out;
  float d_out;
  float out;
} pid_ctrl_t;

void pid_init(pid_ctrl_t *pid, const pid_params_t *params);
/** 清空积分与微分历史（切换模式、解锁时调用） */
void pid_reset(pid_ctrl_t *pid);
/** 计算一次。dt 单位 s，必须 > 0 */
float pid_calc(pid_ctrl_t *pid, float ref, float fdb, float ffd, float dt);

/** 把角度规范到 [-π, π] */
float pid_wrap_pi(float a);
/** 限幅 */
float pid_clampf(float v, float lo, float hi);

#ifdef __cplusplus
}
#endif

#endif /* GIMBAL_PID_H_ */
