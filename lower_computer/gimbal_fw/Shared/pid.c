/**
 * @file    pid.c
 */
#include "pid.h"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

float pid_wrap_pi(float a) {
  const float two_pi = (float)(2.0 * M_PI);
  a = fmodf(a + (float)M_PI, two_pi);
  if (a < 0.0f) {
    a += two_pi;
  }
  return a - (float)M_PI;
}

float pid_clampf(float v, float lo, float hi) {
  if (v < lo) {
    return lo;
  }
  if (v > hi) {
    return hi;
  }
  return v;
}

void pid_init(pid_ctrl_t *pid, const pid_params_t *params) {
  pid->p = *params;
  if (!(pid->p.d_lpf_alpha > 0.0f) || pid->p.d_lpf_alpha > 1.0f) {
    pid->p.d_lpf_alpha = 1.0f;
  }
  pid_reset(pid);
}

void pid_reset(pid_ctrl_t *pid) {
  pid->integral = 0.0f;
  pid->d_filtered = 0.0f;
  pid->last_fdb = 0.0f;
  pid->has_last = false;
  pid->err = pid->p_out = pid->i_out = pid->d_out = pid->out = 0.0f;
}

float pid_calc(pid_ctrl_t *pid, float ref, float fdb, float ffd, float dt) {
  const pid_params_t *p = &pid->p;
  if (!(dt > 0.0f)) {
    return pid->out; /* 非法 dt：保持上次输出，不污染积分 */
  }

  float e = ref - fdb;
  if (p->angle_wrap) {
    e = pid_wrap_pi(e);
  }
  pid->err = e;

  /* P */
  pid->p_out = p->kp * e;

  /* I：积分分离 + 积分限幅 */
  if (p->ki != 0.0f) {
    const bool separated = (p->i_sep > 0.0f) && (fabsf(e) > p->i_sep);
    if (!separated) {
      pid->integral += p->ki * e * dt;
    }
    if (p->i_max > 0.0f) {
      pid->integral = pid_clampf(pid->integral, -p->i_max, p->i_max);
    }
  } else {
    pid->integral = 0.0f;
  }
  pid->i_out = pid->integral;

  /* D：作用在反馈上，带一阶低通 */
  if (p->kd != 0.0f && pid->has_last) {
    float dfdb = fdb - pid->last_fdb;
    if (p->angle_wrap) {
      dfdb = pid_wrap_pi(dfdb);
    }
    const float d_raw = -p->kd * dfdb / dt;
    pid->d_filtered += p->d_lpf_alpha * (d_raw - pid->d_filtered);
  } else if (p->kd == 0.0f) {
    pid->d_filtered = 0.0f;
  }
  pid->d_out = pid->d_filtered;
  pid->last_fdb = fdb;
  pid->has_last = true;

  float out = ffd + pid->p_out + pid->i_out + pid->d_out;
  if (p->out_max > 0.0f) {
    const float unclamped = out;
    out = pid_clampf(out, -p->out_max, p->out_max);
    /* 条件积分（抗饱和的第二道保护）：输出已饱和且误差还在把它往饱和方向推，
       就撤销本周期的积分增量，防止饱和期间积分持续变大 */
    if (p->ki != 0.0f && out != unclamped && (e * unclamped) > 0.0f) {
      const bool separated = (p->i_sep > 0.0f) && (fabsf(e) > p->i_sep);
      if (!separated) {
        pid->integral -= p->ki * e * dt;
        pid->i_out = pid->integral;
      }
    }
  }
  pid->out = out;
  return out;
}
