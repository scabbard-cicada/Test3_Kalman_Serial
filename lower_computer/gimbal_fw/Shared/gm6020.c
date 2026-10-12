/**
 * @file    gm6020.c
 */
#include "gm6020.h"

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define TWO_PI_F ((float)(2.0 * M_PI))
#define RPM_TO_RADS ((float)(2.0 * M_PI / 60.0))

/* ---------------------------------------------------------------- 编解码 */
bool gm6020_decode(const uint8_t data[8], uint8_t len, gm6020_raw_t *out) {
  if (data == 0 || out == 0 || len != 8) {
    return false;
  }
  out->ecd = (uint16_t)(((uint16_t)data[0] << 8) | data[1]);
  out->rpm = (int16_t)(((uint16_t)data[2] << 8) | data[3]);
  out->current = (int16_t)(((uint16_t)data[4] << 8) | data[5]);
  out->temperature = data[6];
  return out->ecd < GM6020_ENCODER_RANGE;
}

void gm6020_encode_voltage(uint8_t data[8], uint8_t motor_id, int16_t voltage) {
  if (motor_id < 1 || motor_id > 4) {
    return;
  }
  if (voltage > GM6020_VOLTAGE_MAX) {
    voltage = GM6020_VOLTAGE_MAX;
  } else if (voltage < -GM6020_VOLTAGE_MAX) {
    voltage = -GM6020_VOLTAGE_MAX;
  }
  const uint8_t k = (uint8_t)(2u * (motor_id - 1u));
  data[k] = (uint8_t)((uint16_t)voltage >> 8);
  data[k + 1] = (uint8_t)((uint16_t)voltage & 0xFFu);
}

void gm6020_encode_feedback(uint8_t data[8], const gm6020_raw_t *raw) {
  data[0] = (uint8_t)(raw->ecd >> 8);
  data[1] = (uint8_t)(raw->ecd & 0xFFu);
  data[2] = (uint8_t)((uint16_t)raw->rpm >> 8);
  data[3] = (uint8_t)((uint16_t)raw->rpm & 0xFFu);
  data[4] = (uint8_t)((uint16_t)raw->current >> 8);
  data[5] = (uint8_t)((uint16_t)raw->current & 0xFFu);
  data[6] = raw->temperature;
  data[7] = 0;
}

/* ---------------------------------------------------------------- 角度展开 */
void gm6020_unwrap_init(gm6020_unwrap_t *u, uint16_t offset_ecd, int8_t dir) {
  u->offset_ecd = (uint16_t)(offset_ecd % GM6020_ENCODER_RANGE);
  u->dir = (dir >= 0) ? 1 : -1;
  u->has_last = false;
  u->last_ecd = 0;
  u->turns = 0;
}

float gm6020_unwrap_update(gm6020_unwrap_t *u, uint16_t ecd) {
  if (u->has_last) {
    const int32_t d = (int32_t)ecd - (int32_t)u->last_ecd;
    /* 相邻两帧（1 ms）转子不可能转过半圈：跳变超过半圈一定是跨过了 0/8191 */
    if (d > GM6020_ENCODER_RANGE / 2) {
      u->turns--;
    } else if (d < -GM6020_ENCODER_RANGE / 2) {
      u->turns++;
    }
  } else {
    /* 首帧：选择离零位最近的那一圈，让上电时的角度落在 [-π, π) */
    int32_t rel = (int32_t)ecd - (int32_t)u->offset_ecd;
    u->turns = 0;
    if (rel >= GM6020_ENCODER_RANGE / 2) {
      u->turns = -1;
    } else if (rel < -GM6020_ENCODER_RANGE / 2) {
      u->turns = 1;
    }
  }
  u->last_ecd = ecd;
  u->has_last = true;

  const int32_t total =
      u->turns * GM6020_ENCODER_RANGE + (int32_t)ecd - (int32_t)u->offset_ecd;
  return (float)u->dir * (float)total * (TWO_PI_F / (float)GM6020_ENCODER_RANGE);
}

float gm6020_rpm_to_rads(int16_t rpm, int8_t dir) {
  return (float)((dir >= 0) ? 1 : -1) * (float)rpm * RPM_TO_RADS;
}

/* ---------------------------------------------------------------- 仿真电机 */
void gm6020_model_default_params(gm6020_model_params_t *p) {
  /*
   * 空载转速：ω0 = V/Ke = 24/0.7 ≈ 34 rad/s ≈ 327 rpm，与手册 320 rpm 量级一致
   * 堵转电流：V/R = 24/1.8 ≈ 13 A，被 i_max = 3 A 截断（与电调电流环限幅相当）
   * 转矩常数：0.741 N·m/A（与队内组件库 kGM6020MotorBaseInfo.torq_const 一致）
   */
  p->J = 0.012f;
  p->b = 0.02f;
  p->tau_c = 0.03f;
  p->R = 1.8f;
  p->L = 0.0016f;
  p->Kt = 0.741f;
  p->Ke = 0.70f;
  p->V_bus = 24.0f;
  p->i_max = 3.0f;
}

void gm6020_model_init(gm6020_model_t *m, const gm6020_model_params_t *p, float theta0) {
  m->p = *p;
  m->theta = theta0;
  m->omega = 0.0f;
  m->i = 0.0f;
  m->temperature = 30.0f;
}

void gm6020_model_step(gm6020_model_t *m, int16_t voltage_cmd, float dt) {
  const gm6020_model_params_t *p = &m->p;
  if (!(dt > 0.0f)) {
    return;
  }
  if (voltage_cmd > GM6020_VOLTAGE_MAX) {
    voltage_cmd = GM6020_VOLTAGE_MAX;
  } else if (voltage_cmd < -GM6020_VOLTAGE_MAX) {
    voltage_cmd = -GM6020_VOLTAGE_MAX;
  }
  const float u = (float)voltage_cmd / (float)GM6020_VOLTAGE_MAX * p->V_bus;

  /* 电气时间常数 L/R ≈ 0.9 ms，与 1 ms 步长同量级：用 0.1 ms 子步，避免显式欧拉发散 */
  const int sub = 10;
  const float h = dt / (float)sub;
  for (int k = 0; k < sub; ++k) {
    float di = (u - p->R * m->i - p->Ke * m->omega) / p->L;
    m->i += di * h;
    if (m->i > p->i_max) {
      m->i = p->i_max;
    } else if (m->i < -p->i_max) {
      m->i = -p->i_max;
    }

    float tau = p->Kt * m->i - p->b * m->omega;
    /* 库仑摩擦：静止时能抵消的部分直接抵消，避免在零速附近来回抖 */
    if (fabsf(m->omega) < 1e-3f) {
      if (fabsf(tau) <= p->tau_c) {
        tau = 0.0f;
        m->omega = 0.0f;
      } else {
        tau -= (tau > 0.0f ? p->tau_c : -p->tau_c);
      }
    } else {
      tau -= (m->omega > 0.0f ? p->tau_c : -p->tau_c);
    }
    m->omega += tau / p->J * h;
    m->theta += m->omega * h;
  }

  /* 温度：I²R 发热 + 向 30 °C 环境散热，时间常数几分钟。只为了让温度字段有变化 */
  m->temperature += (m->i * m->i * p->R * 0.02f - (m->temperature - 30.0f) * 0.005f) * dt;
}

void gm6020_model_raw(const gm6020_model_t *m, gm6020_raw_t *out) {
  float turns = m->theta / TWO_PI_F;
  float frac = turns - floorf(turns);
  int32_t ecd = (int32_t)lroundf(frac * (float)GM6020_ENCODER_RANGE);
  if (ecd >= GM6020_ENCODER_RANGE) {
    ecd -= GM6020_ENCODER_RANGE;
  }
  out->ecd = (uint16_t)ecd;
  out->rpm = (int16_t)lroundf(m->omega / RPM_TO_RADS);
  /* 实际电流原始值：±16384 对应 ±3 A（与组件库 curr_rat = 3/16384 一致） */
  out->current = (int16_t)lroundf(m->i / 3.0f * 16384.0f);
  float t = m->temperature;
  if (t < 0.0f) {
    t = 0.0f;
  } else if (t > 255.0f) {
    t = 255.0f;
  }
  out->temperature = (uint8_t)t;
}
