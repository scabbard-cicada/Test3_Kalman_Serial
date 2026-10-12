/**
 * @file    gimbal_ctrl.c
 * @brief   见 gimbal_ctrl.h
 */
#include "gimbal_ctrl.h"

#include <math.h>
#include <string.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define DEG2RAD(d) ((float)(d) * 0.017453292519943295f)

static uint32_t elapsed(uint32_t now, uint32_t then) { return (uint32_t)(now - then); }

void gimbal_ctrl_default_config(gimbal_config_t *cfg) {
  memset(cfg, 0, sizeof(*cfg));
  /* 推荐站点下目标相对 yaw 为 -14.4° ~ 25.0°，限位留余量 */
  cfg->yaw_min = DEG2RAD(-35.0);
  cfg->yaw_max = DEG2RAD(35.0);
  /* 轨迹最快约 62°/s，留 2 倍余量 */
  cfg->yaw_vel_max = DEG2RAD(120.0);
  cfg->interp_max_ms = 30;
  cfg->cmd_timeout_ms = 100;
  cfg->relax_timeout_ms = 500;
  cfg->arm_fresh_ms = 200;
  cfg->key_debounce_ms = 20;
  cfg->motor_offline_ms = 50;
  cfg->over_temp_c = 70;
  cfg->bad_float_limit = 3;
  cfg->output_scale = 1.0f;

  /*
   * 默认增益：
   *   - 角度环只用 P：误差 1°（0.0175 rad）→ 0.35 rad/s 的速度修正
   *   - 速度环 PI：1 rad/s 误差 → 3000 电压给定；积分消除摩擦和负载带来的稳态误差
   * 在 gm6020.c 的仿真模型上，这组参数第三圈跟踪误差 RMS 约 0.05°，10° 阶跃超调 < 0.3°。
   * 实物有负载、摩擦更大，上台架后按 README 的步骤从速度环开始重新整定。
   */
  cfg->angle_pid.kp = 20.0f;
  cfg->angle_pid.ki = 0.0f;
  cfg->angle_pid.kd = 0.0f;
  cfg->angle_pid.out_max = 10.0f; /* rad/s，角度环输出的修正速度上限 */
  cfg->angle_pid.d_lpf_alpha = 1.0f;
  cfg->angle_pid.angle_wrap = true;

  cfg->speed_pid.kp = 3000.0f;
  cfg->speed_pid.ki = 20000.0f;
  cfg->speed_pid.kd = 0.0f;
  cfg->speed_pid.i_max = 8000.0f;
  cfg->speed_pid.i_sep = 0.0f;
  cfg->speed_pid.out_max = (float)GM6020_VOLTAGE_MAX_F;
  cfg->speed_pid.d_lpf_alpha = 1.0f;
  cfg->speed_pid.angle_wrap = false;
}

void gimbal_ctrl_init(gimbal_ctrl_t *g, const gimbal_config_t *cfg, uint32_t now_ms) {
  memset(g, 0, sizeof(*g));
  g->cfg = *cfg;
  if (!(g->cfg.output_scale > 0.0f) || g->cfg.output_scale > 1.0f) {
    g->cfg.output_scale = 1.0f;
  }
  pid_init(&g->angle_pid, &g->cfg.angle_pid);
  pid_init(&g->speed_pid, &g->cfg.speed_pid);
  g->state = GP_STATE_SAFE;
  g->key_change_ms = now_ms;
}

/* ---------------------------------------------------------------- 内部 */
static bool cmd_is_finite(const gp_cmd_t *c) { return isfinite(c->yaw) && isfinite(c->yaw_vel); }

static void update_key(gimbal_ctrl_t *g, bool raw, uint32_t now) {
  g->key_edge = false;
  if (raw != g->key_raw_last) {
    g->key_raw_last = raw;
    g->key_change_ms = now;
  }
  if (raw != g->key_stable && elapsed(now, g->key_change_ms) >= g->cfg.key_debounce_ms) {
    g->key_stable = raw;
    if (raw) {
      g->key_edge = true; /* 按下沿 */
    }
  }
}

/** 接收新指令：拒绝非有限值；有限但越界的值钳位（不丢帧，不报故障） */
static void accept_cmd(gimbal_ctrl_t *g, const gimbal_input_t *in) {
  if (!in->new_cmd) {
    return;
  }
  if (!cmd_is_finite(&in->cmd)) {
    if (g->bad_float_count < 255u) {
      g->bad_float_count++;
    }
    g->sticky_errors |= GP_ERR_BAD_FLOAT;
    return;
  }
  g->bad_float_count = 0;
  g->last_cmd = in->cmd;
  g->cmd_clamped = false;
  if (g->last_cmd.yaw < g->cfg.yaw_min || g->last_cmd.yaw > g->cfg.yaw_max) {
    g->last_cmd.yaw = pid_clampf(g->last_cmd.yaw, g->cfg.yaw_min, g->cfg.yaw_max);
    g->cmd_clamped = true;
  }
  g->last_cmd.yaw_vel = pid_clampf(g->last_cmd.yaw_vel, -g->cfg.yaw_vel_max, g->cfg.yaw_vel_max);
  g->last_cmd_ms = in->now_ms;
  g->has_cmd = true;
}

static uint8_t detect_faults(gimbal_ctrl_t *g, const gimbal_input_t *in) {
  uint8_t f = 0;
  if (!g->has_motor || elapsed(in->now_ms, g->last_motor_ms) > g->cfg.motor_offline_ms) {
    f |= GP_ERR_MOTOR_OFFLINE;
  }
  if (g->has_motor && in->temperature >= g->cfg.over_temp_c) {
    f |= GP_ERR_OVER_TEMP;
  }
  if (g->bad_float_count >= g->cfg.bad_float_limit) {
    f |= GP_ERR_BAD_FLOAT;
  }
  return f;
}

static void enter(gimbal_ctrl_t *g, gp_state_t s, float yaw_fb) {
  if (g->state == s) {
    return;
  }
  /* 进入任何会驱动电机的状态时都清空 PID 历史，并让参考从当前位置开始，
     避免积分残留或参考跳变造成冲击 */
  pid_reset(&g->angle_pid);
  pid_reset(&g->speed_pid);
  g->yaw_ref = yaw_fb;
  g->ref_init = true;
  g->hold_yaw = yaw_fb;
  g->state = s;
}

/* ---------------------------------------------------------------- 主循环 */
void gimbal_ctrl_step(gimbal_ctrl_t *g, const gimbal_input_t *in, gimbal_output_t *out) {
  const uint32_t now = in->now_ms;
  const float dt = 0.001f;

  if (in->motor_fresh) {
    g->last_motor_ms = now;
    g->has_motor = true;
  }
  update_key(g, in->key_pressed, now);
  accept_cmd(g, in);

  const bool cmd_ever = g->has_cmd;
  const uint32_t cmd_age = cmd_ever ? elapsed(now, g->last_cmd_ms) : UINT32_MAX;
  const uint8_t faults = detect_faults(g, in);

  /* ------------------------------------------------ 安全状态机 */
  if (faults & (GP_ERR_OVER_TEMP | GP_ERR_BAD_FLOAT)) {
    enter(g, GP_STATE_FAULT, in->yaw_fb);
  } else if ((faults & GP_ERR_MOTOR_OFFLINE) && g->state != GP_STATE_SAFE) {
    /* SAFE 下电机离线只报错不进 FAULT：没接电机时也能看串口和指示灯 */
    enter(g, GP_STATE_FAULT, in->yaw_fb);
  }

  switch (g->state) {
    case GP_STATE_SAFE:
      if (g->key_edge && cmd_age <= g->cfg.arm_fresh_ms && faults == 0) {
        enter(g, GP_STATE_ARMED, in->yaw_fb);
      }
      break;
    case GP_STATE_ARMED:
      if (cmd_age > g->cfg.cmd_timeout_ms) {
        enter(g, GP_STATE_TIMEOUT, in->yaw_fb);
      }
      break;
    case GP_STATE_TIMEOUT:
      if (cmd_age <= g->cfg.cmd_timeout_ms) {
        enter(g, GP_STATE_ARMED, in->yaw_fb);
      } else if (cmd_age > g->cfg.relax_timeout_ms) {
        enter(g, GP_STATE_SAFE, in->yaw_fb);
      }
      break;
    case GP_STATE_FAULT:
      /* 故障消失后需按键确认，回到 SAFE（不直接回 ARMED，需要再按一次解锁） */
      if (g->key_edge && faults == 0) {
        g->bad_float_count = 0;
        g->sticky_errors = 0;
        enter(g, GP_STATE_SAFE, in->yaw_fb);
      }
      break;
  }

  /* ------------------------------------------------ 参考生成 */
  bool drive = false;
  float target = in->yaw_fb;
  float ffd = 0.0f;
  bool laser = false;

  if (g->state == GP_STATE_ARMED) {
    switch (g->last_cmd.mode) {
      case GP_MODE_TRACK: {
        /* 两帧之间按角速度外推，外推时长有上限 */
        uint32_t tau_ms = cmd_age < g->cfg.interp_max_ms ? cmd_age : g->cfg.interp_max_ms;
        target = g->last_cmd.yaw + g->last_cmd.yaw_vel * (float)tau_ms * 0.001f;
        ffd = g->last_cmd.yaw_vel;
        g->hold_yaw = in->yaw_fb;
        drive = true;
        laser = (g->last_cmd.flags & GP_FLAG_LASER_ON) && (g->last_cmd.flags & GP_FLAG_TARGET_VALID);
        break;
      }
      case GP_MODE_HOLD:
        target = g->hold_yaw;
        drive = true;
        break;
      case GP_MODE_RELAX:
      default:
        g->hold_yaw = in->yaw_fb;
        drive = false;
        break;
    }
  } else if (g->state == GP_STATE_TIMEOUT) {
    target = g->hold_yaw;
    drive = true;
  }

  float voltage = 0.0f;
  if (drive) {
    target = pid_clampf(target, g->cfg.yaw_min, g->cfg.yaw_max);
    /* 参考变化率限幅：即使上位机给出跳变，送进角度环的参考也只按 yaw_vel_max 移动 */
    if (!g->ref_init) {
      g->yaw_ref = in->yaw_fb;
      g->ref_init = true;
    }
    const float step = g->cfg.yaw_vel_max * dt;
    g->yaw_ref += pid_clampf(target - g->yaw_ref, -step, step);

    const float w_ref = pid_calc(&g->angle_pid, g->yaw_ref, in->yaw_fb, ffd, dt);
    const float w_ref_lim = pid_clampf(w_ref, -g->cfg.yaw_vel_max, g->cfg.yaw_vel_max);
    voltage = pid_calc(&g->speed_pid, w_ref_lim, in->yaw_vel_fb, 0.0f, dt);
    voltage *= g->cfg.output_scale;
  } else {
    pid_reset(&g->angle_pid);
    pid_reset(&g->speed_pid);
    g->yaw_ref = in->yaw_fb;
    g->ref_init = true;
  }

  /* ------------------------------------------------ 输出 */
  uint8_t err = faults;
  if (g->cmd_clamped && g->state == GP_STATE_ARMED) {
    err |= GP_ERR_CMD_CLAMPED;
  }
  if (cmd_ever && cmd_age > g->cfg.cmd_timeout_ms) {
    err |= GP_ERR_CMD_TIMEOUT;
  }
  if (in->can_error) {
    err |= GP_ERR_CAN;
  }
  if (in->uart_error) {
    err |= GP_ERR_UART;
  }
  if (g->wdg_reset) {
    err |= GP_ERR_WDG_RESET;
  }
  err |= g->sticky_errors;

  out->voltage = (int16_t)lroundf(pid_clampf(voltage, -GM6020_VOLTAGE_MAX_F, GM6020_VOLTAGE_MAX_F));
  out->motor_enable = drive;
  out->laser_on = laser;
  out->state = g->state;
  out->error_flags = err;
  out->yaw_ref = drive ? g->yaw_ref : in->yaw_fb;
}

void gimbal_ctrl_fill_feedback(const gimbal_ctrl_t *g, const gimbal_input_t *in,
                               const gimbal_output_t *out, gp_fb_t *fb) {
  fb->yaw = in->yaw_fb;
  fb->yaw_vel = in->yaw_vel_fb;
  fb->yaw_ref = out->yaw_ref;
  fb->current = in->current_raw;
  fb->temperature = in->temperature;
  fb->state = (uint8_t)out->state;
  fb->error_flags = out->error_flags;
  fb->echo_seq = g->has_cmd ? g->last_cmd.seq : 0u;
  fb->echo_host_time_ms = g->has_cmd ? g->last_cmd.host_time_ms : 0u;
}
