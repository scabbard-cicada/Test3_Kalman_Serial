/**
 * @file    app.c
 * @brief   下位机应用层：把 Shared/ 里与硬件无关的逻辑接到 HAL 上
 *
 * 时序：
 *
 *   TIM6 中断（1 kHz，优先级 2）
 *     ├─ 取出 1 ms 内新到的 C2 指令（邮箱）
 *     ├─ 读电机反馈（CAN 接收中断写入）
 *     ├─ gimbal_ctrl_step()：安全状态机 + 串级 PID
 *     ├─ 发 CAN 控制帧（0x1FF）
 *     └─ 每 10 ms 编码一帧 C1 放进发送队列
 *
 *   USART6 DMA（循环模式）+ IDLE 中断（优先级 1）：只更新写指针，不解析
 *   CAN1 RX0 中断（优先级 1）：解码 GM6020 反馈，写入 g_motor
 *   主循环：从环形缓冲取字节喂给解析器；启动 C1 DMA 发送；刷新指示灯；喂狗
 *
 * 为什么解析放在主循环而不是中断：解析要逐字节跑状态机，放在 IDLE 中断里会让中断变长，
 * 进而推迟 TIM6 控制中断。环形缓冲 256 B，115200 bps 下约 22 ms 才写满，主循环远快于此。
 */
#include "app.h"

#include <math.h>
#include <string.h>

#include "app_config.h"
#include "can.h"
#include "gimbal_ctrl.h"
#include "gimbal_proto.h"
#include "gm6020.h"
#include "iwdg.h"
#include "main.h"
#include "tim.h"
#include "usart.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif
/* 全程单精度：Cortex-M4F 只有单精度 FPU，double 运算会退化成软件库调用 */
#define DEG2RAD(d) ((float)(d) * 0.017453292519943295f)

/* ================================================================ 串口接收 */
#define RX_RING_SIZE 256u
static uint8_t s_rx_ring[RX_RING_SIZE];
static volatile uint16_t s_rx_head; /* DMA 写到哪里（由回调更新） */
static uint16_t s_rx_tail;          /* 主循环读到哪里 */
static volatile uint32_t s_uart_errors;

static gp_parser_t s_parser;

/* 指令邮箱：主循环写、TIM6 读。用序号双检代替关中断 */
static volatile uint32_t s_mbox_seq; /* 奇数 = 正在写 */
static gp_cmd_t s_mbox_cmd;
static volatile uint32_t s_mbox_count; /* 累计写入次数 */

/* ================================================================ 串口发送 */
#define TX_QUEUE_LEN 4u
static uint8_t s_tx_queue[TX_QUEUE_LEN][GP_FB_LEN];
static volatile uint8_t s_tx_head; /* TIM6 写 */
static volatile uint8_t s_tx_tail; /* 主循环读 */
static volatile uint32_t s_tx_drop;
static volatile uint8_t s_tx_busy;
static uint8_t s_tx_dma_buf[GP_FB_LEN];

/* ================================================================ 电机 */
typedef struct {
  volatile uint32_t update_count;
  gm6020_raw_t raw;
} motor_rx_t;
static motor_rx_t s_motor_rx;
static gm6020_unwrap_t s_unwrap;
static volatile uint32_t s_can_tx_fail;
static volatile uint32_t s_can_bus_off;
#if GIMBAL_VIRTUAL_MOTOR
static gm6020_model_t s_model;
#endif

/* ================================================================ 控制 */
static gimbal_ctrl_t s_ctrl;
static gimbal_input_t s_in;
static gimbal_output_t s_out;
static volatile uint32_t s_ms;
static volatile uint8_t s_state_for_led;
static volatile uint8_t s_err_for_led;
static volatile uint8_t s_flags_for_led;

/* 调试用：Ozone / STM32CubeIDE 里直接看这几个变量 */
volatile gm6020_raw_t g_motor_raw;
volatile float g_yaw_fb_deg;
volatile float g_yaw_ref_deg;

/* ---------------------------------------------------------------- 指示灯 */
static void led_rgb(uint16_t r, uint16_t g, uint16_t b) {
  /* TIM5 周期 1000 计数，占空 0~999；共阳 RGB，高电平点亮 */
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_3, r);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2, g);
  __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1, b);
}

static void buzzer(bool on) {
  /* TIM4：84 MHz / 84 / 250 = 4 kHz（无源蜂鸣器额定频率），50% 占空 */
  __HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_3, on ? 125u : 0u);
}

static void laser(bool on) { __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, on ? 999u : 0u); }

/* 蜂鸣器序列：beeps 次，每次 on_ms 响 + off_ms 停 */
static struct {
  uint8_t remaining;
  uint16_t on_ms;
  uint16_t off_ms;
  uint32_t t0;
} s_beep;

static void beep(uint8_t times, uint16_t on_ms, uint16_t off_ms) {
  s_beep.remaining = times;
  s_beep.on_ms = on_ms;
  s_beep.off_ms = off_ms;
  s_beep.t0 = s_ms;
}

static void indicator_update(void) {
  const uint32_t now = s_ms;
  const uint8_t state = s_state_for_led;
  const uint8_t err = s_err_for_led;
  const uint32_t rx_age = now - s_ctrl.last_cmd_ms;
  const bool link = s_ctrl.has_cmd && rx_age < 200u;
  const bool blink1 = ((now / 500u) & 1u) != 0u; /* 1 Hz */
  const bool blink5 = ((now / 100u) & 1u) != 0u; /* 5 Hz */

  switch (state) {
    case GP_STATE_SAFE:
      if (!link) {
        led_rgb(0, 0, blink1 ? 400u : 0u); /* 蓝灯慢闪：没有串口数据 */
      } else if (err & GP_ERR_MOTOR_OFFLINE) {
        led_rgb(blink1 ? 300u : 0u, 0, 400u); /* 蓝紫交替：有数据但电机不在线 */
      } else {
        led_rgb(0, 0, 400u); /* 蓝灯常亮：链路通了，等按键解锁 */
      }
      break;
    case GP_STATE_ARMED:
      if (s_flags_for_led & GP_FLAG_OUTLIER) {
        led_rgb(500u, 400u, 0); /* 黄灯闪一下：上位机报离群点 */
      } else if (s_ctrl.last_cmd.mode == GP_MODE_TRACK &&
                 (s_ctrl.last_cmd.flags & GP_FLAG_TARGET_VALID)) {
        /* 绿灯，亮度随跟踪误差变化：误差越大越暗 */
        float e = fabsf(g_yaw_ref_deg - g_yaw_fb_deg);
        float k = 1.0f - e / 5.0f;
        if (k < 0.15f) {
          k = 0.15f;
        }
        led_rgb(0, (uint16_t)(600.0f * k), 0);
      } else {
        led_rgb(blink1 ? 500u : 0u, blink1 ? 400u : 0u, 0); /* 黄灯慢闪：HOLD / 目标丢失 */
      }
      break;
    case GP_STATE_TIMEOUT:
      led_rgb(blink5 ? 600u : 0u, 0, 0); /* 红灯快闪 */
      break;
    case GP_STATE_FAULT:
    default:
      led_rgb(600u, 0, 0); /* 红灯常亮 */
      break;
  }

  /* 蜂鸣器 */
  if (s_beep.remaining > 0u) {
    const uint32_t period = (uint32_t)s_beep.on_ms + s_beep.off_ms;
    const uint32_t t = now - s_beep.t0;
    if (t >= period) {
      s_beep.remaining--;
      s_beep.t0 = now;
      buzzer(false);
    } else {
      buzzer(t < s_beep.on_ms);
    }
  } else {
    buzzer(false);
  }
}

/* ---------------------------------------------------------------- CAN */
static void can_filter_init(void) {
  CAN_FilterTypeDef f;
  memset(&f, 0, sizeof(f));
  /* 只收本电机的反馈帧：标识符列表模式，标准帧 ID 左移 5 位放在高 16 位 */
  const uint32_t id = GM6020_FB_ID_BASE + GIMBAL_MOTOR_ID;
  f.FilterBank = 0;
  f.FilterMode = CAN_FILTERMODE_IDLIST;
  f.FilterScale = CAN_FILTERSCALE_16BIT;
  f.FilterIdHigh = (uint16_t)(id << 5);
  f.FilterIdLow = (uint16_t)(id << 5);
  f.FilterMaskIdHigh = (uint16_t)(id << 5);
  f.FilterMaskIdLow = (uint16_t)(id << 5);
  f.FilterFIFOAssignment = CAN_RX_FIFO0;
  f.FilterActivation = ENABLE;
  f.SlaveStartFilterBank = 14;
  if (HAL_CAN_ConfigFilter(&hcan1, &f) != HAL_OK) {
    Error_Handler();
  }
}

static void can_send_voltage(int16_t v) {
#if GIMBAL_VIRTUAL_MOTOR
  (void)v;
#else
  CAN_TxHeaderTypeDef h;
  uint8_t data[8] = {0};
  uint32_t mailbox;
  h.StdId = GM6020_CTRL_ID_1_4;
  h.ExtId = 0;
  h.IDE = CAN_ID_STD;
  h.RTR = CAN_RTR_DATA;
  h.DLC = 8;
  h.TransmitGlobalTime = DISABLE;
  gm6020_encode_voltage(data, GIMBAL_MOTOR_ID, v);
  if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0u ||
      HAL_CAN_AddTxMessage(&hcan1, &h, data, &mailbox) != HAL_OK) {
    s_can_tx_fail++;
  }
#endif
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan) {
  CAN_RxHeaderTypeDef h;
  uint8_t data[8];
  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0u) {
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &h, data) != HAL_OK) {
      break;
    }
    if (h.IDE == CAN_ID_STD && h.StdId == GM6020_FB_ID_BASE + GIMBAL_MOTOR_ID) {
      gm6020_raw_t r;
      if (gm6020_decode(data, (uint8_t)h.DLC, &r)) {
        s_motor_rx.raw = r;
        s_motor_rx.update_count++;
      }
    }
  }
}

void HAL_CAN_ErrorCallback(CAN_HandleTypeDef *hcan) {
  /* ABOM=ENABLE：bus-off 由硬件自动恢复，这里只计数，不重新初始化（重新初始化会丢掉过滤器和中断使能） */
  if ((hcan->ErrorCode & HAL_CAN_ERROR_BOF) != 0u) {
    s_can_bus_off++;
  }
  HAL_CAN_ResetError(hcan);
}

/* ---------------------------------------------------------------- 串口 */
static void uart_rx_start(void) {
  s_rx_head = 0;
  s_rx_tail = 0;
  /* 循环 DMA + IDLE：HAL 在 IDLE、半满、全满三种事件时调用 RxEventCallback，Size 为 DMA 当前写指针 */
  if (HAL_UARTEx_ReceiveToIdle_DMA(&huart6, s_rx_ring, RX_RING_SIZE) != HAL_OK) {
    s_uart_errors++;
  }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size) {
  if (huart->Instance == USART6) {
    s_rx_head = (uint16_t)(size % RX_RING_SIZE);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART6) {
    /* ORE/FE/NE 时 HAL 会停掉 DMA 接收：清标志后重新开启，否则串口从此再也收不到数据 */
    s_uart_errors++;
    __HAL_UART_CLEAR_OREFLAG(huart);
    HAL_UART_AbortReceive(huart);
    uart_rx_start();
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
  if (huart->Instance == USART6) {
    s_tx_busy = 0;
  }
}

static void on_cmd_frame(const uint8_t *frame, size_t len, void *user) {
  (void)user;
  gp_cmd_t c;
  if (!gp_decode_cmd(frame, len, &c)) {
    return;
  }
  /* 序号双检：写之前和写完都加 1，读方看到奇数或前后不一致就重读 */
  s_mbox_seq++;
  __DMB();
  s_mbox_cmd = c;
  __DMB();
  s_mbox_seq++;
  s_mbox_count++;
}

static void uart_poll_rx(void) {
  const uint16_t head = s_rx_head;
  while (s_rx_tail != head) {
    uint16_t end = (head > s_rx_tail) ? head : RX_RING_SIZE;
    gp_parser_feed(&s_parser, &s_rx_ring[s_rx_tail], (size_t)(end - s_rx_tail), on_cmd_frame, 0);
    s_rx_tail = (uint16_t)(end % RX_RING_SIZE);
  }
}

static void uart_poll_tx(void) {
  if (s_tx_busy || s_tx_tail == s_tx_head) {
    return;
  }
  memcpy(s_tx_dma_buf, s_tx_queue[s_tx_tail], GP_FB_LEN);
  s_tx_tail = (uint8_t)((s_tx_tail + 1u) % TX_QUEUE_LEN);
  s_tx_busy = 1;
  if (HAL_UART_Transmit_DMA(&huart6, s_tx_dma_buf, GP_FB_LEN) != HAL_OK) {
    s_tx_busy = 0;
  }
}

/** TIM6 中断里调用：把一帧 C1 放进队列，满了丢最旧的 */
static void tx_enqueue(const gp_fb_t *fb) {
  uint8_t next = (uint8_t)((s_tx_head + 1u) % TX_QUEUE_LEN);
  if (next == s_tx_tail) {
    s_tx_tail = (uint8_t)((s_tx_tail + 1u) % TX_QUEUE_LEN);
    s_tx_drop++;
  }
  gp_encode_fb(fb, s_tx_queue[s_tx_head], GP_FB_LEN);
  s_tx_head = next;
}

/* ---------------------------------------------------------------- 控制中断 */
static uint32_t s_last_mbox_count;
static uint32_t s_last_motor_count;
static uint8_t s_prev_state = GP_STATE_SAFE;

static void control_tick(void) {
  s_ms++;
  const uint32_t now = s_ms;

  /* 1. 取指令 */
  s_in.new_cmd = false;
  if (s_mbox_count != s_last_mbox_count) {
    uint32_t a, b;
    gp_cmd_t c;
    do {
      a = s_mbox_seq;
      __DMB();
      c = s_mbox_cmd;
      __DMB();
      b = s_mbox_seq;
    } while ((a & 1u) || a != b);
    s_last_mbox_count = s_mbox_count;
    s_in.cmd = c;
    s_in.new_cmd = true;
    s_flags_for_led = c.flags;
  }

  /* 2. 读电机 */
#if GIMBAL_VIRTUAL_MOTOR
  gm6020_raw_t raw;
  gm6020_model_raw(&s_model, &raw);
  s_motor_rx.raw = raw;
  s_motor_rx.update_count++;
#endif
  s_in.motor_fresh = (s_motor_rx.update_count != s_last_motor_count);
  s_last_motor_count = s_motor_rx.update_count;
  if (s_in.motor_fresh) {
    const gm6020_raw_t r = s_motor_rx.raw;
    s_in.yaw_fb = gm6020_unwrap_update(&s_unwrap, r.ecd);
    s_in.yaw_vel_fb = gm6020_rpm_to_rads(r.rpm, GIMBAL_DIR);
    s_in.current_raw = r.current;
    s_in.temperature = r.temperature;
    g_motor_raw = r;
  }

  /* 3. 其它输入 */
  s_in.now_ms = now;
  s_in.key_pressed = (HAL_GPIO_ReadPin(KEY_GPIO_Port, KEY_Pin) == GPIO_PIN_RESET);
  s_in.can_error = (s_can_tx_fail != 0u) || (s_can_bus_off != 0u);
  s_in.uart_error = (s_uart_errors != 0u);

  /* 4. 控制 */
  gimbal_ctrl_step(&s_ctrl, &s_in, &s_out);

  /* 5. 输出 */
  const int16_t v = s_out.motor_enable ? s_out.voltage : 0;
  can_send_voltage(v);
#if GIMBAL_VIRTUAL_MOTOR
  gm6020_model_step(&s_model, v, 0.001f);
#endif
  laser(s_out.laser_on);

  g_yaw_fb_deg = s_in.yaw_fb * 57.29578f;
  g_yaw_ref_deg = s_out.yaw_ref * 57.29578f;
  s_state_for_led = (uint8_t)s_out.state;
  s_err_for_led = s_out.error_flags;

  /* 状态变化时的提示音 */
  if ((uint8_t)s_out.state != s_prev_state) {
    switch (s_out.state) {
      case GP_STATE_ARMED:
        beep(2, 60, 60);
        break;
      case GP_STATE_TIMEOUT:
        beep(1, 300, 0);
        break;
      case GP_STATE_FAULT:
        beep(3, 80, 80);
        break;
      default:
        beep(1, 40, 0);
        break;
    }
    s_prev_state = (uint8_t)s_out.state;
  }

  /* 6. 回传 */
  if ((now % GIMBAL_FB_PERIOD_MS) == 0u) {
    gp_fb_t fb;
    gimbal_ctrl_fill_feedback(&s_ctrl, &s_in, &s_out, &fb);
    tx_enqueue(&fb);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
  if (htim->Instance == TIM6) {
    control_tick();
  }
}

/* ---------------------------------------------------------------- 入口 */
void App_Init(void) {
  /* 上次复位是否来自看门狗：回传给上位机，说明程序曾经卡死 */
  const bool wdg_reset = (__HAL_RCC_GET_FLAG(RCC_FLAG_IWDGRST) != RESET);
  __HAL_RCC_CLEAR_RESET_FLAGS();

  gimbal_config_t cfg;
  gimbal_ctrl_default_config(&cfg);
  cfg.yaw_min = DEG2RAD(GIMBAL_YAW_MIN_DEG);
  cfg.yaw_max = DEG2RAD(GIMBAL_YAW_MAX_DEG);
  cfg.yaw_vel_max = DEG2RAD(GIMBAL_YAW_VEL_MAX_DPS);
  cfg.output_scale = GIMBAL_OUTPUT_SCALE;
  cfg.angle_pid.kp = ANGLE_KP;
  cfg.angle_pid.ki = ANGLE_KI;
  cfg.angle_pid.kd = ANGLE_KD;
  cfg.angle_pid.out_max = ANGLE_OUT_MAX;
  cfg.speed_pid.kp = SPEED_KP;
  cfg.speed_pid.ki = SPEED_KI;
  cfg.speed_pid.kd = SPEED_KD;
  cfg.speed_pid.i_max = SPEED_I_MAX;
  cfg.speed_pid.out_max = SPEED_OUT_MAX;
  gimbal_ctrl_init(&s_ctrl, &cfg, 0);
  s_ctrl.wdg_reset = wdg_reset;

  memset(&s_in, 0, sizeof(s_in));
  gm6020_unwrap_init(&s_unwrap, GIMBAL_ZERO_ECD, GIMBAL_DIR);
#if GIMBAL_VIRTUAL_MOTOR
  {
    gm6020_model_params_t mp;
    gm6020_model_default_params(&mp);
    /* 虚拟电机的转子角和编码器零位对齐，上电时云台角 = 0 */
    gm6020_model_init(&s_model, &mp,
                      (float)GIMBAL_ZERO_ECD * (float)(2.0 * M_PI / GM6020_ENCODER_RANGE));
  }
#endif

  gp_parser_init(&s_parser, GP_HEADER_CMD);

  /* PWM：RGB、蜂鸣器、激光 */
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(&htim5, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
  HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
  led_rgb(0, 0, 0);
  buzzer(false);
  laser(false);

  /* 上电自检：白灯闪一下 + 短鸣一声（阻塞约 150 ms，此时 TIM6 还没启动，看门狗约 300 ms） */
  led_rgb(500, 500, 500);
  buzzer(true);
  HAL_Delay(80);
  buzzer(false);
  HAL_Delay(70);
  led_rgb(0, 0, 0);
  HAL_IWDG_Refresh(&hiwdg);

  /* CAN */
  can_filter_init();
  HAL_CAN_Start(&hcan1);
  HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING | CAN_IT_ERROR | CAN_IT_BUSOFF |
                                           CAN_IT_LAST_ERROR_CODE);

  /* 串口 */
  uart_rx_start();

  /* TIM2：1 MHz 自由计数，用作微秒时间戳（调试测量中断耗时） */
  HAL_TIM_Base_Start(&htim2);
  /* TIM6：1 kHz 控制中断，最后启动 */
  HAL_TIM_Base_Start_IT(&htim6);
}

void App_Loop(void) {
  uart_poll_rx();
  uart_poll_tx();
  indicator_update();
  /* 看门狗只在主循环里喂：中断里死循环会让主循环停住，约 300 ms 后自动复位 */
  HAL_IWDG_Refresh(&hiwdg);
}
